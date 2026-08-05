#!/usr/bin/env python3
"""CSI 인코더를 ESP32 가 읽는 형식으로 내보낸다.

export.py 의 BN 폴딩과 참조 구현을 그대로 쓴다 — 인코더 구조가 같으므로 재발명할
이유가 없다. 다른 것은 헤더 상수뿐이다: 멜 필터뱅크·FFT·Hann 대신 CSI 창 파라미터를 낸다.

세 단계 수치 검증도 그대로 이어간다:
  1. BN 폴딩      torch 원본 ↔ numpy 참조
  2. C 이식        numpy 참조 ↔ gcc 로 컴파일한 cn_infer.c
  3. 온보드 검증   selftest.h 로 부팅 시 자동
"""
from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import numpy as np
import torch

import csi_features as CF
from export import MAGIC, RefRunner, c_float_array, collect
from train import Encoder


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="model/out_csi/csi_encoder.pt")
    ap.add_argument("--protos", default="model/out_csi/csi_prototypes.npz")
    ap.add_argument("--outdir", default="firmware/csi_infer")
    ap.add_argument("--bindir", default="model/out_csi")
    args = ap.parse_args()

    ck = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    model = Encoder(dim=ck["dim"], w=ck["width"])
    model.load_state_dict(ck["model"])
    model.eval()
    T, Fq = int(ck["n_frames"]), int(ck["n_feat"])

    layers = collect(model)
    ref = RefRunner(layers)

    # ── 1) BN 폴딩 검증
    rng = np.random.default_rng(0)
    win = np.clip(rng.standard_normal((T, Fq)) * 1.0, -CF.CLIP, CF.CLIP).astype(np.float32)
    with torch.no_grad():
        want = model(torch.from_numpy(win)[None]).numpy()[0]
    got = ref(win)
    err = float(np.abs(want - got).max())
    cos = float(want @ got / (np.linalg.norm(want) * np.linalg.norm(got) + 1e-12))
    print(f"BN 폴딩 검증: 최대오차 {err:.3e}, 코사인 {cos:.8f}")
    if err > 2e-4:
        print("  폴딩이 어긋났다. 중단.")
        return 1

    # ── 2) 바이너리
    tensors = [(L["name"], L["w"], L["b"]) for L in layers if "w" in L]
    blob = bytearray()
    meta = []
    for name, w, bb in tensors:
        meta.append({"name": name, "shape": list(w.shape),
                     "w_off": len(blob), "w_len": int(w.size)})
        blob += np.asarray(w, np.float32).tobytes()
        meta[-1]["b_off"] = len(blob); meta[-1]["b_len"] = int(bb.size)
        blob += np.asarray(bb, np.float32).tobytes()

    bindir = Path(args.bindir); bindir.mkdir(parents=True, exist_ok=True)
    (bindir / "csi_encoder.bin").write_bytes(
        struct.pack("<IIII", MAGIC, 1, ck["dim"], len(tensors)) + bytes(blob))
    n_par = sum(int(w.size + b.size) for _, w, b in tensors)
    print(f"csi_encoder.bin: 텐서 {len(tensors)}개, 파라미터 {n_par/1000:.1f}K, "
          f"{(len(blob)+16)/1024:.1f}KB")

    # ── 3) C 헤더. cn_infer.c 가 model_data.h 에서 기대하는 이름을 그대로 쓴다.
    outdir = Path(args.outdir); outdir.mkdir(parents=True, exist_ok=True)
    h = ["// 자동 생성 — model/scripts/export_csi.py. 직접 수정하지 말 것.",
         "// CSI 창 분류기. cn_infer.c 를 그대로 쓰되 프런트엔드가 로그멜이 아니라",
         "// CSI 진폭 창이다 (csi_features.py 참조).",
         "#pragma once", "#include <stdint.h>", "",
         f"#define CN_N_FRAMES    {T}     // 리샘플 후 시간 프레임",
         f"#define CN_N_MELS      {Fq}     // 여기서는 서브캐리어 수",
         f"#define CN_EMB_DIM     {ck['dim']}",
         f"#define CN_N_TENSORS   {len(tensors)}",
         f"#define CN_N_CLASS     {int(ck['n_cls'])}", "",
         "// CSI 창 파라미터 — csi_features.py 와 반드시 같아야 한다",
         f"#define CSI_WIN_SEC    {CF.WIN_SEC}f",
         f"#define CSI_HOP_SEC    {CF.HOP_SEC}f",
         f"#define CSI_MIN_PKT    {CF.MIN_PKT}",
         f"#define CSI_EPS        {CF.EPS}f",
         f"#define CSI_CLIP       {CF.CLIP}f", "",
         "// 레이어 실행 순서. kind: 0=conv 1=dwconv 2=pwconv 3=gap 4=fc 5=l2norm",
         "typedef struct { uint8_t kind; uint8_t sh, sw; uint8_t relu;"
         " uint16_t cout, cin, kh, kw; uint32_t w_off, b_off; } cn_layer_t;"]
    rows, ti = [], 0
    for L in layers:
        kind = {"conv": 0, "dwconv": 1, "pwconv": 2, "gap": 3, "fc": 4, "l2norm": 5}[L["kind"]]
        if "w" in L:
            m = meta[ti]; ti += 1
            sh_, sw_ = L["stride"]
            if L["kind"] == "fc":
                cout, cin, kh, kw = m["shape"][0], m["shape"][1], 1, 1
            else:
                cout, cin, kh, kw = m["shape"]
            rows.append(f"  {{ {kind}, {sh_}, {sw_}, {int(L['relu'])}, {cout}, {cin}, "
                        f"{kh}, {kw}, {m['w_off']}, {m['b_off']} }},  // {L['name']}")
        else:
            rows.append(f"  {{ {kind}, 1, 1, 0, 0, 0, 0, 0, 0, 0 }},  // {L['name']}")
    h.append(f"#define CN_N_LAYERS {len(rows)}")
    h.append("static const cn_layer_t cn_layers[CN_N_LAYERS] = {\n" + "\n".join(rows) + "\n};\n")
    h.append(c_float_array("cn_weights",
                           np.frombuffer(bytes(blob), dtype=np.float32)))
    (outdir / "model_data.h").write_text("\n".join(h), encoding="utf-8")
    print(f"model_data.h: {(outdir/'model_data.h').stat().st_size/1024/1024:.2f}MB 소스 "
          f"→ 플래시 {len(blob)/1024:.0f}KB")

    # ── 4) 프로토타입
    pz = np.load(args.protos)
    P = pz["protos"].astype(np.float32)
    ph = ["// 자동 생성 — model/scripts/export_csi.py",
          "// 클래스 프로토타입(L2 정규화). cn_match() 에 그대로 넘긴다.",
          "#pragma once", "#include <stdint.h>", "",
          f"#define CN_N_PROTO   {len(P)}", "",
          c_float_array("cn_protos", P),
          "static const uint8_t cn_proto_class[CN_N_PROTO] = {"
          + ", ".join(str(int(v)) for v in pz["labels"]) + "};\n"]
    (outdir / "prototypes.h").write_text("\n".join(ph), encoding="utf-8")
    print(f"prototypes.h: {len(P)}×{P.shape[1]} float = {P.nbytes/1024:.1f}KB")

    # ── 5) 자기검증 벡터
    st = ["// 자동 생성 — 부팅 시 추론 경로 검증용.",
          "#pragma once", "",
          c_float_array("cn_selftest_mel", win),
          c_float_array("cn_selftest_emb", want),
          "#define CN_SELFTEST_TOL 2.0e-3f", ""]
    (outdir / "selftest.h").write_text("\n".join(st), encoding="utf-8")
    (bindir / "csi_layers.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    print(f"selftest.h:   {(outdir/'selftest.h').stat().st_size/1024:.1f}KB")
    print(f"\n→ {outdir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
