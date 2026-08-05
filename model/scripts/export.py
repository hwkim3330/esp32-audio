#!/usr/bin/env python3
"""학습된 인코더를 ESP32 가 읽는 형식으로 내보낸다.

세 가지를 낸다.

  1. encoder.bin   BatchNorm 을 conv 에 접어 넣은 float32 가중치 (헤더 + 텐서)
  2. model_data.h  레이어 구조와 상수 (C 쪽이 파싱 없이 바로 쓰도록)
  3. selftest.h    입력 멜 1개 + 기대 임베딩. 보드가 부팅 시 스스로 검증한다.

3번이 중요하다. 임베디드 추론에서 가장 흔한 실패는 "돌긴 도는데 값이 미묘하게 다름"
이고, 그건 데모 중에 정확도 문제로 위장해서 나타난다. 보드가 부팅할 때 알려진
입력으로 알려진 출력이 나오는지 스스로 확인하면 그 부류를 전부 걸러낸다.

BN 폴딩은 필수다. 추론 시 BN 은 채널별 스케일+시프트일 뿐이므로 conv 가중치에
곱해 넣으면 레이어가 사라진다 — 연산과 코드가 같이 줄어든다.
"""
from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

import features as FE
from train import DSConv, Encoder

MAGIC = 0x4E424143  # 'CABN'


def fold_conv_bn(conv: nn.Conv2d, bn: nn.BatchNorm2d):
    """conv(bias 없음) + bn → (W', b'). 추론에서 수치적으로 동일하다."""
    w = conv.weight.detach().double()
    g = bn.weight.detach().double()
    b = bn.bias.detach().double()
    mu = bn.running_mean.detach().double()
    var = bn.running_var.detach().double()
    s = g / torch.sqrt(var + bn.eps)                 # 채널별 스케일
    w2 = w * s.reshape(-1, 1, 1, 1)
    b2 = b - mu * s
    return w2.float().numpy(), b2.float().numpy()


def collect(model: Encoder) -> list[dict]:
    """레이어를 실행 순서대로 평평하게 만든다. C 쪽은 이 순서를 그대로 돈다."""
    layers: list[dict] = []

    stem_conv, stem_bn = model.stem[0], model.stem[1]
    w, b = fold_conv_bn(stem_conv, stem_bn)
    layers.append({"kind": "conv", "name": "stem",
                   "stride": list(stem_conv.stride), "groups": 1,
                   "w": w, "b": b, "relu": True})

    for i, blk in enumerate(model.blocks):
        assert isinstance(blk, DSConv)
        w, b = fold_conv_bn(blk.dw, blk.bn1)
        layers.append({"kind": "dwconv", "name": f"b{i}_dw",
                       "stride": list(blk.dw.stride), "groups": blk.dw.groups,
                       "w": w, "b": b, "relu": True})
        w, b = fold_conv_bn(blk.pw, blk.bn2)
        layers.append({"kind": "pwconv", "name": f"b{i}_pw",
                       "stride": [1, 1], "groups": 1,
                       "w": w, "b": b, "relu": True})

    layers.append({"kind": "gap", "name": "gap"})
    hw = model.head.weight.detach().numpy()
    layers.append({"kind": "fc", "name": "head", "w": hw,
                   "b": np.zeros(hw.shape[0], np.float32), "relu": False,
                   "stride": [1, 1], "groups": 1})
    layers.append({"kind": "l2norm", "name": "l2"})
    return layers


class RefRunner:
    """numpy 참조 구현. C 포팅의 정답지이자 폴딩 검증 도구.

    C 코드는 이 함수와 같은 순서·같은 산술을 하면 된다.
    """

    def __init__(self, layers: list[dict]):
        self.layers = layers

    def __call__(self, mel: np.ndarray) -> np.ndarray:
        x = mel.astype(np.float32)[None, :, :]            # (C=1, H=T, W=M)
        for L in self.layers:
            k = L["kind"]
            if k in ("conv", "pwconv"):
                x = self._conv(x, L["w"], L["b"], L["stride"], 1, L["relu"])
            elif k == "dwconv":
                x = self._conv(x, L["w"], L["b"], L["stride"], L["groups"], L["relu"])
            elif k == "gap":
                x = x.mean(axis=(1, 2))
            elif k == "fc":
                x = L["w"] @ x + L["b"]
            elif k == "l2norm":
                x = x / (np.linalg.norm(x) + 1e-12)
        return x

    @staticmethod
    def _conv(x, w, b, stride, groups, relu):
        cin, H, W = x.shape
        cout, cing, kh, kw = w.shape
        sh, sw = stride
        pad = 1 if kh == 3 else 0
        if pad:
            x = np.pad(x, ((0, 0), (pad, pad), (pad, pad)))
        Ho = (x.shape[1] - kh) // sh + 1
        Wo = (x.shape[2] - kw) // sw + 1
        out = np.empty((cout, Ho, Wo), np.float32)
        gsize_in, gsize_out = cin // groups, cout // groups
        for oc in range(cout):
            g = oc // gsize_out
            acc = np.full((Ho, Wo), b[oc], np.float32)
            for ic in range(cing):
                src = x[g * gsize_in + ic]
                ww = w[oc, ic]
                for i in range(kh):
                    for j in range(kw):
                        acc += ww[i, j] * src[i:i + Ho * sh:sh, j:j + Wo * sw:sw]
            out[oc] = acc
        return np.maximum(out, 0.0, out=out) if relu else out


def c_float_array(name: str, a: np.ndarray, per_line: int = 8) -> str:
    v = np.asarray(a, dtype=np.float32).ravel()
    body = []
    for i in range(0, len(v), per_line):
        body.append("    " + ", ".join(f"{x:.8e}f" for x in v[i:i + per_line]))
    return (f"static const float {name}[{len(v)}] = {{\n"
            + ",\n".join(body) + "\n};\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="model/out/encoder.pt")
    ap.add_argument("--commands", default="model/commands.json")
    ap.add_argument("--outdir", default="firmware/cabin_node")
    ap.add_argument("--bindir", default="model/out")
    args = ap.parse_args()

    ck = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    model = Encoder(dim=ck["dim"], w=ck["width"])
    model.load_state_dict(ck["model"])
    model.eval()

    layers = collect(model)
    ref = RefRunner(layers)

    # ── 폴딩 검증: torch 원본과 참조 구현이 같은 값을 내는지
    rng = np.random.default_rng(0)
    mel = (rng.standard_normal((FE.N_FRAMES, FE.N_MELS)) * 3.0).astype(np.float32)
    with torch.no_grad():
        want = model(torch.from_numpy(mel)[None]).numpy()[0]
    got = ref(mel)
    err = float(np.abs(want - got).max())
    cos = float(want @ got / (np.linalg.norm(want) * np.linalg.norm(got) + 1e-12))
    print(f"BN 폴딩 검증: 최대오차 {err:.3e}, 코사인 {cos:.8f}")
    if err > 2e-4:
        print("  폴딩이 어긋났다. 진행 중단.")
        return 1

    # ── 바이너리
    tensors = [(L["name"], L["w"], L["b"]) for L in layers if "w" in L]
    blob = bytearray()
    meta = []
    for name, w, bb in tensors:
        meta.append({"name": name, "shape": list(w.shape),
                     "w_off": len(blob), "w_len": int(w.size)})
        blob += np.asarray(w, np.float32).tobytes()
        meta[-1]["b_off"] = len(blob)
        meta[-1]["b_len"] = int(bb.size)
        blob += np.asarray(bb, np.float32).tobytes()

    bindir = Path(args.bindir); bindir.mkdir(parents=True, exist_ok=True)
    header = struct.pack("<IIII", MAGIC, 1, ck["dim"], len(tensors))
    (bindir / "encoder.bin").write_bytes(header + bytes(blob))
    n_par = sum(int(w.size + b.size) for _, w, b in tensors)
    print(f"encoder.bin: 텐서 {len(tensors)}개, 파라미터 {n_par/1000:.1f}K, "
          f"{(len(blob)+16)/1024:.1f}KB (float32)")

    # ── C 헤더
    outdir = Path(args.outdir); outdir.mkdir(parents=True, exist_ok=True)
    spec = json.loads(Path(args.commands).read_text(encoding="utf-8"))

    h = ["// 자동 생성 — model/scripts/export.py. 직접 수정하지 말 것.",
         "#pragma once", "#include <stdint.h>", "",
         f"#define CN_SR          {FE.SR}",
         f"#define CN_N_FFT       {FE.N_FFT}",
         f"#define CN_WIN         {FE.WIN}",
         f"#define CN_HOP         {FE.HOP}",
         f"#define CN_N_MELS      {FE.N_MELS}",
         f"#define CN_N_FRAMES    {FE.N_FRAMES}",
         f"#define CN_EMB_DIM     {ck['dim']}",
         f"#define CN_LOG_FLOOR   {FE.LOG_FLOOR:.8e}f",
         f"#define CN_N_TENSORS   {len(tensors)}",
         f"#define CN_N_INTENTS   {len(spec['intents'])}", ""]

    # 멜 필터뱅크는 희소하다 — 0 아닌 구간만 (start, len, weights) 로 담는다.
    fb = FE.mel_filterbank()
    nz = [(int(np.nonzero(row)[0][0]), row[np.nonzero(row)[0]]) for row in fb]
    total_nz = sum(len(w) for _, w in nz)
    h.append(f"#define CN_MEL_NZ      {total_nz}   "
             f"// 40x257 중 0 아닌 계수만. 프레임당 MAC 을 {fb.size}→{total_nz} 로 줄인다.")
    h.append("")
    h.append(c_float_array("cn_mel_w",
                           np.concatenate([w for _, w in nz]).astype(np.float32)))
    h.append("static const uint16_t cn_mel_start[%d] = {%s};\n"
             % (len(nz), ", ".join(str(s) for s, _ in nz)))
    h.append("static const uint16_t cn_mel_len[%d] = {%s};\n"
             % (len(nz), ", ".join(str(len(w)) for _, w in nz)))
    h.append(c_float_array("cn_hann", FE.hann()))

    h.append("\n// 레이어 실행 순서. kind: 0=conv 1=dwconv 2=pwconv 3=gap 4=fc 5=l2norm")
    h.append("typedef struct { uint8_t kind; uint8_t sh, sw; uint8_t relu;"
             " uint16_t cout, cin, kh, kw; uint32_t w_off, b_off; } cn_layer_t;")
    rows = []
    ti = 0
    for L in layers:
        kind = {"conv": 0, "dwconv": 1, "pwconv": 2,
                "gap": 3, "fc": 4, "l2norm": 5}[L["kind"]]
        if "w" in L:
            m = meta[ti]; ti += 1
            sh_, sw_ = L["stride"]
            if L["kind"] == "fc":
                cout, cin, kh, kw = m["shape"][0], m["shape"][1], 1, 1
            else:
                cout, cin, kh, kw = m["shape"]
            rows.append(f"  {{ {kind}, {sh_}, {sw_}, {int(L['relu'])}, "
                        f"{cout}, {cin}, {kh}, {kw}, "
                        f"{m['w_off']}, {m['b_off']} }},  // {L['name']}")
        else:
            rows.append(f"  {{ {kind}, 1, 1, 0, 0, 0, 0, 0, 0, 0 }},  // {L['name']}")
    h.append(f"#define CN_N_LAYERS {len(rows)}")
    h.append("static const cn_layer_t cn_layers[CN_N_LAYERS] = {\n"
             + "\n".join(rows) + "\n};\n")

    h.append("// 인텐트 id (등록 임베딩 순서와 동일)")
    h.append("static const char *const cn_intent_ids[CN_N_INTENTS] = {\n"
             + "\n".join(f'  "{it["id"]}",' for it in spec["intents"]) + "\n};\n")

    (outdir / "model_data.h").write_text("\n".join(h), encoding="utf-8")
    print(f"model_data.h: {(outdir/'model_data.h').stat().st_size/1024:.1f}KB")

    # ── 자기검증 벡터
    st = ["// 자동 생성 — 부팅 시 추론 경로 검증용.",
          "// 보드가 이 멜 입력으로 이 임베딩을 못 내면 추론 경로가 깨진 것이다.",
          "#pragma once", "",
          c_float_array("cn_selftest_mel", mel),
          c_float_array("cn_selftest_emb", want),
          f"#define CN_SELFTEST_TOL 2.0e-3f", ""]
    (outdir / "selftest.h").write_text("\n".join(st), encoding="utf-8")
    print(f"selftest.h:   {(outdir/'selftest.h').stat().st_size/1024:.1f}KB")

    # ── 가중치를 C 배열로도 낸다.
    # ESP32 는 상수 데이터가 플래시에 메모리 매핑되므로, 이렇게 두면 파일시스템도
    # PSRAM 복사도 필요 없다. 플래시를 268KB 쓰지만 4MB 중이라 여유가 있고,
    # 부팅 시 로딩 단계가 사라져서 실패 지점이 하나 줄어든다.
    wh = ["// 자동 생성 — model/scripts/export.py. 직접 수정하지 말 것.",
          "// ESP32 플래시에 상주하는 float32 가중치. cn_ctx_init() 에 그대로 넘긴다.",
          "#pragma once", "",
          c_float_array("cn_weights", np.frombuffer(bytes(blob), dtype=np.float32)),
          ""]
    (outdir / "model_weights.h").write_text("\n".join(wh), encoding="utf-8")
    print(f"model_weights.h: {(outdir/'model_weights.h').stat().st_size/1024/1024:.2f}MB "
          f"소스 → 플래시 {len(blob)/1024:.0f}KB")

    (bindir / "layers.json").write_text(
        json.dumps(meta, indent=1), encoding="utf-8")
    print(f"\n→ {bindir/'encoder.bin'}  {outdir/'model_data.h'}  {outdir/'selftest.h'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
