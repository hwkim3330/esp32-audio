#!/usr/bin/env python3
"""인텐트별 프로토타입 임베딩을 만들어 ESP32 헤더로 낸다.

ESP32 는 학습을 못 하지만 "평균" 은 낼 수 있다. 그래서 명령 등록은 이렇게 된다:

    문장 → (Supertonic 합성) → 인코더 → 임베딩 여러 개 → 평균 → L2 정규화 = 프로토타입

런타임 인식은 마이크 임베딩과 프로토타입들의 코사인 유사도 최대값을 고르는 것뿐이다.
그래서 새 명령 추가에 재학습이 필요 없다 — 이 스크립트를 다시 돌리거나, 태블릿이
같은 계산을 하고 벡터만 보내주면 된다.

혼동 행렬을 같이 낸다. "어느 두 명령이 서로 헷갈리는가" 를 알면 명령 문구를 바꿔서
해결할 수 있고, 그게 모델을 키우는 것보다 훨씬 값싼 개선이다.
"""
from __future__ import annotations

import argparse
import json
import wave
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

import features as FE
from train import Encoder


def read_wav_i16(path: Path) -> np.ndarray:
    with wave.open(str(path), "rb") as w:
        return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)


def c_float_array(name: str, a: np.ndarray, per_line: int = 8) -> str:
    v = np.asarray(a, dtype=np.float32).ravel()
    body = ["    " + ", ".join(f"{x:.8e}f" for x in v[i:i + per_line])
            for i in range(0, len(v), per_line)]
    return (f"static const float {name}[{len(v)}] = {{\n"
            + ",\n".join(body) + "\n};\n")


@torch.no_grad()
def embed(model: Encoder, pcms: list[np.ndarray], dev: str, bs: int = 128):
    out = []
    for i in range(0, len(pcms), bs):
        mels = np.stack([FE.features(p) for p in pcms[i:i + bs]])
        out.append(model(torch.from_numpy(mels).to(dev)).cpu())
    return torch.cat(out) if out else torch.zeros(0, model.dim)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="model/out/encoder.pt")
    ap.add_argument("--data", default="model/data/utterances")
    ap.add_argument("--commands", default="model/commands.json")
    ap.add_argument("--outdir", default="firmware/cabin_node")
    ap.add_argument("--report", default="model/out/enroll_report.json")
    ap.add_argument("--enroll-voices", nargs="+",
                    default=["F1", "F2", "F3", "F4", "M1", "M2", "M3", "M4"],
                    help="등록에 쓸 보이스. 평가용 보류 보이스는 제외한다.")
    ap.add_argument("--test-voices", nargs="+", default=["F5", "M5"])
    ap.add_argument("--ood-bank", action="store_true",
                    help="OOD 프로토타입 은행을 함께 낸다. 실측 이득이 오수락 "
                         "15.8%%→13.1%% 인데 플래시 109KB 를 먹어서 기본은 끈다")
    ap.add_argument("--bank-k", type=int, default=0,
                    help="은행을 구면 k-means 로 이 행 수까지 줄인다. 0 은 그대로")
    ap.add_argument("--ood-trained-from", default=None,
                    help="학습에 쓴 OOD 문장 목록을 가져올 체크포인트. 조건이 다른 두 "
                         "모델을 '같은 OOD 클립' 으로 비교할 때 지정한다. 기본은 자기 자신")
    args = ap.parse_args()

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    ck = torch.load(args.ckpt, map_location=dev, weights_only=False)
    model = Encoder(dim=ck["dim"], w=ck["width"]).to(dev)
    model.load_state_dict(ck["model"])
    model.eval()

    root = Path(args.data)
    man = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    spec = json.loads(Path(args.commands).read_text(encoding="utf-8"))
    intent_ids = [it["id"] for it in spec["intents"]]

    items = man["items"]
    en_v, te_v = set(args.enroll_voices), set(args.test_voices)

    # ── 프로토타입은 "문장당 1개" 다. 인텐트당 1개로 묶으면 안 된다.
    #
    # 인코더는 문장 단위 라벨로 학습됐다. 즉 같은 인텐트라도 다른 문장은 일부러
    # 서로 멀리 떨어뜨려 놨다("에어컨 켜줘" vs "시원하게 해줘"). 그걸 평균하면
    # 어느 문장에도 가깝지 않은 점이 나온다 — 실측으로 인텐트 정확도가
    # 0.93 에서 0.67 로 떨어졌다. 학습 목표와 등록 방식이 싸운 결과다.
    #
    # 그래서 문장별로 프로토타입을 만들고, 인텐트는 조회 테이블로 얻는다.
    phrases: list[tuple[str, str]] = []       # (text, intent_id)
    for it in spec["intents"]:
        for t in it["phrases"]:
            phrases.append((t, it["id"]))
    text2row = {t: i for i, (t, _) in enumerate(phrases)}
    proto_intent = np.array([intent_ids.index(iid) for _, iid in phrases], np.int32)

    protos = np.zeros((len(phrases), ck["dim"]), np.float32)
    n_used = []
    for r, (text, _) in enumerate(phrases):
        pcms = [read_wav_i16(root / it["path"]) for it in items
                if it.get("text") == text and it["voice"] in en_v]
        e = embed(model, pcms, dev)
        protos[r] = F.normalize(e.mean(0), dim=0).numpy()
        n_used.append(len(pcms))
    print(f"프로토타입 {len(phrases)}개 (문장당 1개, 인텐트 {len(intent_ids)}개)")
    print(f"  문장당 등록 클립 {min(n_used)}~{max(n_used)}개")

    # ── 평가: 보류 보이스. 문장 정확도와 인텐트 정확도를 따로 본다.
    te = [it for it in items if it["voice"] in te_v and it["label"] != "_ood"]
    pcms = [read_wav_i16(root / it["path"]) for it in te]
    y_row = np.array([text2row[it["text"]] for it in te])
    y = proto_intent[y_row]
    e = embed(model, pcms, dev).numpy()
    sim = e @ protos.T
    pred_row = sim.argmax(1)
    pred = proto_intent[pred_row]
    conf = sim.max(1)
    acc_phrase = float((pred_row == y_row).mean())
    acc = float((pred == y).mean())
    print(f"\n문장 정확도   {acc_phrase:.4f}")

    # ── OOD 거부.
    #
    # 학습에 쓴 OOD 문장으로 평가하면 안 된다. 체크포인트가 어느 문장을 학습에
    # 썼는지 들고 있으므로(ood_train_texts) 그건 평가에서 빼고, 대신 그 문장들로
    # "OOD 프로토타입 은행" 을 만든다 — 명령 프로토타입보다 OOD 쪽이 더 가까우면
    # 거부한다. 임계값 하나로 자르는 것보다 강하고 플래시 비용은 행 몇 개다.
    src = ck
    if args.ood_trained_from:
        src = torch.load(args.ood_trained_from, map_location="cpu", weights_only=False)
    trained_ood = set(src.get("ood_train_texts") or [])
    ood_eval = [it for it in items if it["label"] == "_ood"
                and it["voice"] in te_v and it["text"] not in trained_ood]
    ood_bank = [it for it in items if it["label"] == "_ood"
                and it["voice"] in en_v and it["text"] in trained_ood] \
        if args.ood_bank else []

    ood_protos = np.zeros((0, ck["dim"]), np.float32)
    if ood_bank:
        by_text: dict[str, list] = {}
        for it in ood_bank:
            by_text.setdefault(it["text"], []).append(read_wav_i16(root / it["path"]))
        ood_protos = np.stack([
            F.normalize(embed(model, p, dev).mean(0), dim=0).numpy()
            for p in by_text.values()])
        if args.bank_k and args.bank_k < len(ood_protos):
            # 문장 하나당 한 행씩 두면 플래시가 아깝다. 구면 k-means 로 줄인다 —
            # 몇 행까지 줄여도 되는지는 eval_reject.py 로 정한다.
            from eval_reject import spherical_kmeans
            ood_protos = spherical_kmeans(ood_protos, args.bank_k)
        print(f"OOD 프로토타입 {len(ood_protos)}개 "
              f"(학습 OOD 문장 {len(by_text)}개 → {ood_protos.nbytes/1024:.1f}KB)")

    # 판정식은 마진 하나다: (명령 최고 코사인) − (OOD 최고 코사인) ≥ CN_REJECT_MARGIN.
    # 은행이 비면 두 번째 항이 0 이라 예전 절대 임계값 판정으로 축퇴한다.
    # 임계값은 명령 재현율 95% 지점에서 잡는다 — 기제를 바꿔도 재현율이 고정되므로
    # 오수락 숫자를 그대로 비교할 수 있다.
    far = thr = None
    by_group: dict[str, list[int]] = {}
    margin_cmd = conf - ((e @ ood_protos.T).max(1) if len(ood_protos) else 0.0)
    if ood_eval:
        eo = embed(model, [read_wav_i16(root / it["path"]) for it in ood_eval], dev).numpy()
        margin_ood = (eo @ protos.T).max(1) - (
            (eo @ ood_protos.T).max(1) if len(ood_protos) else 0.0)
        thr = float(np.quantile(margin_cmd, 0.05))
        far = float((margin_ood >= thr).mean())
        for i, it in enumerate(ood_eval):
            by_group.setdefault(it.get("group", "unknown"), []).append(i)

    cm = np.zeros((len(intent_ids), len(intent_ids)), int)
    for t, p in zip(y, pred):
        cm[t, p] += 1

    print(f"\n인텐트 정확도 (보류 보이스 {sorted(te_v)}): {acc:.4f}  "
          f"({int((pred==y).sum())}/{len(y)})")
    if thr is not None:
        print(f"거부 마진 임계값 {thr:.3f} (OOD 프로토타입 {len(ood_protos)}개) "
              f"→ 오수락 {far*100:.1f}%  (명령 재현율 95%, 평가 클립 {len(ood_eval)}개)")
        for g, idx in sorted(by_group.items()):
            print(f"  그룹 {g:14s} 오수락 "
                  f"{float((margin_ood[idx] >= thr).mean())*100:5.1f}%  [n={len(idx)}]")

    # 가장 헷갈리는 쌍 — 문구를 바꿔 해결할 후보
    pairs = [(cm[i, j], intent_ids[i], intent_ids[j])
             for i in range(len(intent_ids)) for j in range(len(intent_ids))
             if i != j and cm[i, j] > 0]
    pairs.sort(reverse=True)
    if pairs:
        print("\n혼동 상위 (실제 → 오인):")
        for n, a, b in pairs[:8]:
            print(f"  {n:3d}회  {a:14s} → {b}")
    else:
        print("\n혼동 없음.")

    # ── C 헤더
    outdir = Path(args.outdir); outdir.mkdir(parents=True, exist_ok=True)
    h = ["// 자동 생성 — model/scripts/enroll.py. 직접 수정하지 말 것.",
         "// 문장별 프로토타입 임베딩(L2 정규화). cn_match() 에 그대로 넘긴다.",
         "//",
         "// 인텐트당 1개가 아니라 '문장당 1개' 다. 인코더가 문장 단위로 학습됐으므로",
         "// 같은 인텐트의 다른 문장을 평균하면 정확도가 무너진다(0.93 → 0.67 실측).",
         "// 매칭된 행을 cn_proto_intent[] 로 인텐트에 사상한다.",
         "//",
         "// 새 명령 추가는 이 배열에 행 하나를 더하는 것뿐이다 — 재학습이 필요 없다.",
         "// 런타임에는 태블릿이 같은 계산을 해서 PSRAM 쪽 배열로 보내줄 수 있다.",
         "#pragma once", "#include <stdint.h>", "",
         f"#define CN_N_PROTO   {len(phrases)}",
         f"#define CN_REJECT_MARGIN {thr if thr is not None else 0.5:.4f}f"
         "   // (명령 최고 코사인 − OOD 최고 코사인) 이 값 미만이면 버린다", "",
         c_float_array("cn_protos", protos),
         "// 프로토타입 행 → 인텐트 인덱스",
         "static const uint8_t cn_proto_intent[CN_N_PROTO] = {"
         + ", ".join(str(int(v)) for v in proto_intent) + "};\n",
         "// 프로토타입 행 → 원문(디버깅·태블릿 표시용)",
         "static const char *const cn_proto_text[CN_N_PROTO] = {",
         *[f'  "{t}",' for t, _ in phrases],
         "};\n",
         "// ── 거부용 OOD 프로토타입. 명령이 아닌 말들의 중심이다.",
         "// 판정: 명령 최고점이 CN_REJECT_THR 미만이거나, OOD 최고점보다 낮으면 거부.",
         "// 임계값 하나로만 자르면 오수락이 안 잡혀서(측정) 이 은행을 같이 쓴다.",
         f"#define CN_N_OOD_PROTO {len(ood_protos)}",
         (c_float_array("cn_ood_protos", ood_protos) if len(ood_protos)
          else "static const float cn_ood_protos[1] = {0.0f};  // 비어 있음\n")]
    (outdir / "prototypes.h").write_text("\n".join(h), encoding="utf-8")
    print(f"\nprototypes.h: {(outdir/'prototypes.h').stat().st_size/1024:.1f}KB "
          f"({len(intent_ids)}x{ck['dim']} float = {protos.nbytes/1024:.1f}KB 플래시)")

    Path(args.report).write_text(json.dumps({
        "intent_accuracy": acc, "phrase_accuracy": acc_phrase, "n_test": int(len(y)),
        "reject_margin": thr, "ood_false_accept": far,
        "n_ood_eval": len(ood_eval), "n_ood_protos": int(len(ood_protos)),
        "ood_false_accept_by_group": {
            g: float((margin_ood[idx] >= thr).mean()) for g, idx in by_group.items()
        } if thr is not None else None,
        "ood_mode": ck.get("ood_mode", "exclude"),
        "enroll_voices": sorted(en_v), "test_voices": sorted(te_v),
        "confusion_top": [{"n": int(n), "true": a, "pred": b}
                          for n, a, b in pairs[:20]],
        "per_intent_recall": {intent_ids[i]: float(cm[i, i] / max(cm[i].sum(), 1))
                              for i in range(len(intent_ids))},
    }, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"→ {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
