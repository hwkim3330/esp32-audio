#!/usr/bin/env python3
"""호출어 후보를 고른다. 재학습 없이 등록만으로 되는지도 같이 확인한다.

호출어 게이트는 상시 인식을 없애는 실전 완화책이다. 지금 구조에서 오수락이 남는
이유는 마이크가 늘 열려 있고 프로토타입이 86개라 무엇과든 우연히 가까울 수 있기
때문인데, 게이트를 두면 그 86개 앞에 **1개**가 서는 셈이다.

측정 방식이 중요하다. 호출어의 negative 는 두 종류이고 둘 다 세야 한다:

  1. 명령 문구 — "볼륨 올려줘" 가 호출어로 잡히면 게이트가 무의미해진다
  2. 명령 아닌 말(OOD) — 이쪽이 잡히면 게이트가 오히려 오작동을 만든다

그래서 임계값은 **호출어 재현율**로 잡고, 그 지점에서 두 negative 의 오각성률을
따로 낸다. 보류 보이스(F5/M5)로만 재고, 프로토타입은 학습 보이스로 등록한다 —
호출어는 학습 데이터에 없으므로 이 측정은 "재학습 없이 새 문구가 되는가" 를
그대로 시험하는 것이기도 하다.
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


@torch.no_grad()
def embed(model, items, root, dev, bs=64):
    out = []
    for i in range(0, len(items), bs):
        mels = np.stack([FE.features(read_wav_i16(root / it["path"]))
                         for it in items[i:i + bs]])
        out.append(model(torch.from_numpy(mels).to(dev)).cpu().numpy())
    return np.concatenate(out) if out else np.zeros((0, model.dim), np.float32)


def auroc(pos: np.ndarray, neg: np.ndarray) -> float:
    if not len(pos) or not len(neg):
        return float("nan")
    a = np.concatenate([pos, neg])
    r = a.argsort().argsort().astype(np.float64) + 1
    return float((r[:len(pos)].sum() - len(pos) * (len(pos) + 1) / 2)
                 / (len(pos) * len(neg)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="model/out/encoder.pt")
    ap.add_argument("--data", default="model/data/utterances")
    ap.add_argument("--commands", default="model/commands.json")
    ap.add_argument("--enroll-voices", nargs="+",
                    default=["F1", "F2", "F3", "F4", "M1", "M2", "M3", "M4"])
    ap.add_argument("--test-voices", nargs="+", default=["F5", "M5"])
    ap.add_argument("--recall", type=float, default=0.95,
                    help="이 호출어 재현율 지점에서 오각성률을 읽는다")
    ap.add_argument("--out", default="model/out/wake_eval.json")
    args = ap.parse_args()

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    try:
        torch.zeros(1).to(dev)
    except Exception:
        dev = "cpu"
    torch.set_num_threads(8)
    ck = torch.load(args.ckpt, map_location=dev, weights_only=False)
    model = Encoder(dim=ck["dim"], w=ck["width"]).to(dev)
    model.load_state_dict(ck["model"])
    model.eval()

    root = Path(args.data)
    items = json.loads((root / "manifest.json").read_text(encoding="utf-8"))["items"]
    spec = json.loads(Path(args.commands).read_text(encoding="utf-8"))
    weak = set(spec.get("weak_phrases") or [])
    trained_ood = set(ck.get("ood_train_texts") or [])
    en_v, te_v = set(args.enroll_voices), set(args.test_voices)

    wake_texts = spec.get("wake_candidates") or []
    if not wake_texts:
        raise SystemExit("commands.json 에 wake_candidates 가 없다")

    # negative 두 종류. 명령은 weak_phrases 를 뺀 실제 등록 대상만,
    # OOD 는 학습에 쓰지 않은 것만 (enroll.py 와 같은 규칙).
    neg_cmd = [it for it in items if it["label"] not in ("_ood", "_wake")
               and it["voice"] in te_v and it["text"] not in weak]
    neg_ood = [it for it in items if it["label"] == "_ood"
               and it["voice"] in te_v and it["text"] not in trained_ood]
    E_cmd = embed(model, neg_cmd, root, dev)
    E_ood = embed(model, neg_ood, root, dev)
    print(f"장치 {dev} | 명령 negative {len(neg_cmd)} / OOD negative {len(neg_ood)}\n")

    rows = {}
    print(f"{'호출어':14s} {'길이':>6s} {'AUROC(명령)':>11s} {'AUROC(OOD)':>11s} "
          f"{'임계':>6s} {'오각성(명령)':>12s} {'오각성(OOD)':>12s}")
    for text in wake_texts:
        enr = [it for it in items if it["label"] == "_wake"
               and it["text"] == text and it["voice"] in en_v]
        tst = [it for it in items if it["label"] == "_wake"
               and it["text"] == text and it["voice"] in te_v]
        if not enr or not tst:
            print(f"{text:14s} 클립이 없다 (합성 먼저: gen_data.py --wake-only)")
            continue
        e = embed(model, enr, root, dev)
        proto = e.mean(0)
        proto = proto / (np.linalg.norm(proto) + 1e-9)
        s_pos = embed(model, tst, root, dev) @ proto
        s_cmd = E_cmd @ proto
        s_ood = E_ood @ proto
        thr = float(np.quantile(s_pos, 1.0 - args.recall))
        far_c = float((s_cmd >= thr).mean())
        far_o = float((s_ood >= thr).mean())
        dur = float(np.mean([it["samples"] / 16000 for it in tst]))
        rows[text] = {"threshold": thr, "n_enroll": len(enr), "n_test": len(tst),
                      "dur_s": dur, "far_cmd": far_c, "far_ood": far_o,
                      "auroc_cmd": auroc(s_pos, s_cmd), "auroc_ood": auroc(s_pos, s_ood),
                      "recall": float((s_pos >= thr).mean())}
        print(f"{text:14s} {dur:5.2f}s {rows[text]['auroc_cmd']:11.3f} "
              f"{rows[text]['auroc_ood']:11.3f} {thr:6.3f} "
              f"{far_c*100:11.1f}% {far_o*100:11.1f}%")

    if rows:
        # 고르는 기준: 두 오각성률의 최댓값이 가장 작은 것. 어느 쪽이든 새면 게이트가
        # 무의미해지므로 평균이 아니라 최댓값으로 본다.
        best = min(rows, key=lambda t: max(rows[t]["far_cmd"], rows[t]["far_ood"]))
        print(f"\n호출어 재현율 {args.recall*100:.0f}% 지점 기준 최선: \"{best}\"  "
              f"(오각성 명령 {rows[best]['far_cmd']*100:.1f}% / "
              f"OOD {rows[best]['far_ood']*100:.1f}%)")
        print("호출어는 학습 데이터에 없다 — 이 숫자는 '재학습 없이 등록만으로 되는가' "
              "의 답이기도 하다.")
        Path(args.out).write_text(
            json.dumps({"recall_target": args.recall, "best": best, "rows": rows},
                       ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"→ {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
