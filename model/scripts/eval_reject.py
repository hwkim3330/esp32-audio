#!/usr/bin/env python3
"""거부(OOD) 성능만 따로 재는 스크립트. 기제 선택을 숫자로 결정하기 위한 것.

왜 따로 있나. enroll.py 는 "임계값을 명령 재현율 95% 에 맞추고 오수락을 본다".
그런데 OOD 프로토타입 은행처럼 판정식을 바꾸는 기제는 재현율도 같이 바꾼다.
서로 다른 재현율의 오수락을 나란히 적으면 비교가 아니라 착각이다. 여기서는

  1) 판정 점수를 정의하고(임계값만 / 은행과의 마진),
  2) 재현율을 95% 로 맞춘 지점에서 오수락을 읽고,
  3) 임계값과 무관한 AUROC 도 같이 낸다.

은행 크기(플래시 비용)도 여기서 정한다 — 구면 k-means 로 줄여가며 오수락을 본다.
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
def embed(model: Encoder, pcms: list[np.ndarray], dev: str, bs: int = 128):
    out = []
    for i in range(0, len(pcms), bs):
        mels = np.stack([FE.features(p) for p in pcms[i:i + bs]])
        out.append(model(torch.from_numpy(mels).to(dev)).cpu().numpy())
    return np.concatenate(out) if out else np.zeros((0, model.dim), np.float32)


def protos_by_text(model, items, root, dev):
    """문장별 프로토타입 = 임베딩 평균 후 L2 정규화. ESP32 등록 절차와 같다."""
    by: dict[str, list] = {}
    for it in items:
        by.setdefault(it["text"], []).append(read_wav_i16(root / it["path"]))
    keys = list(by)
    if not keys:
        return np.zeros((0, model.dim), np.float32), []
    P = np.stack([_l2(embed(model, by[k], dev).mean(0)) for k in keys])
    return P, keys


def _l2(v):
    return v / (np.linalg.norm(v) + 1e-9)


def spherical_kmeans(X: np.ndarray, k: int, iters: int = 50, seed: int = 0):
    """코사인 공간의 k-means. 은행을 몇 행까지 줄일 수 있는지 보려고 쓴다."""
    if k >= len(X):
        return X
    rng = np.random.default_rng(seed)
    C = X[rng.choice(len(X), size=k, replace=False)].copy()
    for _ in range(iters):
        a = (X @ C.T).argmax(1)
        for j in range(k):
            m = X[a == j]
            if len(m):
                C[j] = _l2(m.mean(0))
    return C


def auroc(pos: np.ndarray, neg: np.ndarray) -> float:
    """명령(pos)이 OOD(neg)보다 높은 점수를 받을 확률. 임계값과 무관한 분리도."""
    if not len(pos) or not len(neg):
        return float("nan")
    a = np.concatenate([pos, neg])
    r = a.argsort().argsort().astype(np.float64) + 1     # 동점은 무시할 수준
    return float((r[:len(pos)].sum() - len(pos) * (len(pos) + 1) / 2)
                 / (len(pos) * len(neg)))


def far_at_recall(s_cmd: np.ndarray, s_ood: np.ndarray, recall: float):
    """명령 재현율을 정확히 맞춘 지점의 임계값과 오수락률."""
    thr = float(np.quantile(s_cmd, 1.0 - recall))
    return thr, float((s_ood >= thr).mean()), float((s_cmd >= thr).mean())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", nargs="+", required=True)
    ap.add_argument("--data", default="model/data/utterances")
    ap.add_argument("--commands", default="model/commands.json",
                    help="weak_phrases 를 읽는다. enroll.py 와 같은 문구 집합을 써야 "
                         "두 도구가 같은 숫자를 낸다")
    ap.add_argument("--no-weak-filter", action="store_true",
                    help="weak_phrases 를 무시하고 전 문구로 등록한다(비교용)")
    ap.add_argument("--ood-trained-from", required=True,
                    help="평가에서 뺄 OOD 문장 목록. 조건이 다른 모델을 같은 클립으로 본다")
    ap.add_argument("--enroll-voices", nargs="+",
                    default=["F1", "F2", "F3", "F4", "M1", "M2", "M3", "M4"])
    ap.add_argument("--test-voices", nargs="+", default=["F5", "M5"])
    ap.add_argument("--recall", type=float, default=0.95)
    ap.add_argument("--bank-k", type=int, nargs="+", default=[0, 32, 16, 8],
                    help="은행 행 수. 0 은 줄이지 않음")
    ap.add_argument("--out", default="model/out/reject_eval.json")
    args = ap.parse_args()

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    root = Path(args.data)
    items = json.loads((root / "manifest.json").read_text(encoding="utf-8"))["items"]
    trained = set(torch.load(args.ood_trained_from, map_location="cpu",
                             weights_only=False).get("ood_train_texts") or [])
    en_v, te_v = set(args.enroll_voices), set(args.test_voices)

    # enroll.py 와 **같은 문구 집합**을 써야 한다. 예전에는 이 스크립트가
    # commands.json 을 아예 안 봐서, 같은 체크포인트로 enroll.py 는 오수락 8.1% 를
    # eval_reject.py 는 15.8% 를 냈다 — 판단이 두 곳에서 갈라진 것이다.
    weak: set[str] = set()
    if not args.no_weak_filter:
        spec = json.loads(Path(args.commands).read_text(encoding="utf-8"))
        weak = set(spec.get("weak_phrases") or [])
        # 문구가 전부 빠지는 인텐트가 없어야 한다(enroll.py 와 같은 안전장치).
        for it in spec["intents"]:
            if not [t for t in it["phrases"] if t not in weak]:
                weak -= set(it["phrases"])
        if weak:
            print(f"약한 문구 {len(weak)}개 제외 (enroll.py 와 동일)")

    def is_cmd(it):
        return it["label"] != "_ood" and it["text"] not in weak

    cmd_enroll = [it for it in items if is_cmd(it) and it["voice"] in en_v]
    cmd_test = [it for it in items if is_cmd(it) and it["voice"] in te_v]
    ood_bank_items = [it for it in items if it["label"] == "_ood"
                      and it["voice"] in en_v and it["text"] in trained]
    ood_test = [it for it in items if it["label"] == "_ood"
                and it["voice"] in te_v and it["text"] not in trained]
    groups = sorted({it.get("group", "?") for it in ood_test})
    print(f"명령 등록 {len(cmd_enroll)} / 명령 평가 {len(cmd_test)} / "
          f"OOD 은행 {len(ood_bank_items)} / OOD 평가 {len(ood_test)}")
    print(f"OOD 평가 그룹: " + ", ".join(
        f"{g}={sum(1 for it in ood_test if it.get('group') == g)}" for g in groups))

    results = {}
    for ckpt in args.ckpt:
        ck = torch.load(ckpt, map_location=dev, weights_only=False)
        model = Encoder(dim=ck["dim"], w=ck["width"]).to(dev)
        model.load_state_dict(ck["model"]); model.eval()

        Pc, _ = protos_by_text(model, cmd_enroll, root, dev)
        Po, _ = protos_by_text(model, ood_bank_items, root, dev)
        Ec = embed(model, [read_wav_i16(root / it["path"]) for it in cmd_test], dev)
        Eo = embed(model, [read_wav_i16(root / it["path"]) for it in ood_test], dev)
        gidx = {g: [i for i, it in enumerate(ood_test) if it.get("group") == g]
                for g in groups}

        name = Path(ckpt).stem
        r = {"mode": ck.get("ood_mode", "exclude"), "n_cmd_proto": len(Pc),
             "rules": {}}
        # 규칙 1 — 임계값만 (지금 펌웨어가 하는 것)
        c1, o1 = (Ec @ Pc.T).max(1), (Eo @ Pc.T).max(1)
        # 규칙 2 — 은행과의 마진. 은행 크기를 줄여가며 본다.
        rules = {"thr_only": (c1, o1, 0)}
        for k in args.bank_k:
            if not len(Po):
                break
            B = Po if k == 0 else spherical_kmeans(Po, k)
            rules[f"margin_bank{len(B)}"] = (c1 - (Ec @ B.T).max(1),
                                             o1 - (Eo @ B.T).max(1), len(B))
        for rname, (sc, so, nb) in rules.items():
            thr, far, rec = far_at_recall(sc, so, args.recall)
            r["rules"][rname] = {
                "threshold": thr, "far": far, "cmd_recall": rec,
                "auroc": auroc(sc, so), "bank_rows": nb,
                "bank_flash_kb": nb * ck["dim"] * 4 / 1024,
                "far_by_group": {g: float((so[i] >= thr).mean()) if i else None
                                 for g, i in gidx.items()},
            }
        results[name] = r

    print(f"\n{'모델':22s} {'규칙':16s} {'AUROC':>7s} {'오수락':>7s} "
          f"{'재현율':>7s} {'은행플래시':>9s}   그룹별 오수락")
    for name, r in results.items():
        for rname, v in r["rules"].items():
            gb = " ".join(f"{g[:9]}={v['far_by_group'][g]*100:.0f}%"
                          for g in groups if v["far_by_group"][g] is not None)
            print(f"{name:22s} {rname:16s} {v['auroc']:7.3f} {v['far']*100:6.1f}% "
                  f"{v['cmd_recall']*100:6.1f}% {v['bank_flash_kb']:8.1f}KB   {gb}")

    Path(args.out).write_text(json.dumps(results, ensure_ascii=False, indent=1),
                              encoding="utf-8")
    print(f"\n→ {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
