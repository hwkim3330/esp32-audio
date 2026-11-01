#!/usr/bin/env python3
"""시스템 평가: 깨끗한 음성 / 캐빈 잡음 / 분포 이동 잡음에서 인식·거부·호출어를 한 번에 잰다.

eval_reject.py 와 같은 프로토콜을 따른다 — 그래야 숫자가 이어진다:

  - 명령 프로토타입은 문장당 1개, 학습 보이스(F1-4, M1-4)의 **깨끗한** 클립으로 등록
    (실제 등록도 태블릿이 깨끗하게 합성한 음성으로 한다)
  - 평가는 보류 보이스(F5/M5)만, OOD 는 학습에 쓰지 않은 문장만(222클립)
  - weak_phrases 는 enroll.py·eval_reject.py 와 같은 함수로 뺀다
  - 명령 재현율 95% 지점에서 오수락을 읽는다

새로 더한 것:

  1. **잡음 조건.** `noisy` 는 cabin_aug.cabin (학습 증강과 같은 생성기 계열, 다른 시드),
     `shift` 는 학습에 한 번도 안 쓴 잡음(대화 babble·음악·빗소리/딸깍임)과 다른 잔향·마이크
     경로. SNR 은 0/5/10/15/20 dB 를 고르게 돌린다. 디스크에 쓰지 않고 클립 경로로 시드를
     정해 매번 같은 잡음을 만든다.
  2. **임계값 두 가지.** 조건마다 재현율 95% 로 다시 맞춘 오수락(분리도)과, 깨끗한 음성으로
     정한 임계값을 그대로 쓴 재현율·오수락(보드에 실제로 굽히는 것은 이쪽이다).
  3. **신뢰구간.** 같은 문장의 클립(보이스 2개 × 잡음 복제)은 독립이 아니므로 **문장 단위**
     군집 부트스트랩(명령 문장·OOD 문장을 함께 재표집, 임계값도 다시 계산)으로 95% 구간을 낸다.
  4. **호출어 게이트.** 호출어 자체의 오거부(FR)·오각성(FA)과 게이트를 포함한 시스템 수치.
  5. **접두어 옵션.** `--prefix N` 이면 N음절 이하 명령 문구를 commands.json 의 alt_phrases
     ('캐빈 ...') 로 바꿔 등록한다. 재학습 없이 문구만 바꾸는 제품 옵션이다.
"""
from __future__ import annotations

import argparse
import json
import wave
from pathlib import Path

import numpy as np
import torch

import cabin_aug as CA
import features as FE
from train import Encoder

SNR_GRID = [0.0, 5.0, 10.0, 15.0, 20.0]


def read_f32(path: Path) -> np.ndarray:
    with wave.open(str(path), "rb") as w:
        return np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32) / 32768.0


def weak_set(spec: dict, disable: bool = False) -> set[str]:
    """enroll.py / eval_reject.py 와 같은 규칙: 문구가 전부 빠지는 인텐트는 필터를 무시."""
    if disable:
        return set()
    weak = set(spec.get("weak_phrases") or [])
    for it in spec["intents"]:
        if not [t for t in it["phrases"] if t not in weak]:
            weak -= set(it["phrases"])
    return weak


def phrase_table(spec: dict, weak: set[str], prefix_max: int):
    """(등록 문구, 인텐트, 클립 라벨) 목록. prefix_max>0 이면 짧은 문구를 alt 로 바꾼다."""
    alt = {a["replaces"]: a for a in spec.get("alt_phrases", [])}
    rows = []
    for it in spec["intents"]:
        for p in it["phrases"]:
            if p in weak:
                continue
            n = len(p.replace(" ", ""))
            if prefix_max and n <= prefix_max and p in alt:
                rows.append((alt[p]["text"], it["id"], "_alt"))
            else:
                rows.append((p, it["id"], it["id"]))
    return rows


def corrupt(x: np.ndarray, path: str, cond: str, rep: int, babble):
    if cond == "clean":
        return x, None
    rng = np.random.default_rng(CA.clip_seed(path, cond, rep))
    snr = SNR_GRID[(CA.clip_seed(path, "snr", rep) + rep) % len(SNR_GRID)]
    if cond == "noisy":
        return CA.cabin(x, rng, snr=snr, p_rir=0.8), snr
    if cond == "shift":
        return CA.shifted(x, rng, snr, babble), snr
    raise ValueError(cond)


@torch.no_grad()
def embed_items(model, items, root, cond, reps, babble, bs=128):
    """반환: 임베딩 (N*reps, D), 원 항목 인덱스, SNR."""
    feats, idx, snrs = [], [], []
    R = 1 if cond == "clean" else reps
    for i, it in enumerate(items):
        x = read_f32(root / it["path"])
        for r in range(R):
            y, snr = corrupt(x, it["path"], cond, r, babble)
            feats.append(FE.features(y))
            idx.append(i)
            snrs.append(-1.0 if snr is None else snr)
    out = []
    for i in range(0, len(feats), bs):
        out.append(model(torch.from_numpy(np.stack(feats[i:i + bs]))).numpy())
    E = np.concatenate(out) if out else np.zeros((0, model.dim), np.float32)
    return E, np.array(idx), np.array(snrs)


def l2(v):
    return v / (np.linalg.norm(v, axis=-1, keepdims=True) + 1e-9)


def auroc(pos, neg):
    if not len(pos) or not len(neg):
        return float("nan")
    a = np.concatenate([pos, neg])
    r = a.argsort().argsort().astype(np.float64) + 1
    return float((r[:len(pos)].sum() - len(pos) * (len(pos) + 1) / 2) / (len(pos) * len(neg)))


def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def boot_far(s_cmd, cmd_cl, s_ood, ood_cl, ood_mask, recall, B, seed=0):
    """문장 군집 부트스트랩. 명령·OOD 문장을 함께 재표집하고 임계값을 다시 잡는다."""
    rng = np.random.default_rng(seed)
    cc = np.unique(cmd_cl); oc = np.unique(ood_cl)
    cidx = {c: np.where(cmd_cl == c)[0] for c in cc}
    oidx = {c: np.where(ood_cl == c)[0] for c in oc}
    out = []
    for _ in range(B):
        ci = np.concatenate([cidx[c] for c in rng.choice(cc, len(cc))])
        oi = np.concatenate([oidx[c] for c in rng.choice(oc, len(oc))])
        oi = oi[ood_mask[oi]]
        if not len(oi):
            continue
        thr = np.quantile(s_cmd[ci], 1 - recall)
        out.append(float((s_ood[oi] >= thr).mean()))
    return (float(np.percentile(out, 2.5)), float(np.percentile(out, 97.5)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", nargs="+", required=True)
    ap.add_argument("--data", default="../data/utterances")
    ap.add_argument("--commands", default="../commands.json")
    ap.add_argument("--ood-trained-from", default=None,
                    help="평가에서 뺄 OOD 문장 목록의 출처. 기본은 각 체크포인트 자신 — "
                         "같은 시드면 학습 OOD 분할이 같다")
    ap.add_argument("--conds", nargs="+", default=["clean", "noisy", "shift"])
    ap.add_argument("--reps", type=int, default=3, help="잡음 조건에서 클립당 복제 수")
    ap.add_argument("--prefix", type=int, default=0,
                    help="이 음절 수 이하 명령 문구를 alt_phrases('캐빈 ...')로 바꿔 등록")
    ap.add_argument("--recall", type=float, default=0.95)
    ap.add_argument("--boot", type=int, default=1000)
    ap.add_argument("--enroll-voices", nargs="+",
                    default=["F1", "F2", "F3", "F4", "M1", "M2", "M3", "M4"])
    ap.add_argument("--test-voices", nargs="+", default=["F5", "M5"])
    ap.add_argument("--out", default="../out/system_eval.json")
    args = ap.parse_args()

    torch.set_num_threads(4)
    root = Path(args.data)
    items = json.loads((root / "manifest.json").read_text(encoding="utf-8"))["items"]
    spec = json.loads(Path(args.commands).read_text(encoding="utf-8"))
    weak = weak_set(spec)
    rows = phrase_table(spec, weak, args.prefix)
    en_v, te_v = set(args.enroll_voices), set(args.test_voices)
    intent_ids = [it["id"] for it in spec["intents"]]
    key = {(t, lab): r for r, (t, _, lab) in enumerate(rows)}
    row_int = np.array([intent_ids.index(i) for _, i, _ in rows])

    def row_of(it):
        lab = "_alt" if it["label"] == "_alt" else it["label"]
        return key.get((it["text"], lab))

    cmd_en = [it for it in items if it["voice"] in en_v and row_of(it) is not None]
    cmd_te = [it for it in items if it["voice"] in te_v and row_of(it) is not None]
    wake_text = spec.get("wake_phrase") or ""
    wk_en = [it for it in items if it["label"] == "_wake" and it["text"] == wake_text
             and it["voice"] in en_v]
    wk_te = [it for it in items if it["label"] == "_wake" and it["text"] == wake_text
             and it["voice"] in te_v]
    # babble: 학습 보이스가 말한 OOD 문장(평가 OOD 와 문장도 보이스도 겹치지 않게 아래에서 거름)
    print(f"등록 문구 {len(rows)}개 (prefix≤{args.prefix}) | 명령 등록 {len(cmd_en)} / "
          f"평가 {len(cmd_te)} | 호출어 등록 {len(wk_en)} / 평가 {len(wk_te)}")

    results = {"prefix": args.prefix, "n_proto": len(rows), "conds": args.conds,
               "reps": args.reps, "models": {}}
    for ck_path in args.ckpt:
        ck = torch.load(ck_path, map_location="cpu", weights_only=False)
        model = Encoder(dim=ck["dim"], w=ck["width"])
        model.load_state_dict(ck["model"]); model.eval()
        src = ck if not args.ood_trained_from else torch.load(
            args.ood_trained_from, map_location="cpu", weights_only=False)
        trained = set(src.get("ood_train_texts") or [])
        ood_te = [it for it in items if it["label"] == "_ood" and it["voice"] in te_v
                  and it["text"] not in trained]
        babble_items = [it for it in items if it["label"] == "_ood" and it["voice"] in en_v
                        and it["text"] in trained][:200]
        babble = [read_f32(root / it["path"]) for it in babble_items]

        # 등록은 언제나 깨끗한 음성
        E, idx, _ = embed_items(model, cmd_en, root, "clean", 1, babble)
        r_of = np.array([row_of(cmd_en[i]) for i in idx])
        P = l2(np.stack([E[r_of == r].mean(0) for r in range(len(rows))]))
        Ew, _, _ = embed_items(model, wk_en, root, "clean", 1, babble)
        W = l2(Ew.mean(0)) if len(Ew) else None

        name = Path(ck_path).stem
        mres = {"aug": ck.get("aug", "legacy"), "seed": ck.get("seed"), "conds": {}}
        thr_clean = wthr_clean = wthr_fa1 = None
        for cond in args.conds:
            Ec, ic, snr_c = embed_items(model, cmd_te, root, cond, args.reps, babble)
            Eo, io, snr_o = embed_items(model, ood_te, root, cond, args.reps, babble)
            y_row = np.array([row_of(cmd_te[i]) for i in ic])
            sim = Ec @ P.T
            pred = sim.argmax(1)
            s_cmd = sim.max(1)
            s_ood = (Eo @ P.T).max(1)
            thr = float(np.quantile(s_cmd, 1 - args.recall))
            if cond == "clean":
                thr_clean = thr
            grp = np.array([ood_te[i].get("group", "?") for i in io])
            short = grp == "short"
            cmd_cl = np.array([cmd_te[i]["text"] for i in ic])
            ood_cl = np.array([ood_te[i]["text"] for i in io])
            allm = np.ones(len(io), bool)
            r = {
                "n_cmd": int(len(s_cmd)), "n_ood": int(len(s_ood)),
                "intent_acc": float((row_int[pred] == row_int[y_row]).mean()),
                "phrase_acc": float((pred == y_row).mean()),
                "auroc": auroc(s_cmd, s_ood),
                "thr": thr,
                "far": float((s_ood >= thr).mean()),
                "far_ci": boot_far(s_cmd, cmd_cl, s_ood, ood_cl, allm, args.recall, args.boot),
                "far_short": float((s_ood[short] >= thr).mean()),
                "far_short_ci": boot_far(s_cmd, cmd_cl, s_ood, ood_cl, short, args.recall,
                                         args.boot),
                "far_noshort": float((s_ood[~short] >= thr).mean()),
                "far_by_group": {g: float((s_ood[grp == g] >= thr).mean())
                                 for g in sorted(set(grp))},
                # 보드에 실제로 굽히는 것: 깨끗한 음성으로 정한 임계값 하나
                "fixed_thr": thr_clean,
                "fixed_recall": float((s_cmd >= thr_clean).mean()),
                "fixed_far": float((s_ood >= thr_clean).mean()),
                "fixed_far_short": float((s_ood[short] >= thr_clean).mean()),
                # 정확도까지 포함한 '명령이 제대로 실행된 비율'
                "fixed_success": float(((s_cmd >= thr_clean) &
                                        (row_int[pred] == row_int[y_row])).mean()),
            }
            if cond != "clean":
                r["by_snr"] = {}
                for s in SNR_GRID:
                    mc, mo = snr_c == s, snr_o == s
                    r["by_snr"][str(int(s))] = {
                        "intent_acc": float((row_int[pred[mc]] == row_int[y_row[mc]]).mean()),
                        "fixed_recall": float((s_cmd[mc] >= thr_clean).mean()),
                        "fixed_far": float((s_ood[mo] >= thr_clean).mean()),
                        "far_at95": float((s_ood[mo] >= np.quantile(s_cmd[mc], 1 - args.recall)).mean()),
                    }
            # ── 호출어 게이트
            if W is not None and wk_te:
                Ewt, _, _ = embed_items(model, wk_te, root, cond, args.reps, babble)
                s_w = Ewt @ W
                wc, wo = Ec @ W, Eo @ W
                if cond == "clean":
                    wthr_clean = float(np.quantile(s_w, 1 - args.recall))
                    # 두 번째 임계 정책: 깨끗한 negative(명령+OOD 738클립)의 오각성 1%.
                    # 양성 18클립의 5% 분위수는 사실상 최저점 1개라 불안정하고, 잡음에서
                    # 오거부를 키운다(측정). negative 쪽이 표본이 40배 많다.
                    wthr_fa1 = float(np.quantile(np.concatenate([wc, wo]), 0.99))
                k_fr = int((s_w < wthr_clean).sum())
                k_fc = int((wc >= wthr_clean).sum()); k_fo = int((wo >= wthr_clean).sum())
                r["wake"] = {
                    "n_pos": int(len(s_w)), "thr": wthr_clean,
                    "fr": k_fr / len(s_w), "fr_ci": wilson(k_fr, len(s_w)),
                    "fa_cmd": k_fc / len(wc), "fa_cmd_ci": wilson(k_fc, len(wc)),
                    "fa_ood": k_fo / len(wo), "fa_ood_ci": wilson(k_fo, len(wo)),
                    "auroc_cmd": auroc(s_w, wc), "auroc_ood": auroc(s_w, wo),
                    "thr_fa1": wthr_fa1,
                    "fr_fa1": float((s_w < wthr_fa1).mean()),
                    "fr_fa1_ci": wilson(int((s_w < wthr_fa1).sum()), len(s_w)),
                    "fa_cmd_fa1": float((wc >= wthr_fa1).mean()),
                    "fa_ood_fa1": float((wo >= wthr_fa1).mean()),
                }
                # 시스템: 사용자가 "호출어 → 명령" 을 말했을 때 제대로 실행될 확률,
                # 그리고 호출어 없이 오가는 말 한 마디가 동작을 일으킬 확률.
                # 후자는 (OOD 가 호출어로 잡힘) × (8초 창 안의 다음 OOD 가 명령으로 통과)
                # 로 근사한다 — 두 발화의 독립을 가정한 값이라 대략치다.
                w = r["wake"]
                r["system"] = {
                    "success_gated": (1 - w["fr"]) * r["fixed_success"],
                    "success_gated_fa1": (1 - w["fr_fa1"]) * r["fixed_success"],
                    "false_action_per_ood_gated_idle_fa1": w["fa_ood_fa1"] * r["fixed_far"],
                    "success_ungated": r["fixed_success"],
                    "false_action_per_ood_ungated": r["fixed_far"],
                    "false_action_per_ood_gated_idle": w["fa_ood"] * r["fixed_far"],
                    "false_action_per_ood_in_window": r["fixed_far"],
                }
            mres["conds"][cond] = r
            print(f"{name:24s} {cond:6s} intent {r['intent_acc']:.4f}  "
                  f"FA@95 {r['far']*100:5.1f}% [{r['far_ci'][0]*100:4.1f},{r['far_ci'][1]*100:4.1f}]  "
                  f"short {r['far_short']*100:5.1f}% [{r['far_short_ci'][0]*100:4.1f},"
                  f"{r['far_short_ci'][1]*100:4.1f}]  rest {r['far_noshort']*100:4.1f}%  | "
                  f"fixed thr: recall {r['fixed_recall']*100:5.1f}% FA {r['fixed_far']*100:5.1f}%"
                  + (f"  | wake FR {r['wake']['fr']*100:4.1f}% FAc {r['wake']['fa_cmd']*100:4.1f}% "
                     f"FAo {r['wake']['fa_ood']*100:4.1f}% | FA1%-thr FR {r['wake']['fr_fa1']*100:4.1f}%"
                     if "wake" in r else ""),
                  flush=True)
        results["models"][name] = mres

    Path(args.out).write_text(json.dumps(results, ensure_ascii=False, indent=1),
                              encoding="utf-8")
    print(f"→ {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
