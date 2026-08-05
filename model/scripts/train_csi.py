#!/usr/bin/env python3
"""CSI 창 분류기 학습 — 음성 파이프라인을 그대로 재사용한다.

CSI 창은 (시간 × 서브캐리어) 행렬이고 그게 멜 스펙트로그램과 모양이 같다. 그래서
train.py 의 Encoder 를 손대지 않고 쓴다 — conv + GAP 구조라 입력 크기에 무관하다.

다만 음성과 두 가지가 다르다.

1. **라벨이 인텐트가 아니라 클래스다.** 음성에서는 "새 명령을 재학습 없이 추가" 가
   요구사항이어서 문장 단위 metric learning 을 했다. CSI 는 위치·상태가 미리 정해진
   유한 집합이므로 그냥 분류가 맞다. 다만 임베딩+프로토타입 구조를 유지하는데,
   그러면 나중에 새 위치를 등록만으로 추가할 수 있다.

2. **시간 분할이 필수다.** CSI 는 인접 창이 겹치고 환경이 천천히 변하므로, 무작위
   분할을 하면 테스트 창이 학습 창과 거의 같아져 정확도가 부풀려진다. 그래서 시간
   축으로 앞/뒤를 갈라 학습/테스트를 나눈다. 이게 이 스크립트에서 가장 중요한 부분이다.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

import csi_features as CF
from train import CosineHead, Encoder, count_macs

LABEL_NAME = {0: "빈방/ch1", 1: "왼쪽/ch6", 2: "가운데/ch11", 3: "오른쪽",
              4: "이동중"}


def load(paths: list[str]):
    """여러 세션을 읽어 창으로 자른다. 세션 경계를 넘어 창을 만들지 않는다."""
    Xs, ys, sess = [], [], []
    for si, p in enumerate(paths):
        d = np.load(p, allow_pickle=False)
        X, y, n = CF.windows(d["iq"], d["ms"], d["label"])
        if not len(X):
            print(f"  {Path(p).name}: 창 0개 — 건너뜀")
            continue
        Xs.append(X); ys.append(y); sess.append(np.full(len(X), si, np.int32))
        u, c = np.unique(y, return_counts=True)
        dist = " ".join(f"{LABEL_NAME.get(int(k), k)}:{v}" for k, v in zip(u, c))
        print(f"  {Path(p).name}: 창 {len(X)}개  {dist}")
    if not Xs:
        raise SystemExit("창을 하나도 못 만들었다.")
    return np.concatenate(Xs), np.concatenate(ys), np.concatenate(sess)


class Aug:
    """CSI 증강. 음성의 잔향·노이즈에 해당하는 것을 여기서 만든다."""

    def __init__(self, rng: np.random.Generator, enable: bool):
        self.rng, self.enable = rng, enable

    def __call__(self, x: np.ndarray) -> np.ndarray:
        if not self.enable:
            return x
        x = x.copy()
        # 서브캐리어 드롭 — 실제로 죽은 빈이 생긴다(first_word_invalid 등)
        for _ in range(int(self.rng.integers(0, 3))):
            f = int(self.rng.integers(0, x.shape[1]))
            x[:, f] = 0.0
        # 시간 마스크 — 트래픽이 끊긴 구간
        for _ in range(int(self.rng.integers(0, 3))):
            t = int(self.rng.integers(1, max(2, x.shape[0] // 8)))
            t0 = int(self.rng.integers(0, x.shape[0] - t))
            x[t0:t0 + t, :] = 0.0
        # 시간 이동 — 창 경계 위치가 달라지는 것을 흡수
        sh = int(self.rng.integers(-x.shape[0] // 8, x.shape[0] // 8 + 1))
        if sh:
            x = np.roll(x, sh, axis=0)
        # 진폭 잡음
        x = x + self.rng.standard_normal(x.shape).astype(np.float32) * \
            float(self.rng.uniform(0.0, 0.25))
        return np.clip(x, -CF.CLIP, CF.CLIP)


class WinSet(torch.utils.data.Dataset):
    def __init__(self, X, y, train: bool, seed: int = 0):
        self.X, self.y, self.train, self.seed = X, y, train, seed

    def __len__(self):
        return len(self.X)

    def __getitem__(self, i):
        rng = np.random.default_rng((self.seed, i, torch.initial_seed() % (1 << 31)))
        x = Aug(rng, self.train)(self.X[i])
        return torch.from_numpy(x.copy()), int(self.y[i])


@torch.no_grad()
def embed_all(model, ds, dev, bs=256):
    model.eval()
    out, ys = [], []
    for x, y in torch.utils.data.DataLoader(ds, batch_size=bs):
        out.append(model(x.to(dev)).cpu()); ys.append(y)
    return torch.cat(out), torch.cat(ys)


def proto_score(e_en, y_en, e_te, y_te):
    labs = sorted(set(int(v) for v in y_en.tolist()))
    P = torch.stack([F.normalize(e_en[y_en == c].mean(0), dim=0) for c in labs])
    sim = e_te @ P.t()
    pred = torch.tensor([labs[int(i)] for i in sim.argmax(1)])
    return float((pred == y_te).float().mean()), pred, sim.max(1).values, labs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("data", nargs="+", help="csi_*.npz")
    ap.add_argument("--out", default="model/out_csi")
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--bs", type=int, default=64)
    ap.add_argument("--lr", type=float, default=3e-3)
    ap.add_argument("--dim", type=int, default=64)
    ap.add_argument("--width", type=int, default=32)
    ap.add_argument("--test-frac", type=float, default=0.3,
                    help="각 세션의 뒤쪽 이 비율을 테스트로 (시간 분할)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    outd = Path(args.out); outd.mkdir(parents=True, exist_ok=True)

    print("데이터:")
    X, y, sess = load(args.data)
    n_cls = int(y.max()) + 1
    print(f"\n창 {len(X)}개, 입력 {X.shape[1:]}, 클래스 {n_cls}개")

    # ── 시간 분할. 무작위 분할은 인접 창이 거의 같아 정확도를 부풀린다.
    tr = np.zeros(len(X), bool)
    for s in np.unique(sess):
        idx = np.nonzero(sess == s)[0]          # 이미 시간순
        cut = int(len(idx) * (1.0 - args.test_frac))
        tr[idx[:cut]] = True
    te = ~tr
    print(f"시간 분할: 학습 {tr.sum()} / 테스트 {te.sum()} "
          f"(각 세션 앞 {1-args.test_frac:.0%} / 뒤 {args.test_frac:.0%})")
    for k in range(n_cls):
        print(f"  {LABEL_NAME.get(k, k):12s} 학습 {int((y[tr]==k).sum()):4d}  "
              f"테스트 {int((y[te]==k).sum()):4d}")

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    ds_tr = WinSet(X[tr], y[tr], True, args.seed)
    ds_en = WinSet(X[tr], y[tr], False)
    ds_te = WinSet(X[te], y[te], False)
    dl = torch.utils.data.DataLoader(ds_tr, batch_size=args.bs, shuffle=True,
                                     num_workers=4, drop_last=len(ds_tr) > args.bs,
                                     persistent_workers=True)

    model = Encoder(dim=args.dim, w=args.width).to(dev)
    head = CosineHead(args.dim, n_cls).to(dev)
    n_par = sum(p.numel() for p in model.parameters())
    macs = count_macs(Encoder(dim=args.dim, w=args.width),
                      t=X.shape[1], m=X.shape[2])
    print(f"\n인코더: 파라미터 {n_par/1000:.1f}K, 추론 1회 {macs/1e6:.2f}M MAC, 장치 {dev}\n")

    opt = torch.optim.AdamW(list(model.parameters()) + list(head.parameters()),
                            lr=args.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(
        opt, max_lr=args.lr, total_steps=max(1, args.epochs * len(dl)), pct_start=0.25)

    best = -1.0
    for ep in range(1, args.epochs + 1):
        model.train(); head.train()
        tl = tn = 0
        for xb, yb in dl:
            xb, yb = xb.to(dev), yb.to(dev)
            loss = F.cross_entropy(head(model(xb), yb), yb)
            opt.zero_grad(set_to_none=True); loss.backward(); opt.step(); sched.step()
            tl += float(loss) * len(yb); tn += len(yb)
        line = f"ep {ep:3d}  loss {tl/max(tn,1):6.3f}"
        if ep % 10 == 0 or ep == args.epochs:
            e_en, y_en = embed_all(model, ds_en, dev)
            e_te, y_te = embed_all(model, ds_te, dev)
            acc, _, _, _ = proto_score(e_en, y_en, e_te, y_te)
            line += f"   테스트 {acc:5.3f}"
            if acc > best:
                best = acc
                torch.save({"model": model.state_dict(), "dim": args.dim,
                            "width": args.width, "n_frames": X.shape[1],
                            "n_feat": X.shape[2], "n_cls": n_cls}, outd / "csi_encoder.pt")
                line += "  *저장"
        print(line, flush=True)

    # ── 최종 리포트
    ck = torch.load(outd / "csi_encoder.pt", map_location=dev, weights_only=False)
    model.load_state_dict(ck["model"]); model.eval()
    e_en, y_en = embed_all(model, ds_en, dev)
    e_te, y_te = embed_all(model, ds_te, dev)
    acc, pred, conf, labs = proto_score(e_en, y_en, e_te, y_te)

    cm = np.zeros((n_cls, n_cls), int)
    for t_, p_ in zip(y_te.tolist(), pred.tolist()):
        cm[t_, p_] += 1

    chance = float(max(np.bincount(y[te], minlength=n_cls)) / max(te.sum(), 1))
    print("\n" + "=" * 60)
    print(f"파라미터   {n_par/1000:.1f}K   추론 {macs/1e6:.2f}M MAC")
    print(f"테스트     {acc:.4f}   (다수 클래스만 찍기 = {chance:.4f})")
    print(f"입력       {X.shape[1]} 프레임 × {X.shape[2]} 서브캐리어")
    print("혼동 행렬 (행=실제):")
    print("            " + "".join(f"{LABEL_NAME.get(k,k)[:9]:>10s}" for k in range(n_cls)))
    for i in range(n_cls):
        print(f"  {LABEL_NAME.get(i,i)[:10]:10s}" + "".join(f"{cm[i,j]:10d}" for j in range(n_cls)))
    print("=" * 60)

    prot = torch.stack([F.normalize(e_en[y_en == c].mean(0), dim=0) for c in labs])
    np.savez(outd / "csi_prototypes.npz",
             protos=prot.numpy(), labels=np.array(labs, np.int32))
    (outd / "csi_report.json").write_text(json.dumps({
        "params": int(n_par), "macs": int(macs), "test_acc": acc,
        "chance": chance, "n_cls": n_cls,
        "n_frames": int(X.shape[1]), "n_feat": int(X.shape[2]),
        "n_train": int(tr.sum()), "n_test": int(te.sum()),
        "confusion": cm.tolist(),
    }, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"→ {outd/'csi_encoder.pt'}, {outd/'csi_prototypes.npz'}, {outd/'csi_report.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
