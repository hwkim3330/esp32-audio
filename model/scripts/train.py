#!/usr/bin/env python3
"""캐빈 노드 음성 임베딩 인코더 학습.

설계 의도가 하나 있고, 나머지는 거기서 따라온다.

  라벨을 인텐트(27개)가 아니라 "문장"(약 95개)으로 둔다.

인텐트로 학습하면 그 27개 결정 경계에만 맞춰지고, 28번째 명령을 추가하려면 다시
학습해야 한다. 문장 단위로 "같은 말은 같은 벡터" 를 배우게 하면 처음 보는 문장도
등록(임베딩 평균)만으로 동작한다. 그게 이 프로젝트의 요구사항이다 —
태블릿에서 문장 한 줄 넣으면 재학습 없이 새 명령이 생겨야 한다.

그래서 그게 진짜 되는지 반드시 측정한다: 문장 일부를 학습에서 완전히 빼고
(--holdout-phrases), 등록만으로 맞히는지 본다. 이 숫자가 낮으면 설계가 실패한 것이다.

보이스도 2개 빼서 (--holdout-voices) "학습에 없던 목소리" 일반화를 따로 본다.
"""
from __future__ import annotations

import argparse
import json
import math
import time
import wave
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

import features as FE

try:
    from scipy.signal import fftconvolve as _fftconv
except ImportError:                       # scipy 없으면 느리지만 동작은 한다
    def _fftconv(a, b):
        return np.convolve(a, b)

# ─────────────────────────────────────────── 데이터

def read_wav_i16(path: Path) -> np.ndarray:
    with wave.open(str(path), "rb") as w:
        assert w.getnchannels() == 1 and w.getsampwidth() == 2, path
        return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)


class Augment:
    """차량 캐빈 원거리 마이크를 가정한 증강.

    합성 음성은 너무 깨끗해서 그대로 학습하면 실제 마이크에서 무너진다.
    잔향·노이즈·이득·시간이동을 웨이브폼에서, 마스킹을 스펙트럼에서 건다.
    """

    def __init__(self, rng: np.random.Generator, enable: bool = True):
        self.rng = rng
        self.enable = enable

    def _reverb(self, x: np.ndarray) -> np.ndarray:
        rt60 = self.rng.uniform(0.08, 0.35)
        n = int(rt60 * FE.SR)
        if n < 8:
            return x
        t = np.arange(n) / FE.SR
        ir = self.rng.standard_normal(n).astype(np.float32) * np.exp(-6.9 * t / rt60)
        ir[0] += 1.0                      # 직접음
        ir /= np.sqrt((ir ** 2).sum()) + 1e-9
        # np.convolve 는 여기서 O(len(x)*len(ir)) 라 5600탭 IR 이면 샘플당 1.7억 연산이
        # 되고, 데이터로더가 GPU 를 굶긴다(실측: GPU 2%). FFT 컨볼루션이 100배 빠르다.
        return _fftconv(x, ir)[: len(x)].astype(np.float32)

    def _noise(self, x: np.ndarray) -> np.ndarray:
        snr_db = self.rng.uniform(0.0, 25.0)
        kind = self.rng.integers(0, 3)
        n = self.rng.standard_normal(len(x)).astype(np.float32)
        if kind == 1:                      # 핑크 (1/f 근사)
            n = np.cumsum(n)
            n -= n.mean()
        elif kind == 2:                    # 차량 저주파 럼블
            k = 64
            n = np.convolve(n, np.ones(k, np.float32) / k)[: len(x)]
        sp = float((x ** 2).mean()) + 1e-12
        npow = float((n ** 2).mean()) + 1e-12
        scale = math.sqrt(sp / npow / (10.0 ** (snr_db / 10.0)))
        return x + n * scale

    def wave(self, pcm: np.ndarray) -> np.ndarray:
        x = pcm.astype(np.float32) / 32768.0
        if not self.enable:
            return x
        shift = int(self.rng.integers(-FE.SR // 5, FE.SR // 5))   # ±200ms
        if shift > 0:
            x = np.concatenate([np.zeros(shift, np.float32), x])
        elif shift < 0:
            x = x[-shift:] if -shift < len(x) else x[:1]
        if self.rng.random() < 0.6:
            x = self._reverb(x)
        if self.rng.random() < 0.9:
            x = self._noise(x)
        x = x * (10.0 ** (self.rng.uniform(-12.0, 6.0) / 20.0))
        return np.clip(x, -1.0, 1.0)

    def spec(self, m: np.ndarray) -> np.ndarray:
        if not self.enable:
            return m
        m = m.copy()
        for _ in range(int(self.rng.integers(0, 3))):             # 주파수 마스크
            f = int(self.rng.integers(1, 7))
            f0 = int(self.rng.integers(0, max(1, FE.N_MELS - f)))
            m[:, f0:f0 + f] = 0.0
        for _ in range(int(self.rng.integers(0, 3))):             # 시간 마스크
            t = int(self.rng.integers(1, 21))
            t0 = int(self.rng.integers(0, max(1, m.shape[0] - t)))
            m[t0:t0 + t, :] = 0.0
        return m


class Utterances(torch.utils.data.Dataset):
    def __init__(self, items: list[dict], root: Path, text2id: dict[str, int],
                 train: bool, seed: int = 0):
        self.items = items
        self.root = root
        self.text2id = text2id
        self.train = train
        self.seed = seed
        self.pcm = [read_wav_i16(root / it["path"]) for it in items]

    def __len__(self) -> int:
        return len(self.items)

    def __getitem__(self, i: int):
        # 워커별·에폭별로 다른 증강이 나오되 재현 가능하게.
        rng = np.random.default_rng((self.seed, i, torch.initial_seed() % (1 << 31)))
        aug = Augment(rng, enable=self.train)
        x = aug.wave(self.pcm[i])
        m = FE.normalize(FE.logmel(x))
        m = aug.spec(m)
        # OOD 클립은 text2id 에 없다. 라벨 -1 로 흘려보낸다 — 거부 임계값 측정에만
        # 쓰이고 분류 대상이 아니다. (여기서 KeyError 를 내면 학습은 끝났는데
        # 리포트 단계에서 죽는다.)
        return (torch.from_numpy(m.copy()),
                self.text2id.get(self.items[i]["text"], -1))


# ─────────────────────────────────────────── 모델

class DSConv(nn.Module):
    """Depthwise-separable conv. MCU 에서 표준적으로 쓰이는 블록."""

    def __init__(self, cin: int, cout: int, stride=(1, 1)):
        super().__init__()
        self.dw = nn.Conv2d(cin, cin, 3, stride, 1, groups=cin, bias=False)
        self.bn1 = nn.BatchNorm2d(cin)
        self.pw = nn.Conv2d(cin, cout, 1, 1, 0, bias=False)
        self.bn2 = nn.BatchNorm2d(cout)

    def forward(self, x):
        x = F.relu(self.bn1(self.dw(x)))
        return F.relu(self.bn2(self.pw(x)))


class Encoder(nn.Module):
    """(B, 192, 40) 로그-멜 → (B, dim) L2 정규화 임베딩."""

    def __init__(self, dim: int = 128, w: int = 64):
        super().__init__()
        self.stem = nn.Sequential(
            nn.Conv2d(1, w // 2, 3, (2, 2), 1, bias=False),
            nn.BatchNorm2d(w // 2), nn.ReLU(inplace=True))
        self.blocks = nn.Sequential(
            DSConv(w // 2, w,     (2, 2)),
            DSConv(w,      w,     (1, 1)),
            DSConv(w,      2 * w, (2, 2)),
            DSConv(2 * w,  2 * w, (1, 1)),
            DSConv(2 * w,  2 * w, (2, 1)),
        )
        self.head = nn.Linear(2 * w, dim, bias=False)
        self.dim = dim

    def forward(self, x):
        x = x.unsqueeze(1)                       # (B,1,T,M)
        x = self.blocks(self.stem(x))
        x = x.mean(dim=(2, 3))                   # GAP
        return F.normalize(self.head(x), dim=1)


class CosineHead(nn.Module):
    """정규화된 프록시와의 코사인 + 마진. 임베딩이 클래스 수에 덜 묶이게 한다."""

    def __init__(self, dim: int, n_cls: int, scale: float = 24.0, margin: float = 0.25):
        super().__init__()
        self.w = nn.Parameter(torch.randn(n_cls, dim) * 0.01)
        self.scale, self.margin = scale, margin

    def forward(self, emb, y):
        cos = emb @ F.normalize(self.w, dim=1).t()
        oh = F.one_hot(y, self.w.shape[0]).float()
        return self.scale * (cos - self.margin * oh)


def count_macs(model: Encoder, t: int = FE.N_FRAMES, m: int = FE.N_MELS) -> int:
    """추론 1회 MAC 수. ESP32 실시간성의 실제 제약이라 파라미터 수보다 중요하다."""
    macs = 0
    hooks = []

    def hook(mod, inp, out):
        nonlocal macs
        if isinstance(mod, nn.Conv2d):
            cin = mod.in_channels // mod.groups
            macs += out.numel() * cin * mod.kernel_size[0] * mod.kernel_size[1]
        elif isinstance(mod, nn.Linear):
            macs += mod.in_features * mod.out_features

    for mod in model.modules():
        if isinstance(mod, (nn.Conv2d, nn.Linear)):
            hooks.append(mod.register_forward_hook(hook))
    model.eval()
    with torch.no_grad():
        model(torch.zeros(1, t, m))
    for h in hooks:
        h.remove()
    return macs


# ─────────────────────────────────────────── 평가

@torch.no_grad()
def embed_all(model: Encoder, ds: Utterances, dev: str, bs: int = 256):
    model.eval()
    out, ys = [], []
    dl = torch.utils.data.DataLoader(ds, batch_size=bs, num_workers=4)
    for x, y in dl:
        out.append(model(x.to(dev)).cpu())
        ys.append(y)
    return torch.cat(out), torch.cat(ys)


def enroll_and_score(emb_enroll, y_enroll, emb_test, y_test, ood_mask_test=None):
    """등록 셋으로 클래스 중심(프로토타입)을 만들고 테스트를 최근접으로 맞힌다.

    실제 ESP32 동작과 같은 방식이다: 명령마다 임베딩 평균 하나를 갖고 코사인 비교.
    """
    labels = sorted(set(int(v) for v in y_enroll.tolist()))
    protos = torch.stack([
        F.normalize(emb_enroll[y_enroll == c].mean(0), dim=0) for c in labels])
    sim = emb_test @ protos.t()                       # (N, C)
    best = sim.argmax(1)
    pred = torch.tensor([labels[int(i)] for i in best])
    conf = sim.max(1).values
    keep = torch.ones(len(y_test), dtype=torch.bool) if ood_mask_test is None \
        else ~ood_mask_test
    acc = float((pred[keep] == y_test[keep]).float().mean()) if keep.any() else float("nan")
    return acc, conf, pred, keep


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="model/data/utterances")
    ap.add_argument("--commands", default="model/commands.json")
    ap.add_argument("--out", default="model/out")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--bs", type=int, default=128)
    ap.add_argument("--lr", type=float, default=3e-3)
    ap.add_argument("--dim", type=int, default=128)
    ap.add_argument("--width", type=int, default=64)
    ap.add_argument("--holdout-voices", nargs="+", default=["F5", "M5"])
    ap.add_argument("--holdout-phrases", type=float, default=0.15)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    rng = np.random.default_rng(args.seed)
    root = Path(args.data)
    man = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    items = man["items"]
    outd = Path(args.out); outd.mkdir(parents=True, exist_ok=True)

    # 인텐트 문장만 대상 (OOD 는 거부 임계값 측정에만 쓴다)
    cmd_items = [it for it in items if it["label"] != "_ood"]
    ood_items = [it for it in items if it["label"] == "_ood"]
    texts = sorted({it["text"] for it in cmd_items})

    # 문장 홀드아웃: 인텐트마다 최소 1개는 학습에 남긴다.
    by_intent: dict[str, list[str]] = {}
    for it in cmd_items:
        by_intent.setdefault(it["label"], [])
        if it["text"] not in by_intent[it["label"]]:
            by_intent[it["label"]].append(it["text"])
    unseen: set[str] = set()
    for lab, ph in by_intent.items():
        if len(ph) < 2:
            continue
        k = max(1, int(round(len(ph) * args.holdout_phrases)))
        k = min(k, len(ph) - 1)
        unseen.update(rng.choice(ph, size=k, replace=False).tolist())

    hv = set(args.holdout_voices)
    text2id = {t: i for i, t in enumerate(texts)}

    train_items = [it for it in cmd_items
                   if it["voice"] not in hv and it["text"] not in unseen]
    # 테스트: 보류 보이스만. 등록(enroll)은 학습 보이스로 한다 — 실제 등록 절차와 같다.
    enroll_seen = [it for it in cmd_items
                   if it["voice"] not in hv and it["text"] not in unseen]
    test_seen = [it for it in cmd_items
                 if it["voice"] in hv and it["text"] not in unseen]
    enroll_unseen = [it for it in cmd_items
                     if it["voice"] not in hv and it["text"] in unseen]
    test_unseen = [it for it in cmd_items
                   if it["voice"] in hv and it["text"] in unseen]
    test_ood = [it for it in ood_items if it["voice"] in hv]

    print(f"문장 {len(texts)}개 (학습 {len(texts)-len(unseen)} / 홀드아웃 {len(unseen)})")
    print(f"보류 보이스 {sorted(hv)}")
    print(f"학습 클립 {len(train_items)}, "
          f"테스트(학습된 문장) {len(test_seen)}, "
          f"테스트(처음 보는 문장) {len(test_unseen)}, OOD {len(test_ood)}")

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    ds_tr = Utterances(train_items, root, text2id, train=True, seed=args.seed)
    mk = lambda its: Utterances(its, root, text2id, train=False)
    ds_en_s, ds_te_s = mk(enroll_seen), mk(test_seen)
    ds_en_u, ds_te_u = mk(enroll_unseen), mk(test_unseen)
    ds_ood = mk(test_ood) if test_ood else None

    dl = torch.utils.data.DataLoader(
        ds_tr, batch_size=args.bs, shuffle=True, num_workers=8,
        drop_last=True, persistent_workers=True)

    model = Encoder(dim=args.dim, w=args.width).to(dev)
    head = CosineHead(args.dim, len(texts)).to(dev)
    n_par = sum(p.numel() for p in model.parameters())
    macs = count_macs(Encoder(dim=args.dim, w=args.width))
    print(f"\n인코더: 파라미터 {n_par/1000:.1f}K (int8 약 {n_par/1024:.0f}KB), "
          f"추론 1회 {macs/1e6:.2f}M MAC")
    print(f"장치: {dev}\n")

    opt = torch.optim.AdamW(list(model.parameters()) + list(head.parameters()),
                            lr=args.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(
        opt, max_lr=args.lr, total_steps=args.epochs * len(dl), pct_start=0.25)

    best = -1.0
    for ep in range(1, args.epochs + 1):
        model.train(); head.train()
        tl = tn = tc = 0
        t0 = time.time()
        for x, y in dl:
            x, y = x.to(dev, non_blocking=True), y.to(dev, non_blocking=True)
            logits = head(model(x), y)
            loss = F.cross_entropy(logits, y)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step(); sched.step()
            tl += float(loss) * len(y); tn += len(y)
            tc += int((logits.argmax(1) == y).sum())
        line = (f"ep {ep:3d}  loss {tl/tn:6.3f}  train-acc {tc/tn:5.3f}  "
                f"{time.time()-t0:4.1f}s")

        if ep % 10 == 0 or ep == args.epochs:
            e_s, ys = embed_all(model, ds_en_s, dev)
            t_s, yt = embed_all(model, ds_te_s, dev)
            acc_s, conf_s, _, _ = enroll_and_score(e_s, ys, t_s, yt)
            msg = f"  |  학습된문장 {acc_s:5.3f}"
            if len(ds_te_u):
                e_u, yu = embed_all(model, ds_en_u, dev)
                t_u, ytu = embed_all(model, ds_te_u, dev)
                acc_u, conf_u, _, _ = enroll_and_score(e_u, yu, t_u, ytu)
                msg += f"  처음보는문장 {acc_u:5.3f}"
            else:
                acc_u, conf_u = float("nan"), None
            line += msg
            score = acc_u if not math.isnan(acc_u) else acc_s
            if score > best:
                best = score
                torch.save({"model": model.state_dict(),
                            "dim": args.dim, "width": args.width,
                            "texts": texts, "n_frames": FE.N_FRAMES,
                            "n_mels": FE.N_MELS},
                           outd / "encoder.pt")
                line += "  *저장"
        print(line, flush=True)

    # ── 최종 리포트: 거부 임계값까지 함께 잡는다
    ck = torch.load(outd / "encoder.pt", map_location=dev, weights_only=False)
    model.load_state_dict(ck["model"]); model.eval()

    e_s, ys = embed_all(model, ds_en_s, dev)
    t_s, yt = embed_all(model, ds_te_s, dev)
    acc_s, conf_s, _, _ = enroll_and_score(e_s, ys, t_s, yt)

    report = {"params": int(n_par), "macs_per_inference": int(macs),
              "dim": args.dim, "width": args.width,
              "n_texts": len(texts), "unseen_texts": sorted(unseen),
              "holdout_voices": sorted(hv),
              "acc_seen_phrases": acc_s}

    if len(ds_te_u):
        e_u, yu = embed_all(model, ds_en_u, dev)
        t_u, ytu = embed_all(model, ds_te_u, dev)
        acc_u, conf_u, _, _ = enroll_and_score(e_u, yu, t_u, ytu)
        report["acc_unseen_phrases"] = acc_u

    if ds_ood is not None:
        # 전체 명령으로 등록한 프로토타입에 OOD 를 넣어 유사도 분포를 본다.
        e_all = torch.cat([e_s] + ([e_u] if len(ds_te_u) else []))
        y_all = torch.cat([ys] + ([yu] if len(ds_te_u) else []))
        t_o, _ = embed_all(model, ds_ood, dev)
        labels = sorted(set(int(v) for v in y_all.tolist()))
        protos = torch.stack([
            F.normalize(e_all[y_all == c].mean(0), dim=0) for c in labels])
        conf_ood = (t_o @ protos.t()).max(1).values
        conf_cmd = torch.cat([conf_s] + ([conf_u] if len(ds_te_u) else []))
        # 명령 95% 를 통과시키는 임계값에서 OOD 오수락률
        thr = float(torch.quantile(conf_cmd, 0.05))
        far = float((conf_ood >= thr).float().mean())
        report.update({
            "reject_threshold": thr,
            "ood_false_accept_at_95pct_recall": far,
            "conf_cmd_mean": float(conf_cmd.mean()),
            "conf_ood_mean": float(conf_ood.mean()),
        })

    (outd / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")

    print("\n" + "=" * 62)
    print(f"파라미터        {n_par/1000:.1f}K   (int8 약 {n_par/1024:.0f}KB)")
    print(f"추론 1회        {macs/1e6:.2f}M MAC")
    print(f"학습된 문장     {acc_s:.3f}   (보류 보이스에서)")
    if "acc_unseen_phrases" in report:
        print(f"처음 보는 문장  {report['acc_unseen_phrases']:.3f}   "
              f"← 재학습 없이 명령 추가가 되는지의 지표")
    if "reject_threshold" in report:
        print(f"거부 임계값     {report['reject_threshold']:.3f}  "
              f"→ OOD 오수락 {report['ood_false_accept_at_95pct_recall']*100:.1f}% "
              f"(명령 재현율 95%)")
        print(f"코사인 평균     명령 {report['conf_cmd_mean']:.3f} / "
              f"OOD {report['conf_ood_mean']:.3f}")
    print("=" * 62)
    print(f"→ {outd/'encoder.pt'}, {outd/'report.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
