#!/usr/bin/env python3
"""INT8 양자화가 임베딩을 얼마나 망가뜨리는지 잰다. FPGA 이식의 go/no-go.

이 모델은 임베딩 인코더이고 판정은 프로토타입과의 코사인 거리로 한다.
따라서 "분류 정확도" 보다 **임베딩이 얼마나 보존되는가** 가 먼저다.
코사인이 무너지면 그 아래 모든 판정이 같이 무너진다.

가중치는 레이어별·출력채널별 대칭 양자화(per-channel symmetric)를 쓴다.
활성값은 float 로 둔다 — 가중치만 BRAM 에 올리면 되는지부터 보는 단계다.
"""
from __future__ import annotations
import sys, wave, argparse
from pathlib import Path
import numpy as np, torch
sys.path.insert(0, str(Path(__file__).parent))
import features as FE
from train import Encoder
from export import collect, RefRunner

def read_wav(p: Path) -> np.ndarray:
    with wave.open(str(p), "rb") as w:
        return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)

def quantize_layers(layers, bits=8):
    """출력채널별 대칭 양자화 후 다시 float 로 복원(가짜 양자화)."""
    qmax = (1 << (bits - 1)) - 1
    out, stats = [], []
    for L in layers:
        if "w" not in L:
            out.append(L); continue
        M = dict(L); w = np.asarray(L["w"], np.float32)
        axes = tuple(range(1, w.ndim))
        s = np.abs(w).max(axis=axes, keepdims=True) / qmax
        s[s == 0] = 1e-12
        q = np.clip(np.round(w / s), -qmax, qmax)
        M["w"] = (q * s).astype(np.float32)
        M["_q"] = q.astype(np.int8); M["_s"] = s.astype(np.float32)
        err = float(np.abs(M["w"] - w).max() / (np.abs(w).max() + 1e-12))
        stats.append((L["name"], w.size, err))
        out.append(M)
    return out, stats

ap = argparse.ArgumentParser()
ap.add_argument("--ckpt", default="../out/encoder_exclude2.pt")
ap.add_argument("--bits", type=int, default=8)
ap.add_argument("--n", type=int, default=200)
a = ap.parse_args()

ck = torch.load(a.ckpt, map_location="cpu", weights_only=False)
model = Encoder(dim=ck["dim"], w=ck["width"])
model.load_state_dict(ck["model"]); model.eval()

fl = collect(model)
ql, stats = quantize_layers(fl, a.bits)
nw = sum(s[1] for s in stats)
print(f"모델   : {Path(a.ckpt).name}   실배포 가중치 {nw:,}개 (BN 폴딩 후)")
print(f"양자화 : INT{a.bits} per-channel symmetric   -> {nw*a.bits/8/1024:.1f} KB")
print()
print(f"{'레이어':<10} {'params':>8}  {'최대상대오차':>10}")
for n, sz, e in stats: print(f"  {n:<8} {sz:>8,}  {e*100:>9.3f}%")

wavs = sorted(Path("../data/utterances").rglob("*.wav"))[: a.n]
rng = np.random.default_rng(0)
if len(wavs) > a.n: wavs = list(rng.choice(wavs, a.n, replace=False))
print(f"\n발화 {len(wavs)}개로 임베딩 비교")

rf, rq = RefRunner(fl), RefRunner(ql)
cos, l2 = [], []
for p in wavs:
    mel = FE.logmel(read_wav(p))
    ef, eq = rf(mel), rq(mel)
    cos.append(float(ef @ eq / (np.linalg.norm(ef) * np.linalg.norm(eq) + 1e-12)))
    l2.append(float(np.linalg.norm(ef - eq)))
cos, l2 = np.array(cos), np.array(l2)
print(f"  코사인 유사도  평균 {cos.mean():.6f}  최소 {cos.min():.6f}  (1.0 이 완전보존)")
print(f"  L2 거리        평균 {l2.mean():.6f}  최대 {l2.max():.6f}")
print()
if cos.min() > 0.999:   print("  판정: 손실 무시 가능. FPGA 이식 진행 가능")
elif cos.min() > 0.99:  print("  판정: 경미. 실제 거부 성능 재확인 권장")
else:                   print("  판정: 손실 큼. 비트수를 올리거나 QAT 필요")
