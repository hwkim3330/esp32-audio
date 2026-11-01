#!/usr/bin/env python3
"""차량 캐빈 음향 시뮬레이션 — 학습 증강과 잡음 평가셋이 같이 쓴다.

예전 증강(train.py 의 Augment)은 백색·핑크·저역 잡음과 지수감쇠 잔향이었다. 실제
캐빈 마이크가 듣는 것은 그보다 구조가 있다:

  - 노면 잡음  — 저역에 몰린 광대역, 타이어 결에 따라 천천히 출렁인다
  - 엔진/모터  — 회전수에 묶인 고조파열(내연) 또는 좁은 고음 휘슬(전기차)
  - 공조(HVAC) — 중역 광대역 + 송풍기 날개 톤
  - 바람       — 고역 쪽 광대역, 돌풍으로 출렁인다
  - 잔향       — 작고 흡음이 큰 방: RT60 30~200ms, 앞유리의 강한 초기 반사
  - 마이크 경로 — 대역 제한(고역통과·저역통과), 이득, AGC, 클리핑, 자기잡음

외부 잡음 데이터셋은 쓰지 않는다(라이선스 확인 없이 받지 않기로 했다). 전부
절차적으로 만든다 — 그래서 **같은 생성기로 학습하고 평가하면 평가가 쉬워진다**.
이걸 막으려고 평가용 '분포 이동' 셋을 따로 둔다(`shift`): 학습에 한 번도 안 나온
잡음 종류(옆사람 대화 babble, 음악, 빗소리, 방향지시등 딸깍임)와 다른 방식의 잔향
(직육면체 이미지 소스)을 쓴다. 학습 증강이 그 셋에서도 이득을 내야 진짜다.

모든 함수는 numpy 만 쓰고 rng 를 인자로 받는다 — 평가셋은 클립 경로로 시드를 정해
재현 가능하게 만들고, 디스크에 쓰지 않는다(NVMe 발열 때문에 파일을 늘리지 않는다).
"""
from __future__ import annotations

import hashlib
import math

import numpy as np

try:
    from scipy.signal import fftconvolve, butter, sosfilt
except ImportError:  # pragma: no cover
    fftconvolve = None

SR = 16000


# ─────────────────────────────────────────── 공용

def _shape(n: int, rng: np.random.Generator, mag_fn) -> np.ndarray:
    """백색 잡음을 주파수 영역에서 mag_fn(f) 로 빚는다. 단위 RMS 로 돌려준다."""
    m = 1 << int(math.ceil(math.log2(max(n, 256))))
    spec = np.fft.rfft(rng.standard_normal(m))
    f = np.fft.rfftfreq(m, 1.0 / SR)
    spec *= mag_fn(np.maximum(f, 1.0))
    x = np.fft.irfft(spec, m)[:n].astype(np.float32)
    return x / (np.sqrt((x ** 2).mean()) + 1e-12)


def _slow_mod(n: int, rng: np.random.Generator, rate=(0.3, 3.0), depth=(0.0, 0.4)):
    """천천히 출렁이는 진폭 포락선(평균 1)."""
    t = np.arange(n) / SR
    env = np.ones(n, np.float32)
    for _ in range(2):
        r = rng.uniform(*rate)
        d = rng.uniform(*depth) / 2
        env += d * np.sin(2 * np.pi * r * t + rng.uniform(0, 2 * np.pi)).astype(np.float32)
    return np.maximum(env, 0.05)


def _rms(x: np.ndarray) -> float:
    return float(np.sqrt((x.astype(np.float64) ** 2).mean()) + 1e-12)


def speech_power(x: np.ndarray) -> float:
    """발화 구간의 전력. 무음 패딩까지 평균하면 SNR 이 실제보다 높게 잡힌다."""
    fr = 160
    k = len(x) // fr
    if k < 2:
        return float((x ** 2).mean()) + 1e-12
    e = (x[:k * fr].reshape(k, fr).astype(np.float64) ** 2).mean(1)
    act = e[e > e.max() * 10 ** (-30 / 10)]               # 최대 대비 -30 dB 이상
    return float(act.mean()) + 1e-12


# ─────────────────────────────────────────── 잡음 원천 (학습·평가 공통)

def road(n, rng):
    fc = rng.uniform(60, 250)
    slope = rng.uniform(1.0, 2.0)
    bump_f, bump_w = rng.uniform(500, 1200), rng.uniform(0.3, 0.6)
    bump_g = rng.uniform(0.0, 0.5)

    def mag(f):
        base = 1.0 / (1.0 + (f / fc) ** 2) ** (slope / 2)
        bump = bump_g * np.exp(-0.5 * (np.log(f / bump_f) / bump_w) ** 2)
        return base + bump * (fc / bump_f) ** slope
    return _shape(n, rng, mag) * _slow_mod(n, rng, (0.5, 4.0), (0.0, 0.5))


def engine(n, rng):
    t = np.arange(n) / SR
    x = np.zeros(n, np.float64)
    if rng.random() < 0.6:                                 # 내연기관
        rpm0 = rng.uniform(700, 3500)
        rpm1 = rpm0 * rng.uniform(0.85, 1.15)
        cyl = rng.choice([3, 4, 6])
        f0 = (rpm0 + (rpm1 - rpm0) * t / max(t[-1], 1e-3)) / 60.0 * cyl / 2.0
        ph = 2 * np.pi * np.cumsum(f0) / SR
        a = rng.uniform(0.8, 1.6)
        for k in np.arange(0.5, 8.5, 0.5):                # 반차수 포함
            amp = (1.0 if k == int(k) else 0.3) / k ** a
            x += amp * np.sin(k * ph + rng.uniform(0, 2 * np.pi))
        x = x.astype(np.float32) / (_rms(x) + 1e-12)
        x += 0.3 * _shape(n, rng, lambda f: 1.0 / (1.0 + (f / 300.0) ** 2))
    else:                                                  # 전기차: 인버터·기어 휘슬
        f0 = rng.uniform(400, 4000)
        f1 = f0 * rng.uniform(0.9, 1.1)
        ff = f0 + (f1 - f0) * t / max(t[-1], 1e-3)
        ph = 2 * np.pi * np.cumsum(ff) / SR
        x = np.sin(ph) + 0.4 * np.sin(2 * ph + 1.0) + 0.2 * np.sin(3.1 * ph)
        x = x.astype(np.float32) / (_rms(x) + 1e-12)
        x = 0.4 * x + _shape(n, rng, lambda f: 1.0 / (1.0 + (f / 150.0) ** 2))
    return (x / (_rms(x) + 1e-12)).astype(np.float32)


def hvac(n, rng):
    pk = rng.uniform(250, 1500)
    w = rng.uniform(0.8, 1.5)
    x = _shape(n, rng, lambda f: np.exp(-0.5 * (np.log(f / pk) / w) ** 2) + 0.05)
    bpf = rng.uniform(80, 400)                             # 송풍기 날개 통과 톤
    t = np.arange(n) / SR
    tone = np.sin(2 * np.pi * bpf * t) + 0.5 * np.sin(4 * np.pi * bpf * t + 0.7)
    x = x + rng.uniform(0.0, 0.4) * tone.astype(np.float32) / 0.79
    return (x / (_rms(x) + 1e-12)).astype(np.float32)


def wind(n, rng):
    pk = rng.uniform(800, 3000)
    x = _shape(n, rng, lambda f: 1.0 / (1.0 + (pk / f) ** 2) / (1.0 + (f / 6000.0) ** 2))
    return x * _slow_mod(n, rng, (0.2, 2.0), (0.2, 0.9))


def cabin_noise(n, rng):
    """캐빈 잡음 한 덩어리 = 원천 4개의 무작위 혼합 (단위 RMS)."""
    w = rng.dirichlet([1.2, 0.8, 0.8, 0.5])
    x = (w[0] * road(n, rng) + w[1] * engine(n, rng)
         + w[2] * hvac(n, rng) + w[3] * wind(n, rng))
    return (x / (_rms(x) + 1e-12)).astype(np.float32)


# ─────────────────────────────────────────── 잔향 (학습용: 확률적 캐빈 RIR)

def cabin_rir(rng):
    rt60 = rng.uniform(0.03, 0.20)
    n = int(SR * min(0.4, rt60 * 1.5)) + 64
    h = np.zeros(n, np.float32)
    d0 = int(rng.uniform(0.6, 3.0) * SR / 1000)            # 입→마이크 0.2~1m
    h[d0] = 1.0
    for _ in range(int(rng.integers(4, 14))):              # 초기 반사(앞유리·천장·창)
        dt = d0 + int(rng.uniform(0.4, 8.0) * SR / 1000)
        if dt < n:
            h[dt] += rng.uniform(-0.8, 0.8)
    t = np.arange(n) / SR
    st = d0 + int(0.004 * SR)
    tail_lo = rng.standard_normal(n) * np.exp(-6.9 * t / rt60)
    tail_hi = rng.standard_normal(n) * np.exp(-6.9 * t / (rt60 * rng.uniform(0.3, 0.7)))
    # 고역이 저역보다 빨리 죽는다 — 시트·천장재의 흡음
    k = 8
    tail_lo = np.convolve(tail_lo, np.ones(k) / k, mode="same")
    tail = tail_lo * rng.uniform(0.6, 1.2) + (tail_hi - np.convolve(
        tail_hi, np.ones(k) / k, mode="same")) * rng.uniform(0.2, 0.8)
    tail[:st] = 0.0
    h += (tail * rng.uniform(0.05, 0.3)).astype(np.float32)
    return h / (np.sqrt((h ** 2).sum()) + 1e-9)


# ─────────────────────────────────────────── 마이크 경로

def mic_chain(x, rng):
    """대역 제한 → 이득 → (AGC) → 클리핑 → 자기잡음."""
    hp = rng.uniform(50, 300)
    lp = rng.uniform(3400, 4000) if rng.random() < 0.3 else rng.uniform(5500, 7600)
    sos = butter(2, [hp, min(lp, SR / 2 - 100)], btype="band", fs=SR, output="sos")
    y = sosfilt(sos, x).astype(np.float32)
    y *= 10 ** (rng.uniform(-20, 10) / 20) * (0.1 / (_rms(y) + 1e-9))
    if rng.random() < 0.5:                                 # AGC: 프레임 RMS 추적
        fr = 160
        target = 10 ** (rng.uniform(-26, -14) / 20)
        att, rel = rng.uniform(0.3, 0.8), rng.uniform(0.02, 0.1)
        g, out = 1.0, np.empty_like(y)
        gmax = 10 ** (rng.uniform(15, 30) / 20)
        for i in range(0, len(y), fr):
            seg = y[i:i + fr]
            want = min(gmax, target / (_rms(seg) + 1e-6))
            a = att if want < g else rel
            g += a * (want - g)
            out[i:i + fr] = seg * g
        y = out
    y = y + rng.standard_normal(len(y)).astype(np.float32) * 10 ** (rng.uniform(-75, -55) / 20)
    return np.clip(y, -1.0, 1.0)


def mix_at_snr(s, noise, snr_db):
    ps = speech_power(s)
    pn = float((noise ** 2).mean()) + 1e-12
    return s + noise * math.sqrt(ps / pn / (10 ** (snr_db / 10)))


def cabin(x, rng, snr=(0.0, 20.0), p_rir=0.8):
    """학습 증강 한 번: 캐빈 잔향 + 캐빈 잡음(SNR 균등) + 마이크 경로."""
    if rng.random() < p_rir:
        x = fftconvolve(x, cabin_rir(rng))[: len(x)].astype(np.float32)
    snr_db = rng.uniform(*snr) if isinstance(snr, tuple) else float(snr)
    x = mix_at_snr(x, cabin_noise(len(x), rng), snr_db)
    return mic_chain(x, rng)


# ─────────────────────────────────────────── 분포 이동 평가 전용 (학습에 절대 안 씀)

def shoebox_rir(rng):
    """직육면체 방 이미지 소스(1~3차 반사, 주파수 무관 흡음). cabin_rir 와 다른 방식."""
    L = np.array([rng.uniform(1.6, 2.2), rng.uniform(1.3, 1.6), rng.uniform(1.0, 1.3)])
    src = L * rng.uniform(0.2, 0.8, 3)
    mic = L * rng.uniform(0.2, 0.8, 3)
    beta = rng.uniform(0.5, 0.85)                          # 반사 계수
    n = int(0.12 * SR)
    h = np.zeros(n, np.float64)
    c = 343.0
    for i in range(-3, 4):
        for j in range(-3, 4):
            for k in range(-2, 3):
                o = abs(i) + abs(j) + abs(k)
                if o > 5:
                    continue
                img = np.array([
                    (i * L[0] + (src[0] if i % 2 == 0 else L[0] - src[0])),
                    (j * L[1] + (src[1] if j % 2 == 0 else L[1] - src[1])),
                    (k * L[2] + (src[2] if k % 2 == 0 else L[2] - src[2]))])
                d = float(np.linalg.norm(img - mic)) + 0.05
                tt = int(round(d / c * SR))
                if tt < n:
                    h[tt] += (beta ** o) / d
    h = h.astype(np.float32)
    return h / (np.sqrt((h ** 2).sum()) + 1e-9)


def music(n, rng):
    t = np.arange(n) / SR
    x = np.zeros(n, np.float64)
    bpm = rng.uniform(80, 140)
    beat = 60.0 / bpm
    root = rng.uniform(110, 220)
    for b in range(int(t[-1] / beat) + 1):
        i0 = int(b * beat * SR)
        chord = root * 2 ** (np.array(rng.choice([[0, 4, 7], [0, 3, 7], [0, 5, 9], [-2, 2, 5]]) +
                                     rng.choice([0, 5, 7, -5])) / 12)
        seg = t[i0:i0 + int(beat * SR)] - t[i0] if i0 < n else None
        if seg is None or not len(seg):
            continue
        env = np.exp(-seg * rng.uniform(2, 6))
        for f in chord:
            for h in (1, 2, 3):
                x[i0:i0 + len(seg)] += env * np.sin(2 * np.pi * f * h * seg) / h ** 1.5
        kick = np.exp(-seg * 30) * np.sin(2 * np.pi * 60 * seg)
        x[i0:i0 + len(seg)] += 1.5 * kick
    return (x / (_rms(x) + 1e-12)).astype(np.float32)


def rain_clicks(n, rng):
    x = 0.3 * _shape(n, rng, lambda f: 1.0 / (1.0 + (2000.0 / f) ** 2))
    for _ in range(int(rng.poisson(40 * n / SR))):         # 빗방울
        i = int(rng.integers(0, max(1, n - 80)))
        x[i:i + 80] += rng.uniform(0.5, 3.0) * np.exp(-np.arange(80) / 8) * rng.standard_normal(80)
    per = rng.uniform(0.6, 0.8)                            # 방향지시등 딸깍
    for k in range(int(n / SR / per) + 1):
        i = int((k * per + rng.uniform(0, 0.05)) * SR)
        if i + 200 < n:
            x[i:i + 200] += 4.0 * np.exp(-np.arange(200) / 20) * np.sin(np.arange(200) * 1.3)
    return (x / (_rms(x) + 1e-12)).astype(np.float32)


def shifted(x, rng, snr_db, babble_pool):
    """분포 이동 평가 한 번. babble_pool 은 '다른 사람 말' 파형 목록(float)."""
    x = fftconvolve(x, shoebox_rir(rng))[: len(x)].astype(np.float32)
    n = len(x)
    kind = rng.integers(0, 3)
    if kind == 0 and babble_pool:                          # 옆사람 대화 1~3명
        b = np.zeros(n, np.float32)
        for _ in range(int(rng.integers(1, 4))):
            s = babble_pool[int(rng.integers(0, len(babble_pool)))]
            o = int(rng.integers(-len(s) // 2, n // 2))
            seg = np.zeros(n, np.float32)
            src = s[max(0, -o):max(0, -o) + n - max(0, o)]
            seg[max(0, o):max(0, o) + len(src)] = src
            b += seg / (_rms(s) + 1e-9)
        noise = b + 0.3 * road(n, rng)
    elif kind == 1:
        noise = music(n, rng) + 0.3 * road(n, rng)
    else:
        noise = rain_clicks(n, rng) + 0.5 * road(n, rng)
    y = mix_at_snr(x, noise, snr_db)
    # 마이크 경로도 다르게: 고정 전화 대역 + 강한 압축(AGC 대신 정적 tanh)
    sos = butter(4, [rng.uniform(150, 400), rng.uniform(3000, 3600)], btype="band",
                 fs=SR, output="sos")
    y = sosfilt(sos, y).astype(np.float32)
    y = y * (0.3 / (_rms(y) + 1e-9))
    return np.tanh(y * rng.uniform(1.0, 3.0)).astype(np.float32) * 0.5


def clip_seed(path: str, cond: str, rep: int) -> int:
    """평가셋 시드. 같은 클립·조건·복제면 언제 돌려도 같은 잡음이 붙는다."""
    return int(hashlib.sha1(f"{path}|{cond}|{rep}".encode()).hexdigest()[:12], 16)
