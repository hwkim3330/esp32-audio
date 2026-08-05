#!/usr/bin/env python3
"""로그-멜 프런트엔드. 학습(파이썬)과 추론(ESP32 C)이 같은 값을 내야 하는 유일한 지점.

여기가 어긋나면 모델은 학습에서 잘 되고 보드에서 망하는데, 원인 파악이 가장 어려운
종류의 버그가 된다. 그래서 필터뱅크를 numpy 로 직접 만들고, 같은 상수를 C 헤더로
내보낸다 (export.py 가 이 모듈의 mel_filterbank() 를 그대로 덤프한다).

파라미터는 MCU KWS 에서 표준적으로 쓰는 값을 따랐다.
"""
from __future__ import annotations

import numpy as np

SR = 16000
N_FFT = 512
WIN = 400          # 25ms
HOP = 160          # 10ms
N_MELS = 40
FMIN = 20.0
FMAX = 7600.0
LOG_FLOOR = 1e-5
N_FRAMES = 192     # 1.92s. 가장 긴 명령도 담기는 길이 (manifest 로 확인)


def hz_to_mel(f: np.ndarray | float) -> np.ndarray | float:
    return 2595.0 * np.log10(1.0 + np.asarray(f, dtype=np.float64) / 700.0)


def mel_to_hz(m: np.ndarray | float) -> np.ndarray | float:
    return 700.0 * (10.0 ** (np.asarray(m, dtype=np.float64) / 2595.0) - 1.0)


def mel_filterbank(n_mels: int = N_MELS, n_fft: int = N_FFT, sr: int = SR,
                   fmin: float = FMIN, fmax: float = FMAX) -> np.ndarray:
    """(n_mels, n_fft//2+1) 삼각 필터뱅크. slaney 정규화는 쓰지 않는다 (C 쪽 단순화)."""
    n_bins = n_fft // 2 + 1
    fft_freqs = np.linspace(0.0, sr / 2.0, n_bins, dtype=np.float64)
    mel_pts = np.linspace(hz_to_mel(fmin), hz_to_mel(fmax), n_mels + 2)
    hz_pts = mel_to_hz(mel_pts)

    fb = np.zeros((n_mels, n_bins), dtype=np.float64)
    for m in range(n_mels):
        left, center, right = hz_pts[m], hz_pts[m + 1], hz_pts[m + 2]
        # 상승부
        lo = (fft_freqs > left) & (fft_freqs <= center)
        fb[m, lo] = (fft_freqs[lo] - left) / max(center - left, 1e-12)
        # 하강부
        hi = (fft_freqs > center) & (fft_freqs < right)
        fb[m, hi] = (right - fft_freqs[hi]) / max(right - center, 1e-12)
    return fb.astype(np.float32)


def hann(win: int = WIN) -> np.ndarray:
    """주기형 Hann (STFT 표준). np.hanning 은 대칭형이라 값이 다르다 — 섞지 말 것."""
    n = np.arange(win, dtype=np.float64)
    return (0.5 - 0.5 * np.cos(2.0 * np.pi * n / win)).astype(np.float32)


_FB = mel_filterbank()
_WIN = hann()


def logmel(pcm: np.ndarray, n_frames: int | None = N_FRAMES,
           center_pad: bool = True) -> np.ndarray:
    """int16 또는 float PCM → (n_frames, N_MELS) float32 로그-멜.

    n_frames 가 주어지면 가운데를 기준으로 자르거나 채운다. 발화가 창보다 길면
    가운데를 남기는 게 앞뒤 무음을 버리는 것보다 안전하다.
    """
    x = np.asarray(pcm)
    if x.dtype == np.int16:
        x = x.astype(np.float32) / 32768.0
    else:
        x = x.astype(np.float32)

    if len(x) < WIN:
        x = np.pad(x, (0, WIN - len(x)))

    n_f = 1 + (len(x) - WIN) // HOP
    idx = np.arange(WIN)[None, :] + HOP * np.arange(n_f)[:, None]
    frames = x[idx] * _WIN                                    # (n_f, WIN)
    spec = np.fft.rfft(frames, n=N_FFT, axis=1)
    power = (spec.real ** 2 + spec.imag ** 2).astype(np.float32)  # (n_f, n_bins)
    mel = power @ _FB.T                                       # (n_f, N_MELS)
    out = np.log(np.maximum(mel, LOG_FLOOR)).astype(np.float32)

    if n_frames is None:
        return out
    if out.shape[0] >= n_frames:
        s = (out.shape[0] - n_frames) // 2
        return out[s:s + n_frames]
    pad = n_frames - out.shape[0]
    lo = pad // 2
    return np.pad(out, ((lo, pad - lo), (0, 0)),
                  constant_values=float(np.log(LOG_FLOOR)))


def normalize(m: np.ndarray) -> np.ndarray:
    """발화 단위 평균 차감. 마이크 감도와 거리 차이를 상당히 지운다.

    분산 정규화는 일부러 안 한다 — ESP32 에서 나눗셈이 늘고, 실측 이득이 작았다.
    """
    return (m - m.mean(axis=0, keepdims=True)).astype(np.float32)


def features(pcm: np.ndarray) -> np.ndarray:
    return normalize(logmel(pcm))


if __name__ == "__main__":
    fb = mel_filterbank()
    print(f"필터뱅크 {fb.shape}, 0 아닌 계수 {int((fb > 0).sum())}개")
    print(f"프레임당 {N_MELS} 빈, 창 {WIN} 홉 {HOP} → {N_FRAMES} 프레임 = "
          f"{N_FRAMES * HOP / SR:.2f}s")
    rng = np.random.default_rng(0)
    pcm = (rng.standard_normal(16000) * 3000).astype(np.int16)
    f = features(pcm)
    print(f"랜덤 1초 입력 → {f.shape}, mean {f.mean():.4f}, std {f.std():.4f}")
