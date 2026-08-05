#!/usr/bin/env python3
"""CSI 프런트엔드 — 학습(파이썬)과 추론(ESP32 C)이 같은 값을 내야 하는 지점.

음성의 features.py 와 같은 역할이다. CSI 창은 (시간 × 서브캐리어) 행렬이라
멜 스펙트로그램(192 × 40)과 모양이 같고, 그래서 인코더 구조를 그대로 쓴다.

CSI 에만 있는 문제 세 가지를 여기서 처리한다.

1. **불균일한 샘플링.** CSI 는 주변 트래픽이 올 때만 생긴다 — 실측에서 같은 방인데
   채널에 따라 4~268 Hz 로 흔들렸다. 그래서 창을 **프레임 수가 아니라 시간으로**
   자르고 고정 프레임 수로 리샘플한다. 프레임 수로 자르면 128프레임이 10Hz 에서는
   12.8초, 180Hz 에서는 0.7초가 되어 모델이 보는 시간 폭이 매번 달라진다.

2. **서브캐리어별 상수 오프셋.** 안테나·경로·하드웨어 이득 때문에 서브캐리어마다
   평균 진폭이 다르다. 음성의 발화 단위 평균 차감에 해당하는 것이 여기서는
   서브캐리어별 기준선 차감이다. 안 하면 모델이 "사람" 대신 "이 하드웨어의 주파수
   응답" 을 배운다.

3. **진폭 스케일.** 자동 스케일링(manu_scale=false)이라 절대 진폭에 의미가 적다.
   로그를 취해 곱셈 이득을 덧셈 오프셋으로 바꾸고, 그 오프셋을 2번이 지운다.
"""
from __future__ import annotations

import numpy as np

N_SC     = 64      # 실측 서브캐리어 수 (HT 패킷이 섞이면 128 도 온다)
WIN_SEC  = 2.0     # 창의 시간 폭. 사람이 지나가는 데 1~2초라 그에 맞춘다.
HOP_SEC  = 0.5     # 창 이동
N_FRAMES = 64      # 리샘플 후 프레임 수 → 모델 입력은 항상 (64, n_sc)
MIN_PKT  = 16      # 창 안에 이만큼도 안 오면 버린다 (트래픽이 끊긴 구간)
EPS      = 1e-3
CLIP     = 6.0     # 정규화 후 클리핑 범위 (죽은 서브캐리어의 로그 폭주 차단)


def amp_from_iq(iq: np.ndarray) -> np.ndarray:
    """(N, 2*n_sc) int8 → (N, n_sc) float32 진폭."""
    a = np.asarray(iq, dtype=np.float32).reshape(len(iq), -1, 2)
    return np.sqrt(a[:, :, 0] ** 2 + a[:, :, 1] ** 2)


def log_amp(amp: np.ndarray) -> np.ndarray:
    """로그 진폭. 곱셈 이득을 덧셈 오프셋으로 바꿔 기준선 차감이 지울 수 있게 한다."""
    return np.log(np.maximum(np.asarray(amp, np.float32), EPS))


def resample_time(w: np.ndarray, t: np.ndarray, n_out: int) -> np.ndarray:
    """불균일 시각 t 의 (T, F) 를 균일 n_out 프레임으로 선형 보간한다."""
    t = np.asarray(t, np.float64)
    grid = np.linspace(t[0], t[-1], n_out)
    out = np.empty((n_out, w.shape[1]), np.float32)
    for f in range(w.shape[1]):
        out[:, f] = np.interp(grid, t, w[:, f])
    return out


def normalize_window(w: np.ndarray) -> np.ndarray:
    """서브캐리어별 기준선 차감 + 전체 스케일 정규화.

    중앙값을 빼는 이유: 평균은 사람이 지나간 큰 변화에 끌려간다. 중앙값은 창의
    대부분을 차지하는 정적 상태를 잡으므로 기준선으로 더 안전하다.
    """
    w = np.asarray(w, np.float32)
    w = w - np.median(w, axis=0, keepdims=True)
    s = float(np.std(w))
    w = (w / s) if s > 1e-6 else w
    # 클리핑이 필요하다. first_word_invalid 로 진폭 0 인 서브캐리어에 로그를 씌우면
    # log(EPS) 가 되고, 정규화 후 -27 까지 튄다(실측). 죽은 빈 몇 개가 전체 분포를
    # 지배하면 학습이 그쪽에 끌려간다.
    return np.clip(w, -CLIP, CLIP)


def windows(iq: np.ndarray, ms: np.ndarray, label: np.ndarray | None = None,
            win_sec: float = WIN_SEC, hop_sec: float = HOP_SEC,
            n_frames: int = N_FRAMES, min_pkt: int = MIN_PKT,
            drop_unlabeled: bool = True):
    """시간 기준 창 자르기 + 리샘플.

    반환: (X, y, npkt) — X (M, n_frames, F), y (M,), npkt (M,) 창 안 실제 프레임 수
    라벨이 창 안에서 섞이면 버린다 — 경계에서 오염된 창이 정확도를 갉아먹는다.
    """
    la = log_amp(amp_from_iq(iq))
    t = np.asarray(ms, np.float64) / 1000.0
    if len(t) < min_pkt:
        return (np.zeros((0, n_frames, la.shape[1]), np.float32),
                np.zeros((0,), np.int64), np.zeros((0,), np.int32))

    Xs, ys, ns = [], [], []
    t0 = t[0]
    while t0 + win_sec <= t[-1]:
        m = (t >= t0) & (t < t0 + win_sec)
        k = int(m.sum())
        if k >= min_pkt:
            if label is not None:
                seg = np.unique(label[m])
                if len(seg) == 1 and not (drop_unlabeled and seg[0] == 255):
                    ys.append(int(seg[0]))
                else:
                    t0 += hop_sec
                    continue
            Xs.append(normalize_window(resample_time(la[m], t[m], n_frames)))
            ns.append(k)
        t0 += hop_sec

    if not Xs:
        return (np.zeros((0, n_frames, la.shape[1]), np.float32),
                np.zeros((0,), np.int64), np.zeros((0,), np.int32))
    return (np.stack(Xs).astype(np.float32),
            np.array(ys, np.int64) if label is not None else np.zeros(len(Xs), np.int64),
            np.array(ns, np.int32))


if __name__ == "__main__":
    import sys
    from pathlib import Path

    if len(sys.argv) < 2:
        print("사용법: csi_features.py <csi_*.npz> [...]")
        sys.exit(2)
    for p in sys.argv[1:]:
        d = np.load(p, allow_pickle=False)
        iq, ms = d["iq"], d["ms"]
        amp = amp_from_iq(iq)
        dur = (ms[-1] - ms[0]) / 1000.0
        print(f"{Path(p).name}")
        print(f"  프레임 {len(iq)}, 서브캐리어 {amp.shape[1]}, {dur:.1f}s, "
              f"{len(iq)/max(dur,1e-9):.1f} Hz, 진폭 {amp.mean():.1f}±{amp.std():.1f}")
        # 라벨 없이도 창이 나오는지 본다 (프런트엔드 자체 검증)
        X, y, n = windows(iq, ms, None)
        if len(X):
            print(f"  창 {len(X)}개 → {X.shape[1:]}, 창당 실측 프레임 "
                  f"{n.mean():.0f}±{n.std():.0f}")
            print(f"  정규화 후: 평균 {X.mean():+.4f} 표준편차 {X.std():.4f} "
                  f"범위 {X.min():+.2f}..{X.max():+.2f}")
        else:
            print(f"  창 0개 — {WIN_SEC}s 안에 {MIN_PKT}개 프레임이 안 온다")
