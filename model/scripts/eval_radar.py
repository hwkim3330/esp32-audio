#!/usr/bin/env python3
"""레이더 감지기를 **빈 방에서** 채점한다. 사람 없이 검증할 수 있는 절반이 이것이다.

사람 감지는 사람이 지나가야 확인된다. 그런데 그 전에 확인해야 하는 것이 있다:
**빈 방에서 울리지 않는가.** 빈 방에서 울리는 감지기는 민감도와 무관하게 고장난 것이고,
그건 우리가 이미 가진 데이터로 지금 잴 수 있다(우리 CSI 녹음은 전부 빈 방이다).

펌웨어(`firmware/radar_display/radar_display.ino`)의 판정식을 그대로 옮겼다:

    채널마다 처음 WARMUP_N(40) 프레임으로 기준선(평균·표준편차)을 학습하고,
    이후 프레임마다  w_dev = mean_i |amp_i - mu_i| / sd_i   (sd_i > 0.5 인 것만)
    band_score = 채널별 w_dev 의 최댓값,  thresh(3.0) 이상이면 사건

BLE 기기 점수와 앵커 점수는 npz 에 없어서 제외했다 — 즉 여기서 나오는 오탐률은
**하한**이다. 실제 펌웨어는 세 신호의 최댓값을 쓰므로 오탐이 더 잦다.

이 스크립트가 답하는 것:
  1) 지금 임계값에서 빈 방 오탐률이 얼마인가
  2) 기준선을 한 번 배우고 고정하는 것이 시간이 갈수록 무너지는가(드리프트)
  3) 그것을 어떻게 고치면 오탐이 사라지는가
"""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np

WARMUP_N = 40          # 펌웨어와 같은 값
SD_FLOOR = 0.5         # 이보다 작은 표준편차의 서브캐리어는 무시한다(펌웨어와 같음)


def load(path: Path):
    d = np.load(path, allow_pickle=True)
    amp = np.asarray(d["amp"], dtype=np.float64)
    ms = np.asarray(d["ms"], dtype=np.float64)
    lab = np.asarray(d["label"]) if "label" in d else np.full(len(amp), 255, np.uint8)
    if len(set(lab.tolist())) <= 1:            # 채널 라벨이 없으면 한 채널로 본다
        lab = np.zeros(len(amp), np.uint8)
    o = np.argsort(ms)
    return ms[o], amp[o], lab[o]


def w_dev_stream(amp: np.ndarray, mode: str, half_life_s: float,
                 t: np.ndarray, warmup: int = WARMUP_N):
    """한 채널의 프레임별 w_dev. mode 로 기준선 갱신 방식을 바꾼다.

    fixed  : 처음 warmup 프레임으로 배우고 고정 (지금 펌웨어)
    ema    : 고정 기준선으로 시작해 사건이 아닐 때만 지수이동평균으로 따라간다
    robust : 중앙값·MAD 로 배운다(고정). 이상치 한 프레임에 기준선이 끌려가지 않는다
    """
    n, m = amp.shape
    if n <= warmup:
        return np.zeros(0), np.zeros(0)
    if mode == "robust":
        mu = np.median(amp[:warmup], axis=0)
        sd = 1.4826 * np.median(np.abs(amp[:warmup] - mu), axis=0)
    else:
        mu = amp[:warmup].mean(axis=0)
        sd = amp[:warmup].std(axis=0)
    use = sd > SD_FLOOR
    out = np.zeros(n - warmup)
    # EMA 계수는 시간 기준으로 준다 — CSI 는 표본 간격이 들쭉날쭉해서 프레임 기준으로
    # 두면 트래픽이 몰릴 때 기준선이 훅 따라가고 조용할 때는 굳는다.
    for k in range(warmup, n):
        if use.any():
            z = np.abs(amp[k, use] - mu[use]) / sd[use]
            out[k - warmup] = z.mean()
        if mode == "ema" and out[k - warmup] < 3.0:      # 사건 중에는 갱신하지 않는다
            dt = max(t[k] - t[k - 1], 0.0) / 1000.0
            a = 1.0 - 0.5 ** (dt / half_life_s) if half_life_s > 0 else 0.0
            mu = mu + a * (amp[k] - mu)
    return out, t[warmup:]


def count_events(t: np.ndarray, score: np.ndarray, thresh: float,
                 off_ratio: float = 0.70, min_dur_s: float = 2.0,
                 merge_gap_s: float = 5.0):
    """펌웨어의 사건 판정을 그대로 옮긴다.

    프레임 오탐률은 실제로 중요한 수가 아니다. 펌웨어는 임계값 하나로 켜고 끄지 않는다 —
    켜짐은 thresh, 꺼짐은 그 70%(히스테리시스), 2초 미만은 잡음으로 버리고, 5초 안에
    다시 오르면 같은 사건으로 이어붙인다. 그래서 화면에 남는 것은 **사건**이고,
    빈 방에서 의미 있는 숫자는 **시간당 오경보 건수**다.
    """
    on = False
    t_on = 0.0
    ev: list[tuple[float, float]] = []
    for i in range(len(score)):
        if not on and score[i] >= thresh:
            on, t_on = True, t[i]
        elif on and score[i] < thresh * off_ratio:
            on = False
            if t[i] - t_on >= min_dur_s:
                ev.append((t_on, t[i]))
    if on and t[-1] - t_on >= min_dur_s:
        ev.append((t_on, t[-1]))
    merged: list[list[float]] = []
    for a, b in ev:
        if merged and a - merged[-1][1] <= merge_gap_s:
            merged[-1][1] = b
        else:
            merged.append([a, b])
    return merged


def run(path: Path, thresh: float, mode: str, half_life_s: float):
    ms, amp, lab = load(path)
    chans = sorted(set(lab.tolist()))
    # 채널별 스트림을 시간순으로 합쳐 "마지막으로 안 값의 최댓값" 을 만든다.
    # 펌웨어의 band_score 가 그렇게 동작한다(채널을 순환하며 각 채널의 최신 w_dev 를 남긴다).
    ev = []
    for c in chans:
        m = lab == c
        d, tt = w_dev_stream(amp[m], mode, half_life_s, ms[m])
        for v, tv in zip(d, tt):
            ev.append((tv, c, v))
    if not ev:
        return None
    ev.sort()
    last = {c: 0.0 for c in chans}
    score = np.empty(len(ev))
    for i, (tv, c, v) in enumerate(ev):
        last[c] = v
        score[i] = max(last.values())
    tarr = np.array([e[0] for e in ev]) / 1000.0
    tarr -= tarr[0]
    fp = float((score >= thresh).mean())
    ev = count_events(tarr, score, thresh)
    hours = max(tarr[-1], 1e-9) / 3600.0
    # 드리프트: 앞 20% 와 뒤 20% 의 중위 점수 비교. 기준선이 굳어 있으면 뒤가 올라간다.
    k = max(1, len(score) // 5)
    return {"file": path.name, "n": len(score), "span_s": float(tarr[-1]),
            "fp_rate": fp, "n_events": len(ev), "events_per_hour": len(ev) / hours,
            "event_secs": float(sum(b - a for a, b in ev)), "median": float(np.median(score)),
            "p95": float(np.percentile(score, 95)), "max": float(score.max()),
            "first20_med": float(np.median(score[:k])),
            "last20_med": float(np.median(score[-k:])),
            "n_chan": len(chans)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csi", nargs="*", default=sorted(glob.glob("model/data/csi/*.npz")))
    ap.add_argument("--thresh", type=float, default=3.0, help="펌웨어 기본값")
    ap.add_argument("--modes", nargs="+", default=["fixed", "robust", "ema"])
    ap.add_argument("--half-life", type=float, default=60.0,
                    help="ema 모드의 기준선 반감기(초)")
    ap.add_argument("--out", default="model/out/radar_empty_room.json")
    args = ap.parse_args()

    print(f"빈 방 오탐 채점 — 임계값 {args.thresh} (펌웨어 기본), WARMUP_N={WARMUP_N}")
    print("BLE·앵커 점수는 npz 에 없어 제외했다 → 여기 숫자는 오탐률의 하한이다\n")
    res = {}
    for mode in args.modes:
        print(f"[{mode}]")
        rows = []
        for p in args.csi:
            r = run(Path(p), args.thresh, mode, args.half_life)
            if not r:
                continue
            rows.append(r)
            print(f"  {r['file']:22s} {r['span_s']:6.0f}s ch{r['n_chan']}  "
                  f"프레임오탐 {r['fp_rate']*100:5.1f}%  "
                  f"**오경보 {r['n_events']:2d}건** ({r['events_per_hour']:5.1f}/시간, "
                  f"{r['event_secs']:4.0f}초)  중위 {r['median']:4.2f} 최대 {r['max']:5.2f}")
        if rows:
            w = np.array([r["n"] for r in rows], dtype=float)
            fp = np.array([r["fp_rate"] for r in rows])
            tot_s = sum(r["span_s"] for r in rows)
            tot_e = sum(r["n_events"] for r in rows)
            print(f"  {'합계':22s} {tot_s:6.0f}s        프레임오탐 "
                  f"{100*np.average(fp, weights=w):5.1f}%  "
                  f"**오경보 {tot_e}건 = {tot_e/(tot_s/3600):.1f}/시간**\n")
        res[mode] = rows
    Path(args.out).write_text(json.dumps(res, ensure_ascii=False, indent=1),
                             encoding="utf-8")
    print(f"→ {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
