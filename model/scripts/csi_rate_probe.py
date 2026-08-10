#!/usr/bin/env python3
"""CSI 진폭에 호흡 대역 주기성이 실제로 있는지 오프라인으로 확인한다.

**반드시 `--fs-sweep` 으로 확인하고 나서 믿을 것.** 불균일 CSI 를 균일 격자로 옮기는
순간 앨리어싱이 생기고, 그것이 관심 대역 안에 떨어질 수 있다. 실제로 그렇게 속았다:
RuView 밤샘 녹음에서 10Hz 리샘플만 14.0bpm·일관성 100%·뾰족함 31.6 을 냈고, 원
표본율(19.5Hz)이나 다른 어떤 표본율에서도 없었다. 원인은 9.767Hz 시스템 주기가
|9.767 − 10| = 0.2333Hz = 14.0bpm 으로 접힌 것이었다. 상세는 model/CSI_PIPELINE.md.

왜 필요한가. 지금 우리 감지기는 "분산 비슷한 대역 점수 > 임계값" 하나다. 그런데
전파 환경은 사람 없이도 드리프트하고, 드리프트는 분산을 올린다 — 그래서 빈 방
기준선만으로는 오탐과 감지를 가를 수 없다. **주기성은 다르다.** 드리프트는
0.2~0.3Hz 의 안정된 주기를 만들지 않는다. 그래서 주기성이 잡히면 그건 훨씬 강한
증거이고, 안 잡히면 그 방향에 시간을 쓸 이유가 없다.

이 스크립트는 그 판단을 **보드에 굽기 전에** 내리기 위한 것이다.

내부 대조군을 반드시 같이 낸다:
  - 대역 대비: 호흡 대역(0.1~0.5Hz) 대 그 위 대역(0.6~2.0Hz) 에너지 비.
    같은 신호 안에서 비교하므로 기기·방·게인 차이가 상쇄된다.
  - 합성 널 두 개: 평탄한 백색잡음과 랜덤워크(드리프트). 널이 하나면 해석이 안 된다.
  - bpm 일관성: 창마다 지배 주기가 같은 곳에 오는가. **이게 진짜 판별자다.**
    드리프트는 대비를 올리지만 일정한 주기를 만들지 않는다.

교차 파일 비교(우리 빈 방 대 남의 밤샘 녹음)는 방·AP·보드·표본율이 전부 달라서
그 자체로는 증거가 아니다. 그래서 파일 사이 차이가 아니라 **각 파일 안에서의
대비와 합성 대조군과의 차이**를 본다.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

BREATH_LO, BREATH_HI = 0.10, 0.50      # Hz  → 6~30 bpm
CTRL_LO, CTRL_HI = 0.60, 2.00          # Hz  → 대조 대역(심박대는 여기 들어간다)


# ─────────────────────────────────────────── 로더

def load_npz(path: Path):
    """우리 csi_logger 가 남긴 npz. amp(N,64) float32 + ms(N) uint32."""
    d = np.load(path, allow_pickle=True)
    amp = np.asarray(d["amp"], dtype=np.float64)
    t = np.asarray(d["ms"], dtype=np.float64) / 1000.0
    grp = None
    if "label" in d:
        lab = np.asarray(d["label"])
        if len(set(lab.tolist())) > 1:
            grp = lab                  # 채널 순환 데이터 — 구간을 갈라야 한다
    return t, amp, grp


def load_ruview_jsonl(path: Path, node: int | None = None, max_frames: int = 0,
                      want_sc: int = 64):
    """RuView 녹음. iq_hex 는 서브캐리어당 (I,Q) int8 쌍이다(64 → 128 바이트).

    서브캐리어 개수는 고정이 아니다 — AP 가 HT40 으로 올라가면 128/192 가 섞인다.
    우리 레포가 실기에서 같은 함정을 밟았다(radar_display/README.md). 조용히 버리면
    "왜 프레임이 안 들어오나" 를 오해하므로 세어서 알린다.
    """
    ts, amps, nodes = [], [], []
    other = {}
    with open(path) as f:
        for line in f:
            try:
                d = json.loads(line)
            except Exception:
                continue
            if d.get("type") != "raw_csi" or "iq_hex" not in d:
                continue
            if node is not None and d.get("node_id") != node:
                continue
            raw = np.frombuffer(bytes.fromhex(d["iq_hex"]), dtype=np.int8)
            n_sc = len(raw) // 2
            if n_sc != want_sc:
                other[n_sc] = other.get(n_sc, 0) + 1
                continue
            iq = raw[: 2 * n_sc].astype(np.float64).reshape(n_sc, 2)
            amps.append(np.hypot(iq[:, 0], iq[:, 1]))
            ts.append(float(d["timestamp"]))
            nodes.append(d.get("node_id"))
            if max_frames and len(ts) >= max_frames:
                break
    if not ts:
        raise SystemExit(f"{path}: sc{want_sc} raw_csi 프레임이 없다 (본 것: {other})")
    if other:
        print(f"  [{path.name}] sc{want_sc} {len(ts)}프레임 사용, "
              f"다른 폭 건너뜀: {other}")
    return (np.asarray(ts), np.asarray(amps),
            np.asarray(nodes) if node is None else None)


# ─────────────────────────────────────────── 신호처리

def resample_uniform(t: np.ndarray, x: np.ndarray, fs: float):
    """불균일 표본을 고정 격자로 옮긴다.

    CSI 는 트래픽이 올 때만 생기므로 간격이 들쭉날쭉하다(실측 1~307ms). 이걸 그냥
    FFT 에 넣으면 주파수 축이 의미를 잃는다 — 그래서 창을 시간으로 자르고
    리샘플하는 것이 우리 파이프라인의 규칙이기도 하다(CSI_PIPELINE.md).
    """
    order = np.argsort(t)
    t, x = t[order], x[order]
    grid = np.arange(t[0], t[-1], 1.0 / fs)
    out = np.empty((len(grid), x.shape[1]))
    for c in range(x.shape[1]):
        out[:, c] = np.interp(grid, t, x[:, c])
    return grid, out


def band_energy(x: np.ndarray, fs: float, lo: float, hi: float):
    """주기도(periodogram)에서 대역 에너지. 창은 해닝, 평균 제거."""
    x = x - x.mean()
    if not np.any(x):
        return 0.0, np.array([]), np.array([])
    w = np.hanning(len(x))
    sp = np.abs(np.fft.rfft(x * w)) ** 2
    fr = np.fft.rfftfreq(len(x), 1.0 / fs)
    m = (fr >= lo) & (fr <= hi)
    return float(sp[m].sum()), fr, sp


def dominant_rate(x: np.ndarray, fs: float, lo: float, hi: float):
    """대역 안 최대 봉우리의 주파수와 '뾰족함'.

    뾰족함 = 봉우리 / 그 대역 중앙값. 값이 클수록 넓게 퍼진 잡음이 아니라 한 주기가
    지배한다는 뜻이다. 드리프트는 대역을 고르게 채우므로 이 값이 1 근처에 머문다.
    """
    e, fr, sp = band_energy(x, fs, lo, hi)
    if not len(fr):
        return 0.0, 0.0
    m = (fr >= lo) & (fr <= hi)
    if not m.any():
        return 0.0, 0.0
    band = sp[m]
    i = int(np.argmax(band))
    med = float(np.median(band))
    sharp = float(band[i] / med) if med > 0 else 0.0
    return float(fr[m][i]), sharp


def analyse_window(seg: np.ndarray, fs: float):
    """한 창의 최선 서브캐리어를 골라 호흡 대역 지표를 낸다.

    서브캐리어를 평균하면 안 된다 — 호흡 변조는 일부 서브캐리어에만 강하게 나타나고,
    평균은 그것을 나머지 63개로 희석한다. 대비가 가장 큰 하나를 고른다.
    """
    best = None
    for c in range(seg.shape[1]):
        x = seg[:, c]
        if np.allclose(x, x[0]):
            continue
        x = x - np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x)))
        eb, _, _ = band_energy(x, fs, BREATH_LO, BREATH_HI)
        ec, _, _ = band_energy(x, fs, CTRL_LO, CTRL_HI)
        if ec <= 0:
            continue
        contrast = eb / ec
        if best is None or contrast > best[0]:
            f0, sharp = dominant_rate(x, fs, BREATH_LO, BREATH_HI)
            best = (contrast, f0 * 60.0, sharp, c)
    return best



# ─────────────────────────────────────────── 자기상관 (펌웨어가 실제로 쓸 방식)

def biquad_bandpass_sos(fs: float, lo: float, hi: float):
    """2차 버터워스 대역통과. ESP32 에서는 biquad 두 단이면 끝난다.

    이 단계를 빼면 자기상관이 통째로 실패한다(실측: 실제 데이터 0.25 대 백색잡음 널
    0.24 — 분리 없음). 선형 추세 제거만으로는 0.1Hz **이하** 드리프트가 남고, 그
    드리프트가 자기상관 전 구간을 지배해 호흡 주기를 덮는다. RuView 가 주기 탐색
    앞에 biquad 를 둔 이유가 이것이다 — 값싸 보여서 건너뛰었다가 되돌아왔다.
    """
    from scipy.signal import butter
    return butter(2, [lo, hi], btype="band", fs=fs, output="sos")


def acf_rate(x: np.ndarray, fs: float, sos, lo_bpm: float = 6.0, hi_bpm: float = 30.0):
    """대역통과 후 자기상관으로 지배 주기를 찾는다. 보드에서 돌릴 것과 같은 계산이다.

    FFT 대신 이걸 쓰는 이유: 360표본에 지연 범위 하나면 백만 MAC 대이고, FFT 버퍼도
    트위들 인자도 필요 없다. 우리 cn_infer.c 의 FFT 는 CSI 빌드에서 아예 컴파일되지
    않으므로(CN_N_FFT 미정의) 새로 들여올 이유가 없다.

    뾰족함 = 정규화 자기상관의 봉우리 값(0~1). 백색잡음은 1/sqrt(N) 근처에 머문다.
    """
    from scipy.signal import sosfilt
    n = len(x)
    x = np.asarray(x, dtype=np.float64) - float(np.mean(x))
    x = sosfilt(sos, x)
    # 필터 과도응답을 버린다. 안 버리면 앞머리 급변이 봉우리를 만든다.
    skip = min(n // 4, int(fs * 5))
    x = x[skip:]
    n = len(x)
    e0 = float(np.dot(x, x))
    if n < 32 or e0 <= 0:
        return 0.0, 0.0
    lag_lo = int(round(fs * 60.0 / hi_bpm))     # 30bpm → 2초
    lag_hi = int(round(fs * 60.0 / lo_bpm))     # 6bpm  → 10초
    lag_hi = min(lag_hi, n // 2)
    if lag_hi <= lag_lo + 2:
        return 0.0, 0.0
    r = np.empty(lag_hi - lag_lo + 1)
    for k in range(lag_lo, lag_hi + 1):
        r[k - lag_lo] = float(np.dot(x[: n - k], x[k:])) / e0
    i = int(np.argmax(r))
    peak = float(r[i])
    # 경계에서 잡힌 것은 봉우리가 아니라 단조 감소의 끝일 수 있다. 국소 최대만 받는다.
    if i == 0 or i == len(r) - 1:
        return 0.0, 0.0
    if not (r[i] > r[i - 1] and r[i] > r[i + 1]):
        return 0.0, 0.0
    return 60.0 * fs / (lag_lo + i), peak


_SOS_CACHE = {}


def analyse_window_acf(seg: np.ndarray, fs: float):
    """서브캐리어마다 자기상관을 돌려 가장 뾰족한 것을 고른다."""
    key = (fs, BREATH_LO, BREATH_HI)
    if key not in _SOS_CACHE:
        _SOS_CACHE[key] = biquad_bandpass_sos(fs, BREATH_LO, BREATH_HI)
    sos = _SOS_CACHE[key]
    best = None
    for c in range(seg.shape[1]):
        v = seg[:, c]
        if np.allclose(v, v[0]):
            continue
        bpm, sharp = acf_rate(v, fs, sos)
        if bpm <= 0:
            continue
        if best is None or sharp > best[2]:
            best = (sharp, bpm, sharp, c)
    return best


def synth_control(n: int, n_ch: int, fs: float, seed: int, kind: str):
    """합성 대조군 두 종류.

    널이 하나면 해석이 안 된다. 랜덤워크는 1/f^2 스펙트럼이라 저주파에 에너지가
    쏠리고, 그래서 **대역대비는 실제 데이터보다도 높게 나온다** — 즉 대역대비만
    보면 드리프트를 호흡으로 오독한다. 평탄한 백색잡음을 같이 둬서 두 널 사이
    어디에 있는지를 본다. 진짜 호흡의 표지는 대비가 아니라 **일정한 bpm 에
    반복되는 뾰족한 봉우리**다.
    """
    rng = np.random.default_rng(seed)
    if kind == "white":
        return rng.standard_normal((n, n_ch))
    return np.cumsum(rng.standard_normal((n, n_ch)), axis=0) * 0.5 \
        + rng.standard_normal((n, n_ch))


def run(name: str, t: np.ndarray, amp: np.ndarray, fs: float,
        win_s: float, hop_s: float, seed: int = 0, method: str = "fft"):
    win_fn = analyse_window_acf if method == "acf" else analyse_window
    grid, x = resample_uniform(t, amp, fs)
    nwin = int(win_s * fs)
    nhop = int(hop_s * fs)
    if len(grid) < nwin:
        print(f"{name:38s} 창({win_s:.0f}s)보다 짧다 — 건너뜀 ({len(grid)/fs:.0f}s)")
        return None

    def sweep(sig):
        out = []
        for st in range(0, len(sig) - nwin + 1, nhop):
            r = win_fn(sig[st:st + nwin], fs)
            if r:
                out.append(r)
        return out

    rows = sweep(x)
    if not rows:
        print(f"{name:38s} 유효 창이 없다")
        return None
    nulls = {k: sweep(synth_control(len(grid), x.shape[1], fs, seed, k))
             for k in ("white", "walk")}

    def stats(rs):
        if not rs:
            return None
        con = np.array([r[0] for r in rs])
        bpm = np.array([r[1] for r in rs])
        shp = np.array([r[2] for r in rs])
        # bpm 일관성: 창마다 지배 주기가 같은 곳에 오는가. 호흡이면 몰리고,
        # 드리프트면 흩어지거나 대역 최저 칸(6bpm)에 쌓인다.
        med = float(np.median(bpm))
        agree = float(np.mean(np.abs(bpm - med) <= 2.0))
        # 대역 최저 칸에 쌓이는 것은 호흡이 아니라 잘리지 않은 드리프트다.
        edge = float(np.mean(bpm <= BREATH_LO * 60.0 + 0.6))
        return {"contrast": float(np.median(con)), "sharp": float(np.median(shp)),
                "bpm": med, "agree": agree, "edge": edge, "n": len(rs),
                "in_human": float(np.mean((bpm >= 12) & (bpm <= 20)))}

    r = stats(rows)
    nw, nk = stats(nulls["white"]), stats(nulls["walk"])
    print(f"{name:38s} 창 {r['n']:3d}개  ({win_s:.0f}s/{hop_s:.0f}s hop, {fs:g}Hz)")
    print(f"{'':38s}   대역대비 {r['contrast']:7.2f}   "
          f"널: 백색 {nw['contrast']:.2f} / 랜덤워크 {nk['contrast']:.2f}")
    print(f"{'':38s}   뾰족함   {r['sharp']:7.2f}   "
          f"널: 백색 {nw['sharp']:.2f} / 랜덤워크 {nk['sharp']:.2f}")
    print(f"{'':38s}   지배 bpm {r['bpm']:7.1f}   일관성(±2bpm) {r['agree']*100:3.0f}%"
          f"   대역최저칸 {r['edge']*100:3.0f}%   12~20bpm {r['in_human']*100:3.0f}%")
    print(f"{'':38s}   널 일관성: 백색 {nw['agree']*100:.0f}% (최저칸 {nw['edge']*100:.0f}%)"
          f" / 랜덤워크 {nk['agree']*100:.0f}% (최저칸 {nk['edge']*100:.0f}%)")
    return {"real": r, "null_white": nw, "null_walk": nk}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", nargs="*", default=[])
    ap.add_argument("--ruview", nargs="*", default=[],
                    help="RuView *.csi.jsonl (노드별로 따로 본다)")
    ap.add_argument("--fs", type=float, default=10.0, help="리샘플 표본율(Hz)")
    ap.add_argument("--win", type=float, default=60.0, help="창 길이(초)")
    ap.add_argument("--hop", type=float, default=30.0)
    ap.add_argument("--max-frames", type=int, default=40000,
                    help="RuView 파일에서 읽을 최대 프레임(0=전부)")
    ap.add_argument("--fs-sweep", type=float, nargs="*", default=None,
                    help="여러 표본율로 같은 측정을 반복한다. 한 표본율에서만 나오는 "
                         "봉우리는 리샘플 앨리어싱이다 — 이 검사를 건너뛰지 말 것")
    ap.add_argument("--method", choices=["fft", "acf"], default="fft",
                    help="acf 는 펌웨어가 실제로 쓸 계산이다")
    ap.add_argument("--out", default="model/out/csi_rate_probe.json")
    args = ap.parse_args()

    fs_list = args.fs_sweep if args.fs_sweep else [args.fs]
    print(f"호흡 대역 {BREATH_LO}~{BREATH_HI}Hz ({BREATH_LO*60:.0f}~{BREATH_HI*60:.0f} bpm), "
          f"대조 대역 {CTRL_LO}~{CTRL_HI}Hz")
    if len(fs_list) > 1:
        print(f"표본율 스윕 {fs_list} — 한 표본율에서만 나오는 봉우리는 앨리어싱이다")
    print()
    res = {}
    for p in args.npz:
        t, amp, grp = load_npz(Path(p))
        base = Path(p).name
        if grp is None:
            for fsv in fs_list:
                r = run(f"{base} @{fsv:g}Hz (우리, 빈 방)", t, amp, fsv,
                        args.win, args.hop, method=args.method)
                if r:
                    res[f"{base}@{fsv:g}"] = r
            r = None
            if r:
                res[base] = r
        else:
            # 채널이 바뀌면 주파수 응답이 통째로 바뀐다. 이어 붙이면 그 계단이
            # 저주파 성분으로 들어와 호흡처럼 보인다. 라벨별로 갈라서 본다.
            for g in sorted(set(grp.tolist())):
                m = grp == g
                for fsv in fs_list:
                    r = run(f"{base} label={g} @{fsv:g}Hz", t[m], amp[m],
                            fsv, args.win, args.hop, method=args.method)
                    if r:
                        res[f"{base}#{g}@{fsv:g}"] = r
                r = None
                if r:
                    res[f"{base}#{g}"] = r
    for p in args.ruview:
        ts, amp, nodes = load_ruview_jsonl(Path(p), max_frames=args.max_frames)
        base = Path(p).name
        for nd in sorted(set(nodes.tolist())):
            m = nodes == nd
            for fsv in fs_list:
                r = run(f"{base} node{nd} @{fsv:g}Hz", ts[m], amp[m],
                        fsv, args.win, args.hop, method=args.method)
                if r:
                    res[f"{base}#node{nd}@{fsv:g}"] = r
            r = None
            if r:
                res[f"{base}#node{nd}"] = r

    Path(args.out).write_text(json.dumps(res, ensure_ascii=False, indent=1),
                              encoding="utf-8")
    print(f"\n→ {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
