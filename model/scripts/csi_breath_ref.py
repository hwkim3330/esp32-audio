#!/usr/bin/env python3
"""호흡 탐침의 참조 구현. **C 이식은 하지 않았다 — 전제가 철회됐다.**

이 파일이 겨누던 신호는 리샘플 앨리어싱이었다. 10Hz 리샘플에서만 나오던 14.0bpm 은
9.767Hz 시스템 주기가 |9.767 − 10| = 0.2333Hz 로 접힌 것이고, 원 표본율에서는 없다.
상세와 경과는 `model/CSI_PIPELINE.md` 의 "호흡 주기성 — 시도했고 철회했다".

그래도 남겨 둔다. 여기서 확정한 알고리즘 판단들은 나중에 **사람 라벨이 붙은 실측**이
생기면 그대로 쓸 수 있고, 셋 다 실측으로 얻은 것이다:
  - 선형 추세 제거를 빼면 무너진다(실제 6.53 대 백색잡음 널 8.51 로 역전). 드리프트가
    저주파 칸을 부풀리면 대역 중앙값이 올라가고, 뾰족함은 봉우리/중앙값이라 분모가 커진다.
  - 분산 상위 K개만 FFT 하는 최적화는 틀렸다. 전체 분산은 드리프트가 지배하므로 호흡이
    실린 서브캐리어와 정반대를 고른다.
  - 고르는 양과 재는 양을 분리해야 한다. 뾰족함으로 고르고 뾰족함으로 판정하면 널도
    64개 중 최댓값을 받아 같이 올라간다(11.4 대 13.3 역전).

`csi_rate_probe.py` 는 "방법에 신호가 있나" 를 묻는 탐색용이었고, 이 파일은 보드에
올릴 계산을 한 줄씩 확정한 것이다. 그래서 두 가지가 다르다:

  - 보드가 할 수 있는 것만 한다. scipy 없음, 다항식 적합 없음, 서브캐리어 전수 FFT 없음.
  - 데시메이션까지 포함한다. 보드는 CSI 를 오는 대로 받아 125ms 칸에 평균해 넣는다.

계산 순서 (보드와 동일):
  1) 진폭을 8Hz 칸에 평균 (칸이 비면 앞 값 유지 — 보드도 같다)
  2) 분산 상위 K개 서브캐리어만 고른다 (전수 FFT 는 낭비다)
  3) 각각 평균 제거 → 해닝 창 → 512점 실수 주기도
  4) 호흡대역(0.1~0.5Hz) 봉우리/중앙값 = 뾰족함, 봉우리 위치 = bpm
  5) 대역 최저 칸에 걸린 것은 버린다 (잘리지 않은 드리프트다)
  6) **대역 대비**가 가장 큰 서브캐리어를 고르고, 그것의 뾰족함으로 판정한다
     (고르는 양과 재는 양이 같으면 널이 같이 올라간다 — analyse 주석)

왜 자기상관이 아닌가: 대역통과된 잡음은 협대역 신호라서 자기상관에 원래 봉우리가
있다 — 대역을 자르는 행위가 재려는 주기성을 만들어낸다. 대역 **안에서의** 집중도만이
그 함정을 피한다. 상세는 `model/CSI_PIPELINE.md`.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

BR_FS = 8.0                  # Hz. 0.5Hz 대역에 Nyquist 여유가 충분하다
BR_SEC = 60                  # 초. 30초가 하한, 45초면 확실, 60초면 넉넉
BR_N = int(BR_FS * BR_SEC)   # 480 표본
BR_NFFT = 512                # 480 → 512 로 0 채움
BR_TOPK = 0                  # 0 = 전수. 분산 상위 K개 최적화는 폐기했다(아래 analyse 주석)
BREATH_LO, BREATH_HI = 0.10, 0.50
CTRL_LO, CTRL_HI = 0.60, 2.00


def decimate_bins(t: np.ndarray, amp: np.ndarray, fs: float = BR_FS):
    """불균일 CSI 를 고정 칸에 평균해 넣는다. 보드의 `br_push` 와 같은 규칙.

    빈 칸은 앞 값을 유지한다. 0 으로 두면 없던 급변이 생겨 스펙트럼에 광대역 잡음이
    깔린다 — 보간이 아니라 유지인 이유는 보드가 미래 표본을 모르기 때문이다.
    """
    order = np.argsort(t)
    t, amp = t[order], amp[order]
    step = 1.0 / fs
    n_bin = int((t[-1] - t[0]) / step) + 1
    out = np.zeros((n_bin, amp.shape[1]))
    acc = np.zeros(amp.shape[1])
    cnt = 0
    b_prev = 0
    filled = np.zeros(n_bin, dtype=bool)
    for i in range(len(t)):
        b = int((t[i] - t[0]) / step)
        if b != b_prev:
            if cnt:
                out[b_prev] = acc / cnt
                filled[b_prev] = True
            acc[:] = 0.0
            cnt = 0
            b_prev = b
        acc += amp[i]
        cnt += 1
    if cnt:
        out[b_prev] = acc / cnt
        filled[b_prev] = True
    # 유지(hold): 빈 칸은 직전 값
    last = None
    for b in range(n_bin):
        if filled[b]:
            last = out[b].copy()
        elif last is not None:
            out[b] = last
    return out, filled


def detrend_linear(x: np.ndarray):
    """최소제곱 직선을 뺀다. 닫힌 식이라 보드에서도 누적 두 번이면 된다.

    이 단계를 빼면 전부 무너진다(실측: 실제 6.53 대 백색잡음 널 8.51 — 역전). 드리프트가
    저주파 칸을 부풀리면 호흡 대역의 **중앙값**이 같이 올라가고, 뾰족함은 봉우리/중앙값
    이므로 분모가 커져 신호가 지워진다. 평균만 빼는 것으로는 부족하다.
    """
    n = len(x)
    i = np.arange(n, dtype=np.float64)
    sx = i.sum(); sy = float(x.sum())
    sxx = float((i * i).sum()); sxy = float((i * x).sum())
    den = n * sxx - sx * sx
    if den == 0:
        return x - x.mean()
    b = (n * sxy - sx * sy) / den
    a = (sy - b * sx) / n
    return x - (a + b * i)


def periodogram_sharp(x: np.ndarray, fs: float = BR_FS):
    """추세 제거 → 해닝 → 512점 주기도 → 대역 봉우리/중앙값.

    반환: (bpm, 뾰족함, 대비, 최저칸에 걸렸나)
    """
    n = len(x)
    x = detrend_linear(np.asarray(x, dtype=np.float64))
    w = 0.5 - 0.5 * np.cos(2.0 * np.pi * np.arange(n) / (n - 1))   # 해닝
    buf = np.zeros(BR_NFFT)
    buf[:n] = x * w
    sp = np.abs(np.fft.rfft(buf)) ** 2
    fr = np.fft.rfftfreq(BR_NFFT, 1.0 / fs)
    mb = (fr >= BREATH_LO) & (fr <= BREATH_HI)
    mc = (fr >= CTRL_LO) & (fr <= CTRL_HI)
    band = sp[mb]
    if not len(band):
        return 0.0, 0.0, 0.0, True
    i = int(np.argmax(band))
    med = float(np.median(band))
    sharp = float(band[i] / med) if med > 0 else 0.0
    ec = float(sp[mc].sum())
    contrast = float(band.sum() / ec) if ec > 0 else 0.0
    f0 = float(fr[mb][i])
    # 대역 최저 칸에 걸리면 호흡이 아니라 잘리지 않은 드리프트다.
    edge = (i == 0)
    return f0 * 60.0, sharp, contrast, edge


def analyse(win: np.ndarray, topk: int = 0):
    """창 하나 → 최선 서브캐리어의 호흡 추정.

    **고르는 양과 재는 양을 분리한다.** 서브캐리어를 뾰족함으로 고르고 그 뾰족함을
    판정에 쓰면 64개 중 최댓값을 고르는 셈이라 널도 자기 최고치를 받는다 — 실측으로
    실제 11.4 대 백색잡음 널 13.3 으로 역전됐다. 선택 편향이고, 앞서 절제 실험을
    실제 데이터에만 돌려서 놓쳤다.
    그래서 선택은 **대역 대비**(호흡대역/대조대역 에너지비)로 하고, 판정은 뾰족함으로
    한다. 대비는 신호대잡음 성격이라 뾰족함과 거의 독립이다.

    서브캐리어는 전수로 본다. 분산 상위 K개만 고르는 최적화도 폐기했다 — 전체 분산은
    드리프트가 지배하므로 호흡이 실린 것과 정반대를 고른다(실측 6.53 대 널 8.51).
    64개 전수 FFT 는 512점 × 64 ≈ 0.9M MAC 이고 음성 인코더 1회가 8.14M 이다.
    """
    var = win.var(axis=0)
    cand = range(win.shape[1]) if topk <= 0 else np.argsort(var)[::-1][:topk]
    best = None
    for c in cand:
        if var[c] <= 0:
            continue
        bpm, sharp, contrast, edge = periodogram_sharp(win[:, c])
        if edge or bpm <= 0:
            continue
        if best is None or contrast > best["contrast"]:
            best = {"bpm": bpm, "sharp": sharp, "contrast": contrast, "sc": int(c)}
    return best


def synth(n: int, n_ch: int, kind: str, seed: int = 0):
    rng = np.random.default_rng(seed)
    if kind == "white":
        return rng.standard_normal((n, n_ch)) * 10.0 + 100.0
    return (np.cumsum(rng.standard_normal((n, n_ch)), axis=0) * 0.5
            + rng.standard_normal((n, n_ch)) + 100.0)


def sweep(name: str, dec: np.ndarray, hop: int):
    out = []
    for s in range(0, len(dec) - BR_N + 1, hop):
        r = analyse(dec[s:s + BR_N])
        if r:
            out.append(r)
    if not out:
        print(f"{name:44s} 유효 창 없음")
        return None
    bpm = np.array([r["bpm"] for r in out])
    shp = np.array([r["sharp"] for r in out])
    med = float(np.median(bpm))
    agree = float(np.mean(np.abs(bpm - med) <= 2.0))
    print(f"{name:44s} 창 {len(out):3d}  뾰족함 중위 {np.median(shp):7.2f}  "
          f"5%={np.percentile(shp,5):6.2f}  bpm {med:5.1f}  일관성 {agree*100:3.0f}%  "
          f"12~20bpm {100*np.mean((bpm>=12)&(bpm<=20)):3.0f}%")
    return {"sharp_med": float(np.median(shp)),
            "sharp_p5": float(np.percentile(shp, 5)),
            "sharp_p95": float(np.percentile(shp, 95)),
            "bpm_med": med, "agree": agree, "n_win": len(out)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ruview", nargs="*", default=[])
    ap.add_argument("--npz", nargs="*", default=[])
    ap.add_argument("--max-frames", type=int, default=60000)
    ap.add_argument("--hop-s", type=float, default=15.0)
    ap.add_argument("--dump-vector", default="",
                    help="C 패리티용 테스트 벡터 경로(.json)")
    ap.add_argument("--out", default="model/out/csi_breath_ref.json")
    args = ap.parse_args()

    import csi_rate_probe as P

    print(f"보드 설정 그대로: {BR_FS:g}Hz × {BR_SEC}s = {BR_N}표본, "
          f"{BR_NFFT}점 FFT, 서브캐리어 {'전수' if BR_TOPK <= 0 else BR_TOPK}\n")
    hop = int(args.hop_s * BR_FS)
    res = {}
    vec_src = None

    for p in args.npz:
        t, amp, grp = P.load_npz(Path(p))
        dec, _ = decimate_bins(t, amp)
        r = sweep(f"{Path(p).name} (우리, 빈 방)", dec, hop)
        if r:
            res[Path(p).name] = r

    for p in args.ruview:
        ts, amp, nodes = P.load_ruview_jsonl(Path(p), max_frames=args.max_frames)
        for nd in sorted(set(nodes.tolist())):
            m = nodes == nd
            dec, _ = decimate_bins(ts[m], amp[m])
            r = sweep(f"{Path(p).name} node{nd} (사람 있음)", dec, hop)
            if r:
                res[f"{Path(p).name}#node{nd}"] = r
            if vec_src is None and len(dec) >= BR_N:
                vec_src = dec

    # 널: 창 길이·서브캐리어 수를 실제와 같게 맞춘다
    n_ch = vec_src.shape[1] if vec_src is not None else 64
    n_len = max(len(vec_src) if vec_src is not None else 0, BR_N * 6)
    for kind in ("white", "walk"):
        r = sweep(f"합성 널 ({kind})", synth(n_len, n_ch, kind), hop)
        if r:
            res[f"null_{kind}"] = r

    if args.dump_vector and vec_src is not None:
        # 첫 창 하나와 그 답을 그대로 남긴다. C 가 같은 값을 내야 한다.
        win = vec_src[:BR_N]
        r = analyse(win)
        sc = r["sc"]
        Path(args.dump_vector).write_text(json.dumps({
            "n": BR_N, "nfft": BR_NFFT, "fs": BR_FS, "topk": BR_TOPK,
            "n_sc": int(win.shape[1]),
            "expect": r,
            "window_sc": [float(v) for v in win[:, sc]],
            "window_all": [[float(v) for v in row] for row in win],
        }, ensure_ascii=False), encoding="utf-8")
        print(f"\n패리티 벡터 → {args.dump_vector}  (기대: sc{sc} "
              f"{r['bpm']:.2f}bpm 뾰족함 {r['sharp']:.4f})")

    Path(args.out).write_text(json.dumps(res, ensure_ascii=False, indent=1),
                              encoding="utf-8")
    print(f"→ {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
