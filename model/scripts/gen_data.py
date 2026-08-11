#!/usr/bin/env python3
"""Supertonic 으로 캐빈 노드 학습 데이터를 합성한다.

학습 데이터를 사람이 녹음할 필요가 없다는 게 이 파이프라인의 핵심이다.
같은 문장을 10개 보이스 × 여러 속도로 합성하면, 인코더가 "말한 사람"이 아니라
"말한 내용"으로 뭉치도록 배울 재료가 나온다.

거부(OOD) 클래스가 없으면 모델이 무슨 소리를 들어도 가장 가까운 명령을 고른다.
그래서 명령과 무관한 문장도 같이 합성한다 — 이게 실사용 오작동을 막는다.

출력: 16kHz mono int16 WAV (ESP32 의 마이크 샘플레이트와 맞춘다)
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

SUPERTONIC_SR = 44100
TARGET_SR = 16000
VOICES = ["F1", "F2", "F3", "F4", "F5", "M1", "M2", "M3", "M4", "M5"]

# 명령과 무관한 발화는 ood_texts.json 에 있다. 30문장으로는 거부가 안 됐다 —
# 오수락 52.8% 의 원인이 데이터 부족이었고, 특히 명령과 어휘를 공유하는
# "하드 네거티브" 가 아예 없었다. 목록이 길어져서 파일로 뺐다.
OOD_TEXTS_JSON = "model/ood_texts.json"


def load_ood_groups(path: str | Path) -> dict[str, list[str]]:
    spec = json.loads(Path(path).read_text(encoding="utf-8"))
    return {g: list(v) for g, v in spec["groups"].items()}


def clip_name(text: str, voice: str, speed: float) -> str:
    """파일명. hash() 는 프로세스마다 값이 달라져 재실행이 재현되지 않는다."""
    d = hashlib.sha1(text.encode("utf-8")).hexdigest()[:8]
    return f"{voice}_sp{int(speed * 100)}_{d}.wav"


def to_16k(a: np.ndarray) -> np.ndarray:
    """44.1kHz float → 16kHz int16. scipy 가 있으면 다상 필터, 없으면 선형 보간."""
    a = np.asarray(a, dtype=np.float32).squeeze()
    try:
        from scipy.signal import resample_poly

        # 44100 / 16000 = 441/160
        y = resample_poly(a, 160, 441).astype(np.float32)
    except ImportError:
        n = int(round(len(a) * TARGET_SR / SUPERTONIC_SR))
        y = np.interp(
            np.linspace(0, len(a) - 1, n, dtype=np.float64),
            np.arange(len(a)),
            a,
        ).astype(np.float32)
    peak = float(np.abs(y).max())
    if peak > 0:
        y = y / peak * 0.95  # 클리핑 없이 정규화. 음량 차이는 학습 증강에서 다시 만든다.
    return (y * 32767.0).astype(np.int16)


def write_wav(path: Path, pcm: np.ndarray, sr: int = TARGET_SR) -> None:
    import wave

    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(pcm.tobytes())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--commands", default="model/commands.json")
    ap.add_argument("--assets", required=True, help="supertonic-3 assets 디렉터리")
    ap.add_argument("--out", default="model/data/utterances")
    ap.add_argument("--steps", type=int, default=4,
                    help="확산 스텝. 4면 품질/속도 균형이 좋다 (RTF 약 0.18)")
    ap.add_argument("--speeds", type=float, nargs="+", default=[0.95, 1.05, 1.15],
                    help="합성 속도 배수. 발화 속도 다양성을 여기서 만든다")
    ap.add_argument("--voices", nargs="+", default=VOICES)
    ap.add_argument("--ood-texts", default=OOD_TEXTS_JSON)
    ap.add_argument("--ood-only", action="store_true",
                    help="명령은 건너뛰고 OOD 만 합성해 기존 manifest 에 덧붙인다")
    ap.add_argument("--wake-only", action="store_true",
                    help="commands.json 의 wake_candidates 만 합성한다 (label=_wake). "
                         "호출어는 보류 보이스에서도 재야 하므로 전 보이스·전 속도로 만든다")
    ap.add_argument("--ood-train-voices", type=int, default=3,
                    help="OOD 문장당 학습 보이스 수 (보류 보이스는 별도로 전부 붙는다)")
    ap.add_argument("--holdout-voices", nargs="+", default=["F5", "M5"],
                    help="train.py 의 --holdout-voices 와 같아야 한다")
    ap.add_argument("--ood-groups", nargs="+",
                    default=["hard_negative", "chitchat", "short"],
                    help="합성할 OOD 그룹. baseline30 은 이미 합성돼 있고 평가 전용이라 제외")
    args = ap.parse_args()

    import supertonic as st

    spec = json.loads(Path(args.commands).read_text(encoding="utf-8"))
    intents = spec["intents"]
    ood_groups = load_ood_groups(args.ood_texts)

    out = Path(args.out)
    tts = st.TTS(model="supertonic-3", model_dir=args.assets, auto_download=False)
    styles = {v: tts.get_voice_style(v) for v in args.voices}

    # (label, text, voice, speed, group) 작업 목록
    jobs: list[tuple[str, str, str, float, str]] = []

    # 호출어는 명령과 같은 취급으로 전 보이스·전 속도를 만든다. 보류 보이스에서
    # 재현율을 재야 하고, 학습 보이스로 프로토타입을 등록해야 하기 때문이다.
    if args.wake_only:
        for text in spec.get("wake_candidates", []):
            for v in args.voices:
                for sp in args.speeds:
                    jobs.append(("_wake", text, v, sp, "wake"))

    if not args.ood_only and not args.wake_only:
        for it in intents:
            for text in it["phrases"]:
                for v in args.voices:
                    for sp in args.speeds:
                        jobs.append((it["id"], text, v, sp, "command"))

    # OOD 보이스 배정: 문장마다 학습 보이스 몇 개 + 보류 보이스 전부.
    #
    # 보류 보이스를 문장마다 반드시 붙이는 이유: 학습에서 뺀 OOD 문장을 "처음 듣는
    # 목소리 + 처음 듣는 문장" 으로 평가해야 정직한 숫자가 나온다. 예전 배정 방식은
    # (vi*6+k)%30 순환이라 F5/M5 에 걸린 OOD 문장이 6개뿐이었고, 그래서 52.8% 는
    # 클립 36개(19/36) 위의 수였다 — ±16%p 짜리 지표였다.
    if args.wake_only:
        args.ood_groups = []          # 호출어만 만든다

    hv = [v for v in args.holdout_voices if v in args.voices]
    train_pool = [v for v in args.voices if v not in hv]
    n_tv = max(1, min(args.ood_train_voices, len(train_pool)))
    idx = 0
    for group in args.ood_groups:
        for text in ood_groups.get(group, []):
            picked = [train_pool[(idx * n_tv + j) % len(train_pool)] for j in range(n_tv)]
            for j, v in enumerate(picked + hv):
                sp = args.speeds[(idx + j) % len(args.speeds)]
                jobs.append(("_ood", text, v, sp, group))
            idx += 1

    # 이미 있는 클립은 다시 합성하지 않는다 (증분 실행).
    man_path = out / "manifest.json"
    existing: list[dict] = []
    if man_path.exists():
        existing = json.loads(man_path.read_text(encoding="utf-8"))["items"]
    have = {(it["text"], it["voice"], round(float(it["speed"]), 3)) for it in existing}
    jobs = [j for j in jobs if (j[1], j[2], round(j[3], 3)) not in have]

    print(f"합성 대상 {len(jobs)}개 (기존 {len(existing)}개 유지, "
          f"보이스 {len(args.voices)}개, 속도 {args.speeds})", flush=True)

    t_start = time.time()
    audio_sec = 0.0
    manifest: list[dict] = list(existing)
    for i, (label, text, voice, sp, group) in enumerate(jobs):
        wav, _ = tts.synthesize(
            text, styles[voice], lang="ko", total_steps=args.steps, speed=sp
        )
        pcm = to_16k(wav)
        audio_sec += len(pcm) / TARGET_SR
        rel = f"{label}/{clip_name(text, voice, sp)}"
        write_wav(out / rel, pcm)
        manifest.append({"path": rel, "label": label, "text": text,
                         "voice": voice, "speed": sp, "group": group,
                         "samples": int(len(pcm))})
        if (i + 1) % 100 == 0 or i + 1 == len(jobs):
            el = time.time() - t_start
            rate = (i + 1) / el
            eta = (len(jobs) - i - 1) / rate
            print(f"  {i+1:5d}/{len(jobs)}  {el:6.1f}s 경과  "
                  f"{rate:4.1f}개/s  ETA {eta:5.1f}s  "
                  f"(음성 {audio_sec:6.1f}s, RTF {el/max(audio_sec,1e-9):.3f})",
                  flush=True)

    (out / "manifest.json").write_text(
        json.dumps({"sample_rate": TARGET_SR, "items": manifest},
                   ensure_ascii=False, indent=1),
        encoding="utf-8",
    )
    el = time.time() - t_start
    print(f"\n완료: 클립 {len(manifest)}개 (새로 {len(jobs)}개), "
          f"새 음성 {audio_sec/60:.1f}분, "
          f"소요 {el/60:.1f}분 (RTF {el/max(audio_sec, 1e-9):.3f})")
    print(f"→ {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
