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
import json
import sys
import time
from pathlib import Path

import numpy as np

SUPERTONIC_SR = 44100
TARGET_SR = 16000
VOICES = ["F1", "F2", "F3", "F4", "F5", "M1", "M2", "M3", "M4", "M5"]

# 명령과 무관한 발화. 이걸 "기타"로 배워야 아무 말에나 반응하지 않는다.
OOD_TEXTS = [
    "오늘 점심 뭐 먹을까", "그 영화 봤어？", "회의는 세 시에 시작합니다",
    "비가 올 것 같은데", "어제 잠을 못 잤어", "이번 주말에 뭐 해",
    "커피 한 잔 마시고 싶다", "책상 위에 서류가 있어", "전화 좀 받아볼게",
    "택배가 아직 안 왔네", "고양이가 소파에서 자고 있어", "환율이 많이 올랐다",
    "그건 좀 어려울 것 같아요", "다음 정류장에서 내립니다", "사진 좀 찍어줄래",
    "숙제 다 했어？", "이 옷 어때？", "은행에 가야 하는데",
    "축구 경기 결과 봤어", "지난달보다 매출이 늘었습니다",
    "여기 와이파이 비밀번호 뭐예요", "머리가 좀 아프네",
    "내일 아침에 일찍 나가야 해", "그 사람 이름이 뭐였지",
    "설명서를 먼저 읽어보세요", "생각보다 훨씬 크네요",
    "약속 시간을 조금 늦춰도 될까요", "이번 학기 시간표 나왔어",
    "빨래를 개어 놓았어요", "공원에서 산책하기 좋은 날씨야",
]


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
    ap.add_argument("--ood-per-voice", type=int, default=6,
                    help="보이스당 OOD 문장 수 (전체에서 순환 선택)")
    args = ap.parse_args()

    import supertonic as st

    spec = json.loads(Path(args.commands).read_text(encoding="utf-8"))
    intents = spec["intents"]

    out = Path(args.out)
    tts = st.TTS(model="supertonic-3", model_dir=args.assets, auto_download=False)
    styles = {v: tts.get_voice_style(v) for v in args.voices}

    # (label, text) 작업 목록. OOD 는 보이스마다 다른 문장을 쓰게 오프셋을 준다.
    jobs: list[tuple[str, str, str, float]] = []
    for it in intents:
        for text in it["phrases"]:
            for v in args.voices:
                for sp in args.speeds:
                    jobs.append((it["id"], text, v, sp))
    for vi, v in enumerate(args.voices):
        for k in range(args.ood_per_voice):
            text = OOD_TEXTS[(vi * args.ood_per_voice + k) % len(OOD_TEXTS)]
            for sp in args.speeds:
                jobs.append(("_ood", text, v, sp))

    print(f"합성 대상 {len(jobs)}개 "
          f"(인텐트 {len(intents)}개 + OOD, 보이스 {len(args.voices)}개, "
          f"속도 {args.speeds})", flush=True)

    t_start = time.time()
    audio_sec = 0.0
    manifest: list[dict] = []
    for i, (label, text, voice, sp) in enumerate(jobs):
        wav, _ = tts.synthesize(
            text, styles[voice], lang="ko", total_steps=args.steps, speed=sp
        )
        pcm = to_16k(wav)
        audio_sec += len(pcm) / TARGET_SR
        name = f"{voice}_sp{int(sp * 100)}_{abs(hash(text)) % 10**8:08d}.wav"
        rel = f"{label}/{name}"
        write_wav(out / rel, pcm)
        manifest.append({"path": rel, "label": label, "text": text,
                         "voice": voice, "speed": sp,
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
    print(f"\n완료: {len(manifest)}개 클립, 음성 {audio_sec/60:.1f}분, "
          f"소요 {el/60:.1f}분 (RTF {el/audio_sec:.3f})")
    print(f"→ {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
