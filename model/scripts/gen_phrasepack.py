#!/usr/bin/env python3
"""번역 무전기 구문 팩을 만든다 — ESP32 의 4MB 플래시 + 4MB PSRAM 을 실제로 채운다.

두 가지를 낸다.

  1. 인식용   : 한국어 문장 × 여러 보이스 → 인코더 등록에 쓸 클립
  2. 재생용   : 같은 뜻의 대상 언어 음성 → IMA-ADPCM 4:1 압축 → 보드에 상주

ADPCM 을 쓰는 이유는 용량이 전부다. 16kHz 16bit 모노는 32KB/s 라 7MB 에 218초밖에
안 들어가지만, 4비트 ADPCM 은 8KB/s 라 875초가 들어간다. 4배 차이가 "언어 1개" 와
"언어 4개" 를 가른다. ADPCM 은 곱셈이 없는 정수 코덱이라 ESP32 디코딩 비용도 거의 0 이다.

배치는 이렇게 나눈다:
  플래시  — 자주 쓰는 문구. 태블릿 없이 단독 동작하는 최소 집합.
  PSRAM  — 나머지. 태블릿이 WiFi 로 밀어 넣고 캐시한다.
"""
from __future__ import annotations

import argparse
import json
import struct
import time
import wave
from pathlib import Path

import numpy as np

SR = 16000
PACK_MAGIC = 0x4B505043  # 'CPPK'

# ── IMA ADPCM (표준 테이블). 인코더/디코더 한 쌍이 같은 테이블을 써야 한다.
_STEP = [
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230,
    253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963,
    1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024,
    3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493,
    10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623,
    27086, 29794, 32767,
]
_INDEX = [-1, -1, -1, -1, 2, 4, 6, 8]


def adpcm_encode(pcm: np.ndarray) -> bytes:
    """int16 → 4비트 IMA ADPCM (니블 2개가 1바이트, 하위 니블이 먼저)."""
    pred, idx = 0, 0
    out = bytearray()
    hi = False
    cur = 0
    for s in pcm.astype(np.int32):
        step = _STEP[idx]
        diff = int(s) - pred
        code = 0
        if diff < 0:
            code = 8
            diff = -diff
        if diff >= step:
            code |= 4; diff -= step
        if diff >= step >> 1:
            code |= 2; diff -= step >> 1
        if diff >= step >> 2:
            code |= 1
        # 디코더와 동일한 재구성 (누적 오차가 갈라지지 않게)
        d = step >> 3
        if code & 4: d += step
        if code & 2: d += step >> 1
        if code & 1: d += step >> 2
        pred = pred - d if code & 8 else pred + d
        pred = max(-32768, min(32767, pred))
        idx = max(0, min(88, idx + _INDEX[code & 7]))
        if hi:
            out.append(cur | (code << 4)); hi = False
        else:
            cur = code; hi = True
    if hi:
        out.append(cur)
    return bytes(out)


def adpcm_decode(data: bytes, n: int) -> np.ndarray:
    """검증용 디코더. C 구현이 이것과 같은 값을 내야 한다."""
    pred, idx = 0, 0
    out = np.empty(n, np.int16)
    for i in range(n):
        byte = data[i >> 1]
        code = (byte & 0x0F) if (i & 1) == 0 else (byte >> 4)
        step = _STEP[idx]
        d = step >> 3
        if code & 4: d += step
        if code & 2: d += step >> 1
        if code & 1: d += step >> 2
        pred = pred - d if code & 8 else pred + d
        pred = max(-32768, min(32767, pred))
        idx = max(0, min(88, idx + _INDEX[code & 7]))
        out[i] = pred
    return out


def to_16k(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=np.float32).squeeze()
    from scipy.signal import resample_poly
    y = resample_poly(a, 160, 441).astype(np.float32)
    peak = float(np.abs(y).max())
    if peak > 0:
        y = y / peak * 0.95
    return (y * 32767.0).astype(np.int16)


def write_wav(path: Path, pcm: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR)
        w.writeframes(pcm.tobytes())


def c_bytes(name: str, data: bytes, per_line: int = 16) -> str:
    lines = ["    " + ", ".join(f"0x{b:02x}" for b in data[i:i + per_line])
             for i in range(0, len(data), per_line)]
    return (f"static const uint8_t {name}[{len(data)}] = {{\n"
            + ",\n".join(lines) + "\n};\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--assets", required=True)
    ap.add_argument("--phrases", default="model/phrases.json")
    ap.add_argument("--out", default="model/data/phrasepack")
    ap.add_argument("--outdir", default="firmware/cabin_node")
    ap.add_argument("--steps", type=int, default=8, help="재생용이라 품질을 올린다")
    ap.add_argument("--play-voice", default="F2", help="대상 언어 재생에 쓸 보이스")
    ap.add_argument("--enroll-voices", nargs="+",
                    default=["F1", "F2", "F3", "F4", "M1", "M2", "M3", "M4"])
    ap.add_argument("--enroll-speeds", type=float, nargs="+", default=[0.95, 1.05, 1.15])
    ap.add_argument("--flash-budget-kb", type=int, default=2600,
                    help="플래시에 담을 상한. 앱 공간을 남긴다.")
    args = ap.parse_args()

    import supertonic as st

    spec = json.loads(Path(args.phrases).read_text(encoding="utf-8"))
    langs = spec["target_langs"]
    phrases = spec["phrases"]
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)

    tts = st.TTS(model="supertonic-3", model_dir=args.assets, auto_download=False)
    play_style = tts.get_voice_style(args.play_voice)
    en_styles = {v: tts.get_voice_style(v) for v in args.enroll_voices}

    t0 = time.time()

    # ── 1. 재생용: 대상 언어 음성 → ADPCM
    print(f"재생용 합성: {len(phrases)}문장 × {len(langs)}언어 = "
          f"{len(phrases)*len(langs)}개")
    clips: list[dict] = []
    blob = bytearray()
    audio_sec = 0.0
    for pi, p in enumerate(phrases):
        for li, lang in enumerate(langs):
            text = p[lang]
            wav, _ = tts.synthesize(text, play_style, lang=lang,
                                    total_steps=args.steps)
            pcm = to_16k(wav)
            enc = adpcm_encode(pcm)
            # 왕복 검증: 인코더/디코더가 어긋나면 보드에서 잡음이 된다
            rt = adpcm_decode(enc, len(pcm))
            snr = 10 * np.log10(
                float((pcm.astype(np.float64) ** 2).mean()) /
                max(float(((pcm - rt).astype(np.float64) ** 2).mean()), 1e-9))
            clips.append({"phrase": p["id"], "lang": lang, "text": text,
                          "off": len(blob), "bytes": len(enc),
                          "samples": int(len(pcm)), "snr_db": round(snr, 1)})
            blob += enc
            audio_sec += len(pcm) / SR
        if (pi + 1) % 10 == 0:
            print(f"  {pi+1}/{len(phrases)} 문장  "
                  f"{len(blob)/1024:.0f}KB  {time.time()-t0:.0f}s", flush=True)

    snrs = [c["snr_db"] for c in clips]
    print(f"\nADPCM: {len(blob)/1024:.1f}KB, 음성 {audio_sec:.0f}s, "
          f"압축 {audio_sec*SR*2/len(blob):.2f}:1")
    print(f"  왕복 SNR {min(snrs):.1f}~{max(snrs):.1f} dB "
          f"(평균 {np.mean(snrs):.1f})")

    # ── 2. 배치: 플래시 / PSRAM 분할
    budget = args.flash_budget_kb * 1024
    flash_n = 0
    acc = 0
    for c in clips:
        if acc + c["bytes"] > budget:
            break
        acc += c["bytes"]; flash_n += 1
    print(f"\n배치: 플래시 {flash_n}개 ({acc/1024:.0f}KB / {args.flash_budget_kb}KB), "
          f"PSRAM {len(clips)-flash_n}개 ({(len(blob)-acc)/1024:.0f}KB)")

    # ── 3. 팩 파일 (태블릿이 PSRAM 쪽으로 보낼 것)
    idx = bytearray()
    for c in clips:
        idx += struct.pack("<IIH", c["off"], c["bytes"], c["samples"] // 16)
    hdr = struct.pack("<IIII", PACK_MAGIC, 1, len(clips), len(idx))
    (out / "phrasepack.bin").write_bytes(hdr + bytes(idx) + bytes(blob))
    (out / "phrasepack.json").write_text(
        json.dumps({"sample_rate": SR, "langs": langs, "clips": clips},
                   ensure_ascii=False, indent=1), encoding="utf-8")

    # ── 4. 플래시 헤더
    h = ["// 자동 생성 — model/scripts/gen_phrasepack.py. 직접 수정하지 말 것.",
         "// 대상 언어 음성, IMA-ADPCM 4비트 16kHz. 플래시에 상주해 단독 동작을 보장한다.",
         "// 나머지는 태블릿이 WiFi 로 PSRAM 에 밀어 넣는다.",
         "#pragma once", "#include <stdint.h>", "",
         f"#define CN_PP_SR        {SR}",
         f"#define CN_PP_N_LANG    {len(langs)}",
         f"#define CN_PP_N_PHRASE  {len(phrases)}",
         f"#define CN_PP_N_FLASH   {flash_n}", "",
         "static const char *const cn_pp_langs[CN_PP_N_LANG] = {"
         + ", ".join(f'"{l}"' for l in langs) + "};",
         "static const char *const cn_pp_phrase_ids[CN_PP_N_PHRASE] = {",
         *[f'  "{p["id"]}",' for p in phrases], "};", "",
         "// 한국어 원문 (태블릿 표시 · 디버깅용)",
         "static const char *const cn_pp_ko[CN_PP_N_PHRASE] = {",
         *[f'  "{p["ko"]}",' for p in phrases], "};", "",
         "typedef struct { uint32_t off; uint32_t bytes; uint16_t frames16; "
         "uint8_t phrase, lang; } cn_pp_entry_t;",
         "static const cn_pp_entry_t cn_pp_index[CN_PP_N_FLASH] = {"]
    pid = {p["id"]: i for i, p in enumerate(phrases)}
    for c in clips[:flash_n]:
        h.append(f"  {{ {c['off']}, {c['bytes']}, {c['samples']//16}, "
                 f"{pid[c['phrase']]}, {langs.index(c['lang'])} }},"
                 f"  // {c['phrase']}/{c['lang']}")
    h.append("};\n")
    h.append(c_bytes("cn_pp_audio", bytes(blob[:acc])))
    (Path(args.outdir) / "phrasepack.h").write_text("\n".join(h), encoding="utf-8")
    print(f"phrasepack.h: {(Path(args.outdir)/'phrasepack.h').stat().st_size/1024/1024:.2f}MB "
          f"소스 → 플래시 {acc/1024:.0f}KB")

    # ── 5. 인식용: 한국어 클립 (인코더 등록에 쓴다)
    n_en = len(phrases) * len(args.enroll_voices) * len(args.enroll_speeds)
    print(f"\n인식용 합성: {n_en}개 "
          f"({len(phrases)}문장 × 보이스 {len(args.enroll_voices)} × 속도 {len(args.enroll_speeds)})")
    man = []
    for pi, p in enumerate(phrases):
        for v in args.enroll_voices:
            for sp in args.enroll_speeds:
                wav, _ = tts.synthesize(p["ko"], en_styles[v], lang="ko",
                                        total_steps=4, speed=sp)
                pcm = to_16k(wav)
                rel = f"{p['id']}/{v}_sp{int(sp*100)}.wav"
                write_wav(out / "ko" / rel, pcm)
                man.append({"path": rel, "label": p["id"], "text": p["ko"],
                            "voice": v, "speed": sp, "samples": int(len(pcm))})
        if (pi + 1) % 10 == 0:
            print(f"  {pi+1}/{len(phrases)} 문장  {time.time()-t0:.0f}s", flush=True)
    (out / "ko" / "manifest.json").write_text(
        json.dumps({"sample_rate": SR, "items": man}, ensure_ascii=False, indent=1),
        encoding="utf-8")

    print(f"\n완료 {(time.time()-t0)/60:.1f}분")
    print(f"→ {out/'phrasepack.bin'}  {Path(args.outdir)/'phrasepack.h'}  {out/'ko'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
