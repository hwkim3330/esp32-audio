#!/usr/bin/env python3
"""이미 합성해 둔 구문 팩에서 플래시 헤더만 다시 만든다 (재합성 없음).

플래시 예산은 한 번에 맞히기 어렵다. 인코더 가중치 268KB, 프로토타입 46KB, 앱 코드,
그리고 파티션 스킴이 모두 얽힌다 — 실측으로는 앱 파티션 3MB 중 구문 음성에 쓸 수 있는
건 약 1.75MB 였다. 다시 20분 합성할 이유가 없으니 인덱스와 헤더만 다시 뽑는다.

플래시에 안 들어간 나머지는 태블릿이 WiFi 로 PSRAM 에 밀어 넣는다 — 원래 설계다.

언어별로 고르게 남기는 것이 중요하다. 앞에서부터 자르면 마지막 언어가 통째로
빠져서 "그 언어는 아예 안 되는" 상태가 된다.
"""
from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

PACK_MAGIC = 0x4B505043


def c_bytes(name: str, data: bytes, per_line: int = 16) -> str:
    lines = ["    " + ", ".join(f"0x{b:02x}" for b in data[i:i + per_line])
             for i in range(0, len(data), per_line)]
    return (f"static const uint8_t {name}[{len(data)}] = {{\n"
            + ",\n".join(lines) + "\n};\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pack", default="model/data/phrasepack")
    ap.add_argument("--phrases", default="model/phrases.json")
    ap.add_argument("--outdir", default="firmware/cabin_node")
    ap.add_argument("--flash-budget-kb", type=int, default=1700)
    args = ap.parse_args()

    pack = Path(args.pack)
    meta = json.loads((pack / "phrasepack.json").read_text(encoding="utf-8"))
    spec = json.loads(Path(args.phrases).read_text(encoding="utf-8"))
    clips = meta["clips"]
    langs = meta["langs"]
    phrases = spec["phrases"]
    pid = {p["id"]: i for i, p in enumerate(phrases)}

    raw = (pack / "phrasepack.bin").read_bytes()
    magic, ver, n, idx_len = struct.unpack("<IIII", raw[:16])
    assert magic == PACK_MAGIC and n == len(clips), "팩 파일이 메타와 안 맞는다"
    blob = raw[16 + idx_len:]

    # 문장 순서로 돌면서 모든 언어를 함께 넣는다. 예산이 끊기면 그 문장부터 PSRAM.
    budget = args.flash_budget_kb * 1024
    by_phrase: dict[str, list[dict]] = {}
    for c in clips:
        by_phrase.setdefault(c["phrase"], []).append(c)

    keep: list[dict] = []
    used = 0
    dropped_from = None
    for p in phrases:
        group = by_phrase.get(p["id"], [])
        need = sum(c["bytes"] for c in group)
        if used + need > budget:
            dropped_from = p["id"]
            break
        keep.extend(group)
        used += need

    n_ph_flash = len({c["phrase"] for c in keep})
    print(f"플래시 예산 {args.flash_budget_kb}KB")
    print(f"  담김   문장 {n_ph_flash}/{len(phrases)} × 언어 {len(langs)} "
          f"= {len(keep)}개, {used/1024:.0f}KB")
    print(f"  PSRAM  문장 {len(phrases)-n_ph_flash}개, "
          f"{(len(blob)-used)/1024:.0f}KB (태블릿이 전송)")
    if dropped_from:
        print(f"  '{dropped_from}' 부터 PSRAM 으로 넘어간다 "
              f"(언어별로 고르게 — 특정 언어가 통째로 빠지지 않는다)")

    # keep 의 오프셋은 원본 blob 기준이라, 플래시용으로 다시 채운다.
    out_blob = bytearray()
    entries = []
    for c in keep:
        entries.append({**c, "flash_off": len(out_blob)})
        out_blob += blob[c["off"]:c["off"] + c["bytes"]]

    h = ["// 자동 생성 — model/scripts/repack_flash.py. 직접 수정하지 말 것.",
         "// 대상 언어 음성, IMA-ADPCM 4비트 16kHz. 플래시에 상주해 단독 동작을 보장한다.",
         "// 여기 없는 문장은 태블릿이 WiFi 로 PSRAM 에 밀어 넣는다.",
         "#pragma once", "#include <stdint.h>", "",
         f"#define CN_PP_SR        {meta['sample_rate']}",
         f"#define CN_PP_N_LANG    {len(langs)}",
         f"#define CN_PP_N_PHRASE  {len(phrases)}",
         f"#define CN_PP_N_FLASH   {len(entries)}",
         f"#define CN_PP_N_PHRASE_FLASH {n_ph_flash}", "",
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
    for e in entries:
        h.append(f"  {{ {e['flash_off']}, {e['bytes']}, {e['samples']//16}, "
                 f"{pid[e['phrase']]}, {langs.index(e['lang'])} }},"
                 f"  // {e['phrase']}/{e['lang']}")
    h.append("};\n")
    h.append(c_bytes("cn_pp_audio", bytes(out_blob)))

    p = Path(args.outdir) / "phrasepack.h"
    p.write_text("\n".join(h), encoding="utf-8")
    print(f"\nphrasepack.h: {p.stat().st_size/1024/1024:.2f}MB 소스 → "
          f"플래시 {len(out_blob)/1024:.0f}KB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
