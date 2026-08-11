#!/usr/bin/env python3
"""롬을 플래시에 박을 C 헤더로 만든다.

**롬은 레포에 넣지 않는다.** 상용 롬을 공개 저장소에 올리면 배포다. 그래서 이 스크립트가
로컬 파일에서 읽어 `firmware/gb_eink/rom_data.h` 를 만들고, 그 헤더는 gitignore 된다.
레포에 남는 것은 도구뿐이다 — pokedex 쪽에 세운 BYOR 방식과 같다.

    python3 tools/gb_eink/embed_rom.py ~/Downloads/"Pokemon - Red Version (K).gb"

헤더 크기가 소스로 6MB 쯤 되는데(1MB 를 0x.. 로 적으면 그렇다) 컴파일 결과는 1MB 다.
"""
from __future__ import annotations

import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
OUT = ROOT / "firmware/gb_eink/rom_data.h"

MBC = {0x00: "ROM only", 0x01: "MBC1", 0x03: "MBC1+RAM+BAT", 0x0F: "MBC3+RTC+BAT",
       0x10: "MBC3+RTC+RAM+BAT", 0x13: "MBC3+RAM+BAT", 0x19: "MBC5",
       0x1A: "MBC5+RAM", 0x1B: "MBC5+RAM+BAT"}
RAM_SZ = {0x00: 0, 0x01: 2048, 0x02: 8192, 0x03: 32768, 0x04: 131072, 0x05: 65536}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("rom")
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()

    d = Path(args.rom).read_bytes()
    title = d[0x134:0x143].split(b"\0")[0].decode("ascii", "replace")
    cart = d[0x147]
    ram = RAM_SZ.get(d[0x149], 0)
    print(f"{Path(args.rom).name}")
    print(f"  title  {title!r}")
    print(f"  cart   0x{cart:02X}  {MBC.get(cart, '알 수 없음')}")
    print(f"  rom    {len(d)//1024}KB     cart ram {ram//1024}KB")
    if cart not in MBC:
        print("  경고: peanut-gb 가 지원하는 MBC 인지 확인 필요")

    lines = [
        "// 자동 생성 — tools/gb_eink/embed_rom.py. 커밋하지 말 것.",
        "//",
        f"// {title}  cart 0x{cart:02X} ({MBC.get(cart, '?')})  "
        f"{len(d)//1024}KB  cart ram {ram//1024}KB",
        "//",
        "// 상용 롬이므로 이 헤더는 gitignore 된다. 레포에는 만드는 도구만 있다.",
        "#pragma once",
        "#include <stdint.h>",
        "#include <pgmspace.h>",
        "",
        f"#define GB_ROM_LEN {len(d)}",
        f'#define GB_ROM_TITLE "{title}"',
        f"#define GB_CART_RAM_LEN {ram}",
        "",
        "static const uint8_t GB_ROM[GB_ROM_LEN] PROGMEM = {",
    ]
    per = 24
    for i in range(0, len(d), per):
        lines.append("  " + ",".join(f"0x{b:02x}" for b in d[i:i + per]) + ",")
    lines.append("};")
    out = Path(args.out)
    out.write_text("\n".join(lines), encoding="utf-8")
    print(f"→ {out}  (소스 {out.stat().st_size/1024/1024:.1f}MB, 플래시 "
          f"{len(d)/1024:.0f}KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
