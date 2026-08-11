#!/usr/bin/env python3
"""웹 자산을 gzip 해서 C 헤더로 박는다.

왜 이렇게 하나. 보드가 SoftAP 로 뜰 때는 **인터넷이 없다** — 폰이 붙어도 CDN 에
못 나간다. 그래서 차트 라이브러리를 플래시에 같이 넣어야 하고, 넣을 때는 gzip 한
바이트열로 넣는다(WebServer 가 Content-Encoding: gzip 으로 그대로 흘려보낸다).

SPIFFS 를 쓰지 않는 이유: 파티션 표를 바꿔야 하고, 이 스케치는 앱에 3MB 를 다 쓴다.
자산이 30KB 급이면 PROGMEM 이 더 간단하고 마운트 실패라는 실패 모드도 없다.

원본은 `firmware/radar_display/web/` 에 그대로 두고(라이선스 포함) 이 스크립트가
헤더를 만든다 — 헤더를 손으로 고치면 다음 실행에서 덮인다.

    python3 tools/embed_web.py
"""
from __future__ import annotations

import gzip
from pathlib import Path

SRC = Path("firmware/radar_display/web")
OUT = Path("firmware/radar_display/web_assets.h")

ASSETS = [
    ("UPLOT_JS", "uPlot.iife.min.js", "application/javascript"),
    ("UPLOT_CSS", "uPlot.min.css", "text/css"),
]


def c_bytes(name: str, blob: bytes, per_line: int = 16) -> str:
    lines = []
    for i in range(0, len(blob), per_line):
        lines.append("  " + ", ".join(f"0x{b:02x}" for b in blob[i:i + per_line]))
    return (f"static const uint8_t {name}_GZ[{len(blob)}] PROGMEM = {{\n"
            + ",\n".join(lines) + "\n};\n"
            + f"static const size_t {name}_GZ_LEN = {len(blob)};\n")


def main() -> int:
    parts = [
        "// 자동 생성 — tools/embed_web.py. 직접 수정하지 말 것.",
        "//",
        "// SoftAP 에는 인터넷이 없으므로 차트 라이브러리를 플래시에 같이 넣는다.",
        "// gzip 한 바이트열이고 WebServer 가 Content-Encoding: gzip 으로 그대로 보낸다.",
        "//",
        "// uPlot v1.6.32 — MIT (c) 2022 Leon Sorokin. 원본과 라이선스 전문은",
        "// firmware/radar_display/web/ 에 있다.",
        "#pragma once",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "#include <pgmspace.h>",
        "",
    ]
    total_raw = total_gz = 0
    for name, fn, mime in ASSETS:
        raw = (SRC / fn).read_bytes()
        gz = gzip.compress(raw, 9, mtime=0)     # mtime=0 → 실행마다 같은 바이트가 나온다
        total_raw += len(raw)
        total_gz += len(gz)
        parts.append(f"// {fn}  {len(raw):,}B → gzip {len(gz):,}B   ({mime})")
        parts.append(c_bytes(name, gz))
        print(f"{fn:24s} {len(raw):7,}B → {len(gz):6,}B  ({100*len(gz)/len(raw):.0f}%)")
    OUT.write_text("\n".join(parts), encoding="utf-8")
    print(f"\n합계 {total_raw:,}B → {total_gz:,}B 플래시")
    print(f"→ {OUT}  ({OUT.stat().st_size/1024:.0f}KB 소스)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
