#!/usr/bin/env bash
# 보드 식별 — 읽기 전용. 플래시를 쓰지 않는다 (칩 리셋만 발생).
set -euo pipefail
PORT="${1:-/dev/ttyUSB0}"
ESPTOOL="${ESPTOOL:-$HOME/.arduino15/packages/esp32/tools/esptool_py/5.0.0/esptool}"

echo "=== 칩 / 플래시 ==="
"$ESPTOOL" --port "$PORT" flash-id

echo
echo "=== 파티션 테이블 (0x8000) ==="
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
"$ESPTOOL" --port "$PORT" read-flash 0x8000 0xC00 "$TMP/pt.bin" >/dev/null
python3 - "$TMP/pt.bin" <<'PY'
import struct, sys
data = open(sys.argv[1], 'rb').read()
kind = {0: 'app', 1: 'data'}
for i in range(0, len(data), 32):
    e = data[i:i + 32]
    if e[:2] != b'\xaa\x50':
        continue
    off, size = struct.unpack('<II', e[4:12])
    name = e[12:28].rstrip(b'\x00').decode('utf-8', 'replace')
    print(f'{name:16s} {kind.get(e[2], e[2]):4s} sub=0x{e[3]:02x} '
          f'off=0x{off:06x} size=0x{size:06x} ({size // 1024}K)')
PY
