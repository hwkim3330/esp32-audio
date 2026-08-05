#!/usr/bin/env bash
# 플래시 4MB 전체를 파일로 백업. 460800 baud 로 약 100초.
set -euo pipefail
PORT="${1:-/dev/ttyUSB0}"
OUT="${2:-factory_backup_4MB.bin}"
ESPTOOL="${ESPTOOL:-$HOME/.arduino15/packages/esp32/tools/esptool_py/5.0.0/esptool}"

[[ -e "$OUT" ]] && { echo "이미 존재함, 덮지 않는다: $OUT" >&2; exit 1; }
"$ESPTOOL" --port "$PORT" --baud 460800 read-flash 0x0 0x400000 "$OUT"
sha256sum "$OUT"
