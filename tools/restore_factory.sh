#!/usr/bin/env bash
# 백업해 둔 공장 펌웨어로 되돌린다. 현재 플래시 내용은 전부 사라진다.
set -euo pipefail
PORT="${1:-/dev/ttyUSB0}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMG="${2:-$ROOT/firmware/factory/factory_ESP32-Audio-Kit_4MB_4c11aef5a518.bin}"
ESPTOOL="${ESPTOOL:-$HOME/.arduino15/packages/esp32/tools/esptool_py/5.0.0/esptool}"

[[ -f "$IMG" ]] || { echo "이미지 없음: $IMG" >&2; exit 1; }

echo "포트 : $PORT"
echo "이미지: $IMG"
echo "이 이미지는 MAC 4c:11:ae:f5:a5:18 보드에서 뜬 것이다. 다른 보드면 중단할 것."
read -rp "현재 플래시 4MB 를 전부 덮어쓴다. 계속? [yes/NO] " ans
[[ "$ans" == "yes" ]] || { echo "취소"; exit 1; }

"$ESPTOOL" --port "$PORT" --baud 460800 write-flash 0x0 "$IMG"
echo
echo "완료. 리셋 후 시리얼에 esp32_bt_sd_v1.2 / SD-BT-Player 가 뜨면 정상."
