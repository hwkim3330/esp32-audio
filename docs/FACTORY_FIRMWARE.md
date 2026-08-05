# 공장 펌웨어 — 백업과 복구

보드가 도착했을 때 이미 들어 있던 AI-Thinker 공장 펌웨어의 4MB 전체 덤프다.
**아무것도 굽기 전에 이걸 떠 놨다.** 뭘 해도 되돌릴 수 있다.

## 파일

| 파일 | 크기 | 내용 |
| --- | --- | --- |
| `firmware/factory/factory_ESP32-Audio-Kit_4MB_4c11aef5a518.bin` | 4,194,304 B | 플래시 `0x000000`~`0x400000` 전체 |
| `firmware/factory/partition_table.bin` | 3,072 B | `0x8000` 파티션 테이블 원본 |
| `firmware/factory/SHA256SUMS` | — | 무결성 검증용 |

파일명의 `4c11aef5a518` 은 이 보드의 MAC 이다. 다른 보드에 굽지 말 것 — MAC 은 eFuse 라
덮이지 않지만, NVS 에 든 캘리브레이션/페어링 정보는 이 보드 것이다.

## 파티션 맵

`0x8000` 을 읽어서 직접 파싱한 결과다.

```
nvs              data sub=0x02 off=0x009000 size=0x004000 (16K)
otadata          data sub=0x00 off=0x00d000 size=0x002000 (8K)
phy_init         data sub=0x01 off=0x00f000 size=0x001000 (4K)
factory          app  sub=0x00 off=0x010000 size=0x0e1000 (900K)
ota_0            app  sub=0x10 off=0x100000 size=0x180000 (1536K)
ota_1            app  sub=0x11 off=0x280000 size=0x180000 (1536K)
```

세 앱 슬롯 모두 유효한 이미지가 들어 있다 (매직 바이트 `0xE9`):

| 슬롯 | 실사용 | SHA256 (앞 16) | 정체 |
| --- | --- | --- | --- |
| `factory` | 283K | `9c94d88baf2bcc32` | 공장 셀프테스트 (`FACTORY_PARTION`) |
| `ota_0` | 323K | `03e121896733595e` | `OTA1/main/main.c` (`USER_PARTION_1`) |
| `ota_1` | 1084K | `4222ca38fd6999ca` | **현재 부팅 중** — BT 스피커 + SD MP3 플레이어 |
| `bootloader` | 13K | `bb38af2c906dca42` | 2차 부트로더 |

`otadata` 첫 워드가 `0x00000001` → 슬롯 1(`ota_1`) 부팅. 부팅 로그의 `USER_PARTION_2`
출력과 일치한다.

빌드 환경: ESP-IDF **v3.1.1-rc2-2-gd1d2ce8c2**, ESP-ADF, PHY 3662 (2018-05-09).
꽤 오래된 IDF 다 — 이 위에 얹어 개발할 게 아니라, 되돌릴 기준점으로만 쓰는 게 맞다.

## 백업 떠 두기 (다른 보드에도)

```bash
./tools/dump_factory.sh /dev/ttyUSB0 my_backup.bin
```

4MB 를 460800 baud 로 약 100초 걸린다.

## 복구

```bash
./tools/restore_factory.sh /dev/ttyUSB0
```

내부적으로 하는 일:

```bash
esptool --port /dev/ttyUSB0 --baud 460800 write-flash 0x0 \
  firmware/factory/factory_ESP32-Audio-Kit_4MB_4c11aef5a518.bin
```

복구 확인 방법 — 리셋 후 시리얼에 `esp32_bt_sd_v1.2` 와 `SD-BT-Player` 가 뜨고, 폰의
블루투스 목록에 `SD-BT-Player` 가 보이면 정상이다. (이 PC 에는 BT 어댑터가 없어서
호스트에서는 확인할 수 없다.)

## 공장 펌웨어로 지금 할 수 있는 것

굽지 않고 그대로 쓸 수 있는 기능이다.

- **BT 스피커** — 폰에서 `SD-BT-Player` 페어링 → 헤드폰/스피커 출력. 폰의 이전/다음/재생
  버튼도 먹는다 (AVRCP).
- **SD MP3 플레이어** — microSD 에 MP3 넣고 터치키로 조작.
  **MP3 만 된다** — 이 빌드에 wav/aac/flac 디코더가 없다.
- 터치키 6개: prev / mode(SD↔BT) / next / stop-play / vol+ / vol-

WiFi 는 **안 된다.** 칩에는 있지만 이 펌웨어가 WiFi·IP 스택을 아예 안 링크했다
(`esp_wifi_*`, `lwip`, `tcpip`, `esp_netif`, `socket` 심볼 0개). 쓰려면 재플래시해야 한다.
