# esp32-audio

AI-Thinker **ESP32-Audio-Kit** (ESP32-A1S 모듈) 조사 기록 + 공장 펌웨어 백업.

`/dev/ttyUSB0` 에 꽂힌 보드가 뭔지, 뭐가 되고 뭐가 안 되는지, 그리고 이걸로 뭘 할지를
실제로 측정한 값만으로 정리한 저장소다. 추정한 부분은 전부 "미확인"으로 표시했다.

---

## 1. 보드 식별

빈 DevKit이 아니라 **오디오 개발보드**이고, 공장 펌웨어가 살아 있는 상태로 도착했다.

| 항목 | 값 | 근거 |
| --- | --- | --- |
| 칩 | ESP32-D0WD rev v1.0, 듀얼코어 240MHz | `esptool flash-id` |
| 플래시 | 4MB (Winbond, mfr `ef` dev `4016`) | `esptool flash-id` |
| PSRAM | **4MB** (외부 SPI RAM, 정상 동작) | 부팅 로그 `spiram: SPI SRAM memory test OK` + `Adding pool of 4096K` |
| 코덱 | **AC101** | 펌웨어 문자열 `AC101_init`, `AC101_DRIVER` |
| USB-시리얼 | Silicon Labs CP2102 (`10c4:ea60`) | `lsusb` |
| MAC | `4c:11:ae:f5:a5:18` | `esptool flash-id` |
| 무선 | WiFi 2.4GHz + Classic BT + BLE | eFuse feature 비트 |

**AC101 이 결정적 근거다.** Espressif ESP32-LyraT 는 ES8388 을 쓰고, AC101 은 AI-Thinker
ESP32-A1S 계열만 쓴다. 게다가 공장 펌웨어에 빌드 경로가 그대로 박혀 있다:

```
/Users/mac/esp_demo/Aithnker_ESP32-Audio-Kit/factory/main/main.c
/Users/mac/esp_demo/Aithnker_ESP32-Audio-Kit/OTA1/main/main.c
/Users/mac/esp_demo/Aithnker_ESP32-Audio-Kit/Audio_bt_sd_combin/main/play_sdcard_mp3_control_example.c
```

자세한 내용: [docs/HARDWARE.md](docs/HARDWARE.md)

---

## 2. 지금 되는 것 / 하드웨어에 있는 것 / 없는 것

공장 펌웨어는 **순수 블루투스 전용**이다. WiFi·IP 스택 심볼이 정확히 0개
(`esp_wifi_*`, `lwip`, `tcpip`, `esp_netif`, `socket` 전부 매칭 없음). Bluedroid 는 통째로
들어 있고 A2DP + AVRCP 까지 있다.

| 연결 | 상태 | 근거 |
| --- | --- | --- |
| USB 시리얼 | **지금 됨** — `/dev/ttyUSB0` 115200 | 실측 |
| Classic BT A2DP + AVRCP | **지금 됨** — BT 이름 `SD-BT-Player` | 펌웨어 문자열 `A2DP`, `AVRC_MsgReq`, `SD-BT-Player` |
| microSD (MP3 만) | **지금 됨** — 현재 카드 미삽입 | 부팅 로그 `no sdcard detect`, mp3 디코더만 존재 |
| 아날로그 출력 (헤드폰/스피커) | 있음 | AC101 DAC |
| 마이크 / 녹음 | 있음 | 펌웨어 문자열 `record`, `line_in`, `Record` |
| 터치키 6개 | **지금 됨** | 부팅 로그 `[1.4] Initialize Touch peripheral`, 문자열 `Touch_Pad` |
| WiFi 2.4GHz | 칩엔 있으나 **공장 펌웨어가 안 씀** → 재플래시 필요 | WiFi 심볼 0개 |
| BLE GATT 서버 | 스택은 올라와 있으나 앱이 안 씀 → 재플래시 필요 | `gatt_*` 심볼 존재, 서버 등록 없음 |
| 이더넷 / TSN | **없음** (PHY 부재) | ESP32-D0WD 는 MAC 만 있고 이 보드에 PHY 없음 |
| CAN | 컨트롤러(TWAI)는 칩에 있으나 **온보드 트랜시버 없음** | 외부 모듈 필요 |
| DIP 스위치 5개 | 기능 **미확인** — 리비전별로 다름 | 실크스크린/회로도 확인 필요 |

### 이 저장소를 쓰는 사람이 먼저 알아야 할 두 가지

1. **이 PC 에 블루투스 어댑터가 없다.** `bluetoothctl show` → `No default controller available`.
   BT/BLE 동작 검증은 폰이나 태블릿에서 해야 한다.
2. **재플래시하면 공장 BT 플레이어가 사라진다.** 4MB 전체 백업이
   [`firmware/factory/`](firmware/factory/) 에 있으니 되돌릴 수 있다.
   복구 절차: [docs/FACTORY_FIRMWARE.md](docs/FACTORY_FIRMWARE.md)

---

## 3. 공장 펌웨어 — 앱이 3개 들어 있다

파티션 테이블을 읽어 보니 `factory` + `ota_0` + `ota_1` 세 슬롯 모두 유효한 앱
이미지(`0xE9` 매직)를 담고 있다. 문자열의 빌드 경로와 1:1 로 대응된다.

| 파티션 | 오프셋 | 할당 | 실사용 | 정체 |
| --- | --- | --- | --- | --- |
| `bootloader` | `0x001000` | 28K | 13K | 2차 부트로더 |
| `partition_table` | `0x008000` | 3K | — | |
| `nvs` / `otadata` / `phy_init` | `0x009000`~ | 28K | — | |
| `factory` | `0x010000` | 900K | 283K | `factory/main/main.c` — 공장 셀프테스트 (`FACTORY_PARTION`) |
| `ota_0` | `0x100000` | 1536K | 323K | `OTA1/main/main.c` (`USER_PARTION_1`) |
| `ota_1` | `0x280000` | 1536K | 1084K | **현재 부팅 중** — `Audio_bt_sd_combin` BT 스피커 + SD MP3 플레이어 (`USER_PARTION_2`) |

`otadata` 가 슬롯 1(`ota_1`)을 가리키고, 부팅 로그도 `USER_PARTION_2` 를 출력한다.
ESP-IDF **v3.1.1-rc2** / ESP-ADF 기반, PHY 버전 3662 (2018-05-09 빌드).

전체 부팅 로그: [docs/boot_log_factory.txt](docs/boot_log_factory.txt)

---

## 4. 뭘 할까 — 판단

핵심은 **일반 ESP32 와의 유일한 차이가 오디오 입출력**이라는 점이다. BLE·ESP-NOW·GPIO 는
어느 ESP32 나 하고, 이 보드는 오히려 코덱+SD+PSRAM 이 핀을 잡아먹어 **남는 GPIO 가 더 적다.**
그러니 소리를 쓰지 않는 용도로 이 보드를 고르는 건 손해다.

우선순위와 근거, 차량 내 실제 위치, AI 로 뭘 할 수 있고 뭘 못 하는지:
→ **[docs/IDEAS.md](docs/IDEAS.md)**

한 줄 요약:

- **차량에서의 진짜 자리** = EV **AVAS**(저소음차 경고음 발생장치, 법정 의무 장치)의 기능
  프로토타입. ESP32 가 속도 연동 음 합성, AC101+앰프가 스피커 구동 — 실제 AVAS ECU 와
  블록도가 같다. 그 다음이 경고음/차임 모듈, 캐빈 음성 HMI 마이크 노드, 실내 음향 이벤트 감지.
- **태블릿과 조합** = 0줄 코딩으로 되는 게 하나 있다. 태블릿이 이 보드를 A2DP BT 스피커로
  페어링하면 태블릿 TTS·경보가 실제 캐빈 스피커로 나온다. 재플래시 없이 오늘 된다.
- **AI** = 이 칩은 추론 플랫폼이 아니다. ESP-SR 의 신형 모델과 MultiNet 은 ESP32-S3 를
  요구하고, 한국어는 아예 미지원(중/영만). 현실적 역할은 **AI 의 오디오 프런트엔드** —
  보드가 마이크/스피커/VAD, 무거운 건 PC·태블릿·서버에서.

---

## 5. 저장소 구조

```
esp32-audio/
├── README.md                         이 문서
├── docs/
│   ├── HARDWARE.md                   보드 식별 근거, 커넥터, 미확인 항목
│   ├── FACTORY_FIRMWARE.md           파티션 맵, 백업/복구 절차
│   ├── IDEAS.md                      용도 판단 · 차량 내 위치 · AI 범위
│   └── boot_log_factory.txt          공장 펌웨어 부팅 로그 (ANSI 제거)
├── firmware/factory/
│   ├── factory_ESP32-Audio-Kit_4MB_4c11aef5a518.bin   4MB 전체 덤프
│   ├── partition_table.bin                            0x8000 파티션 테이블
│   └── SHA256SUMS
└── tools/
    ├── identify.sh                   칩/플래시/파티션 확인 (읽기 전용)
    ├── dump_factory.sh               4MB 전체 백업
    └── restore_factory.sh            백업으로 되돌리기
```

## 6. 필요한 도구

`esptool` 은 arduino-cli 의 esp32 코어에 같이 들어온다. 별도 설치 안 해도 된다:

```
~/.arduino15/packages/esp32/tools/esptool_py/5.0.0/esptool
```

빌드는 `arduino-cli` + esp32 코어 3.3.0 으로 한다. FQBN 옵션 문자열은 실제 빌드로
확인했다(2026-08-10, `firmware/cabin_node` 기준):

```bash
arduino-cli compile --fqbn \
  'esp32:esp32:esp32:PSRAM=enabled,FlashSize=4M,PartitionScheme=custom,CPUFreq=240' \
  firmware/cabin_node
# → Sketch uses 3,641,491 bytes / partitions.csv 의 앱 파티션 3.9MB
```

`PartitionScheme=custom` 이 스케치 폴더의 `partitions.csv` 를 집어간다. arduino-cli 가
같이 찍는 `Maximum is 16777216` 은 커스텀 파티션에서 잘못 나오는 값이라 무시한다 —
진짜 한도는 `partitions.csv` 다.
