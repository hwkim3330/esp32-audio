# web_mp3 — 브라우저로 넣고 블루투스로 듣는 MP3 플레이어

이 보드의 3.5mm 잭은 죽어 있다. 그래도 MP3 플레이어는 된다 — 출력을 A2DP 로 뺀다.

```
MP3 파일 (SD 또는 내장 플래시)
   → minimp3 디코드 → 44.1kHz 스테레오 리샘플 → PSRAM 링버퍼 256KB
   → A2DP 소스 데이터 콜백 → SBC 인코딩(BT 컨트롤러) → 이어폰/스피커
조작: 웹 UI(WiFi) + 온보드 버튼 6개
```

## 왜 A2DP 인가 — 잭으로는 못 낸다

`firmware/loopback_test` 에서 확정했다: **I2S 가 정확히 0 을 내보낸다.** DIN 후보 8개,
`ADC_SRC` 9개, `ADC_APC_CTRL` 9개, `ADC_VOL_CTRL` 6개, `MODE_ADC`/`ADC_DAC`, APLL on/off —
전부 무신호. AC101 I2C 는 양방향으로 완벽하다. 남은 단일 용의자는 **가운데 DIP 스위치**
(물리 점퍼)다. 소프트웨어로 넘을 수 있는 벽이 아니다.

A2DP 는 그 경로를 안 지나간다. 앱이 PCM 을 만들어 BT 컨트롤러에 바로 넘긴다.

## 실기에서 확인한 것 / 아직 아닌 것

| | 상태 | 근거 |
| --- | --- | --- |
| 빌드 | **됨** — 1,832,791 바이트 | `app0` 1920K 에 들어간다(1664K 로 잡았다가 넘쳤다) |
| 부팅 · PSRAM | **됨** — PSRAM 여유 3800KB, 내부 힙 190KB | 부팅 로그 |
| LittleFS 2112K | **됨** | `[fs] LittleFS 8 / 2112 KB` |
| SD 마운트(1비트) | **됨** — 16GB 카드 인식 | `[sd] 마운트됨` |
| WiFi AP + 웹서버 | **됨** — PC 가 붙어 `/api/sd/list` 응답 받음 | 실측 |
| BT 컨트롤러 + A2DP 소스 init | **됨** — MAC `…f5:a5:1a`, 이름 `KETI-MP3` | 부팅 로그 |
| SD → HTTP 다운로드 | **됨 — 약 420 KB/s** | 1.9GB 백업 실측 |
| **이어폰 A2DP 연결·재생** | **미검증** | 페어링 모드로 둔 이어폰이 필요하다 |
| **MP3 디코드 → 소리** | **미검증** | 위와 같은 이유 |
| 마이크 녹음 | **불가** — I2S 가 죽어서 입력도 0 | 음향 폐루프 자기진단 |

`SD_MMC.usedBytes()` 가 찍는 사용량은 **믿지 마라.** 실제 1.9GB 가 든 카드를 65MB 로
보고했다(exFAT). 용량을 알아야 하면 `tools/sd_backup.py --dry-run` 으로 세라.

## SD 카드는 읽기 전용이다 (의도한 설계)

카드에 남의 데이터가 있을 수 있다(실제로 GoPro HERO4 영상 1.9GB 가 들어 있었다).
그래서 이 펌웨어에는 **SD 쓰기 경로가 아예 없다**:

- 업로드는 언제나 내장 플래시(LittleFS)로 간다
- `/api/delete` 는 플래시에 올린 파일만 지운다. SD 파일 이름을 주면 404
- `SD_MMC.begin(..., format_if_empty=false)` — 빈 카드도 포맷하지 않는다
- 재생은 SD 를 읽기만 한다

카드를 통째로 내려받으려면 PC 에서:

```bash
tools/sd_backup.py 192.168.4.1 --dry-run     # 목록·용량만
tools/sd_backup.py 192.168.4.1               # ~/esp32_audio_kit/sd_backup 으로
```

크기가 같은 파일은 건너뛰므로 중단하고 다시 돌리면 이어서 받는다. (WebServer 의
`streamFile` 은 Range 를 지원하지 않아 **파일 단위**로만 이어받는다 — 받다 만 파일은
처음부터 다시 받는다.)

## 굽기

```bash
arduino-cli compile --fqbn \
  'esp32:esp32:esp32:PSRAM=enabled,FlashSize=4M,PartitionScheme=custom,CPUFreq=240' \
  firmware/web_mp3
arduino-cli upload -p /dev/ttyUSB0 --fqbn \
  'esp32:esp32:esp32:PSRAM=enabled,FlashSize=4M,PartitionScheme=custom,CPUFreq=240' \
  firmware/web_mp3
```

`PartitionScheme=custom` 이 이 폴더의 `partitions.csv` 를 집어간다. arduino-cli 가 찍는
`Maximum is 16777216` 은 커스텀 파티션에서 나오는 헛값이다 — 진짜 한도는 `partitions.csv` 다.

## 쓰는 법

1. 저장한 AP 가 없으면 보드가 **AP 모드**로 뜬다: SSID `esp32-mp3` / 비번 `12345678` →
   `http://192.168.4.1/`
2. 웹 UI 의 WiFi 칸에 집/사무실 AP 를 넣으면 재부팅해서 STA 로 붙는다
   (`http://esp32-mp3.local/` 도 된다)
3. 이어폰을 **페어링 모드**로 두고 `기기 탐색` → 목록에서 고르면 A2DP 로 붙는다.
   한 번 붙은 기기는 NVS 에 저장돼 다음 부팅에 자동 재연결한다
4. MP3 를 끌어다 놓으면 내장 플래시로 올라간다(2MB 남짓 = 128kbps 3분 곡 한 곡쯤).
   여러 곡은 SD 에 넣는 편이 낫다 — 루트와 한 단계 아래 폴더에서 `.mp3` 를 최대 200개 모은다

## 버튼 6개

실기로 확정한 매핑(`KEY1=36 KEY2=13 KEY3=19 KEY4=23 KEY5=18 KEY6=5`)을 그대로 쓴다.

| 키 | 짧게 | 길게(1.2초) |
| --- | --- | --- |
| KEY1 | 재생/일시정지 | 정지 |
| KEY2 | 다음 곡 | |
| KEY3 | 이전 곡 | |
| KEY4 | 음량 +5 | |
| KEY5 | 음량 −5 | |
| KEY6 | 마지막 기기로 재연결 | BT 기기 탐색 |

LED(GPIO22): 켜짐=스트리밍, 느린 깜빡임=연결만, 빠른 깜빡임=미연결.

## 설계에서 조심한 것들

- **`btStart()` 를 써야 한다.** `esp_bt_controller_init()` 직접 호출은 상태가 IDLE 로
  읽히는데도 `ESP_ERR_INVALID_STATE` 를 돌려준다.
- **큰 버퍼는 전부 PSRAM.** 내부 DRAM 은 320KB 뿐이고 BT+WiFi 스택이 먹는다.
  `MALLOC_CAP_INTERNAL` 은 IRAM 을 줄 수 있는데 IRAM 은 16비트 접근이 불법이다.
- **SD 는 1비트 모드.** 4비트는 GPIO13 을 먹어 KEY2 와 부딪힌다.
- **LED 핀을 키보다 먼저 설정한다.** 안 하면 GPIO13(KEY2)이 GPIO22 에 끌려 LOW 로 읽힌다.
- **GPIO16/17 은 PSRAM 것이다.** 만지면 즉시 재부팅 루프.
- **음량은 A2DP 콜백에서 곱한다.** 디코더에서 곱하면 링버퍼 깊이(약 1.5초)만큼 늦게 반응한다.
- **링이 절반 찬 뒤에 스트림을 시작한다.** 시작 직후 언더런을 피한다.
- **타입은 `player.h` 에 둔다.** arduino-cli 가 `#include` 직후에 함수 원형을 자동 삽입해서,
  .ino 본문에 정의한 struct 를 인수로 받는 함수는 "does not name a type" 으로 깨진다.
- **`partitions.csv` 의 플래그 칸 뒤에 주석 금지.** arduino-cli 가 주석을 플래그로 읽고 죽는다.

## 다음

1. **이어폰 페어링 → 실제 소리 확인.** 여기까지가 "된다" 고 말할 수 있는 선이다.
2. 가운데 DIP 스위치. 이게 풀리면 3.5mm 잭 출력과 **온보드 마이크 녹음**이 같이 살아난다.
3. 녹음을 지금 당장 하려면 HFP Audio Gateway 로 **이어폰 마이크**를 입력으로 쓰는 길뿐이다
   (`esp_hf_ag_api.h`, mSBC 광대역이면 16kHz 모노 — `cabin_node` 음성 모델이 쓰는 레이트).
4. HTTP 스트리밍 재생(PC/인터넷 라디오 → 보드 → BT). WiFi 와 A2DP 를 동시에 밀어야 해서
   코엑시스턴스 여유를 재고 붙일 일이다.

## 출처

`minimp3.h` — https://github.com/lieff/minimp3 (CC0/퍼블릭 도메인). 구현은
`minimp3_impl.c` 한 곳에서만 펼친다(`MINIMP3_ONLY_MP3`, `MINIMP3_NO_SIMD`).
