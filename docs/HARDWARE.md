# 하드웨어

## 1. 식별 경로

보드에 아무 라벨도 안 보고, 소프트웨어만으로 특정한 순서다. 같은 상황이 또 오면 이 순서를 쓰면 된다.

### 1단계 — USB 로 계열 좁히기

```
$ lsusb
Bus 001 Device 024: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
```

CP2102 → `/dev/ttyUSB0`. ESP32-S3/C3 계열 개발보드는 보통 네이티브 USB 라서 `/dev/ttyACM*`
로 잡힌다. 즉 이 단계에서 이미 **구형 ESP32(D0WD 계열)** 쪽으로 기운다.

### 2단계 — 칩 자체를 읽기

```
$ esptool --port /dev/ttyUSB0 flash-id
Chip type:          ESP32-D0WD (revision v1.0)
Features:           Wi-Fi, BT, Dual Core + LP Core, 240MHz, Vref calibration in eFuse
Crystal frequency:  40MHz
MAC:                4c:11:ae:f5:a5:18
Manufacturer: ef      Device: 4016      Detected flash size: 4MB
```

### 3단계 — 부팅 로그가 정체를 다 말해준다

`esptool` 은 리셋만 하고 앱을 안 띄우니, DTR/RTS 를 직접 흔들어 하드웨어 리셋 후 캡처했다.
(수동으로 그냥 듣기만 하면 0바이트다 — 이 펌웨어는 유휴 상태에서 아무것도 안 찍는다.)

로그에서 드러난 것:

```
spiram: PSRAM initialized, cache is in low/high (2-core) mode.
spiram: SPI SRAM memory test OK
spiram: Adding pool of 4096K of external SPI memory to heap allocator
esp32_bt_sd_v1.2
USER_PARTION_2
SDCARD_BT_CONTROL_EXAMPLE: [1.1] Start SD card peripheral
PERIPH_SDCARD: no sdcard detect
SDCARD_BT_CONTROL_EXAMPLE: [1.4] Initialize Touch peripheral
SDCARD_BT_CONTROL_EXAMPLE: [3.1] Get Bluetooth stream
```

→ PSRAM 4MB 실장 + SD 슬롯 + 오디오 코덱 + 터치키 + BT 오디오. **오디오 보드다.**

### 4단계 — 어느 벤더인지 (문자열)

4MB 를 덤프해서 `strings` 로 훑었다.

```
$ strings -n 4 factory_..._4MB.bin | grep -oiE "ac101|es8388|audio-kit|lyrat"
AC101      (7회)
Audio-Kit  (5회)
ES8388     (1회)
```

- **AC101** = AI-Thinker ESP32-A1S 계열 코덱. Espressif LyraT 는 ES8388.
  (ES8388 문자열 1개는 ESP-ADF 코덱 드라이버 테이블에 이름만 들어간 것)
- 결정적으로 빌드 경로가 그대로 남아 있다: `Aithnker_ESP32-Audio-Kit`
  (벤더 오타 `Aithnker` 포함 — AI-Thinker 원본 예제 그대로다)

---

## 2. 확인된 주변장치

| 주변장치 | 근거 |
| --- | --- |
| AC101 오디오 코덱 (I2C 제어 + I2S 데이터) | `AC101_init`, `AC101_DRIVER`, `AC101.c` 경로, `i2c_set_pin` |
| microSD (SPI 또는 SDMMC) | `sdcard_init`, `/sdcard`, `no sdcard detect` |
| 정전식 터치키 6개 | `Touch_Pad`, `touch_value`, `[1.4] Initialize Touch peripheral` |
| 마이크 입력 / 녹음 경로 | `record`, `recoder_task`, `line_in`, ADC/DAC 문자열 |
| DAC 출력 (헤드폰/스피커) | AC101 + `i2s_stream` |
| GPIO22 를 출력으로 구동 | 부팅 로그 `GPIO[22]| InputEn: 0| OutputEn: 1` |

공장 펌웨어가 키 6개를 **정전식 터치**로 초기화하는 건 확실하다. 다만 이 보드에는 물리 택트
버튼 KEY1~KEY6 도 있는 것으로 알려져 있어서, 펌웨어가 LyraT 예제를 그대로 가져오며 터치
페리페럴을 초기화한 것일 수 있다. **어느 쪽이 실제로 배선돼 있는지는 미확인.**

---

## 3. 미확인 항목 — 실물을 봐야 하는 것들

소프트웨어로는 여기까지가 한계다. 아래는 추측하면 배선을 잘못 태울 수 있어 일부러 비워 뒀다.

### 3.1 가운데 DIP 스위치 5개

**정체:** 여러 기능이 공유하는 GPIO 를 microSD 쪽으로 붙일지 다른 쪽(JTAG/키)으로 붙일지
갈라주는 **모드 선택 물리 점퍼**다. 버튼이 아니다.

**왜 중요한가:** 카드를 넣었는데도 `no sdcard detect` 가 뜨면 1순위 용의자가 이 스위치다.

**왜 매핑을 안 적었나:** 스위치 1~5 의 개별 핀 매핑이 **리비전마다 다르다** (v2.2 / A247 /
B 계열). 확정하는 방법 세 가지:

1. DIP 옆 실크스크린 읽기 — v2.2 는 보통 기능이 인쇄돼 있다
2. PCB 에 찍힌 버전 문자열을 확인해 해당 회로도로 매핑
3. GPIO 프로브 스케치를 굽고 스위치를 하나씩 토글해 어느 핀이 바뀌는지 실측
   (가장 확실하지만 공장 펌웨어가 지워진다 — 백업은 있음)

### 3.2 커넥터 구성

micro-USB / microSD 슬롯은 확인됨(각각 시리얼·SD 로 동작 중). 그 외 3.5mm 잭, 스피커
JST, 배터리 커넥터, 확장 헤더의 유무·위치는 **소프트웨어로 확인 불가.** 실물 확인 필요.

### 3.3 PSRAM 이 점유하는 GPIO

PSRAM 이 4MB 실장돼 동작하는 것은 확실하다(메모리 테스트 통과). 다만 A1S 모듈이 PSRAM
배선에 **GPIO16/17 을 점유하는 계열인지는 미확인.** WROVER 계열이 그 두 핀을 쓰기 때문에
가능성이 높고, 이게 사실이면 다음이 문제가 된다:

> PLEOS `path_display_node.ino` 는 ST7789 디스플레이의 DC 핀을 `kTftDc = 16` 으로 쓴다.
> GPIO16 이 PSRAM 에 묶여 있으면 그 코드를 이 보드에 그대로 못 올린다.

이것도 3.1 과 같이 실물/데이터시트로 확정해야 한다.

---

## 4. 화면을 붙이려면

이 보드는 코덱+SD+PSRAM 이 핀을 많이 먹어서 남는 GPIO 가 적다. 그래서 사실상 깔끔한
경로가 하나뿐이다:

**AC101 코덱이 I2C 버스에 달려 있으니, SSD1306 OLED 를 같은 SDA/SCL 에 병렬로 물리면
GPIO 를 하나도 더 안 쓴다.** (펌웨어에 `i2c_set_pin`, `i2c_num` 심볼 존재로 I2C 사용은
확인됨. 실제 SDA/SCL 핀 번호는 컴파일된 정수라 문자열로는 안 나온다 — 미확인.)

SPI TFT(ST7789 등)는 위 3.3 의 GPIO16 문제와 SD/I2S 핀 경쟁 때문에 권장하지 않는다.
