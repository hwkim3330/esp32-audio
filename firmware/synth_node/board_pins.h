// AI-Thinker ESP32-Audio-Kit (ESP32-A1S) 핀 배치.
//
// ★ 이 파일이 이 프로젝트에서 유일하게 "확인되지 않은 가정" 이 모여 있는 곳이다.
//
// 보드 실크스크린을 읽지 않고 소프트웨어만으로 식별했기 때문에(docs/HARDWARE.md),
// 아래 핀 번호는 널리 알려진 ESP32-A1S Audio Kit v2.2 배치를 따른 것이고
// 리비전에 따라 다를 수 있다. 틀리면 최악의 경우 핀을 태운다.
//
// 그래서 코드가 런타임에 스스로 증명하게 만들었다:
//   cn_board_probe() 가 I2C 를 스캔해서 AC101 이 응답하는지 본다.
//   응답하면 I2C 핀 가정이 맞다는 실측 증거다. 응답하지 않으면 그 자리에서
//   멈추고 어떤 핀을 확인해야 하는지 출력한다 — 조용히 오작동하지 않는다.
//
// 값을 고칠 때는 여기만 고치면 된다.
#pragma once

// ── I2C: AC101 코덱 제어 (가정)
#define CN_PIN_I2C_SDA      33
#define CN_PIN_I2C_SCL      32
#define CN_I2C_FREQ_HZ      100000
#define CN_AC101_ADDR       0x1A   // AC101 7비트 주소

// ── I2S: AC101 오디오 데이터 (가정)
//    ESP32 가 마스터, 코덱이 슬레이브. 마이크와 스피커가 같은 I2S 포트를
//    공유하므로 전이중(full-duplex) 로 열어야 한다 — 재생 중에도 듣기 위해.
#define CN_PIN_I2S_MCLK      0
#define CN_PIN_I2S_BCLK     27
// ★ 2026-09-08 정정: LRCK 와 DOUT 이 서로 뒤바뀌어 있었다.
//
//    이 저장소의 오디오 결론 전부("I2S 가 정확히 0", "오디오는 죽었다")가 이 상태에서
//    나왔다. 워드싱크를 데이터 라인으로, 데이터를 싱크 라인으로 보내면 클럭은 다 흐르고
//    I2C 도 완벽한데 오디오만 안 흐른다 — 관측된 증상과 정확히 일치한다.
//
//    근거: pschatzmann/arduino-audiokit 의 이 보드(AI-Thinker AC101) 정의.
//    그 라이브러리는 이 보드에서 동작이 확인된 것이다.
//      PIN_I2S_AUDIO_KIT_WS 26 / PIN_I2S_AUDIO_KIT_DATA_OUT 25 / DATA_IN 35
#define CN_PIN_I2S_LRCK     26
#define CN_PIN_I2S_DOUT     25     // ESP32 → 코덱
// DIN = GPIO35. 공개 보드 정의와 같다.
//
// 한때 GPIO34 로 정정했다가 되돌렸다. GPIO34 가 "피크 63" 을 내서 신호로 봤지만,
// 그건 플로팅 입력의 노이즈였다. 판별법은 이렇다:
//   RMS 0.0 (깨끗한 0)  = 누군가 그 라인을 실제로 구동한다 → 연결됨, 내용이 무음
//   RMS 1.0 (노이즈)    = 아무도 안 붙었다 → 플로팅
// GPIO35 는 정확히 0.0, GPIO34 는 1.0 이었다. 즉 35 가 연결된 라인이다.
#define CN_PIN_I2S_DIN      35     // 코덱 → ESP32 (마이크)

// ── 파워앰프 인에이블 (가정)
//    ★ 정정: 앰프 인에이블은 GPIO21 이다. GPIO22 는 초록 LED 다.
//    공장 펌웨어가 GPIO22 를 출력으로 구동한 것("GPIO[22]| OutputEn: 1")은 LED 였다.
//    근거는 위와 같다(arduino-audiokit 의 AI-Thinker AC101 보드 정의):
//      PA_ENABLE_GPIO 21 / GREEN_LED_GPIO 22 / BLUE_LED_GPIO 19 / HEADPHONE_DETECT 5
//    GPIO5 는 KEY6 과 겸용이다 — 이어폰을 꽂으면 그 핀이 눌린 것처럼 읽힌다.
#define CN_PIN_PA_ENABLE    21
#define CN_PIN_LED_GREEN    22
#define CN_PIN_LED_BLUE     19
#define CN_PIN_HP_DETECT     5

// ── 터치/버튼 6개 (가정)
//    공장 펌웨어는 이들을 "정전식 터치" 로 초기화했다(Touch_Pad 심볼).
//    보드에 물리 택트 버튼도 있는 것으로 알려져 있어, 실제 배선은 미확인이다.
//    cn_board_probe() 는 이 핀들을 건드리지 않는다 — 확인 전엔 읽기만 한다.
#define CN_PIN_KEY1          19    // 이전
#define CN_PIN_KEY2          23    // 모드
#define CN_PIN_KEY3         18     // 다음
#define CN_PIN_KEY4          5     // 정지/재생  ※ GPIO5 는 부팅 스트래핑 핀이다
#define CN_PIN_KEY5         36     // 볼륨 +    ※ 입력 전용 핀
#define CN_PIN_KEY6         13     // 볼륨 −

// ── 쓰지 말아야 하는 핀
//    A1S 모듈은 PSRAM 배선에 GPIO16/17 을 점유하는 계열로 보인다(미확인).
//    PSRAM 이 실제로 동작하는 것은 부팅 로그로 확인됐으므로(메모리 테스트 통과),
//    이 두 핀은 어떤 용도로도 쓰지 않는다.
#define CN_PIN_RESERVED_PSRAM_1  16
#define CN_PIN_RESERVED_PSRAM_2  17

// SD 카드는 쓰지 않는다 — 이 데모의 전제다.
// 응답 음성은 플래시(핵심 문구, ADPCM) + PSRAM 캐시(태블릿이 보내준 것)로 해결한다.
