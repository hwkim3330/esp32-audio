// 스케치가 쓰는 타입들.
//
// 왜 헤더로 뺐나: arduino-cli 는 .ino 에 함수 원형을 자동 삽입하는데, 그 삽입 위치가
// #include 직후다. 그래서 .ino 본문에서 정의한 struct 를 인수로 받는 함수는
// "'Track' does not name a type" 로 깨진다. 타입은 헤더에 둬야 한다.
#pragma once
#include <Arduino.h>
#include <vector>
#include <esp_gap_bt_api.h>

enum PState { ST_IDLE, ST_PLAY, ST_PAUSE };

// 출력 경로. "둘 다 동시" 는 없다 — I2S 와 A2DP 는 클록 도메인이 달라서
// 링버퍼 소비자가 둘이 되면 드리프트를 흡수할 곳이 없다.
enum OutMode { OUT_BT, OUT_JACK };

// 보드의 역할. Bluedroid 는 A2DP 소스와 싱크를 동시에 못 든다 —
// 바꿀 때는 NVS 에 적고 재부팅한다(역할 전환 중 상태가 꼬이는 걸 피한다).
//   ROLE_PLAYER : 파일(SD/플래시) → 디코드 → 잭 또는 BT 이어폰
//   ROLE_SINK   : 맥·폰이 이 보드에 연결 → 그 소리가 잭으로 나온다 (BT 스피커)
enum Role { ROLE_PLAYER, ROLE_SINK };

// 실시간 처리 모드. 웹 UI 의 세 버튼과 1:1 로 대응한다.
//   PM_NORMAL  : 처리 없음 — 음악 재생/싱크(기존 동작 그대로)
//   PM_HEARING : 보청기 — 빔포밍 + 적응 잡음억제 + 말소리 EQ + 컴프레서
//   PM_NR      : 잡음억제만 — 같은 빔포밍/NLMS, EQ·컴프레서 없이 평탄하게
enum ProcMode { PM_NORMAL, PM_HEARING, PM_NR };

struct Track {
    String name;
    String path;
    size_t size;
    bool   sd;      // true = SD 카드, false = 내장 플래시(LittleFS)
};

struct BtDev {
    esp_bd_addr_t bda;
    char          name[64];
    int           rssi;
    uint8_t       major;   // CoD major class. 4 = 오디오/비디오
};

struct Key {
    int      pin;
    uint32_t down_at;
    bool     prev;
    bool     longsent;
};
