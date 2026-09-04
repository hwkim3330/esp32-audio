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
