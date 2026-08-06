// Gicisky/PICKSMART 296x128 BWR 전자선반라벨 드라이버.
//
// 프로토콜은 atc1441 의 리버스엔지니어링이 정본이다(ATC_GICISKY_ESL).
//   서비스 0xFEF0, CMD 0xFEF1(쓰기+알림), IMG 0xFEF2(쓰기)
//   1) CMD ← 0x01                     → 태그가 0x01 + 파트크기(LE16) 로 답한다
//   2) CMD ← 0x02 + 총길이(LE32) + 0,0,0
//   3) CMD ← 0x03                     → 전송 시작
//   4) IMG ← [파트번호 LE32][데이터]   → 태그가 0x05 로 다음 파트를 요구한다
//   5) 0x05 의 상태가 0x08 이면 완료
//
// 픽셀 배치는 **열 우선**이다. 열 하나가 16바이트(128비트)이고 296열이므로
// 평면당 4736바이트, BW+RED 두 평면으로 9472바이트다.
//
// 참조 구현과 다르게 만든 것:
//  - MAC 을 박아두지 않고 광고에서 찾는다. 태그를 바꿔 끼워도 코드를 안 고친다.
//  - 전송 시간을 재서 돌려준다. "느리다" 를 고치려면 먼저 숫자가 있어야 한다.
//  - 실패 사유를 열거형으로 돌려준다. 참조 구현은 bool 이라 어디서 죽었는지 모른다.
#pragma once

#include <Arduino.h>
#include <Adafruit_GFX.h>

#define ESL_W        296
#define ESL_H        128
#define ESL_BYTES    ((ESL_W) * ((ESL_H) / 8) * 2)   // 9472
#define ESL_MAX_TAG  8

enum EslResult {
    ESL_OK = 0,
    ESL_ERR_CONNECT,       // 연결 자체가 안 됨
    ESL_ERR_SERVICE,       // 0xFEF0 이 없음 → ESL 이 아니다
    ESL_ERR_CHAR,          // FEF1/FEF2 가 없음
    ESL_ERR_NEGOTIATE,     // 0x01 응답 없음
    ESL_ERR_TRANSFER,      // 전송 중 정지·거부
    ESL_ERR_TIMEOUT,
};

const char *esl_result_name(EslResult r);

// 태그 모델 정의. 같은 "2.9인치" 안에도 BW/BWR/BWRY 가 따로 있고, 크기·회전·좌우반전이
// 모델마다 다르다. 표는 eigger/hass-gicisky 의 devices.py 를 옮긴 것이다.
struct EslModel {
    uint16_t id;
    const char *model;
    uint16_t w, h;
    bool  red;           // 적색 평면이 있는가 — 없으면 페이로드가 절반이다
    bool  four_color;    // BWRY. 우리 렌더러는 아직 지원하지 않는다
    bool  mirror_x, mirror_y;
    uint16_t rotation;
    bool  compression;   // 줄 단위 청크 포맷
    bool  compression2;  // 2비트 패킹 + 압축
    float v_max;
};

const EslModel *esl_model(uint16_t id);

struct EslTag {
    uint8_t addr[6];
    char    name[24];
    int8_t  rssi;
    // 광고의 제조사 데이터(회사 ID 0x5053, 5바이트)에서 그대로 읽는다.
    // **연결하지 않고** 모델·전압·펌웨어를 알 수 있다. 참조 구현은 MAC 을 박아두고
    // 모델을 가정했는데, 그러면 BW 태그에 BWR 페이로드를 보내 화면이 깨진다.
    bool     have_mfg;
    uint16_t device_id;
    uint16_t firmware;
    float    volts;
    const EslModel *m;   // 표에 없으면 nullptr
};

// 광고에서 태그를 찾는다. 0xFEF0 을 광고하거나 MAC 이 FF:FF 로 시작하면 후보다.
// 반환: 찾은 개수. BLEDevice::init() 이 먼저 호출돼 있어야 한다.
int esl_scan(EslTag *out, int max_n, uint32_t secs);

// GFXcanvas1 두 장(검정 평면, 적색 평면)을 태그 페이로드로 만든다.
// 반환값이 실제 길이다 — BW 모델이면 적색 평면을 아예 넣지 않는다.
size_t esl_pack(const EslTag &t, const GFXcanvas1 &bw, const GFXcanvas1 &red,
                uint8_t *buf);

// 이 태그에 보낼 페이로드 길이. BW 모델은 적색 평면이 없어 절반이다.
size_t esl_payload_len(const EslTag &t);

// 페이로드를 태그에 올린다. ms_out 에 걸린 시간을 담는다(널 허용).
EslResult esl_upload(const uint8_t addr[6], const uint8_t *payload, size_t len,
                     uint32_t *ms_out, uint32_t *parts_out);
