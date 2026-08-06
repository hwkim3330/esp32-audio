// ESL 태그를 이 보드의 화면으로 쓴다.
//
// ESP32-Audio-Kit 에는 화면이 없다. 그동안 CSI 채널 일치율도, 추론 지연도, 대역
// 점수도 전부 시리얼로만 나왔다 — 즉 PC 가 붙어 있어야 읽을 수 있었고, 그건
// "PC 에서는 하지 마라" 는 전제와 어긋난다.
//
// Gicisky/PICKSMART BLE 전자선반라벨이 그 구멍을 메운다. 실측으로 3대가 잡혔다:
//   FF:FF:92:95:75:78  NEMR92957578  -46dBm
//   FF:FF:92:95:73:04  NEMR92957304  -50dBm
//   FF:FF:92:95:95:81  NEMR92959581  -58dBm
// 전부 0xFEF0 을 광고한다. MAC 을 박아두지 않고 매번 광고에서 찾는다.
//
// 이 스케치가 확인하는 것: **정말 화면에 뜨는가.** 뜨면 다음은 multiband_sense 의
// 센싱 값을 여기에 실어 보드가 스스로 자기 상태를 보여주게 만든다.

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <BLEDevice.h>
#include <esp_heap_caps.h>

#include "esl_bwr.h"

static GFXcanvas1 *cbw = nullptr, *cred = nullptr;
static uint8_t    *payload = nullptr;
static U8G2_FOR_ADAFRUIT_GFX u8g2;

// 화면에 실을 값. 센싱 코드와 화면 코드를 이 구조체 하나로만 잇는다 —
// 그래야 multiband_sense 에 합칠 때 화면 코드를 안 고친다.
struct Live {
    float    band, thresh;
    int      hz[3], pkt[3], n_sc;
    float    match_pct;
    uint32_t infer_ms;
    int      ble_dev, ble_hot;
    uint32_t ble_adv;
    char     ap_name[20];
    int      ap_rssi;
    int      n_tags;
};
// 아직 센싱과 합치지 않았으므로 실측으로 확인된 값을 넣는다. 지어낸 값은 넣지 않는다
// — 화면에 뜬 숫자가 어디서 왔는지 모르게 되는 순간 화면이 쓸모없어진다.
static Live live = {
    /*band*/      1.2f,  /*thresh*/ 3.0f,
    /*hz*/       {10, 11, 10},
    /*pkt*/      {129, 128, 56},
    /*n_sc*/      64,
    /*match_pct*/ 53.1f, /*infer_ms*/ 289,
    /*ble_dev*/   39,    /*ble_hot*/ 5,
    /*ble_adv*/   154,
    /*ap_name*/   "scan",  /*ap_rssi*/ -51,
    /*n_tags*/    0,
};

// MAC 을 박아둔 표는 없앴다. 모델을 광고에서 읽으므로 태그를 바꿔 끼워도 된다.
// 그래도 **적색에 정보를 의존하지 않는다** — 표에 없는 모델이 오면 적색 유무를
// 모르고, 그때도 화면은 읽혀야 한다. 적색은 강조로만 쓴다.

static EslTag tags[ESL_MAX_TAG];
static int n_tags = 0;
static uint32_t cycle = 0;

// 태그마다 다른 내용을 그린다. 4대를 같은 화면으로 채우면 4대인 의미가 없다.
static void render(int slot, const EslTag &t)
{
    // 두 평면의 극성이 **반대**다. esl_pack 이 BW 는 그대로, RED 는 반전해 보낸다.
    //   BW  : 1 = 흰색,     0 = 검정
    //   RED : 1 = 적색 없음, 0 = 적색
    // 처음에 적색 평면을 0 으로 채웠더니 화면 전체가 빨개지고 글자만 흰색이 됐다.
    cbw->fillScreen(1);
    cred->fillScreen(1);

    u8g2.begin(*cbw);
    u8g2.setFontMode(1);
    u8g2.setFontDirection(0);
    u8g2.setForegroundColor(0);   // 검정
    u8g2.setBackgroundColor(1);

    // ── 머리말: 어느 태그인지. 슬롯 번호가 곧 이 태그의 역할이다.
    u8g2.setFont(u8g2_font_helvB14_tf);
    u8g2.setCursor(4, 18);
    u8g2.printf("CABIN NODE  #%d", slot);

    cbw->drawFastHLine(0, 24, ESL_W, 0);

    // ── 본문. 네 대를 같은 화면으로 채우면 네 대인 의미가 없으므로 역할을 나눈다.
    //    슬롯은 RSSI 순으로 정해진다(esl_scan 이 세기 순으로 정렬한다) — 가장 가까운
    //    태그가 가장 자주 보게 될 화면이므로 거기에 지금 상태를 둔다.
    u8g2.setFont(u8g2_font_helvR12_tf);
    int y = 44;
    switch (slot) {
    case 0:   // 지금 상태 — 센싱이 무엇을 보고 있나
        u8g2.setCursor(6, y);
        u8g2.printf("band score  %.1f   thresh %.1f", live.band, live.thresh); y += 18;
        u8g2.setCursor(6, y);
        u8g2.printf("CSI  %d/%d/%d Hz   sc %d",
                    live.hz[0], live.hz[1], live.hz[2], live.n_sc); y += 18;
        u8g2.setCursor(6, y);
        u8g2.printf("model  ch match %.0f%%  %lums",
                    live.match_pct, (unsigned long)live.infer_ms); y += 18;
        u8g2.setCursor(6, y);
        u8g2.printf("BLE  %d dev   %lu adv", live.ble_dev, (unsigned long)live.ble_adv);
        break;
    case 1:   // 이 보드에서 무엇이 살아 있고 무엇이 죽었나
        u8g2.setCursor(6, y);  u8g2.print("ESP32-A1S  4MB flash + 4MB PSRAM"); y += 18;
        u8g2.setCursor(6, y);  u8g2.print("AC101 / I2S  ->  DEAD (silent)"); y += 18;
        u8g2.setCursor(6, y);  u8g2.print("2.4GHz sensing  ->  ALIVE"); y += 18;
        u8g2.setCursor(6, y);  u8g2.print("display  ->  this BLE tag");
        break;
    case 2:   // 전파 이웃 — 무엇이 주변에 있나
        u8g2.setCursor(6, y);
        u8g2.printf("WiFi ch1/6/11  %d/%d/%d pkt",
                    live.pkt[0], live.pkt[1], live.pkt[2]); y += 18;
        u8g2.setCursor(6, y);
        u8g2.printf("AP  %s  %d dBm", live.ap_name, live.ap_rssi); y += 18;
        u8g2.setCursor(6, y);
        u8g2.printf("BLE dev %d   hot %d (>2 sigma)", live.ble_dev, live.ble_hot); y += 18;
        u8g2.setCursor(6, y);
        u8g2.printf("ESL tags  %d found", live.n_tags);
        break;
    default:  // 이 태그 자신 — 어느 태그가 어느 역할인지 알아야 배치할 수 있다
        u8g2.setCursor(6, y);  u8g2.printf("%s", t.name[0] ? t.name : "(no name)"); y += 18;
        u8g2.setCursor(6, y);
        u8g2.printf("%02X:%02X:%02X:%02X:%02X:%02X   %d dBm",
                    t.addr[0], t.addr[1], t.addr[2], t.addr[3], t.addr[4], t.addr[5],
                    t.rssi); y += 18;
        u8g2.setCursor(6, y);  u8g2.printf("296x128 BWR   %d bytes/frame", ESL_BYTES); y += 18;
        u8g2.setCursor(6, y);  u8g2.print("svc FEF0  cmd FEF1  img FEF2");
        break;
    }

    // ── 회차는 **흑백에** 찍는다. 적색이 안 나오는 태그가 있으므로 여기에만 두면
    //    그 태그에서는 갱신되는지 알 수 없다.
    u8g2.begin(*cbw);
    u8g2.setForegroundColor(0);
    u8g2.setBackgroundColor(1);
    u8g2.setFont(u8g2_font_helvB14_tf);
    {
        char nb[24];
        snprintf(nb, sizeof nb, "#%lu", (unsigned long)cycle);
        u8g2.setCursor(ESL_W - u8g2.getUTF8Width(nb) - 6, 18);
        u8g2.print(nb);
    }

    // ── 적색 평면: 강조만. 적색이 없어도 화면은 완전히 읽힌다.
    if (t.m && !t.m->red) return;        // BW 모델이면 적색을 아예 안 그린다
    u8g2.begin(*cred);
    u8g2.setFontMode(1);
    u8g2.setForegroundColor(0);   // 적색 평면에서는 0 이 "칠한다" 다
    u8g2.setBackgroundColor(1);
    // 슬롯 0(가장 가까운 태그)에서 대역점수가 임계를 넘으면 그것만 빨갛게 한다.
    // 빨강이 아무 때나 켜지면 강조가 아니게 된다.
    u8g2.setFont(u8g2_font_helvB14_tf);
    if (slot == 0 && live.band >= live.thresh) {
        u8g2.setCursor(6, 118);
        u8g2.print("MOTION");
        cred->drawFastHLine(0, 122, ESL_W, 0);
        cred->drawFastHLine(0, 123, ESL_W, 0);
    } else {
        // 평상시에는 얇은 밑줄 하나만. "살아 있다" 는 표시로 충분하다.
        cred->drawFastHLine(0, 126, ESL_W, 0);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== ESL 화면 ===");
    Serial.println("이 보드에 없는 화면을 BLE 전자선반라벨로 대신한다.");

    // 캔버스 두 장 + 페이로드는 내부 DRAM 에 두면 아깝다. PSRAM 으로 보낸다.
    // (296x128 흑백 캔버스 하나가 4736바이트, 페이로드가 9472바이트)
    payload = (uint8_t *)heap_caps_malloc(ESL_BYTES, MALLOC_CAP_SPIRAM);
    cbw  = new GFXcanvas1(ESL_W, ESL_H);
    cred = new GFXcanvas1(ESL_W, ESL_H);
    if (!payload || !cbw || !cred || !cbw->getBuffer() || !cred->getBuffer()) {
        Serial.println("버퍼 할당 실패. 중단."); while (1) delay(1000);
    }
    Serial.printf("[메모리] PSRAM 여유 %u KB, 내부 DRAM 여유 %u KB\n",
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));

    BLEDevice::init("CABIN-NODE");
    // setMTU 는 쓰지 않는다. 태그가 0x01 응답으로 파트 크기를 직접 알려주므로
    // 우리가 MTU 를 키울 이유가 없고, 참조 구현도 쓰지 않는다.
}

void loop()
{
    Serial.println("\n[스캔] ESL 태그 찾는 중 (8초)...");
    n_tags = esl_scan(tags, ESL_MAX_TAG, 8);
    Serial.printf("[스캔] %d대\n", n_tags);
    for (int i = 0; i < n_tags; i++)
        Serial.printf("   #%d %02X:%02X:%02X:%02X:%02X:%02X %s %ddBm  %s  %.1fV  fw%04X\n",
                      i, tags[i].addr[0], tags[i].addr[1], tags[i].addr[2],
                      tags[i].addr[3], tags[i].addr[4], tags[i].addr[5],
                      tags[i].name, tags[i].rssi,
                      tags[i].m ? tags[i].m->model
                                : (tags[i].have_mfg ? "모르는 모델" : "제조사데이터 없음"),
                      tags[i].volts, tags[i].firmware);
    live.n_tags = n_tags;
    if (!n_tags) { Serial.println("태그 없음. 30초 후 재시도."); delay(30000); return; }

    for (int i = 0; i < n_tags; i++) {
        render(i, tags[i]);
        const size_t len = esl_pack(tags[i], *cbw, *cred, payload);

        uint32_t ms = 0, parts = 0;
        Serial.printf("[전송] #%d %s  %u바이트 ... ", i, tags[i].name, (unsigned)len);
        const EslResult r = esl_upload(tags[i].addr, payload, len, &ms, &parts);
        if (r == ESL_OK)
            Serial.printf("성공  %lu파트 %lums (%.1f KB/s)\n",
                          (unsigned long)parts, (unsigned long)ms,
                          ms ? (ESL_BYTES / 1024.0f) / (ms / 1000.0f) : 0.0f);
        else
            Serial.printf("실패 — %s (%lums)\n", esl_result_name(r), (unsigned long)ms);
        delay(500);
    }

    cycle++;
    Serial.println("[대기] 60초");
    delay(60000);
}
