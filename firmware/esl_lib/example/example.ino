// Gicisky BLE 전자종이 태그에 그림 올리기 — 최소 예제.
//
// 아무 ESP32 에서 돈다. 필요한 것:
//   - Arduino ESP32 코어 (Bluedroid BLE. NimBLE 아니다 — 이유는 README 참고)
//   - Adafruit GFX Library
//   - U8g2_for_Adafruit_GFX  (한글·다양한 폰트를 쓸 때만)
//
// 태그 MAC 을 코드에 적지 않는다. 광고에서 모델까지 읽어오므로 태그를 바꿔 끼워도
// 코드를 안 고친다.

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <BLEDevice.h>
#include <esp_heap_caps.h>

#include "esl_bwr.h"

static GFXcanvas1 *cbw = nullptr, *cred = nullptr;
static uint8_t    *buf = nullptr;
static U8G2_FOR_ADAFRUIT_GFX u8g2;
static EslTag      tags[ESL_MAX_TAG];
static int         n = 0;
static uint32_t    cycle = 0;

static void render(const EslTag &t)
{
    // 두 평면의 극성이 **반대**다. BW: 1=흰색, RED: 1=적색 없음.
    // 적색 평면을 0 으로 채우면 화면 전체가 빨개진다.
    cbw->fillScreen(1);
    cred->fillScreen(1);

    u8g2.begin(*cbw);
    u8g2.setFontMode(1);
    u8g2.setForegroundColor(0);      // BW 평면에서 0 = 검정
    u8g2.setBackgroundColor(1);

    u8g2.setFont(u8g2_font_helvB14_tf);
    u8g2.setCursor(4, 20);
    u8g2.print("ESL demo");
    cbw->drawFastHLine(0, 26, ESL_W, 0);

    u8g2.setFont(u8g2_font_helvR12_tf);
    int y = 46;
    u8g2.setCursor(4, y); u8g2.printf("%s", t.name[0] ? t.name : "(no name)"); y += 18;
    u8g2.setCursor(4, y);
    u8g2.printf("%s   %.2f V", t.m ? t.m->model : "unknown model", t.volts); y += 18;
    u8g2.setCursor(4, y); u8g2.printf("refresh #%lu", (unsigned long)cycle);

    // **화면에는 언제 기준인지 적는다.** 전자종이는 보드가 죽어도 화면이 남으므로,
    // 시각이 없으면 멈춘 화면이 살아 있는 것처럼 보인다.
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.setCursor(4, 124);
    u8g2.printf("drawn at uptime %lus", (unsigned long)(millis() / 1000));
}

void setup()
{
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== ESL 최소 예제 ===");

    // 296x128 캔버스 두 장(각 4736B)과 페이로드(9472B)는 PSRAM 으로.
    // PSRAM 이 없는 보드면 MALLOC_CAP_DEFAULT 로 바꿔도 들어간다(내부 DRAM 19KB).
    buf  = (uint8_t *)heap_caps_malloc(ESL_BYTES, MALLOC_CAP_SPIRAM);
    if (!buf) buf = (uint8_t *)malloc(ESL_BYTES);
    cbw  = new GFXcanvas1(ESL_W, ESL_H);
    cred = new GFXcanvas1(ESL_W, ESL_H);
    if (!buf || !cbw->getBuffer() || !cred->getBuffer()) {
        Serial.println("버퍼 할당 실패"); while (1) delay(1000);
    }

    BLEDevice::init("");
    // setMTU 는 부르지 않는다. 태그가 0x01 응답으로 파트 크기를 직접 알려준다.

    n = esl_scan(tags, ESL_MAX_TAG, 8);
    Serial.printf("태그 %d대\n", n);
    for (int i = 0; i < n; i++)
        Serial.printf("  #%d %02X:%02X:%02X:%02X:%02X:%02X %-14s %ddBm  %s  %.2fV  fw%04X\n",
                      i, tags[i].addr[0], tags[i].addr[1], tags[i].addr[2],
                      tags[i].addr[3], tags[i].addr[4], tags[i].addr[5],
                      tags[i].name, tags[i].rssi,
                      tags[i].m ? tags[i].m->model : "모르는 모델",
                      tags[i].volts, tags[i].firmware);

    // 부팅 후 첫 전송은 파트 크기가 20 으로 협상돼 592파트/21초가 걸린다.
    // 여기서 미리 달궈 그 손해를 사람이 볼 첫 화면이 아니라 버리는 세션에 떠넘긴다.
    if (n) {
        Serial.print("BLE 달구는 중... ");
        Serial.println(esl_warmup(tags[0].addr) ? "됨" : "실패(무해)");
    }
}

void loop()
{
    for (int i = 0; i < n; i++) {
        render(tags[i]);
        const size_t len = esl_pack(tags[i], *cbw, *cred, buf);

        uint32_t ms = 0, parts = 0;
        const EslResult r = esl_upload(tags[i].addr, buf, len, &ms, &parts);
        Serial.printf("#%d %-14s %-16s %lu파트 %lums\n", i, tags[i].name,
                      esl_result_name(r), (unsigned long)parts, (unsigned long)ms);
        delay(300);
    }
    cycle++;

    // **여기가 핵심이다.** 실제 제품에서는 이 delay 를 쓰지 말 것 —
    // 타이머로 갱신하면 배터리가 며칠이면 끝난다. 내용이 바뀌었을 때만 부를 것.
    // (같은 내용이면 esl_upload 가 ESL_SKIPPED 로 돌려주므로 헛되게 굽지는 않는다.)
    Serial.println("— 60초 대기 (실제 제품에서는 사건이 있을 때만 부를 것) —\n");
    delay(60000);
}
