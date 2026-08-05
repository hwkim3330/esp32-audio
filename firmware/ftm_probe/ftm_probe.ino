// FTM 탐색 — 이 보드로 2.4GHz "진짜 레이더(거리 측정)" 가 되는가.
//
// 지금까지 한 CSI 센싱은 엄밀히 레이더가 아니다. 레이더는 내가 쏜 신호의 왕복
// 시간으로 거리를 잡는데, CSI 는 AP 가 쏜 것을 받아 채널 변화만 본다 —
// 수동 바이스태틱 센싱이고 거리가 안 나온다(IEEE 802.11bf 가 그렇게 규정한다).
//
// 거리를 재는 표준 경로가 802.11mc FTM(Fine Timing Measurement)이다. 왕복 시간을
// 피코초 단위로 측정해 미터급 거리를 낸다(wifi_ftm_report_entry_t.rtt).
//
// 문제: ESP-IDF 는 헤더를 칩 간 공유하므로 API 가 있어도 하드웨어 지원을 뜻하지 않는다.
// 클래식 ESP32 는 FTM 미지원으로 알려져 있고, 그렇다면 FTM_STATUS_UNSUPPORTED 가 온다.
// 그걸 실측으로 판정한다.
//
// 두 가지를 본다:
//   1. 주변 AP 중 ftm_responder / ftm_initiator 를 광고하는 것이 있는가
//      (있어야 초기자 역할을 시험할 상대가 있다)
//   2. 이 칩이 FTM 세션을 열 수 있는가 — 상태 코드가 답한다

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_event.h>

static volatile bool     got_report = false;
static volatile uint8_t  rep_status = 0xFF;
static volatile uint32_t rep_rtt_ps = 0;
static volatile uint32_t rep_dist_cm = 0;
static volatile uint8_t  rep_entries = 0;

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base != WIFI_EVENT || id != WIFI_EVENT_FTM_REPORT) return;
    wifi_event_ftm_report_t *r = (wifi_event_ftm_report_t *)data;
    rep_status  = (uint8_t)r->status;
    rep_rtt_ps  = r->rtt_raw;
    rep_dist_cm = r->dist_est;
    rep_entries = r->ftm_report_num_entries;
    got_report = true;
}

static const char *ftm_status_name(uint8_t s)
{
    switch (s) {
    case FTM_STATUS_SUCCESS:        return "SUCCESS — 거리 측정 성공";
    case FTM_STATUS_UNSUPPORTED:    return "UNSUPPORTED — 하드웨어가 FTM 을 지원하지 않음";
    case FTM_STATUS_CONF_REJECTED:  return "CONF_REJECTED — 상대가 설정을 거부";
    case FTM_STATUS_NO_RESPONSE:    return "NO_RESPONSE — 상대가 응답 없음";
    case FTM_STATUS_FAIL:           return "FAIL";
    case FTM_STATUS_NO_VALID_MSMT:  return "NO_VALID_MSMT — 유효 측정 없음";
    case FTM_STATUS_USER_TERM:      return "USER_TERM";
    default:                        return "(알 수 없음)";
    }
}

void setup()
{
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== FTM 탐색 (802.11mc 거리 측정) ===");
    Serial.println("CSI 센싱은 채널 변화만 본다. FTM 은 왕복시간으로 실제 거리를 낸다.");
    Serial.println("헤더에 API 가 있어도 하드웨어 지원과는 별개다 — 실측으로 판정한다.\n");

    WiFi.mode(WIFI_MODE_STA);
    esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_FTM_REPORT,
                                        &on_wifi, nullptr, nullptr);

    // ── 1) 스캔. 스캔 레코드의 ftm_responder / ftm_initiator 비트를 본다.
    Serial.println("[1] AP 스캔 — FTM 광고 여부 확인");
    wifi_scan_config_t sc = {};
    sc.show_hidden = true;
    if (esp_wifi_scan_start(&sc, true) != ESP_OK) {
        Serial.println("  스캔 실패");
    }
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n > 24) n = 24;
    wifi_ap_record_t *recs =
        (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * (n ? n : 1));
    if (recs && n) esp_wifi_scan_get_ap_records(&n, recs);
    Serial.printf("  AP %u개\n", n);

    int ftm_resp_idx = -1;
    for (int i = 0; i < n; i++) {
        const wifi_ap_record_t *a = &recs[i];
        const bool fr = a->ftm_responder, fi = a->ftm_initiator;
        if (fr && ftm_resp_idx < 0) ftm_resp_idx = i;
        Serial.printf("  %-24.24s ch%2d %4ddBm  FTM: resp=%d init=%d%s\n",
                      (const char *)a->ssid, a->primary, a->rssi, fr, fi,
                      fr ? "   <<< 응답자" : "");
    }

    // ── 2) FTM 세션 시도. 상대가 없어도 상태 코드로 칩 지원 여부는 갈린다.
    Serial.println("\n[2] FTM 세션 시도");
    wifi_ftm_initiator_cfg_t cfg = {};
    cfg.frm_count = 16;         // 프레임 수 (많으면 정확, 느림)
    cfg.burst_period = 2;       // 100ms 단위
    if (ftm_resp_idx >= 0) {
        memcpy(cfg.resp_mac, recs[ftm_resp_idx].bssid, 6);
        cfg.channel = recs[ftm_resp_idx].primary;
        Serial.printf("  상대: %-20.20s ch%d\n",
                      (const char *)recs[ftm_resp_idx].ssid, cfg.channel);
    } else if (n > 0) {
        // FTM 광고 AP 가 없으면 가장 센 AP 로 시도한다. 성공은 기대하지 않지만,
        // 칩이 아예 미지원이면 UNSUPPORTED 가 즉시 온다 — 그게 알고 싶은 것이다.
        int best = 0;
        for (int i = 1; i < n; i++) if (recs[i].rssi > recs[best].rssi) best = i;
        memcpy(cfg.resp_mac, recs[best].bssid, 6);
        cfg.channel = recs[best].primary;
        Serial.printf("  FTM 광고 AP 없음 — 가장 센 AP 로 시도 (%-16.16s ch%d)\n",
                      (const char *)recs[best].ssid, cfg.channel);
    } else {
        Serial.println("  AP 가 없어 시도할 상대가 없다.");
    }

    if (n > 0) {
        const esp_err_t e = esp_wifi_ftm_initiate_session(&cfg);
        Serial.printf("  esp_wifi_ftm_initiate_session() = %s (0x%x)\n",
                      esp_err_to_name(e), e);
        if (e == ESP_OK) {
            const uint32_t t0 = millis();
            while (!got_report && millis() - t0 < 6000) delay(50);
            if (got_report) {
                Serial.printf("  리포트: status=%s\n", ftm_status_name(rep_status));
                Serial.printf("          rtt_raw=%lu ps  거리추정=%lu cm  항목=%u개\n",
                              (unsigned long)rep_rtt_ps,
                              (unsigned long)rep_dist_cm, rep_entries);
            } else {
                Serial.println("  6초 내 리포트 없음");
            }
            esp_wifi_ftm_end_session();
        }
    }

    Serial.println("\n=== 판정 ===");
    if (got_report && rep_status == FTM_STATUS_SUCCESS) {
        Serial.println("이 보드로 802.11mc 거리 측정이 된다. 진짜 레이더에 가장 가까운 경로다.");
    } else if (got_report && rep_status == FTM_STATUS_UNSUPPORTED) {
        Serial.println("클래식 ESP32 는 FTM 하드웨어 지원이 없다.");
        Serial.println("→ 거리 측정은 불가. CSI 기반 수동 센싱(변화 감지)만 가능하다.");
    } else if (got_report) {
        Serial.println("칩은 세션을 열었지만 측정이 안 됐다 — 상대 AP 가 FTM 응답자가 아닐 가능성.");
        Serial.println("→ FTM 지원 AP(또는 ESP32-S3/C3 를 응답자로)와 다시 시험할 여지가 있다.");
    } else {
        Serial.println("리포트가 오지 않았다. 세션 자체가 성립하지 않았다.");
    }
    free(recs);
}

void loop() { delay(1000); }
