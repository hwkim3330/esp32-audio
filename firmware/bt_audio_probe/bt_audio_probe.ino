// BT 오디오 탐색 — 죽은 아날로그 경로를 블루투스로 우회할 수 있는가.
//
// 이 보드의 I2S/AC101 경로는 실기에서 안 살아났다(firmware/loopback_test/README.md).
// 그런데 A2DP 는 그 경로를 지나가지 않는다 — 앱이 PCM 을 만들어 블루투스로 바로 보낸다.
// 공장 펌웨어가 A2DP 싱크로 동작했으니 BT 자체는 확실히 살아 있다.
//
// 여기서 확인하는 것:
//   1. BT 컨트롤러/Bluedroid 가 올라오는가
//   2. 주변 BT 기기가 보이는가 — 특히 오디오 기기(CoD major class = Audio/Video)
//   3. A2DP 소스로 연결해 실제로 소리를 보낼 수 있는가
//
// 되면 오디오 출력이 살아난다: 부팅 확인음, 번역 무전기 응답, 상태 알림.
// 지연이 100~200ms 라 건반 연주는 못 하지만 알림·응답은 충분하다.
//
// 입력까지 살리려면 HFP Audio Gateway 를 쓴다(esp_hf_ag_api.h). 이어폰이 헤드셋으로
// 붙으면 그 마이크 오디오가 보드로 들어오고, mSBC 광대역이면 16kHz 모노 — 음성 모델이
// 쓰는 바로 그 샘플레이트다. 이 스케치는 출력(A2DP)만 먼저 확인한다.

#include <Arduino.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_bt_api.h>
#include <esp_a2dp_api.h>
#include <math.h>

static const int   SR       = 44100;   // A2DP 표준 샘플레이트
static const float TONE_HZ  = 660.0f;

static volatile bool  connected = false;
static volatile bool  streaming = false;
static volatile int   n_found = 0;
static volatile int   n_audio = 0;
static esp_bd_addr_t  target = {0};
static bool           have_target = false;
static char           target_name[64] = "(이름 없음)";
static float          phase = 0.0f;

// CoD 의 major device class 를 읽는다. 4 = Audio/Video (이어폰·스피커).
static uint8_t cod_major(uint32_t cod) { return (uint8_t)((cod >> 8) & 0x1F); }

static const char *cod_major_name(uint8_t m)
{
    switch (m) {
    case 1:  return "컴퓨터";
    case 2:  return "휴대폰";
    case 4:  return "오디오/비디오";
    case 5:  return "주변기기";
    case 6:  return "이미징";
    case 7:  return "웨어러블";
    default: return "기타";
    }
}

// ── A2DP 소스가 PCM 을 요구할 때 호출된다. 스테레오 16비트 인터리브를 채운다.
static int32_t a2dp_data_cb(uint8_t *buf, int32_t len)
{
    if (!buf || len < 0) return 0;
    const int32_t n = len / 4;                  // 스테레오 16비트 = 4바이트/프레임
    int16_t *o = (int16_t *)buf;
    const float inc = TONE_HZ / SR;
    for (int32_t i = 0; i < n; i++) {
        phase += inc;
        if (phase >= 1.0f) phase -= 1.0f;
        // 삼각파. 사인보다 싸고 작은 스피커에서 더 잘 들린다.
        const float s = (phase < 0.5f) ? (4.0f * phase - 1.0f) : (3.0f - 4.0f * phase);
        const int16_t v = (int16_t)(s * 8000.0f);
        o[2 * i] = v;
        o[2 * i + 1] = v;
    }
    return n * 4;
}

static void a2dp_cb(esp_a2d_cb_event_t ev, esp_a2d_cb_param_t *p)
{
    switch (ev) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        if (p->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            connected = true;
            Serial.println("[a2dp] 연결됨");
        } else if (p->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            connected = false;
            Serial.printf("[a2dp] 연결 끊김 (사유 %d)\n", p->conn_stat.disc_rsn);
        }
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        streaming = (p->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED);
        Serial.printf("[a2dp] 오디오 상태 %s\n", streaming ? "시작 — 소리가 나야 한다" : "정지");
        break;
    default:
        break;
    }
}

static void gap_cb(esp_bt_gap_cb_event_t ev, esp_bt_gap_cb_param_t *p)
{
    if (ev == ESP_BT_GAP_DISC_RES_EVT) {
        n_found++;
        uint32_t cod = 0;
        char name[64] = "";
        int rssi = 0;
        for (int i = 0; i < p->disc_res.num_prop; i++) {
            esp_bt_gap_dev_prop_t *pr = &p->disc_res.prop[i];
            if (pr->type == ESP_BT_GAP_DEV_PROP_COD) cod = *(uint32_t *)pr->val;
            else if (pr->type == ESP_BT_GAP_DEV_PROP_RSSI) rssi = *(int8_t *)pr->val;
            else if (pr->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
                int l = pr->len < (int)sizeof(name) - 1 ? pr->len : (int)sizeof(name) - 1;
                memcpy(name, pr->val, l);
                name[l] = 0;
            }
        }
        const uint8_t mj = cod_major(cod);
        const bool audio = (mj == 4);
        if (audio) n_audio++;
        Serial.printf("  %02x:%02x:%02x:%02x:%02x:%02x  %4ddBm  %-14s  %s%s\n",
                      p->disc_res.bda[0], p->disc_res.bda[1], p->disc_res.bda[2],
                      p->disc_res.bda[3], p->disc_res.bda[4], p->disc_res.bda[5],
                      rssi, cod_major_name(mj), name[0] ? name : "(이름 없음)",
                      audio ? "   <<< 오디오 기기" : "");
        if (audio && !have_target) {
            memcpy(target, p->disc_res.bda, sizeof(esp_bd_addr_t));
            have_target = true;
            if (name[0]) { strncpy(target_name, name, sizeof target_name - 1); }
        }
    } else if (ev == ESP_BT_GAP_DISC_STATE_CHANGED_EVT) {
        if (p->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED)
            Serial.println("[gap] 탐색 종료");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== BT 오디오 탐색 ===");
    Serial.println("A2DP 는 AC101/I2S 를 지나가지 않는다 — 죽은 아날로그 경로를 우회한다.");
    Serial.println("이어폰을 페어링 모드로 두면(보통 전원 버튼 길게) 목록에 나타난다.\n");

    // Arduino 코어가 제공하는 btStart() 를 쓴다.
    //
    // 직접 esp_bt_controller_init() 을 부르면 상태가 IDLE 로 읽히는데도
    // ESP_ERR_INVALID_STATE 가 돌아온다(실측, 기본 설정·mem_release 제거 후에도 동일).
    // 코어가 컨트롤러 설정을 자기 방식으로 관리하므로 그 진입점을 쓰는 게 맞다.
    Serial.printf("[bt] 컨트롤러 상태 %d → btStart()\n",
                  (int)esp_bt_controller_get_status());
    if (!btStart()) {
        Serial.printf("btStart() 실패. 컨트롤러 상태 %d. 중단.\n",
                      (int)esp_bt_controller_get_status());
        Serial.println("PSRAM 과 BT 클래식이 충돌할 수 있다 — PSRAM=disabled 로도 시험할 것.");
        while (1) delay(1000);
    }
    Serial.printf("[bt] btStart() 성공, 상태 %d\n",
                  (int)esp_bt_controller_get_status());

    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        const esp_err_t e = esp_bluedroid_init();
        Serial.printf("[bt] bluedroid_init = %s\n", esp_err_to_name(e));
        if (e != ESP_OK) { while (1) delay(1000); }
    }
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) {
        const esp_err_t e = esp_bluedroid_enable();
        Serial.printf("[bt] bluedroid_enable = %s\n", esp_err_to_name(e));
        if (e != ESP_OK) { while (1) delay(1000); }
    }

    const uint8_t *mac = esp_bt_dev_get_address();
    Serial.printf("[bt] 컨트롤러 정상, MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    esp_bt_gap_set_device_name("CABIN-NODE");

    esp_bt_gap_register_callback(gap_cb);
    esp_a2d_register_callback(a2dp_cb);
    esp_a2d_source_register_data_callback(a2dp_data_cb);
    if (esp_a2d_source_init() != ESP_OK) {
        Serial.println("A2DP 소스 init 실패. 중단."); while (1) delay(1000);
    }
    Serial.println("[a2dp] 소스 모드 준비");

    // 페어링을 위해 검색 가능 + 연결 가능으로 둔다
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    Serial.println("\n[1] 주변 BT 기기 탐색 (12초)");
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    const uint32_t t0 = millis();
    while (millis() - t0 < 12000) delay(100);
    esp_bt_gap_cancel_discovery();
    delay(300);
    Serial.printf("  기기 %d개, 그중 오디오 %d개\n", n_found, n_audio);

    if (!have_target) {
        Serial.println("\n오디오 기기를 찾지 못했다.");
        Serial.println("이어폰을 페어링 모드로 두고 리셋하면 다시 탐색한다.");
        Serial.println("(이미 다른 기기에 붙어 있으면 검색되지 않는다 — 연결을 끊고 시도)");
        return;
    }

    Serial.printf("\n[2] A2DP 연결 시도: %s\n", target_name);
    if (esp_a2d_source_connect(target) != ESP_OK) {
        Serial.println("  connect 호출 실패");
        return;
    }
    const uint32_t t1 = millis();
    while (!connected && millis() - t1 < 10000) delay(100);
    if (!connected) {
        Serial.println("  10초 내 연결 안 됨");
        return;
    }

    Serial.println("\n[3] 660Hz 톤 전송 (8초). 이어폰에서 소리가 나야 한다.");
    esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
    delay(300);
    esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
    const uint32_t t2 = millis();
    while (millis() - t2 < 8000) delay(200);
    esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);

    Serial.println("\n=== 판정 ===");
    if (streaming) {
        Serial.println("A2DP 스트리밍이 시작됐다. 소리가 들렸다면 오디오 출력이 살아난 것이다.");
        Serial.println("→ 부팅 확인음·번역 응답·상태 알림을 블루투스로 낼 수 있다.");
        Serial.println("→ 다음: HFP Audio Gateway 로 이어폰 마이크까지 입력으로 쓴다.");
    } else if (connected) {
        Serial.println("연결은 됐지만 스트리밍이 시작되지 않았다 — 코덱 협상 문제일 수 있다.");
    } else {
        Serial.println("연결되지 않았다.");
    }
}

void loop() { delay(1000); }
