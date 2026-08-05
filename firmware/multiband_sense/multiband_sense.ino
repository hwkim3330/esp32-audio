// 멀티밴드 2.4GHz 센싱 — 이 칩의 라디오를 전부 센서로 쓴다.
//
// 왜 대역을 넓게 봐야 하는가:
//   멀티패스 페이딩은 주파수마다 다르다. 어떤 주파수에서는 사람이 지나가도 신호가
//   거의 안 변하는데(널 지점), 다른 주파수에서는 크게 변한다. 그래서 대역을 넓게
//   보면 환경을 독립적으로 여러 번 보는 셈이 되고, 이게 주파수 다이버시티다.
//
//   WiFi CSI 만 쓰면 2412~2472 의 20MHz 창뿐이다. BLE 광고 채널을 더하면
//   2402 와 2480 — 대역 양끝이 들어온다.
//
// 이 칩의 2.4GHz 라디오 3종:
//   WiFi CSI      ch1/6/11 = 2412/2437/2462, 20MHz, 서브캐리어 64개 (가장 세밀)
//   BLE 광고 스캔  ch37/38/39 = 2402/2426/2480 (양끝 커버), RSSI 만
//   BT Classic    79채널 호핑 — 여기서는 쓰지 않는다(아래 참조)
//
// BT Classic 은 뺐다. inquiry 는 12초씩 걸려 감지 주기가 무너지고, WiFi 와
// 프런트엔드를 다투어 CSI 프레임률을 떨어뜨린다. BLE 스캔은 훨씬 가볍고 대역
// 양끝을 이미 커버하므로 비용 대비 이득이 없다.
//
// 오디오(HFP/A2DP)와는 동시에 못 한다. 라디오 프런트엔드가 하나라서 오디오
// 스트리밍은 BT 를 독점해야 하고, 센싱은 계속 호핑해야 한다. 여기서는 센싱을 택했다.

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include <math.h>

// ── WiFi CSI
#define MAX_SC     128
#define N_WCH      3
static const uint8_t WCH[N_WCH] = { 1, 6, 11 };
static const int WCH_MHZ[N_WCH] = { 2412, 2437, 2462 };

// 채널별 기준선과 최근 편차. 채널을 순환하므로 각각 따로 학습해야 한다 —
// 채널이 다르면 주파수 응답이 계통적으로 달라서 하나의 기준선으로는 못 잡는다.
static float  base_mu[N_WCH][MAX_SC];
static float  base_sd[N_WCH][MAX_SC];
static bool   base_ok[N_WCH] = { false, false, false };
static float  acc_sum[N_WCH][MAX_SC];
static float  acc_sq[N_WCH][MAX_SC];
static uint32_t acc_n[N_WCH] = { 0, 0, 0 };
static volatile float w_dev[N_WCH] = { 0, 0, 0 };
static volatile uint32_t w_pkt[N_WCH] = { 0, 0, 0 };
static volatile uint8_t cur_wch_i = 0;
static uint8_t n_sc = 0;

// ── BLE RSSI. 광고 채널은 하드웨어가 자동 순환하므로 채널을 지정할 수 없다.
// 대신 광고를 많이 받으면 세 채널(2402/2426/2480)이 자연히 섞인다.
#define BLE_MAX_DEV 48
typedef struct { uint8_t addr[6]; float mu, sd, last; uint32_t n; } ble_dev_t;
static ble_dev_t ble_dev[BLE_MAX_DEV];
static volatile int n_ble = 0;
static volatile float ble_dev_score = 0.0f;
static volatile uint32_t ble_pkt = 0;
static volatile int ble_valid = 0, ble_hot = 0;

// 채널당 기준선 프레임 수. 채널을 순환하므로 너무 크면 학습이 안 끝난다
// (실측: 60프레임에 50초 동안 3채널 중 1개만 준비됐다).
#define WARMUP_N 40

// ───────────────────────────────────────── WiFi CSI
static void IRAM_ATTR csi_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf || info->len < 8) return;
    const uint8_t ci = cur_wch_i;
    w_pkt[ci]++;

    const int sc = info->len / 2;
    const int n = (sc > MAX_SC) ? MAX_SC : sc;
    if (!n_sc) n_sc = (uint8_t)n;
    const int start = info->first_word_invalid ? 2 : 0;

    float amp[MAX_SC];
    for (int i = 0; i < n; i++) {
        if (i < start) { amp[i] = 0.0f; continue; }
        const float im = (float)info->buf[2 * i], re = (float)info->buf[2 * i + 1];
        amp[i] = sqrtf(im * im + re * re);
    }

    if (!base_ok[ci]) {
        // 기준선 학습: 합과 제곱합만 쌓는다. 프레임을 다 보관하지 않아도
        // 평균·표준편차가 나오므로 메모리가 채널 수에 비례하지 않는다.
        for (int i = 0; i < n; i++) { acc_sum[ci][i] += amp[i]; acc_sq[ci][i] += amp[i] * amp[i]; }
        if (++acc_n[ci] >= WARMUP_N) {
            for (int i = 0; i < n; i++) {
                const float mu = acc_sum[ci][i] / acc_n[ci];
                const float v = acc_sq[ci][i] / acc_n[ci] - mu * mu;
                base_mu[ci][i] = mu;
                base_sd[ci][i] = sqrtf(v > 0 ? v : 0);
            }
            base_ok[ci] = true;
        }
        return;
    }
    float z = 0.0f; int c = 0;
    for (int i = start; i < n; i++)
        if (base_sd[ci][i] > 0.5f) { z += fabsf(amp[i] - base_mu[ci][i]) / base_sd[ci][i]; c++; }
    w_dev[ci] = c ? (z / c) : 0.0f;
}

// ───────────────────────────────────────── BLE RSSI
//
// 사람이 광고 기기와 보드 사이를 지나면 그 링크의 RSSI 가 변한다. 기기마다
// 기준선을 따로 두고 편차를 본다 — 기기마다 거리·경로가 달라 절대값은 의미가 없다.
static void ble_cb(esp_gap_ble_cb_event_t ev, esp_ble_gap_cb_param_t *p)
{
    if (ev != ESP_GAP_BLE_SCAN_RESULT_EVT) return;
    if (p->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) return;
    ble_pkt++;

    int idx = -1;
    for (int i = 0; i < n_ble; i++)
        if (!memcmp(ble_dev[i].addr, p->scan_rst.bda, 6)) { idx = i; break; }
    if (idx < 0) {
        if (n_ble >= BLE_MAX_DEV) return;
        idx = n_ble++;
        memcpy(ble_dev[idx].addr, p->scan_rst.bda, 6);
        ble_dev[idx].mu = p->scan_rst.rssi;
        ble_dev[idx].sd = 2.0f;
        ble_dev[idx].n = 1;
        ble_dev[idx].last = p->scan_rst.rssi;
        return;
    }
    ble_dev_t *d = &ble_dev[idx];
    const float r = p->scan_rst.rssi;
    d->last = r;
    d->n++;
    // 지수 이동 평균/편차. 기준선이 천천히 따라가되 급변은 편차로 드러난다.
    const float a = (d->n < 40) ? 0.1f : 0.01f;
    const float e = r - d->mu;
    d->mu += a * e;
    d->sd += a * (fabsf(e) - d->sd);

    // 점수는 z-score 최대값이 아니라 **2σ 를 넘는 기기의 비율**이다.
    //
    // 최대값을 쓰면 다중비교 문제에 걸린다. 노이즈가 있는 48개 변수의 최댓값은
    // 우연히 3σ 를 넘는 게 정상이라, 아무도 안 움직이는데 점수가 3.1 을 찍었다(실측).
    // 사람이 지나가면 여러 링크가 동시에 흔들리는데 노이즈는 하나씩 튄다 — 그
    // 차이를 쓰는 것이 옳다.
    int hot = 0, valid = 0;
    for (int i = 0; i < n_ble; i++) {
        if (ble_dev[i].n < 20 || ble_dev[i].sd < 0.5f) continue;
        valid++;
        const float z = fabsf(ble_dev[i].last - ble_dev[i].mu) / ble_dev[i].sd;
        if (z > 2.0f) hot++;
    }
    // 비율을 z 스케일에 맞춰 환산한다(0.3 = 30% 가 흔들림 → 점수 3.0).
    ble_dev_score = valid ? (10.0f * (float)hot / (float)valid) : 0.0f;
    ble_valid = valid; ble_hot = hot;
}

void setup()
{
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== 멀티밴드 2.4GHz 센싱 ===");
    Serial.println("WiFi CSI(ch1/6/11) + BLE 광고 RSSI(ch37/38/39) 를 같이 본다.");
    Serial.println("멀티패스 페이딩은 주파수마다 달라서, 대역을 넓게 보면 환경을 더 본다.\n");

    if (!psramFound()) Serial.println("(PSRAM 없음 — 기준선만 쓰므로 계속 진행)");

    // ── WiFi CSI
    WiFi.mode(WIFI_MODE_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_filter(nullptr);
    wifi_csi_config_t cfg = {};
    cfg.lltf_en = true; cfg.htltf_en = true; cfg.stbc_htltf2_en = true;
    cfg.ltf_merge_en = true; cfg.channel_filter_en = true;
    cfg.manu_scale = false; cfg.shift = 0; cfg.dump_ack_en = false;
    if (esp_wifi_set_csi_config(&cfg) != ESP_OK) {
        Serial.println("CSI 설정 실패. 중단."); while (1) delay(1000);
    }
    esp_wifi_set_csi_rx_cb(csi_cb, nullptr);
    esp_wifi_set_csi(true);
    esp_wifi_set_channel(WCH[0], WIFI_SECOND_CHAN_NONE);
    Serial.println("[wifi] CSI 활성화");

    // ── BLE 스캔. btStart() 를 쓴다 — 직접 컨트롤러를 올리면 상태가 IDLE 로
    //    읽히는데도 INVALID_STATE 가 돌아온다(bt_audio_probe 에서 실측).
    if (!btStart()) {
        Serial.println("[ble] btStart() 실패 — WiFi CSI 만으로 계속한다");
    } else if (esp_bluedroid_init() == ESP_OK && esp_bluedroid_enable() == ESP_OK) {
        esp_ble_gap_register_callback(ble_cb);
        esp_ble_scan_params_t sp = {};
        sp.scan_type          = BLE_SCAN_TYPE_PASSIVE;   // 수동 — 프로브를 쏘지 않는다
        sp.own_addr_type      = BLE_ADDR_TYPE_PUBLIC;
        sp.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
        sp.scan_interval      = 0x50;                    // 50ms
        sp.scan_window        = 0x30;                    // 30ms
        sp.scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE;  // 중복도 받는다(RSSI 시계열)
        esp_ble_gap_set_scan_params(&sp);
        delay(200);
        esp_ble_gap_start_scanning(0);                   // 무한
        Serial.println("[ble] 수동 스캔 시작 (광고 채널 37/38/39)");
    } else {
        Serial.println("[ble] bluedroid 실패 — WiFi CSI 만으로 계속한다");
    }

    Serial.printf("\n기준선 학습 중 (채널당 %d 프레임). 주변에서 움직이지 마세요.\n\n",
                  WARMUP_N);
}

void loop()
{
    // WiFi 채널 순환. 채널마다 기준선이 따로이므로 전환 후 잠깐은 값이 흔들린다.
    static uint32_t ch_t = 0;
    // 채널 체류 시간. 짧으면 기준선 학습이 오래 걸리고, 길면 감지 주기가 느려진다.
    if (millis() - ch_t > 6000) {
        ch_t = millis();
        cur_wch_i = (uint8_t)((cur_wch_i + 1) % N_WCH);
        esp_wifi_set_channel(WCH[cur_wch_i], WIFI_SECOND_CHAN_NONE);
    }

    static uint32_t rep_t = 0;
    if (millis() - rep_t < 1000) { delay(20); return; }
    rep_t = millis();

    // 대역 전체를 아우르는 점수: 준비된 WiFi 채널들의 최대 편차와 BLE 점수 중 큰 것.
    // 최대를 쓰는 이유는 널 지점 때문이다 — 어떤 주파수에서 신호가 안 변해도
    // 다른 주파수에서 변하면 사람이 있는 것이다. 평균을 쓰면 그게 희석된다.
    float wmax = 0.0f;
    int ready = 0;
    for (int i = 0; i < N_WCH; i++) {
        if (base_ok[i]) { ready++; if (w_dev[i] > wmax) wmax = w_dev[i]; }
    }
    const float band = (wmax > ble_dev_score) ? wmax : ble_dev_score;

    Serial.printf("ch");
    for (int i = 0; i < N_WCH; i++)
        Serial.printf("%s%d:%s%.1f", i ? " " : "", WCH_MHZ[i],
                      base_ok[i] ? "" : "(학습)", base_ok[i] ? w_dev[i] : 0.0f);
    Serial.printf("  | BLE %d/%d 흔들림 (유효%d, %lu광고) 점수%.1f  | 대역 %.1f%s  sc%d 준비%d/%d\n",
                  ble_hot, ble_valid, ble_valid, (unsigned long)ble_pkt,
                  ble_dev_score, band, (band > 3.0f) ? "  <<< 움직임" : "",
                  n_sc, ready, N_WCH);
}
