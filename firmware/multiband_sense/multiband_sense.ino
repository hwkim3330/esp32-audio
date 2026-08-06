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

#include "cn_infer.h"      // 학습된 인코더 추론 (음성과 같은 엔진, 조건 컴파일로 로그멜 제외)
#include "csi_front.h"     // CSI 창 프런트엔드 — 파이썬과 1.371e-06 로 대조됨
#include "prototypes.h"
#include "selftest.h"

// ── LED. 어느 GPIO 가 LED 인지 실기로 특정하지 못했다(관찰자가 없었다).
// 그래서 안전한 후보를 전부 같은 상태 신호로 구동한다 — 하나가 LED 면 그게 표시등이 된다.
// 화면이 없는 보드에서 유일한 시각 피드백이다.
//
// 후보에서 뺀 것: 버튼(19/23/18/5/36/13), I2C(32/33), I2S(0/25/26/27/34/35),
// PSRAM(16/17 — 실측 확인), 플래시(6~11), 스트래핑(12/15).
#define N_LED 4
static const int LED_PIN[N_LED] = { 22, 21, 2, 4 };

// ── PSRAM: CSI 원시 프레임 링. 창을 시간으로 잘라 모델에 넣으려면 이력이 필요하다.
#define RING_N 512
static int8_t   *ring_iq;      // RING_N x (2*n_sc)
static uint32_t *ring_ms;
static volatile uint32_t ring_w = 0;
static volatile uint8_t  ring_ch[RING_N];

static cn_ctx_t infer_ctx;
static float   *infer_win;     // CN_N_FRAMES x CN_N_MELS
static int8_t  *infer_packed;  // CF_MAX_IN x (2*n_sc)  — PSRAM
static uint32_t *infer_wms;    // CF_MAX_IN
static float   *infer_scr;     // CF_MAX_IN x n_sc      — cf_window 스크래치
static bool     infer_ready = false;
static volatile int   last_cls = -1;
static volatile float last_score = 0.0f;
static volatile uint32_t infer_ms = 0, infer_n = 0;

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

// ── 버튼 6개. 화면이 없으니 버튼이 유일한 입력이고, 가장 중요한 용도는 **정답 찍기**다.
//
// 이 펌웨어의 유일한 미검증 항목이 "실제 움직임에 반응하는가" 였다. 사람이 지나가면서
// 마크를 누르면 보드가 마크 구간과 비마크 구간의 점수 분포를 스스로 비교한다 —
// PC 도 관찰자도 필요 없다.
//
//   K1 짧게 = 마크 토글(사람 있음)   K1 길게 = 기준선 재학습
//   K2 = 임계값 −0.5                K3 = 임계값 +0.5
//   K4 = 채널 고정/순환 토글         K5 짧게 = BLE 표 리셋, 길게 = 통계 요약
//   K6 = 통계 요약
#define N_KEYS 6
static const int KEY_PIN[N_KEYS] = { 19, 23, 18, 5, 36, 13 };
static bool key_ok[N_KEYS] = { false };
static volatile bool  marked = false;
static volatile float thresh = 3.0f;
static volatile bool  ch_lock = false;

// 마크/비마크 구간의 점수 통계. 이게 검증의 전부다.
static double m_sum = 0, m_sq = 0, u_sum = 0, u_sq = 0;
static uint32_t m_n = 0, u_n = 0;
static uint32_t m_hit = 0, u_hit = 0;      // 임계값 초과 횟수

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

    // PSRAM 링에 원시 IQ 를 적재한다 — 모델 입력용 창을 나중에 만든다.
    if (ring_iq) {
        const uint32_t w = ring_w % RING_N;
        int8_t *dst = ring_iq + (size_t)w * 2 * MAX_SC;
        const int nb = (info->len > 2 * MAX_SC) ? 2 * MAX_SC : info->len;
        for (int i = 0; i < nb; i++) dst[i] = info->buf[i];
        ring_ms[w] = info->rx_ctrl.timestamp / 1000;   // us → ms
        ring_ch[w] = ci;
        ring_w++;
    }

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

static void reset_baseline()
{
    for (int i = 0; i < N_WCH; i++) {
        base_ok[i] = false; acc_n[i] = 0;
        for (int j = 0; j < MAX_SC; j++) { acc_sum[i][j] = 0; acc_sq[i][j] = 0; }
    }
    Serial.println("[key] 기준선 재학습 시작 — 움직이지 마세요");
}

static void print_stats()
{
    Serial.println("\n=== 검증 통계 ===");
    if (!m_n || !u_n) {
        Serial.printf("표본 부족 (마크 %lu, 비마크 %lu). K1 을 눌러 마크하고 지나가세요.\n",
                      (unsigned long)m_n, (unsigned long)u_n);
        return;
    }
    const double mm = m_sum / m_n, um = u_sum / u_n;
    const double ms = sqrt(fmax(m_sq / m_n - mm * mm, 0.0));
    const double us = sqrt(fmax(u_sq / u_n - um * um, 0.0));
    // 분리도(Cohen's d): 두 분포가 얼마나 떨어졌나. 1.0 이상이면 쓸만하다.
    const double pooled = sqrt(((ms * ms) + (us * us)) / 2.0);
    const double d = pooled > 1e-9 ? (mm - um) / pooled : 0.0;
    Serial.printf("사람 있음 (마크)  %6lu 표본  점수 %.2f ± %.2f   임계초과 %.0f%%\n",
                  (unsigned long)m_n, mm, ms, 100.0 * m_hit / m_n);
    Serial.printf("사람 없음        %6lu 표본  점수 %.2f ± %.2f   임계초과 %.0f%%\n",
                  (unsigned long)u_n, um, us, 100.0 * u_hit / u_n);
    Serial.printf("분리도 d = %.2f  (%s)\n", d,
                  d > 1.5 ? "좋다" : (d > 0.8 ? "쓸만하다" : "부족 — 임계값이나 특징을 손봐야 한다"));
    Serial.printf("현재 임계값 %.1f — 감지율 %.0f%%, 오탐율 %.0f%%\n\n",
                  thresh, 100.0 * m_hit / m_n, 100.0 * u_hit / u_n);
}

static void keys_poll()
{
    static uint32_t last = 0;
    static bool down[N_KEYS] = { false };
    static uint32_t t_down[N_KEYS] = { 0 };
    static bool longed[N_KEYS] = { false };
    if (millis() - last < 40) return;
    last = millis();

    for (int k = 0; k < N_KEYS; k++) {
        if (!key_ok[k]) continue;
        const bool d = (digitalRead(KEY_PIN[k]) == LOW);
        if (d && !down[k]) { down[k] = true; t_down[k] = millis(); longed[k] = false; }
        else if (d && down[k] && !longed[k] && millis() - t_down[k] > 700) {
            longed[k] = true;
            if (k == 0) reset_baseline();
            else if (k == 4) print_stats();
        } else if (!d && down[k]) {
            down[k] = false;
            if (longed[k]) continue;
            switch (k) {
            case 0:
                marked = !marked;
                Serial.printf("[key] 마크 %s\n", marked ? "ON — 사람 있음" : "OFF");
                break;
            case 1:
                if (thresh > 1.0f) thresh -= 0.5f;
                Serial.printf("[key] 임계값 %.1f\n", thresh);
                break;
            case 2:
                thresh += 0.5f;
                Serial.printf("[key] 임계값 %.1f\n", thresh);
                break;
            case 3:
                ch_lock = !ch_lock;
                Serial.printf("[key] 채널 %s\n", ch_lock ? "고정" : "순환");
                break;
            case 4:
                n_ble = 0; ble_pkt = 0; ble_hot = 0; ble_valid = 0;
                Serial.println("[key] BLE 표 리셋");
                break;
            case 5:
                print_stats();
                break;
            }
        }
    }
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

    // ── 버튼. 부팅 시 LOW 로 읽히면 배선이 의심스러워 비활성한다(K6=GPIO13 실측).
    for (int k = 0; k < N_KEYS; k++)
        pinMode(KEY_PIN[k], (KEY_PIN[k] == 36 || KEY_PIN[k] == 5) ? INPUT : INPUT_PULLUP);
    Serial.print("[keys] ");
    int n_ok = 0;
    for (int k = 0; k < N_KEYS; k++) {
        key_ok[k] = (digitalRead(KEY_PIN[k]) == HIGH);
        if (key_ok[k]) n_ok++;
        Serial.printf("K%d=%d%s ", k + 1, digitalRead(KEY_PIN[k]),
                      key_ok[k] ? "" : "[비활성]");
    }
    Serial.printf("(%d/%d 사용)\n", n_ok, N_KEYS);
    Serial.println("  K1짧게=마크토글  K1길게=기준선재학습  K2=임계값−  K3=임계값+");
    Serial.println("  K4=채널고정  K5짧게=BLE리셋 K5길게=통계  K6=통계");
    if (!key_ok[5]) Serial.println("  ※ K6 비활성 — 통계는 K5 를 길게 누르세요");

    // ── LED 후보 전부 출력으로
    for (int i = 0; i < N_LED; i++) { pinMode(LED_PIN[i], OUTPUT); digitalWrite(LED_PIN[i], LOW); }
    Serial.printf("[led] 후보 %d핀 구동 (GPIO", N_LED);
    for (int i = 0; i < N_LED; i++) Serial.printf("%s%d", i ? "/" : "", LED_PIN[i]);
    Serial.println(") — 하나가 LED 면 표시등이 된다");

    // ── PSRAM: CSI 원시 링 + 추론 스크래치
    if (psramFound()) {
        ring_iq = (int8_t *)heap_caps_malloc((size_t)RING_N * 2 * MAX_SC, MALLOC_CAP_SPIRAM);
        ring_ms = (uint32_t *)heap_caps_malloc((size_t)RING_N * 4, MALLOC_CAP_SPIRAM);
        void *sc = heap_caps_malloc(cn_scratch_bytes(), MALLOC_CAP_SPIRAM);
        infer_win = (float *)heap_caps_malloc(
            (size_t)CN_N_FRAMES * CN_N_MELS * sizeof(float), MALLOC_CAP_SPIRAM);
        infer_packed = (int8_t *)heap_caps_malloc((size_t)CF_MAX_IN * 2 * MAX_SC,
                                                 MALLOC_CAP_SPIRAM);
        infer_wms = (uint32_t *)heap_caps_malloc((size_t)CF_MAX_IN * 4, MALLOC_CAP_SPIRAM);
        infer_scr = (float *)heap_caps_malloc((size_t)CF_MAX_IN * MAX_SC * sizeof(float),
                                             MALLOC_CAP_SPIRAM);
        if (ring_iq && ring_ms && sc && infer_win && infer_packed && infer_wms && infer_scr) {
            cn_ctx_init(&infer_ctx, cn_weights, sc);
            float err = 0.0f;
            const int rc = cn_selftest(&infer_ctx, &err);
            Serial.printf("[모델] 자기검증 %s (오차 %.2e) — 파라미터 %u, 프로토타입 %d\n",
                          rc == 0 ? "통과" : "실패", err,
                          (unsigned)(sizeof(cn_weights) / sizeof(float)), CN_N_PROTO);
            infer_ready = (rc == 0);
            Serial.printf("[psram] 링 %dKB + 추론 %dKB + 창 %dKB\n",
                          (int)((size_t)RING_N * 2 * MAX_SC / 1024),
                          (int)(cn_scratch_bytes() / 1024),
                          (int)((size_t)CN_N_FRAMES * CN_N_MELS * 4 / 1024));
            Serial.println("      ※ 현재 모델은 '채널 분류기' 다 — 위치 라벨이 없어 그걸로 학습했다.");
            Serial.println("        라이브 CSI 로 사슬이 도는지 확인하는 용도이고, 위치 데이터가");
            Serial.println("        오면 모델만 바꾸면 된다.");
        } else {
            Serial.println("[psram] 할당 실패 — 통계만으로 진행");
        }
    }

    Serial.printf("\n기준선 학습 중 (채널당 %d 프레임). 주변에서 움직이지 마세요.\n",
                  WARMUP_N);
    Serial.println("준비되면 K1 을 눌러 마크하고 보드 앞을 지나가세요 — 보드가 스스로 검증합니다.\n");
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

    // ── 온보드 추론. 현재 채널의 최근 프레임으로 창을 만들어 모델에 넣는다.
    //    라이브 CSI 로 프런트엔드→인코더→매칭이 도는지 확인하는 것이 목적이다.
    if (infer_ready && ring_w > 40) {
        // 현재 채널의 프레임 인덱스를 최신순으로 모은다. 인덱스만 담으므로 작다.
        uint16_t idxs[CF_MAX_IN];
        int k = 0;
        const uint32_t total = ring_w;
        const uint32_t have = (total < RING_N) ? total : RING_N;
        for (uint32_t i2 = 0; i2 < have && k < CF_MAX_IN; i2++) {
            const uint32_t idx = (total - 1 - i2) % RING_N;
            // 채널이 섞이면 주파수 응답이 섞여 무의미하다 — 현재 채널만.
            if (ring_ch[idx] != cur_wch_i) continue;
            idxs[k++] = (uint16_t)idx;
        }
        if (k >= 8) {
            // 시간순으로 되집어 PSRAM 버퍼에 연속 배치한다 (보간이 단조를 요구한다).
            for (int a = 0; a < k; a++) {
                const uint32_t src = idxs[k - 1 - a];
                memcpy(infer_packed + (size_t)a * 2 * n_sc,
                       ring_iq + (size_t)src * 2 * MAX_SC, (size_t)2 * n_sc);
                infer_wms[a] = ring_ms[src];
            }
            const uint32_t t0 = millis();
            if (cf_window(infer_packed, infer_wms, k, n_sc, CN_N_FRAMES,
                          CSI_CLIP, CSI_EPS, infer_scr, infer_win) == 0) {
                float emb[CN_EMB_DIM];
                cn_encode(&infer_ctx, infer_win, emb);
                float sc2 = 0.0f;
                const int best = cn_match(emb, cn_protos, CN_N_PROTO, &sc2);
                last_cls = (best >= 0) ? cn_proto_class[best] : -1;
                last_score = sc2;
                infer_ms = millis() - t0;
                infer_n++;
            }
        }
    }

    // 마크/비마크로 나눠 누적한다. 기준선이 다 준비된 뒤부터만 센다.
    if (ready == N_WCH) {
        const bool hit = band > thresh;
        if (marked) { m_sum += band; m_sq += (double)band * band; m_n++; if (hit) m_hit++; }
        else        { u_sum += band; u_sq += (double)band * band; u_n++; if (hit) u_hit++; }
    }

    Serial.printf("ch");
    for (int i = 0; i < N_WCH; i++)
        Serial.printf("%s%d:%s%.1f", i ? " " : "", WCH_MHZ[i],
                      base_ok[i] ? "" : "(학습)", base_ok[i] ? w_dev[i] : 0.0f);
    // LED: 학습 중 = 느린 깜빡임, 준비 = 켜짐, 감지 = 빠른 깜빡임
    {
        static bool on = false;
        const bool det = band > thresh;
        if (ready < N_WCH) on = !on;                 // 1Hz 깜빡
        else if (det)      on = !on;                 // 리포트마다 토글 (빠르게 보임)
        else               on = true;                // 준비됨 = 켜짐
        for (int i = 0; i < N_LED; i++) digitalWrite(LED_PIN[i], on ? HIGH : LOW);
    }

    if (infer_ready)
        Serial.printf("        [모델] 클래스%d 코사인%.2f %lums (%lu회)\n",
                      last_cls, last_score, (unsigned long)infer_ms,
                      (unsigned long)infer_n);
    Serial.printf("%s  | BLE %d/%d (유효%d, %lu광고) 점수%.1f  | 대역 %.1f%s  sc%d 준비%d/%d\n",
                  marked ? "  [마크]" : "        ",
                  ble_hot, ble_valid, ble_valid, (unsigned long)ble_pkt,
                  ble_dev_score, band, (band > thresh) ? "  <<< 움직임" : "",
                  n_sc, ready, N_WCH);
    if (marked) Serial.print("");
}
