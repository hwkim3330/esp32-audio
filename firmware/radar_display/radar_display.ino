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

#include <Adafruit_GFX.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Preferences.h>

#include "cn_infer.h"      // 학습된 인코더 추론 (음성과 같은 엔진, 조건 컴파일로 로그멜 제외)
#include "csi_front.h"     // CSI 창 프런트엔드 — 파이썬과 1.371e-06 로 대조됨
#include "prototypes.h"
#include "selftest.h"
#include "esl_bwr.h"
#include "web_ui.h"

void ble_note(const uint8_t *bda, int8_t rssi);

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

// 실린 모델은 "채널 분류기" 다. 그러면 예측 클래스는 창을 뜬 채널과 같아야 한다.
// 이 일치율이 파이프라인(PSRAM 링 → 채널 필터 → cf_window → 인코더 → 매칭)이
// 보드에서 **맞게** 도는지 보는 유일한 자동 지표다. 사람이 필요 없다.
static volatile uint32_t cls_hit = 0, cls_tot = 0;
static volatile uint32_t win_span_ms = 0, win_n = 0, win_skip = 0;
// 창을 뜬 시점의 채널. 출력 시점의 cur_wch_i 와 다를 수 있다(6초마다 바뀐다) —
// 로그로 혼동행렬을 다시 계산하려면 이 값이 찍혀야 한다.
static volatile int last_win_ch = -1;
static volatile uint32_t cls_cm[N_WCH][N_WCH];   // [실제][예측]

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
// 서브캐리어 개수가 기대와 다른 프레임 수. 0 이 아니면 AP 대역폭이 섞인 것이다.
static volatile uint32_t sc_mismatch = 0;
static volatile uint16_t sc_other = 0;
static uint32_t ch_t = 0;      // 마지막 채널 전환 시각. 추론 쪽에서도 본다.
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

// 채널 체류 시간. 6초였는데 그중 앞 2초는 링에 이 채널 프레임이 없어 창을 못 만든다.
// 그래서 추론 175회에 프레임 부족 건너뜀이 181회였다 — 절반 넘게 헛돌았다.
// 10초로 늘리면 쓸 수 있는 구간이 4초 → 8초가 되어 비율이 뒤집힌다. 대가는 한
// 채널을 다시 보기까지 18초 → 30초인데, 추세 그래프는 1초마다 찍히므로 문제없다.
#define CH_DWELL_MS  10000
#define CH_SETTLE_MS  2200      // 전환 직후 이만큼은 추론을 아예 시도하지 않는다

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
// **실기로 확정한 매핑** (핀 변화 감시기로 KEY1..KEY6 을 순서대로 눌러 확인).
//   KEY1=GPIO36  KEY2=GPIO13  KEY3=GPIO19  KEY4=GPIO23  KEY5=GPIO18  KEY6=GPIO5
//
// 처음에는 { 19, 23, 18, 5, 36, 13 } 이었다 — **두 칸 밀려 있었다.** 그래서
// "3번 키만 반응한다" 는 증상이 나왔다. 물리 KEY3(GPIO19)이 코드에서는 K1 =
// 마크 토글이었고, 마크 토글만 LED 패턴이라는 눈에 보이는 효과가 있었다.
// 나머지 다섯 개는 잘 작동하면서 시리얼에만 찍혔다.
//
// 교훈: 화면도 LED 도 없이 시리얼로만 확인하면 "안 되는 것" 과 "되는데 안 보이는
// 것" 이 구별되지 않는다. 그래서 아래에서 **모든 키에 LED 응답**을 붙였다.
static const int KEY_PIN[N_KEYS] = { 36, 13, 19, 23, 18, 5 };
static bool key_ok[N_KEYS] = { false };
// 마크는 **한 번 누르면 30초 동안 켜지고 스스로 꺼진다.**
//
// 처음에는 토글이었다. 누르고 걷고 다시 눌러 끄는 방식인데, 실기에서 표본이 하나도
// 안 남았다(마크 0 / 비마크 676). 끄는 것을 기억해야 하는 설계 자체가 문제다 —
// 검증은 한 번만 하면 되는 일이니 사람이 할 일은 "누르고 걷기" 하나여야 한다.
//
// 30초는 대역 점수가 1초에 한 표본이므로 30표본이다. Cohen's d 를 내기에 최소한이고,
// 사람이 방을 몇 번 왕복하기에 충분하다. 여러 번 눌러 쌓으면 표본이 늘어난다.
#define MARK_MS 30000
static volatile bool     marked = false;
static uint32_t          mark_until = 0;
static volatile float thresh = 3.0f;
static volatile bool  ch_lock = false;

// 마크/비마크 구간의 점수 통계. 이게 검증의 전부다.
static double m_sum = 0, m_sq = 0, u_sum = 0, u_sq = 0;
static uint32_t m_n = 0, u_n = 0;
static uint32_t m_hit = 0, u_hit = 0;      // 임계값 초과 횟수

// 키를 누르면 **누른 키 번호만큼 LED 를 깜빡인다.**
//
// 이게 없어서 "3번 키만 반응한다" 는 오진이 나왔다. 여섯 개 다 작동하는데 다섯 개는
// 시리얼에만 찍혀서 안 보였을 뿐이다. 화면도 없는 보드에서는 즉시 보이는 응답이
// 있어야 "먹었는지" 를 알 수 있다. 막는 방식(delay)으로 짜면 CSI 프레임을 놓치므로
// 큐로 만든다.
static volatile int  blink_left = 0;      // 남은 깜빡임 횟수 (×2 = 켜고 끄기)
static uint32_t      blink_t = 0;
#define BLINK_MS 120

static void led_all(bool on)
{
    for (int i = 0; i < N_LED; i++) digitalWrite(LED_PIN[i], on ? HIGH : LOW);
}


static void blink_ack(int key_1based) { blink_left = key_1based * 2; }

static void blink_poll(void)
{
    if (blink_left <= 0) return;
    if (millis() - blink_t < BLINK_MS) return;
    blink_t = millis();
    led_all((blink_left & 1) != 0);       // 홀수 남으면 켠다
    blink_left--;
}

// ───────────────────────────────────────── 영속 저장
//
// 재부팅하면 전부 사라진다는 것이 이 펌웨어의 가장 큰 구멍이었다. 벽에 붙여둘
// 센서인데 며칠에 걸쳐 증거를 모을 수 없으면 K1 검증(마크/비마크 분리도)이
// 무의미하다 — 사람이 한 번 지나가는 것으로는 표본이 안 모인다.
//
// NVS 에 남기는 것은 **누적된 증거와 사람이 정한 값**뿐이다. 기준선(base_mu/sd)은
// 일부러 안 남긴다 — 전파 환경은 시간이 지나면 바뀌므로 묵은 기준선을 되살리면
// 부팅 직후 유령 감지가 난다. 다시 배우는 데 40프레임이면 된다.
static Preferences prefs;
static uint32_t boot_n = 0;

static void store_load(void)
{
    prefs.begin("radar", false);
    boot_n = prefs.getUInt("boot", 0) + 1;
    prefs.putUInt("boot", boot_n);
    thresh   = prefs.getFloat("thresh", 3.0f);
    m_sum = prefs.getDouble("ms", 0.0); m_sq = prefs.getDouble("mq", 0.0);
    u_sum = prefs.getDouble("us", 0.0); u_sq = prefs.getDouble("uq", 0.0);
    m_n = prefs.getUInt("mn", 0); u_n = prefs.getUInt("un", 0);
    m_hit = prefs.getUInt("mh", 0); u_hit = prefs.getUInt("uh", 0);
    cls_hit = prefs.getUInt("ch", 0); cls_tot = prefs.getUInt("ct", 0);
    prefs.getBytes("cm", (void *)cls_cm, sizeof(cls_cm));
    Serial.printf("[저장] 부팅 %lu회째. 검증 표본 마크 %lu / 비마크 %lu, "
                  "채널일치 %lu/%lu, 임계 %.1f\n",
                  (unsigned long)boot_n, (unsigned long)m_n, (unsigned long)u_n,
                  (unsigned long)cls_hit, (unsigned long)cls_tot, thresh);
}

static void store_save(const char *why)
{
    prefs.putFloat("thresh", thresh);
    prefs.putDouble("ms", m_sum); prefs.putDouble("mq", m_sq);
    prefs.putDouble("us", u_sum); prefs.putDouble("uq", u_sq);
    prefs.putUInt("mn", m_n); prefs.putUInt("un", u_n);
    prefs.putUInt("mh", m_hit); prefs.putUInt("uh", u_hit);
    prefs.putUInt("ch", cls_hit); prefs.putUInt("ct", cls_tot);
    prefs.putBytes("cm", (const void *)cls_cm, sizeof(cls_cm));
    Serial.printf("[저장] 기록함 (%s)\n", why);
}

// ───────────────────────────────────────── 키 매핑 탐색
//
// 여섯 개 중 하나만 반응한다(실기 관측). 내 매핑이 틀렸거나 핀이 다른 기능에
// 물려 있다는 뜻이다. 짐작하지 말고 보드가 직접 알려주게 한다.
//
// 유력한 원인: 이 보드의 **GPIO 5/18/19/23 은 SD카드 SPI 와 겸용**이고, 가운데
// DIP 스위치가 그걸 키와 SD 사이에서 가른다. 즉 지금 DIP 가 SD 쪽이면 그 네 개는
// 키로 안 잡힌다. 오디오가 죽은 것의 남은 단일 용의자도 같은 DIP 스위치다 —
// 두 증상이 한 원인일 수 있다.
//
// 여기서는 후보 핀을 넓게 걸어놓고 **변화가 생긴 핀 번호를 그대로 찍는다.**
// 버튼을 하나씩 누르면 어느 핀인지 나온다.
static const int WATCH[] = { 36, 39, 34, 35, 13, 5, 18, 19, 23, 27, 12, 14 };
#define N_WATCH ((int)(sizeof(WATCH) / sizeof(WATCH[0])))
static uint8_t watch_prev[N_WATCH];
static uint32_t watch_hits[N_WATCH];

static void watch_init(void)
{
    Serial.print("[핀탐색] 부팅 시 레벨: ");
    for (int i = 0; i < N_WATCH; i++) {
        const int p = WATCH[i];
        // 34~39 는 입력 전용이라 내부 풀업이 없다. 보드에 외부 풀업이 없으면
        // 값이 떠다니는데, 그것도 정보다("이 핀은 못 쓴다").
        pinMode(p, (p >= 34) ? INPUT : INPUT_PULLUP);
        watch_prev[i] = digitalRead(p);
        Serial.printf("%d=%d ", p, watch_prev[i]);
    }
    Serial.println("\n[핀탐색] 버튼을 하나씩 눌러보세요 — 바뀐 핀 번호가 찍힙니다.");
}

static void watch_poll(void)
{
    for (int i = 0; i < N_WATCH; i++) {
        const uint8_t v = digitalRead(WATCH[i]);
        if (v == watch_prev[i]) continue;
        watch_prev[i] = v;
        watch_hits[i]++;
        Serial.printf("[핀탐색] GPIO%-2d → %s   (이 핀 변화 %lu회)\n",
                      WATCH[i], v ? "HIGH(뗌)" : "LOW(누름)",
                      (unsigned long)watch_hits[i]);
    }
}

// ───────────────────────────────────────── 앵커 링크
//
// 지금 BLE 점수는 주변 기기 47대의 RSSI 변동에서 나온다. 그런데 그 47대의 대부분은
// **휴대폰**이고, 휴대폰은 스스로 움직인다. 즉 신호가 흔들려도 그게 사람이 지나가서인지
// 그 폰이 주머니 안에서 움직여서인지 구분할 수 없다. 게다가 MAC 을 랜덤화해서
// 같은 기기를 계속 추적할 수도 없다(실측: 96대 중 82대가 이름 없는 랜덤 MAC).
//
// 전자종이 태그는 다르다. **벽에 붙어 있고, 움직이지 않고, MAC 을 안 바꾼다.**
// 그러면 태그 하나가 보드-태그 사이의 **경로 하나**를 대표한다. 그 경로의 RSSI 가
// 흔들리면 그 선분을 가로지른 것이 있다는 뜻이다 — 이게 바이스태틱 감지의 정의다.
//
// 태그가 4대면 서로 다른 4개 경로다. 어느 경로가 흔들리는지 보면 방향까지 좁혀진다.
// 익명 47대의 통계보다 이쪽이 훨씬 해석 가능하다.
#define N_ANCHOR 8
struct Anchor {
    uint8_t  addr[6];
    char     name[16];
    float    mu, sd;      // 지수 이동 평균/편차
    int8_t   last;
    uint32_t n;
    float    z;           // 현재 편차 (시그마 단위)
    bool     used;
};
static Anchor anchors[N_ANCHOR];
static int    n_anchor = 0;
static float  anchor_score = 0.0f;   // 앵커 경로 중 최대 편차

// ESL 태그인가. 0xFEF0 을 광고하거나 MAC 이 FF:FF 로 시작하면 후보다.
static bool is_anchor_addr(const uint8_t *a) { return a[0] == 0xFF && a[1] == 0xFF; }

static void anchor_note(const uint8_t *a, int8_t rssi, const char *name)
{
    int i = -1;
    for (int k = 0; k < n_anchor; k++)
        if (!memcmp(anchors[k].addr, a, 6)) { i = k; break; }
    if (i < 0) {
        if (n_anchor >= N_ANCHOR) return;
        i = n_anchor++;
        memset(&anchors[i], 0, sizeof(Anchor));
        memcpy(anchors[i].addr, a, 6);
        anchors[i].mu = rssi;
        anchors[i].sd = 2.0f;
        anchors[i].n  = 1;
        anchors[i].used = true;
        if (name && name[0]) strncpy(anchors[i].name, name, sizeof anchors[i].name - 1);
        return;
    }
    Anchor &d = anchors[i];
    if (name && name[0] && !d.name[0]) strncpy(d.name, name, sizeof d.name - 1);
    d.last = rssi;
    d.n++;
    const float e = (float)rssi - d.mu;
    // 편차를 먼저 재고 나서 평균을 갱신한다. 순서를 바꾸면 자기 자신을 흡수해
    // 큰 변화가 z 에 안 나타난다.
    d.z = (d.sd > 0.5f) ? fabsf(e) / d.sd : 0.0f;
    const float al = (d.n < 40) ? 0.1f : 0.02f;
    d.mu += al * e;
    d.sd += al * (fabsf(e) - d.sd);

    // 앵커는 넷뿐이라 다중비교 걱정이 작다. 그리고 각 경로가 물리적으로 의미가
    // 있으므로 **최댓값**이 맞다 — 한 경로만 가로막혀도 사람이 있는 것이다.
    // (익명 47대에서는 최댓값이 틀렸다. 개수가 많으면 우연한 극단값이 보장된다.)
    float mx = 0.0f;
    for (int k = 0; k < n_anchor; k++)
        if (anchors[k].used && anchors[k].n > 30 && anchors[k].z > mx) mx = anchors[k].z;
    anchor_score = mx;
}

// BLE 센싱을 BLEScan 콜백으로 받는다. 원시 esp_ble_gap_* 경로를 쓰면
// BLEDevice::init() 이 콜백을 덮어 GATT 업로드와 공존할 수 없다.
class SenseCb : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice ad) override {
        const uint8_t *a = ad.getAddress().getNative();
        const int8_t r = (int8_t)ad.getRSSI();
        ble_note(a, r);
        // 고정 위치 앵커는 따로 센다. 익명 기기 통계에 섞으면 그 안정성이 희석된다.
        if (is_anchor_addr(a)) {
            const String nm = ad.haveName() ? ad.getName() : String("");
            anchor_note(a, r, nm.c_str());
        }
    }
};

// 대역 점수 하나로 모은다. 세 군데에서 같은 식을 되풀이하다가 앵커를 넣을 때
// 한 곳을 빠뜨릴 뻔했다.
//
// 최댓값을 쓰는 이유는 널 지점 때문이다 — 어떤 주파수·경로에서 신호가 안 변해도
// 다른 데서 변하면 사람이 있는 것이다. 단 익명 BLE 기기 점수만은 예외로 이미
// "2σ 넘는 비율" 로 바뀌어 있다(개수가 많으면 최댓값이 다중비교로 오염된다).
static float band_score(void)
{
    float b = 0.0f;
    for (int i = 0; i < N_WCH; i++) if (base_ok[i] && w_dev[i] > b) b = w_dev[i];
    if (ble_dev_score > b) b = ble_dev_score;
    if (anchor_score  > b) b = anchor_score;
    return b;
}

// ───────────────────────────────────────── 전자종이 화면
//
// 이 보드에 화면이 없다는 것이 이 프로젝트 내내 가장 큰 제약이었다. 대역 점수도,
// 채널 일치율도, 추론 지연도 전부 시리얼로만 나왔고 그건 PC 가 붙어 있어야 한다.
//
// BLE 전자선반라벨(Gicisky EPD 2.9" BWR, 실측 4대)을 화면으로 쓴다. 전자종이는
// 갱신이 느리지만(태그당 3.5초) **추세 그래프에는 오히려 알맞다** — 초당 갱신할
// 이유가 없고, 전원을 끊어도 화면이 남는다.
//
// 문제는 공존이다. 참조 구현(사용자 레포)은 BLE 업로드 전에 WiFi 를 완전히
// 내린다(esp_wifi_deinit). 우리는 CSI 를 계속 받아야 하므로 그럴 수 없다.
// 그래서 업로드 동안만 프로미스큐어스를 끄고, 끝나면 되돌린다 — CSI 는 그 몇 초만
// 비고 링은 유지된다.
// 추세를 **네 개의 시간 축으로** 동시에 쌓는다.
//
// 전자종이는 갱신이 느리지만 전원을 끊어도 화면이 남는다. 그러면 네 대를 같은 신호의
// 서로 다른 시간 축에 쓰는 것이 가장 값지다 — 1분 창 하나만 보면 "지금 뭔가 지나갔다"
// 까지만 알 수 있고, 6시간 창이 있으면 "이 방은 밤에 조용하다" 를 알 수 있다.
//
// 각 축은 1초 표본을 N개씩 모아 한 칸을 만든다. 평균이 아니라 **최댓값**을 쓴다 —
// 3초 지나간 사람을 300초로 평균하면 사라진다.
#define TREND_N   72          // 296픽셀 폭에 4픽셀씩이면 72칸이다
#define N_SCALE   4
static const uint16_t SCALE_SEC[N_SCALE]  = { 1, 10, 50, 300 };   // 칸당 초
static const char    *SCALE_NAME[N_SCALE] = { "72 s", "12 min", "1 hr", "6 hr" };
static float    trend[N_SCALE][TREND_N];
static uint32_t trend_n[N_SCALE];
static float    acc_max[N_SCALE];
static uint16_t acc_cnt[N_SCALE];

static GFXcanvas1 *cbw = nullptr, *cred = nullptr;
static uint8_t    *esl_buf = nullptr;
static U8G2_FOR_ADAFRUIT_GFX u8g2;
static EslTag      tags[ESL_MAX_TAG];
static int         n_tags = 0;
static uint32_t    esl_cycle = 0;
static uint32_t    esl_ok = 0, esl_fail = 0;
// 적색을 화면에 **안 그리되, 평면은 비워서 반드시 보낸다.**
//
// 처음에는 "흑백만 보내면 되지" 라고 생각해 적색 평면을 아예 빼고 4736바이트만
// 보냈다. 틀렸다. **전자종이는 잔류형이다.** 적색 평면을 안 보내면 태그의 적색
// 레이어는 지워지지 않고 **이전 내용이 그대로 남는다.** 즉 흑백만 보내기는 적색을
// 끄는 방법이 아니라 예전 적색을 박제하는 방법이었다(실기 관측: 흑백 전용으로
// 바꾼 뒤에도 화면이 계속 빨갰다).
//
// 그래서 적색 평면을 **흰색으로 채워서 함께 보낸다.** 그러면 적색 레이어가 지워진다.
// 대가는 페이로드가 두 배(4736 → 9472)인데, 속도 차이가 없다는 것은 이미 쟀다 —
// BWR 4074ms 대 흑백만 4327ms. 병목이 바이트가 아니라 연결 수립과 태그 자신의
// 리프레시라서 그렇다. 그러니 잃는 것이 없다.
static bool        bw_only = false;   // 평면은 둘 다 보낸다
static bool        draw_red = false;  // 다만 적색에 아무것도 그리지 않는다
static uint32_t    esl_ms[2] = {0, 0}, esl_n[2] = {0, 0};   // [0]=BWR, [1]=흑백만
static uint32_t    esl_parts[2] = {0, 0}, esl_len[2] = {0, 0};
static bool        esl_force = false;      // 버튼으로 즉시 갱신
static int         page_shift = 0;         // 어느 페이지를 어느 태그에 보낼지

// ───────────────────────────────────────── 능동 프로빙
//
// 문제: 완전 수동 수신이라 CSI 프레임이 남의 트래픽에 달려 있다. 실측으로 추론 175회에
// 프레임 부족 건너뜀이 181회였고, ch6 은 유효 창이 19/175 뿐이었다. 조용한 채널에서는
// 2초 창이 16프레임을 못 채운다.
//
// 해결: **우리가 트래픽을 만든다.** 브로드캐스트 프로브 리퀘스트를 쏘면 그 채널의
// AP 들이 프로브 응답을 보낸다. 응답은 우리가 받는 프레임이므로 곧 CSI 다.
// 연결도 비밀번호도 필요 없고, 채널 순환도 그대로 유지된다.
//
// 프로브 응답은 레거시 속도로 오는 경우가 많은데 ESP32 는 L-LTF 에서도 CSI 를 뽑으므로
// 문제가 없어야 한다 — 다만 "없어야 한다" 는 추측이고, 아래 A/B 로 실측한다.
static bool probe_on = true;          // 30초마다 뒤집어 A/B 로 효과를 잰다
static uint32_t probe_tx = 0, probe_fail = 0;
static uint32_t rate_frames[2] = {0, 0};   // [0]=프로빙 끔, [1]=켬
static uint32_t rate_ms[2] = {0, 0};

// 채널별로 가장 센 AP 의 MAC. 유니캐스트 널 프레임을 보낼 상대다.
// CSI 콜백의 info->mac 이 송신자 MAC 이므로 별도 스캔이 필요 없다.
static uint8_t  ap_mac[N_WCH][6] = {{0}};
static int8_t   ap_rssi[N_WCH] = { -127, -127, -127 };
static bool     ap_ok[N_WCH] = { false, false, false };

// 프로빙 방식. 두 메커니즘이 근본적으로 다르다:
//   PROBE_BCAST = 브로드캐스트 프로브 요청 → AP **소프트웨어**가 응답을 만든다.
//                 AP 재량이라 억제될 수 있다(실측 1.13배, 사실상 효과 없음).
//   PROBE_NULL  = AP 로 유니캐스트 널 데이터 → AP **하드웨어**가 ACK 한다.
//                 802.11 규약상 주소가 맞으면 무조건 답한다. 연결 여부 무관.
static uint8_t probe_mode = 1;        // 0=브로드캐스트, 1=널+ACK

// 와일드카드 프로브 리퀘스트. SSID 를 비우면 모든 AP 가 답한다.
static uint8_t probe_frame[] = {
    0x40, 0x00,                             // FC: 관리 프레임, 서브타입 4 = 프로브 요청
    0x00, 0x00,                             // duration
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,     // addr1 DA = 브로드캐스트
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // addr2 SA = 우리 MAC (setup 에서 채운다)
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,     // addr3 BSSID = 브로드캐스트
    0x00, 0x00,                             // seq ctrl
    0x00, 0x00,                             // SSID 엘리먼트: id 0, 길이 0 (와일드카드)
    0x01, 0x08, 0x02, 0x04, 0x0b, 0x16,     // 지원 속도: 1,2,5.5,11,
    0x0c, 0x12, 0x18, 0x24,                 //            6,9,12,18 Mbps
};

// 널 데이터 프레임(type=data, subtype=4, ToDS=1). 본문이 없어 24바이트다.
// AP 는 주소가 자기 것이면 하드웨어에서 ACK 를 낸다 — 연결 상태를 보지 않는다.
static uint8_t null_frame[] = {
    0x48, 0x01,                             // FC: 데이터, 서브타입4=널, ToDS=1
    0x00, 0x00,                             // duration
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // addr1 = BSSID (매번 채운다)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // addr2 = 우리 MAC
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // addr3 = DA
    0x00, 0x00,                             // seq ctrl
};

static void probe_send(void)
{
    if (probe_mode == 1) {
        const uint8_t ci = cur_wch_i;
        if (!ap_ok[ci]) return;             // 아직 이 채널의 AP 를 모른다
        memcpy(null_frame + 4,  ap_mac[ci], 6);
        memcpy(null_frame + 16, ap_mac[ci], 6);
        const esp_err_t e = esp_wifi_80211_tx(WIFI_IF_STA, null_frame,
                                              sizeof(null_frame), true);
        if (e == ESP_OK) probe_tx++; else probe_fail++;
        return;
    }
    // en_sys_seq=true 로 두면 시퀀스 번호를 하드웨어가 채운다.
    const esp_err_t e = esp_wifi_80211_tx(WIFI_IF_STA, probe_frame,
                                          sizeof(probe_frame), true);
    if (e == ESP_OK) probe_tx++; else probe_fail++;
}

// ───────────────────────────────────────── WiFi CSI
static void IRAM_ATTR csi_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf || info->len < 8) return;
    const uint8_t ci = cur_wch_i;
    w_pkt[ci]++;
    rate_frames[probe_on ? 1 : 0]++;
    // info->mac 은 **받은 프레임의 송신자**다. 대개 AP 가 아니라 다른 단말이고,
    // 요즘 단말은 MAC 을 랜덤화한다(첫 옥텟의 로컬관리 비트가 켜져 있다).
    // 여기서 그걸 AP 로 착각해 널 프레임을 보냈다가 ACK 을 못 받았다(실측 1.07배,
    // 송신 실패 54%). 진짜 BSSID 는 setup 의 스캔에서 채운다.

    // 서브캐리어 개수는 **고정이 아니다.** 20MHz 프레임은 64개, HT40(40MHz)은 128개다.
    // 실측으로 AP 가 HT40 으로 올라가면서 sc64 → sc128 로 바뀌었다.
    //
    // 이걸 그냥 받으면 조용히 망가진다. 모델은 64개로 학습됐고, 링의 한 행은
    // 2*n_sc 바이트로 고정 배치되므로 프레임마다 개수가 다르면 행이 어긋난다.
    // 증상은 "인식률이 왜 이러지" 로만 보이고 원인이 감춰진다.
    //
    // 그래서 **모델이 기대하는 개수만 받는다.** 나머지는 세어서 알린다 —
    // 조용히 버리면 "왜 프레임이 안 들어오나" 를 또 오해하게 된다.
    const int sc = info->len / 2;
    if (!n_sc) n_sc = (uint8_t)((sc > MAX_SC) ? MAX_SC : sc);
    if (sc != (int)n_sc) { sc_mismatch++; sc_other = (uint16_t)sc; return; }
    const int n = n_sc;
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
// 광고 하나를 반영한다. 호출자는 BLEScan 콜백이다 — 원시 esp_ble_gap_* 경로는
// BLEDevice::init() 이 콜백을 덮어 GATT 업로드와 공존하지 못한다.
void ble_note(const uint8_t *bda, int8_t rssi)
{
    ble_pkt++;

    int idx = -1;
    for (int i = 0; i < n_ble; i++)
        if (!memcmp(ble_dev[i].addr, bda, 6)) { idx = i; break; }
    if (idx < 0) {
        if (n_ble >= BLE_MAX_DEV) return;
        idx = n_ble++;
        memcpy(ble_dev[idx].addr, bda, 6);
        ble_dev[idx].mu = rssi;
        ble_dev[idx].sd = 2.0f;
        ble_dev[idx].n = 1;
        ble_dev[idx].last = rssi;
        return;
    }
    ble_dev_t *d = &ble_dev[idx];
    const float r = rssi;
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
    // 파이프라인 정확성. 사람 없이 자동으로 나오는 유일한 지표이므로
    // 마크 표본이 없어 아래에서 조기 반환하더라도 이건 먼저 찍는다.
    if (n_anchor) {
        // 앵커는 위치가 고정된 링크다. 이 z 가 조용하면 방이 조용한 것이고,
        // 익명 기기 점수와 달리 "그 기기가 움직였을 뿐" 이라는 변명이 안 통한다.
        Serial.printf("\n[앵커] 고정 위치 링크 %d개 — 최대 z %.2f\n", n_anchor, anchor_score);
        for (int k = 0; k < n_anchor; k++) {
            const Anchor &a = anchors[k];
            Serial.printf("   %02X:%02X:%02X:%02X:%02X:%02X %-14s "
                          "평균 %6.1f dBm  편차 %4.2f  z %4.2f  표본 %lu\n",
                          a.addr[0], a.addr[1], a.addr[2], a.addr[3], a.addr[4], a.addr[5],
                          a.name[0] ? a.name : "(이름없음)", a.mu, a.sd, a.z,
                          (unsigned long)a.n);
        }
    }
    if (cls_tot) {
        Serial.printf("[모델] 채널 일치 %lu/%lu = %.1f%% (무작위 기대 %.0f%%)\n",
                      (unsigned long)cls_hit, (unsigned long)cls_tot,
                      100.0 * cls_hit / cls_tot, 100.0 / N_WCH);
        Serial.println("  행=창을뜬채널 열=예측  (대각선이 커야 파이프라인이 맞다)");
        for (int a = 0; a < N_WCH; a++) {
            uint32_t r = 0;
            for (int b = 0; b < N_WCH; b++) r += cls_cm[a][b];
            Serial.printf("   ch%d(%4d):", a, WCH_MHZ[a]);
            for (int b = 0; b < N_WCH; b++)
                Serial.printf(" %5lu", (unsigned long)cls_cm[a][b]);
            Serial.printf("   합%4lu  대각 %5.1f%%\n", (unsigned long)r,
                          r ? 100.0 * cls_cm[a][a] / r : 0.0);
        }
        Serial.printf("  창: %lu프레임 %lums, 프레임 부족으로 건너뜀 %lu회\n",
                      (unsigned long)win_n, (unsigned long)win_span_ms,
                      (unsigned long)win_skip);
    }
    if (rate_ms[0] || rate_ms[1]) {
        const double r0 = rate_ms[0] ? 1000.0 * rate_frames[0] / rate_ms[0] : 0.0;
        const double r1 = rate_ms[1] ? 1000.0 * rate_frames[1] / rate_ms[1] : 0.0;
        Serial.printf("[프로빙] 끔 %.1f Hz (%lu프레임/%lus) | 켬 %.1f Hz (%lu프레임/%lus)"
                      "  →  %.2f배\n",
                      r0, (unsigned long)rate_frames[0], (unsigned long)(rate_ms[0]/1000),
                      r1, (unsigned long)rate_frames[1], (unsigned long)(rate_ms[1]/1000),
                      r0 > 0 ? r1 / r0 : 0.0);
        Serial.printf("         방식 %s, 송신 %lu회, 실패 %lu회\n",
                      probe_mode == 1 ? "널+ACK" : "브로드캐스트프로브",
                      (unsigned long)probe_tx, (unsigned long)probe_fail);
        for (int i = 0; i < N_WCH; i++)
            if (ap_ok[i])
                Serial.printf("         ch%d AP %02x:%02x:%02x:%02x:%02x:%02x %ddBm\n",
                              i, ap_mac[i][0], ap_mac[i][1], ap_mac[i][2],
                              ap_mac[i][3], ap_mac[i][4], ap_mac[i][5], ap_rssi[i]);
    }
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
            blink_ack(k + 1);
            Serial.printf("[key] K%d 길게 (GPIO%d)\n", k + 1, KEY_PIN[k]);
            if (k == 0) reset_baseline();
            else if (k == 3) {
                ch_lock = !ch_lock;
                Serial.printf("[key] 채널 %s\n", ch_lock ? "고정" : "순환");
            } else if (k == 4) print_stats();
            else if (k == 5) {
                // 웹 모드. 채널이 고정되므로 기본이 아니라 눌러서 켜는 모드다.
                if (web_running()) web_stop(); else web_start(WCH[1]);   // ch6
            }
        } else if (!d && down[k]) {
            down[k] = false;
            if (longed[k]) continue;
            // 어느 키가 먹었는지 **먼저** 알린다. 아래 동작이 무엇이든 응답은 온다.
            blink_ack(k + 1);
            Serial.printf("[key] K%d 눌림 (GPIO%d)\n", k + 1, KEY_PIN[k]);
            switch (k) {
            case 0:
                marked = true;
                mark_until = millis() + MARK_MS;
                Serial.printf("\n[검증] 마크 시작 — 앞으로 %d초 동안 보드 앞을 "
                              "왕복해 주세요. 끝나면 스스로 판정합니다.\n",
                              MARK_MS / 1000);
                break;
            case 1:
                if (thresh > 1.0f) thresh -= 0.5f;
                Serial.printf("[key] 임계값 %.1f\n", thresh);
                store_save("임계값");
                break;
            case 2:
                thresh += 0.5f;
                Serial.printf("[key] 임계값 %.1f\n", thresh);
                store_save("임계값");
                break;
            case 3:
                // 화면이 생긴 뒤로 가장 자주 필요한 것은 "지금 갱신해라" 다.
                // 90초를 기다리지 않고 곧바로 네 태그를 다시 그린다.
                esl_force = true;
                Serial.println("[key] 화면 즉시 갱신 요청");
                break;
            case 4:
                // 어느 페이지를 어느 태그에 보낼지 돌린다. 태그를 벽에 붙여둔
                // 상태에서 내용만 바꿀 수 있어야 배치를 실험할 수 있다.
                page_shift = (page_shift + 1) % 4;
                esl_force = true;
                Serial.printf("[key] 페이지 회전 %d\n", page_shift);
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
    // **LLTF 만 받는다.** 셋 다 켜면 한 프레임에 세 벌(64+64+64=192 서브캐리어)이
    // 실려 오고, 프레임 종류에 따라 64/128/192 로 개수가 들쭉날쭉해진다.
    // 모델은 64개로 학습됐으므로 그걸 버리면 파이프라인이 굶고(실측: 창이 4프레임까지
    // 줄고 추론이 159회에서 멈췄다), 안 버리면 링의 행이 어긋난다.
    // 애초에 한 벌만 오게 하는 것이 답이다. 학습 데이터도 LLTF 64개였다.
    cfg.lltf_en = true; cfg.htltf_en = false; cfg.stbc_htltf2_en = false;
    cfg.ltf_merge_en = true; cfg.channel_filter_en = true;
        cfg.manu_scale = false; cfg.shift = 0;
    // ACK 의 CSI 를 받는다. PROBE_NULL 방식은 이게 켜져 있어야 의미가 있다 —
    // 우리가 보낸 널 프레임에 대한 AP 의 ACK 가 곧 우리가 받는 프레임이다.
    cfg.dump_ack_en = true;
    if (esp_wifi_set_csi_config(&cfg) != ESP_OK) {
        Serial.println("CSI 설정 실패. 중단."); while (1) delay(1000);
    }
    esp_wifi_set_csi_rx_cb(csi_cb, nullptr);

    // ── 전자종이 화면 준비. 캔버스와 페이로드는 PSRAM 으로 보낸다.
    esl_buf = (uint8_t *)heap_caps_malloc(ESL_BYTES, MALLOC_CAP_SPIRAM);
    cbw  = new GFXcanvas1(ESL_W, ESL_H);
    cred = new GFXcanvas1(ESL_W, ESL_H);
    if (!esl_buf || !cbw || !cred || !cbw->getBuffer() || !cred->getBuffer()) {
        Serial.println("[화면] 버퍼 할당 실패 — 화면 없이 계속한다");
        esl_buf = nullptr;
    } else {
        Serial.println("[화면] 캔버스 준비 (296x128 x2 평면)");
    }

    // 프로브 요청의 SA 를 우리 MAC 으로 채운다. 남의 MAC 을 쓰면 커널이 거부한다.
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    memcpy(probe_frame + 10, mac, 6);
    memcpy(null_frame + 10, mac, 6);

    // ── 실제 AP 를 스캔해서 채널별 BSSID 를 얻는다.
    //    널+ACK 프로빙은 상대가 진짜 AP 여야 성립한다. 스캔은 여기서 한 번만 한다
    //    (스캔 중에는 채널이 마음대로 바뀌므로 순환 루프와 같이 돌릴 수 없다).
    Serial.println("[프로빙] AP 스캔...");
    esp_wifi_set_promiscuous(false);
    const int n_ap = WiFi.scanNetworks(false, true);
    for (int i = 0; i < n_ap; i++) {
        const int ch = WiFi.channel(i);
        for (int c = 0; c < N_WCH; c++) {
            if (ch != WCH[c]) continue;
            const int8_t r = (int8_t)WiFi.RSSI(i);
            if (!ap_ok[c] || r > ap_rssi[c]) {
                ap_rssi[c] = r; ap_ok[c] = true;
                memcpy(ap_mac[c], WiFi.BSSID(i), 6);
            }
        }
    }
    WiFi.scanDelete();
    esp_wifi_set_promiscuous(true);
    Serial.printf("[프로빙] AP %d개 중 대상 채널에서:\n", n_ap);
    for (int c = 0; c < N_WCH; c++) {
        if (ap_ok[c])
            Serial.printf("  ch%d(%d) %02x:%02x:%02x:%02x:%02x:%02x %ddBm%s\n",
                          c, WCH_MHZ[c], ap_mac[c][0], ap_mac[c][1], ap_mac[c][2],
                          ap_mac[c][3], ap_mac[c][4], ap_mac[c][5], ap_rssi[c],
                          (ap_mac[c][0] & 0x02) ? "  (랜덤화 MAC — AP 답지 않다)" : "");
        else
            Serial.printf("  ch%d(%d) AP 없음 — 이 채널은 널 프로빙을 못 한다\n",
                          c, WCH_MHZ[c]);
    }
    Serial.printf("[프로빙] SA %02x:%02x:%02x:%02x:%02x:%02x, 프레임 %u바이트\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                  (unsigned)sizeof(probe_frame));
    esp_wifi_set_csi(true);
    esp_wifi_set_channel(WCH[0], WIFI_SECOND_CHAN_NONE);
    Serial.println("[wifi] CSI 활성화");

    // ── BLE. 스택을 **하나로** 통일한다.
    //
    // multiband_sense 는 원시 esp_ble_gap_* 콜백으로 광고를 셌다. 그런데 태그에
    // 이미지를 올리려면 GATT 클라이언트가 필요하고, BLEDevice::init() 은 자기
    // GAP 콜백을 등록해 우리 것을 덮어버린다. 두 스택을 섞으면 한쪽이 조용히 죽는다.
    //
    // 그래서 센싱도 Arduino BLEScan 의 콜백으로 옮겼다. 거기서도 주소와 RSSI 가
    // 그대로 오므로 기기별 RSSI 시계열은 똑같이 만들 수 있다.
    BLEDevice::init("CABIN-NODE");
    {
        BLEScan *sc = BLEDevice::getScan();
        sc->setAdvertisedDeviceCallbacks(new SenseCb(), true);   // 중복도 받는다
        sc->setActiveScan(false);        // 수동 — 우리가 프로브를 쏘지 않는다
        sc->setInterval(0x50);
        sc->setWindow(0x30);
        // 무한 스캔. 화면 갱신 때만 esl_scan 이 잠깐 가져간다.
        sc->start(0, nullptr, false);
        Serial.println("[ble] 수동 스캔 시작 (광고 채널 37/38/39, BLEScan 경로)");
    }

    // ── LED 를 **먼저** 설정한다. GPIO13 이 GPIO22 상태에 끌리므로 GPIO22 를
    //    확정한 뒤에 키를 읽어야 값이 안정된다.
    for (int i = 0; i < N_LED; i++) { pinMode(LED_PIN[i], OUTPUT); digitalWrite(LED_PIN[i], LOW); }

    // ── 버튼 6개. 매핑은 핀 변화 감시기로 실기 확정했다.
    for (int k = 0; k < N_KEYS; k++)
        pinMode(KEY_PIN[k], (KEY_PIN[k] == 36 || KEY_PIN[k] == 5) ? INPUT : INPUT_PULLUP);
    // **자동 비활성을 없앴다.**
    //
    // 전에는 부팅 시 LOW 로 읽히는 키를 배선 의심으로 껐다. 그게 GPIO13(KEY2)을
    // 잡았는데, 핀 변화 감시기로 확인해보니 **KEY2 는 정상 작동한다.** LOW 로 읽힌
    // 이유는 순서였다 — 키 스캔이 LED 핀 설정보다 먼저 돌아서 GPIO22 가 떠 있을 때
    // 읽었고, GPIO13 은 GPIO22 상태에 끌린다(이 보드의 알려진 특성).
    //
    // 그래서 LED 를 먼저 설정한 뒤에 읽고, 값이 이상해도 끄지 않고 알리기만 한다.
    // 근거 없는 자동 비활성은 "안 되는 키" 를 스스로 만들어냈다.
    Serial.print("[keys] ");
    for (int k = 0; k < N_KEYS; k++) {
        const int v = digitalRead(KEY_PIN[k]);
        key_ok[k] = true;
        Serial.printf("K%d(GPIO%d)=%d%s ", k + 1, KEY_PIN[k], v,
                      v == LOW ? "[눌림? 확인]" : "");
    }
    Serial.printf("— %d개 전부 사용\n", N_KEYS);
    Serial.println("  K1짧게=마크토글  K1길게=기준선재학습  K2=임계값−  K3=임계값+");
    Serial.println("  K4짧게=화면즉시갱신 K4길게=채널고정  K5짧게=페이지회전 K5길게=통계");
    Serial.println("  K6짧게=통계  K6길게=웹모드(SoftAP \"CABIN-NODE\"/cabinnode, 채널 고정)");

    store_load();
    watch_init();

    // ── LED 후보 전부 출력으로

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

// 대역 점수 추세를 그린다. 전자종이가 가장 잘하는 일이다.
static void draw_trend(int sl, int x0, int y0, int w, int h)
{
    cbw->drawFastHLine(x0, y0 + h, w, 0);        // 시간축
    cbw->drawFastVLine(x0, y0, h, 0);            // 값축

    // 눈금: 임계값 선. 점선으로 그려 데이터와 구별한다.
    const float vmax = 6.0f;                     // 대역 점수 상한(실측 최대 3 근처)
    const int ty = y0 + h - (int)(h * (thresh / vmax));
    if (ty > y0 && ty < y0 + h)
        for (int x = x0; x < x0 + w; x += 4) cbw->drawPixel(x, ty, 0);

    if (!trend_n[sl]) return;
    const uint32_t tn = trend_n[sl];
    const int n = (tn < TREND_N) ? (int)tn : TREND_N;
    const int step = (w - 2) / TREND_N;
    for (int i = 0; i < n; i++) {
        // 오래된 것이 왼쪽. 링 버퍼를 시간순으로 읽는다.
        const int idx = (int)((tn - n + i) % TREND_N);
        float v = trend[sl][idx];
        if (v > vmax) v = vmax;
        const int bh = (int)(h * (v / vmax));
        const int x = x0 + 2 + i * step;
        // 임계 초과는 채운 막대, 아래는 얇은 막대. 흑백만으로 구별된다.
        if (v >= thresh) {
            for (int k = 0; k < step && k < 3; k++)
                cbw->drawFastVLine(x + k, y0 + h - bh, bh, 0);
        } else if (bh > 0) {
            cbw->drawFastVLine(x, y0 + h - bh, bh, 0);
        }
    }
}

static void render(int slot, const EslTag &t)
{
    // 두 평면의 극성이 반대다. BW 는 1=흰색, RED 는 1=적색 없음.
    // 적색 평면을 0 으로 채웠더니 화면 전체가 빨개졌다(실기 관측).
    cbw->fillScreen(1);
    cred->fillScreen(1);

    u8g2.begin(*cbw);
    u8g2.setFontMode(1);
    u8g2.setForegroundColor(0);
    u8g2.setBackgroundColor(1);

    u8g2.setFont(u8g2_font_helvB14_tf);
    u8g2.setCursor(4, 16);
    u8g2.printf("CABIN NODE #%d", slot);
    {
        char nb[24];
        snprintf(nb, sizeof nb, "%lu", (unsigned long)esl_cycle);
        u8g2.setCursor(ESL_W - u8g2.getUTF8Width(nb) - 4, 16);
        u8g2.print(nb);
    }
    cbw->drawFastHLine(0, 21, ESL_W, 0);

    u8g2.setFont(u8g2_font_helvR10_tf);
    // 네 대 모두 **같은 신호의 다른 시간 축**을 그린다.
    //
    // 처음에는 태그마다 다른 내용(그래프/모델/전파이웃/태그정보)을 넣었다. 그런데
    // 그래프가 한 대에만 나오니 나머지 세 대는 한 번 보면 다시 볼 이유가 없는 표였다.
    // 전자종이는 전원을 끊어도 화면이 남고 갱신이 느리다 — 그 성질에 맞는 것은
    // **긴 시간 축**이다. 6시간 창은 화면을 자꾸 고치는 기기로는 못 보여준다.
    //
    // 상세 표는 K5(페이지 회전)로 볼 수 있게 남겨뒀다.
    const int sl = slot % N_SCALE;
    u8g2.setFont(u8g2_font_helvR10_tf);

    u8g2.setCursor(4, 34);
    u8g2.printf("%s window   band %.1f / thr %.1f   %lu bars",
                SCALE_NAME[sl], band_score(), thresh, (unsigned long)trend_n[sl]);

    draw_trend(sl, 4, 40, ESL_W - 8, 58);

    // 앵커 4경로 편차. 어느 경로가 흔들리는지가 방향 정보다.
    {
        int x = 4;
        for (int k = 0; k < n_anchor && k < 4; k++) {
            const int w = (ESL_W - 8) / 4 - 4;
            const int bh = (int)(14.0f * fminf(anchors[k].z, 4.0f) / 4.0f);
            cbw->drawRect(x, 102, w, 14, 0);
            if (bh > 0) cbw->fillRect(x + 1, 102 + 14 - bh, w - 2, bh, 0);
            u8g2.setFont(u8g2_font_5x7_tf);
            u8g2.setCursor(x + 2, 100);
            u8g2.printf("%02X %.1f", anchors[k].addr[5], anchors[k].z);
            x += w + 4;
        }
    }

    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.setCursor(4, 126);
    u8g2.printf("boot %lu  up %luh%02lum | infer %lums %lu/%lu ch-match | "
                "ble %d | sc%d | mark %lu/%lu",
                (unsigned long)boot_n, (unsigned long)(millis() / 3600000UL),
                (unsigned long)(millis() / 60000UL % 60),
                (unsigned long)infer_ms, (unsigned long)cls_hit,
                (unsigned long)cls_tot, n_ble, n_sc,
                (unsigned long)m_n, (unsigned long)u_n);

    // ── 적색 평면은 위에서 fillScreen(1)(=적색 없음)로 비워뒀고, 그대로 보낸다.
    //    그래야 이전 회차의 적색이 지워진다. 여기서 return 하면 "안 그린다" 이지
    //    "안 보낸다" 가 아니다 — 그 구분이 이 화면의 핵심이다.
    if (!draw_red) return;

    // ── 적색을 쓸 자격.
    //
    // 전자종이에서 적색의 유일한 장점은 **읽지 않아도 눈에 들어온다**는 것이다.
    // 숫자·그래프·라벨은 가까이서 읽으므로 검정이 더 선명하고 페이로드도 절반이다.
    // 그래서 적색을 쓸 값이 있는 것은 "안 봐도 알아야 하는 이진 상태" 하나뿐이다.
    //
    // 그런데 지금 그 알람(대역 점수 임계 초과)은 **실제 움직임으로 검증된 적이 없다**.
    // 검증 안 된 감지기에 적색을 걸면 노이즈에도 빨개지고, 그러면 적색이 아무
    // 의미가 없어진다. 그래서 **감지기가 스스로 자격을 증명해야** 적색을 쓴다:
    // K1 로 마크한 구간과 안 한 구간의 분리도(Cohen's d)가 0.8 을 넘을 때만.
    if (bw_only) return;
    if (t.m && !t.m->red) return;    // BW 모델
    if (!(m_n >= 10 && u_n >= 10)) return;   // 아직 대조군이 없다
    {
        const double mm = m_sum / m_n, um = u_sum / u_n;
        const double sm = sqrt(fmax(m_sq / m_n - mm * mm, 0.0));
        const double su = sqrt(fmax(u_sq / u_n - um * um, 0.0));
        const double pooled = sqrt((sm * sm + su * su) / 2.0);
        const double d = (pooled > 1e-9) ? (mm - um) / pooled : 0.0;
        if (d < 0.8) return;         // 감지기가 아직 자격을 못 얻었다
    }
    u8g2.begin(*cred);
    u8g2.setFontMode(1);
    u8g2.setForegroundColor(0);       // 적색 평면에서는 0 이 "칠한다" 다
    u8g2.setBackgroundColor(1);
    if (band_score() >= thresh) {
        u8g2.setFont(u8g2_font_helvB14_tf);
        u8g2.setCursor(ESL_W - 90, 16);
        u8g2.print("MOTION");
    }
    cred->drawFastHLine(0, 126, ESL_W, 0);
}

// 화면 갱신. 업로드 동안만 프로미스큐어스를 내린다 — BLE GATT 전송과 WiFi
// 스니핑을 동시에 하면 한쪽이 굶는다. 링은 유지되므로 CSI 는 몇 초만 빈다.
static void esl_refresh(void)
{
    if (!cbw || !esl_buf) return;

    // 스캔 소유권을 넘겨받는다.
    //
    // 센싱은 무한 스캔(start(0, ...))을 돌리고 있다. 그 상태에서 esl_scan 이 또
    // start() 를 부르면 보드가 조용히 멈춘다(실측: 화면 갱신 시각에 로그가 끊겼다).
    // 게다가 esl_scan 은 끝나면서 콜백을 nullptr 로 지우므로 센싱이 영구히 죽는다.
    // 그래서 여기서 멈추고, 끝난 뒤 콜백과 무한 스캔을 되돌린다.
    BLEScan *sc = BLEDevice::getScan();
    sc->stop();
    delay(200);

    esp_wifi_set_promiscuous(false);
    n_tags = esl_scan(tags, ESL_MAX_TAG, 6);
    Serial.printf("\n[화면] 태그 %d대\n", n_tags);
    for (int i = 0; i < n_tags; i++) {
        render((i + page_shift) % 4, tags[i]);
        const size_t len = esl_pack(tags[i], *cbw, *cred, esl_buf, bw_only);
        uint32_t ms = 0, parts = 0;
        const EslResult r = esl_upload(tags[i].addr, esl_buf, len, &ms, &parts);
        if (r == ESL_OK) {
            esl_ok++;
            // 첫 전송은 파트 크기 협상이 20 으로 나오는 일이 있어(부팅 직후) 21초가
            // 걸린다. 그건 코덱 비교를 오염시키므로 정상 협상만 센다.
            // 첫 전송은 협상이 20 으로 나와 21초가 걸리는 일이 있다(부팅 직후).
            // 그걸 섞으면 비교가 오염되므로 정상 협상만 센다.
            if (parts <= 64) {
                const int arm = bw_only ? 1 : 0;
                esl_ms[arm] += ms; esl_n[arm]++; esl_parts[arm] = parts; esl_len[arm] = len;
            }
        } else esl_fail++;
        Serial.printf("  #%d %-14s %s  %s  %lu파트 %lums\n", i, tags[i].name,
                      tags[i].m ? tags[i].m->model : "?", esl_result_name(r),
                      (unsigned long)parts, (unsigned long)ms);
    }
    esl_cycle++;
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(WCH[cur_wch_i], WIFI_SECOND_CHAN_NONE);

    // 센싱 스캔을 되돌린다. 이걸 빠뜨리면 BLE 점수가 영원히 0 이 되는데,
    // WiFi 쪽은 계속 도니까 "BLE 만 이상하다" 로 보여 원인을 찾기 어렵다.
    sc->setAdvertisedDeviceCallbacks(new SenseCb(), true);
    sc->setActiveScan(false);
    sc->start(0, nullptr, false);
    if (esl_n[0] && esl_n[1])
        Serial.printf("[화면] 평균 전송: BWR %lums (%lu회, %lu바이트/%lu파트)"
                      " | 흑백만 %lums (%lu회, %lu바이트/%lu파트)  →  %.2f배 빠름\n",
                      (unsigned long)(esl_ms[0] / esl_n[0]), (unsigned long)esl_n[0],
                      (unsigned long)esl_len[0], (unsigned long)esl_parts[0],
                      (unsigned long)(esl_ms[1] / esl_n[1]), (unsigned long)esl_n[1],
                      (unsigned long)esl_len[1], (unsigned long)esl_parts[1],
                      (double)(esl_ms[0] / esl_n[0]) / (double)(esl_ms[1] / esl_n[1]));
    Serial.println("[화면] 센싱 스캔 복귀");
}

void loop()
{
    // WiFi 채널 순환. 채널마다 기준선이 따로이므로 전환 후 잠깐은 값이 흔들린다.
    // 채널 체류 시간. 짧으면 기준선 학습이 오래 걸리고, 길면 감지 주기가 느려진다.
    // 웹 모드에서는 채널을 고정한다 — SoftAP 가 채널을 옮기면 붙은 폰이 끊긴다.
    if (!web_running() && !ch_lock && millis() - ch_t > CH_DWELL_MS) {
        ch_t = millis();
        cur_wch_i = (uint8_t)((cur_wch_i + 1) % N_WCH);
        esp_wifi_set_channel(WCH[cur_wch_i], WIFI_SECOND_CHAN_NONE);
    }

    // ── 능동 프로빙 A/B. 30초마다 켜고 끄면서 프레임률을 따로 누적한다.
    //    "프로빙이 효과가 있다" 를 주장하려면 같은 환경에서 번갈아 재야 한다 —
    //    한쪽만 재고 예전 숫자와 비교하면 시간대·트래픽 변화와 구분이 안 된다.
    static uint32_t ab_t = 0, probe_t = 0;
    if (ab_t == 0) ab_t = millis();
    if (millis() - ab_t > 30000) {
        rate_ms[probe_on ? 1 : 0] += millis() - ab_t;
        ab_t = millis();
        probe_on = !probe_on;
    }
    // 초당 10회. 브로드캐스트 프로브 하나에 채널의 AP 여러 대가 답할 수 있으므로
    // 이것만으로도 프레임률이 크게 오를 수 있다. 전파를 아끼려면 이 정도가 적당하다.
    // 널+ACK 는 송신 1회 = 응답 1회이므로 송신 주기가 곧 프레임률 상한이다.
    // 20ms(50Hz)면 2초 창에 100프레임 — 학습 창(중앙 35프레임)보다 촘촘해진다.
    // 20ms(50Hz)에서는 송신 큐가 차서 54%가 실패했다. 실패한 송신은 전파도 안 쓰고
    // 응답도 못 받으니 그냥 낭비다. 40ms(25Hz)면 2초 창에 50프레임으로 충분하다.
    const uint32_t probe_iv = (probe_mode == 1) ? 40 : 100;
    if (probe_on && millis() - probe_t >= probe_iv) { probe_t = millis(); probe_send(); }

    web_poll();     // 요청 처리. 막지 않으므로 CSI 콜백을 방해하지 않는다.
    watch_poll();   // 50Hz 로 훑는다. 사람 손가락에는 충분하다.
    blink_poll();   // LED 응답 큐. 막지 않으므로 CSI 를 놓치지 않는다.

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
    // 전환 직후에는 시도조차 하지 않는다. 시도하면 건너뜀 카운터만 올라가고
    // "왜 이렇게 많이 건너뛰나" 를 오해하게 만든다.
    if (infer_ready && ring_w > 40 && millis() - ch_t > CH_SETTLE_MS) {
        // 현재 채널의 프레임 인덱스를 최신순으로 모은다. 인덱스만 담으므로 작다.
        uint16_t idxs[CF_MAX_IN];
        int k = 0;
        // 아래에서 6초마다 채널이 바뀌므로 지금 값을 박아둔다. 안 그러면
        // 프레임은 A 채널인데 정답 라벨이 B 가 되는 경우가 생긴다.
        const uint8_t win_ch = cur_wch_i;
        const uint32_t total = ring_w;
        const uint32_t have = (total < RING_N) ? total : RING_N;
        // **시간으로 자른다.** 학습 창은 CSI_WIN_SEC(2초)다. 시간 제한 없이
        // "현재 채널 프레임 128개" 를 긁으면 채널이 6초 머물고 18초 뒤 돌아오므로
        // 18초 구멍이 여러 개 뚫린 수십 초 구간이 되고, 보간이 그 구멍을 가로질러
        // 직선으로 메운다. 학습 분포와 완전히 다른 것이 모델에 들어간다.
        uint32_t t_new = 0;
        for (uint32_t i2 = 0; i2 < have; i2++) {
            const uint32_t idx = (total - 1 - i2) % RING_N;
            if (ring_ch[idx] == win_ch) { t_new = ring_ms[idx]; break; }
        }
        const uint32_t t_cut = (t_new > (uint32_t)(CSI_WIN_SEC * 1000.0f))
                             ? t_new - (uint32_t)(CSI_WIN_SEC * 1000.0f) : 0;
        for (uint32_t i2 = 0; i2 < have && k < CF_MAX_IN; i2++) {
            const uint32_t idx = (total - 1 - i2) % RING_N;
            // 채널이 섞이면 주파수 응답이 섞여 무의미하다 — 현재 채널만.
            if (ring_ch[idx] != win_ch) continue;
            if (ring_ms[idx] < t_cut) break;   // 링은 시간순이므로 여기서 멈춘다
            idxs[k++] = (uint16_t)idx;
        }
        win_span_ms = (k >= 2) ? (ring_ms[idxs[0]] - ring_ms[idxs[k - 1]]) : 0;
        win_n = k;
        // 학습과 같은 최소 프레임 수를 요구한다(CSI_MIN_PKT). 8은 근거 없는 수였다.
        if (k < CSI_MIN_PKT) win_skip++;
        if (k >= CSI_MIN_PKT) {
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
                // 창을 뜬 채널이 정답이다. 창 안 프레임은 전부 이 채널만이다.
                last_win_ch = (int)win_ch;
                if (last_cls >= 0 && last_cls < N_WCH && win_ch < N_WCH) {
                    cls_cm[win_ch][last_cls]++;
                    cls_tot++;
                    if (last_cls == (int)win_ch) cls_hit++;
                }
                infer_ms = millis() - t0;
                infer_n++;
            }
        }
    }

    // 마크 자동 종료. 끝나는 즉시 판정하고 저장한다 — 사람이 결과를 보러 돌아올
    // 필요가 없어야 한다.
    if (marked && (int32_t)(millis() - mark_until) >= 0) {
        marked = false;
        Serial.println("\n[검증] 마크 종료 — 판정합니다.");
        print_stats();
        store_save("마크 종료");
        esl_force = true;      // 화면도 곧바로 갱신해 결과를 보여준다
    }

    // ── 웹 스냅샷을 채운다. 웹 코드가 센싱 내부를 직접 들여다보지 않게 여기서만 쓴다.
    {
        g_snap.n_sc = n_sc;
        // 마지막으로 받은 프레임의 진폭을 dB 스케일로 옮긴다. 링의 최신 행을 쓴다.
        if (ring_iq && ring_w) {
            const int8_t *row = ring_iq + (size_t)((ring_w - 1) % RING_N) * 2 * MAX_SC;
            for (int f = 0; f < n_sc && f < 128; f++) {
                const float im = row[2 * f], re = row[2 * f + 1];
                const float a = sqrtf(im * im + re * re);
                // 20*log10 을 -128..127 로. 진폭 1 → -128, 진폭 90 → +100 근처.
                float db = (a > 0.5f) ? (60.0f * log10f(a) - 128.0f) : -128.0f;
                if (db > 127.0f) db = 127.0f;
                if (db < -128.0f) db = -128.0f;
                g_snap.amp[f] = (int8_t)db;
            }
        }
        g_snap.band = band_score();
        g_snap.thresh = thresh;
        g_snap.n_anchor = (uint8_t)n_anchor;
        for (int k = 0; k < n_anchor && k < 8; k++) {
            g_snap.anchor_z[k] = anchors[k].z;
            g_snap.anchor_last[k] = anchors[k].addr[5];
            g_snap.anchor_rssi[k] = anchors[k].last;
        }
        g_snap.infer_ms = infer_ms;
        g_snap.cls_hit = cls_hit; g_snap.cls_tot = cls_tot;
        g_snap.last_cls = last_cls; g_snap.last_score = last_score;
        g_snap.n_ble = n_ble; g_snap.ble_adv = ble_pkt;
        // 1초 리포트 주기이므로 이번 초의 프레임 수가 곧 Hz 다.
        static uint32_t prev_frames = 0;
        const uint32_t now_frames = ring_w;
        g_snap.csi_hz = now_frames - prev_frames;
        prev_frames = now_frames;
        g_snap.uptime_s = millis() / 1000;
        g_snap.boot_n = boot_n;
        g_snap.mark_n = m_n; g_snap.unmark_n = u_n;
        if (m_n >= 2 && u_n >= 2) {
            const double mm = m_sum / m_n, um = u_sum / u_n;
            const double sm = sqrt(fmax(m_sq / m_n - mm * mm, 0.0));
            const double su = sqrt(fmax(u_sq / u_n - um * um, 0.0));
            const double pooled = sqrt((sm * sm + su * su) / 2.0);
            g_snap.cohen_d = (pooled > 1e-9) ? (float)((mm - um) / pooled) : 0.0f;
        } else g_snap.cohen_d = 0.0f;
    }

    // 추세를 적재한다. 1초에 한 번이므로 72표본이면 72초 창이다.
    {
        const float b = band_score();
        for (int sl = 0; sl < N_SCALE; sl++) {
            // 최댓값으로 모은다. 평균이면 3초 지나간 사람이 300초 칸에서 사라진다.
            if (b > acc_max[sl]) acc_max[sl] = b;
            if (++acc_cnt[sl] >= SCALE_SEC[sl]) {
                trend[sl][trend_n[sl] % TREND_N] = acc_max[sl];
                trend_n[sl]++;
                acc_max[sl] = 0.0f;
                acc_cnt[sl] = 0;
            }
        }
    }

    // 화면 갱신. 전자종이는 태그당 3.5초 걸리므로 자주 할 수 없고, 할 이유도 없다.
    // 90초 주기면 추세 그래프가 늘 최근 72초를 보여준다.
    static uint32_t esl_t = 0;
    if (esl_force || (trend_n[0] > 20 && millis() - esl_t > 90000)) {
        esl_force = false; esl_t = millis(); esl_refresh();
    }

    // 60초마다 자동으로 혼동행렬을 뱉는다. 버튼을 누를 사람이 없어도
    // 파이프라인 정확성의 정본 숫자가 로그에 남는다.
    static uint32_t last_auto = 0;
    if (cls_tot >= 20 && millis() - last_auto > 60000) {
        last_auto = millis();
        print_stats();
        // 5분마다 남긴다. NVS 는 10만 회 쓰기 수명이라 이 주기면 1년 이상이다.
        static uint32_t last_save = 0;
        if (millis() - last_save > 300000) { last_save = millis(); store_save("주기"); }
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
    // 단 키 응답 깜빡임 중에는 손대지 않는다 — 덮으면 키를 눌러도 안 보이고,
    // 그게 바로 "3번 키만 반응한다" 는 오진을 만든 구조다.
    if (blink_left <= 0) {
        static bool on = false;
        const bool det = band_score() > thresh;
        if (ready < N_WCH) on = !on;                 // 1Hz 깜빡
        else if (det)      on = !on;                 // 리포트마다 토글 (빠르게 보임)
        else               on = true;                // 준비됨 = 켜짐
        led_all(on);
    }

    if (infer_ready)
        Serial.printf("        [모델] ch%d→클래스%d 코사인%.2f %lums (%lu회, 일치 %lu/%lu %.0f%%) 창%lu프레임/%lums 건너뜀%lu\n",
                      last_win_ch, last_cls, last_score, (unsigned long)infer_ms,
                      (unsigned long)infer_n, (unsigned long)cls_hit,
                      (unsigned long)cls_tot,
                      cls_tot ? 100.0 * cls_hit / cls_tot : 0.0,
                      (unsigned long)win_n, (unsigned long)win_span_ms,
                      (unsigned long)win_skip);
    Serial.printf("%s  | BLE %d/%d (%lu광고) 점수%.1f | 앵커%d z%.1f | 대역 %.1f%s"
                  "  sc%d%s 준비%d/%d\n",
                  marked ? "  [마크]" : "        ",
                  ble_hot, ble_valid, (unsigned long)ble_pkt, ble_dev_score,
                  n_anchor, anchor_score,
                  band, (band > thresh) ? "  <<< 움직임" : "", n_sc,
                  sc_mismatch ? " (다른폭 프레임 버림)" : "", ready, N_WCH);
    if (sc_mismatch)
        Serial.printf("        ※ 서브캐리어 %u개인 프레임 %lu개를 버렸다 — AP 가 대역폭을"
                      " 섞어 쓴다. 모델은 %u개로 학습됐다.\n",
                      (unsigned)sc_other, (unsigned long)sc_mismatch, (unsigned)n_sc);
    if (marked) Serial.print("");
}
