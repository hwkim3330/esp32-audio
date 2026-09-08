// 웹 MP3 플레이어 — 소리는 블루투스로, 곡은 브라우저로 넣는다.
//
// 이 보드의 아날로그 출력(AC101 → 3.5mm 잭)은 실기에서 죽어 있다. I2S 가 정확히 0 을
// 내보내고, 레지스터 조합 40여 개를 다 시도해도 무신호였다(firmware/loopback_test).
// 남은 용의자는 가운데 DIP 스위치 하나 — 물리 작업이라 소프트웨어로는 못 넘는다.
//
// 그래서 출력 경로를 A2DP 소스로 잡는다. A2DP 는 AC101/I2S 를 지나가지 않는다 —
// 앱이 PCM 을 만들어 블루투스 컨트롤러에 바로 넘긴다. 배관은 bt_audio_probe 에서
// 검증했다(연결 실패는 "남의 기기를 골라서"였다).
//
//   MP3 파일 → minimp3 디코드 → 44.1kHz 스테레오로 리샘플 → PSRAM 링버퍼
//            → A2DP 데이터 콜백 → SBC 인코딩(컨트롤러) → 이어폰/스피커
//
// SD 카드는 **읽기 전용으로만** 쓴다. 카드에 남의 데이터(대시캠 영상 등)가 있을 수
// 있어서 이 펌웨어는 SD 에 쓰지도, 지우지도, 포맷하지도 않는다. 업로드는 언제나
// 내장 플래시(LittleFS)로 간다. 카드 내용은 /sd/... 로 내려받아 백업할 수 있다.
//
// 빌드:
//   arduino-cli compile --fqbn \
//     'esp32:esp32:esp32:PSRAM=enabled,FlashSize=4M,PartitionScheme=custom,CPUFreq=240' \
//     firmware/web_mp3
//
// 주의로 배운 것들(반복해 물렸던 것):
//   - BT 컨트롤러는 btStart() 로 올려야 한다. esp_bt_controller_init() 을 직접 부르면
//     상태가 IDLE 로 읽히는데도 ESP_ERR_INVALID_STATE 가 돌아온다.
//   - GPIO16/17 은 PSRAM 것이다. 출력으로 만지면 즉시 재부팅 루프.
//   - 큰 버퍼는 PSRAM 에. 내부 DRAM 은 320KB 뿐이고 MALLOC_CAP_INTERNAL 은 IRAM 을
//     줄 수 있는데 IRAM 은 16비트 접근이 불법이다(LoadStoreError).
//   - 키 6개는 KEY1=36 KEY2=13 KEY3=19 KEY4=23 KEY5=18 KEY6=5. LED 핀(22)을 먼저
//     설정한 뒤 읽어야 GPIO13 이 안정된다.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <FS.h>
#include <LittleFS.h>
#include <SD_MMC.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_bt_api.h>
#include <esp_a2dp_api.h>
#include <esp_avrc_api.h>
#include <esp_heap_caps.h>
#include <Wire.h>
#include <AC101.h>
#include <driver/i2s.h>

#include "minimp3.h"
#include "player.h"
#include "ui.h"

// ─────────────────────────────────────────────────────────────── 하드웨어 상수

static const int KEY_PLAY = 36;   // KEY1
static const int KEY_NEXT = 13;   // KEY2
static const int KEY_PREV = 19;   // KEY3
static const int KEY_VUP  = 23;   // KEY4
static const int KEY_VDN  = 18;   // KEY5
static const int KEY_BT   = 5;    // KEY6
// GPIO22/21 은 **파워앰프 인에이블 후보**다. 이 저장소는 22 를 "PA_ENABLE(가정)" 으로
// 적어 두고도(loopback_test/board_pins.h) 동시에 상태 LED 로 깜빡여 왔다. 22 가 정말
// 앰프 인에이블이면 그 깜빡임이 **청음 시험 내내 앰프를 켜고 끄는 짓**이다.
// ★ 확정(2026-09-08): **앰프 인에이블은 GPIO21, GPIO22 는 초록 LED** 다.
// 근거 둘: arduino-audiokit 의 이 보드 정의(PA_ENABLE_GPIO 21 / GREEN_LED_GPIO 22),
// 그리고 실측 — loopback_test 에서 GPIO22 를 LOW/HIGH 로 바꿔도 출력 검파값이
// 그대로였다(비 0.85). 둘 다 HIGH 로 두면 앰프는 켜지고 LED 는 그냥 켜져 있다.
static const int PIN_PA    = 21;   // 파워앰프 인에이블
static const int PIN_LED_G = 22;   // 초록 LED (앰프가 아니다)

// GPIO5 는 **헤드폰 삽입 검출과 KEY6 겸용**이다(arduino-audiokit: HEADPHONE_DETECT 5).
// 이어폰을 꽂으면 이 핀이 계속 LOW 로 읽혀서 KEY6 이 눌린 것처럼 보인다 — 그래서
// KEY6 의 길게 누르기 동작(BT 탐색)은 웹 UI 로만 두고 여기서는 검출로 쓴다.
static const int PIN_HP_DETECT = 5;

static const int SD_CLK = 14, SD_CMD = 15, SD_D0 = 2;   // SDMMC 슬롯1, 1비트 모드

static const char *AP_SSID = "esp32-mp3";
static const char *AP_PASS = "12345678";
static const char *BT_NAME = "KETI-MP3";

static const int OUT_HZ = 44100;   // A2DP 표준. 여기로 맞춰 리샘플한다.

// 잭 출력(AC101 + I2S). 44100 을 벗어나면 AC101 PLL 이 락을 잃는다 — 우연히
// A2DP 와 같은 레이트라 리샘플러 하나로 두 출력을 다 먹인다.
static const i2s_port_t I2S_PORT_JACK = I2S_NUM_0;
// ★ 정정(2026-09-08): LRCK 와 DOUT 이 25/26 으로 뒤바뀌어 있었다. 이 한 쌍 때문에
// 이 저장소의 "오디오는 죽었다" 결론 전체가 틀렸다. 근거는 arduino-audiokit 의
// 이 보드 정의(WS 26 / DATA_OUT 25 / DATA_IN 35)와 고친 뒤의 실측이다.
static const int PIN_MCLK = 0, PIN_BCLK = 27, PIN_LRCK = 26, PIN_DOUT = 25;
static const int PIN_SDA = 33, PIN_SCL = 32;
static const int I2S_BLOCK = 256;   // 프레임

// SD 쓰기는 **/MP3 폴더 안에서만** 허용한다.
//
// 이 카드에는 GoPro 영상이 들어 있었다(1.9GB, 2026-09-08 백업·검증 완료). 그래서
// 처음에는 SD 쓰기 경로를 아예 만들지 않았다. 이제 곡을 SD 에 넣으라는 요청이
// 있어서 열되, **DCIM/MISC/System Volume Information 은 손댈 수 없게** 경로를
// /MP3 로 못박는다. 삭제도 이 폴더 안에서만 된다. 곡 스캔도 이 폴더만 본다
// (루트를 훑으면 DCIM 의 영상 수천 개를 헛돈다).
static const char *SD_MUSIC_DIR = "/MP3";

// ─────────────────────────────────────────────────────────────── 링버퍼 (PSRAM)

// 2의 거듭제곱으로 잡아 마스킹으로 감싼다. 256KB = 스테레오 16비트 44.1kHz 로 약 1.48초.
// A2DP 지연이 100~200ms 인데 그보다 넉넉해야 업로드·스캔 중 끊김을 흡수한다.
static const size_t RING_BITS = 18;
static const size_t RING_SZ   = (size_t)1 << RING_BITS;
static const size_t RING_MASK = RING_SZ - 1;

static uint8_t *g_ring = nullptr;
static volatile size_t g_wr = 0;   // 디코더만 증가
static volatile size_t g_rd = 0;   // BT 콜백만 증가

static inline size_t ring_used() { return (g_wr - g_rd) & RING_MASK; }
static inline size_t ring_free() { return RING_SZ - 1 - ring_used(); }
static inline void   ring_reset() { g_rd = g_wr = 0; }

// ─────────────────────────────────────────────────────────────── 상태

static volatile PState g_state = ST_IDLE;
static volatile bool   g_req_stop = false;      // 디코더에게 현재 곡을 접으라고 알린다
static volatile int    g_req_track = -1;        // >=0 이면 그 인덱스로 전환
static volatile int    g_vol = 60;              // 0~100, 출력단에서 적용(즉시 반응)
static volatile OutMode g_out = OUT_BT;         // 소리를 어디로 낼지
static volatile Role    g_role = ROLE_PLAYER;   // 플레이어냐 BT 스피커냐
static volatile int     g_sink_hz = 44100;      // 싱크가 협상한 샘플레이트
static volatile int     g_sink_ch = 2;
static volatile uint32_t g_sink_bytes = 0;      // 싱크로 실제로 들어온 바이트
static volatile bool     g_prebuf = false;      // 선버퍼링이 찼는가
static volatile uint32_t g_starves = 0;         // 링이 굶어 무음을 낸 횟수
static volatile bool     g_wifi_off_audio = true;  // 스트리밍 중 WiFi 를 끈다
static volatile int      g_wifi_req = 0;        // 1=끄기 2=켜기 (loop 에서 처리)
static volatile uint32_t g_wifi_on_at = 0;      // 이 시각에 다시 켠다
static volatile bool   g_busy_fs = false;       // 업로드/다운로드 중 — 재생을 양보한다

static std::vector<Track> g_lib;
static volatile int g_idx = -1;

static volatile uint32_t g_played_frames = 0;   // 디코드해 낸 출력 프레임 수(경과 시간)
static volatile size_t   g_file_pos = 0;
static volatile size_t   g_file_len = 0;
static volatile int      g_bitrate = 0;         // 마지막 프레임의 kbps — 길이 추정용
static volatile int      g_src_hz = 0;
static volatile int      g_src_ch = 0;
static volatile uint32_t g_underruns = 0;

static bool   g_have_sd = false;
static String g_store_name = "flash";

static Preferences g_prefs;
static WebServer   g_http(80);
static SemaphoreHandle_t g_fsmux = nullptr;

// ─────────────────────────────────────────────────────────────── 블루투스 상태

static volatile bool g_bt_conn = false;
static volatile bool g_bt_stream = false;
static volatile bool g_bt_scanning = false;

static std::vector<BtDev> g_bt_devs;
static esp_bd_addr_t g_bt_target = {0};
static bool          g_bt_have_target = false;

static String addr_str(const esp_bd_addr_t a)
{
    char b[18];
    snprintf(b, sizeof b, "%02x:%02x:%02x:%02x:%02x:%02x", a[0], a[1], a[2], a[3], a[4], a[5]);
    return String(b);
}

static bool addr_parse(const String &s, esp_bd_addr_t out)
{
    unsigned v[6];
    if (sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6)
        return false;
    for (int i = 0; i < 6; i++) out[i] = (uint8_t)v[i];
    return true;
}

static const char *cod_major_name(uint8_t m)
{
    switch (m) {
    case 1:  return "컴퓨터";
    case 2:  return "휴대폰";
    case 4:  return "오디오";
    case 5:  return "주변기기";
    case 6:  return "이미징";
    case 7:  return "웨어러블";
    default: return "기타";
    }
}

// ── A2DP 가 PCM 을 요구할 때. BT 태스크에서 불린다 — 빨라야 한다.
static int32_t a2dp_data_cb(uint8_t *buf, int32_t len)
{
    if (!buf || len <= 0) return 0;

    // 잭 모드일 때는 링을 건드리지 않는다. 소비자가 둘이 되면 서로 프레임을 뺏는다
    // (이어폰이 아직 연결돼 스트리밍 중일 수 있으므로 이 방어가 필요하다).
    if (g_out != OUT_BT) { memset(buf, 0, (size_t)len); return len; }

    size_t want = (size_t)len;
    size_t have = ring_used();
    have -= have % 4;                       // 프레임(4바이트) 경계로 자른다

    if (have < want) {
        if (g_state == ST_PLAY && have == 0) g_underruns++;
        memset(buf + have, 0, want - have);  // 모자란 만큼은 무음 — 스트림을 끊지 않는다
        want = have;
    }

    // 링에서 꺼내며 음량을 곱한다. 음량을 여기서 적용해야 슬라이더가 즉시 반응한다
    // (디코더에서 곱하면 링 깊이만큼 늦는다).
    const int32_t q = ((int32_t)g_vol * (int32_t)g_vol) / 100;   // 0~100, 청감상 곡선
    size_t rd = g_rd;
    int16_t *o = (int16_t *)buf;
    for (size_t i = 0; i < want; i += 2) {
        int16_t s;
        memcpy(&s, &g_ring[rd & RING_MASK], 2);
        rd += 2;
        *o++ = (int16_t)(((int32_t)s * q) / 100);
    }
    g_rd = rd & RING_MASK;
    return len;
}

// ── A2DP 싱크: 맥·폰이 보낸 소리가 여기로 들어온다 (컨트롤러가 SBC 를 이미 풀어 준다).
//
// BT 태스크에서 불리므로 절대 막히면 안 된다 — 링이 꽉 차면 **버린다**.
// 디코더 경로(ring_put_frame)는 기다리지만 여기서 기다리면 BT 스택이 밀린다.
static float   g_sk_pos = 0.0f;
static int16_t g_sk_prev_l = 0, g_sk_prev_r = 0;
static volatile uint32_t g_sink_drops = 0;

static inline bool sink_put(int16_t l, int16_t r)
{
    if (ring_free() < 4) { g_sink_drops++; return false; }
    size_t w = g_wr;
    memcpy(&g_ring[w & RING_MASK], &l, 2); w += 2;
    memcpy(&g_ring[w & RING_MASK], &r, 2); w += 2;
    g_wr = w & RING_MASK;
    return true;
}

static void a2dp_sink_data_cb(const uint8_t *data, uint32_t len)
{
    if (!data || !g_ring) return;
    g_sink_bytes += len;

    const int ch = g_sink_ch >= 2 ? 2 : 1;
    const int n = (int)(len / (2 * ch));               // 프레임 수
    const int16_t *in = (const int16_t *)data;
    if (n <= 0) return;

    if (g_sink_hz == OUT_HZ) {                          // 보통 맥·폰은 44.1kHz 다
        for (int i = 0; i < n; i++) {
            const int16_t l = in[ch == 2 ? 2 * i : i];
            const int16_t r = (ch == 2) ? in[2 * i + 1] : l;
            if (!sink_put(l, r)) return;                // 꽉 찼으면 이 블록은 포기
        }
        g_sk_prev_l = in[ch == 2 ? 2 * (n - 1) : n - 1];
        g_sk_prev_r = (ch == 2) ? in[2 * (n - 1) + 1] : g_sk_prev_l;
        return;
    }

    // 협상 결과가 44.1kHz 가 아니면 리샘플한다. AC101 은 44100 을 벗어나면
    // PLL 이 락을 잃으므로 I2S 쪽 레이트를 바꾸는 건 답이 아니다.
    const float step = (float)g_sink_hz / (float)OUT_HZ;
    float pos = g_sk_pos;
    while (pos < (float)n) {
        const int i = (int)pos;
        const float f = pos - (float)i;
        int16_t al, ar, bl, br;
        if (i == 0) { al = g_sk_prev_l; ar = g_sk_prev_r; }
        else if (ch == 2) { al = in[2 * (i - 1)]; ar = in[2 * (i - 1) + 1]; }
        else { al = ar = in[i - 1]; }
        if (ch == 2) { bl = in[2 * i]; br = in[2 * i + 1]; }
        else { bl = br = in[i]; }
        if (!sink_put((int16_t)(al + (bl - al) * f), (int16_t)(ar + (br - ar) * f))) break;
        pos += step;
    }
    g_sk_pos = pos - (float)n;
    if (g_sk_pos < 0) g_sk_pos = 0;
    g_sk_prev_l = in[ch == 2 ? 2 * (n - 1) : n - 1];
    g_sk_prev_r = (ch == 2) ? in[2 * (n - 1) + 1] : g_sk_prev_l;
}

// ── AVRCP 컨트롤러: 싱크가 **상대(맥)의 재생을 조작한다**.
//
// BT 스피커의 버튼이 폰·맥의 재생/다음곡을 움직이는 게 바로 이 프로파일이다.
// 트랜잭션 라벨은 0~15 를 돌려 쓴다.
static uint8_t g_tl = 0;
static bool    g_remote_playing = true;

static void avrc_send(uint8_t key)
{
    const uint8_t tl = g_tl++ & 0x0F;
    esp_avrc_ct_send_passthrough_cmd(tl, key, ESP_AVRC_PT_CMD_STATE_PRESSED);
    vTaskDelay(pdMS_TO_TICKS(30));
    esp_avrc_ct_send_passthrough_cmd(tl, key, ESP_AVRC_PT_CMD_STATE_RELEASED);
}

static void avrc_cmd(const String &c)
{
    if (g_role != ROLE_SINK) return;
    if (c == "next")       avrc_send(ESP_AVRC_PT_CMD_FORWARD);
    else if (c == "prev")  avrc_send(ESP_AVRC_PT_CMD_BACKWARD);
    else if (c == "stop")  avrc_send(ESP_AVRC_PT_CMD_STOP);
    else if (c == "play")  { avrc_send(ESP_AVRC_PT_CMD_PLAY);  g_remote_playing = true; }
    else if (c == "pause") { avrc_send(ESP_AVRC_PT_CMD_PAUSE); g_remote_playing = false; }
    else if (c == "toggle") {
        avrc_send(g_remote_playing ? ESP_AVRC_PT_CMD_PAUSE : ESP_AVRC_PT_CMD_PLAY);
        g_remote_playing = !g_remote_playing;
    }
    Serial.printf("[avrc] %s\n", c.c_str());
}

static void avrc_ct_cb(esp_avrc_ct_cb_event_t ev, esp_avrc_ct_cb_param_t *p)
{
    if (ev == ESP_AVRC_CT_CONNECTION_STATE_EVT)
        Serial.printf("[avrc] %s\n", p->conn_stat.connected ? "연결됨 — 버튼이 맥을 조작한다" : "끊김");
}

static void a2dp_cb(esp_a2d_cb_event_t ev, esp_a2d_cb_param_t *p)
{
    // 싱크가 협상한 코덱 설정. SBC 헤더에서 샘플레이트·채널을 꺼낸다.
    if (ev == ESP_A2D_AUDIO_CFG_EVT) {
        if (p->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            const uint8_t *c = p->audio_cfg.mcc.cie.sbc;
            const uint8_t fs = c[0] & 0xF0;
            g_sink_hz = (fs & 0x80) ? 16000 : (fs & 0x40) ? 32000 : (fs & 0x20) ? 44100 : 48000;
            g_sink_ch = ((c[0] & 0x08) != 0) ? 1 : 2;   // 0x08 = MONO
            Serial.printf("[싱크] 코덱 협상 %dHz %dch\n", g_sink_hz, g_sink_ch);
            g_sk_pos = 0; g_sk_prev_l = g_sk_prev_r = 0;
            ring_reset();
        }
        return;
    }

    switch (ev) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        if (p->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            g_bt_conn = true;
            memcpy(g_bt_target, p->conn_stat.remote_bda, sizeof(esp_bd_addr_t));
            g_bt_have_target = true;
            Serial.printf("[a2dp] 연결됨 %s\n", addr_str(g_bt_target).c_str());
            g_prefs.putString("bt", addr_str(g_bt_target));
        } else if (p->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            g_bt_conn = g_bt_stream = false;
            Serial.printf("[a2dp] 연결 끊김 (사유 %d)\n", p->conn_stat.disc_rsn);
        }
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        g_bt_stream = (p->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED);
        Serial.printf("[a2dp] 스트리밍 %s\n", g_bt_stream ? "시작" : "정지");
        // WiFi 와 클래식 BT 는 같은 2.4GHz 프런트엔드를 나눠 쓴다. 싱크로 음악을
        // 받는 동안 AP 를 띄워 두면 그게 그대로 끊김(지지직)이 된다. 스트리밍 중에는
        // 끄고, 멈춘 뒤 3초 있다가 되살린다(웹 UI 를 다시 쓸 수 있게).
        // WiFi API 를 BT 콜백에서 직접 부르면 위험하므로 요청만 남기고 loop 에서 한다.
        if (g_role == ROLE_SINK && g_wifi_off_audio) {
            if (g_bt_stream) { g_wifi_req = 1; }
            else { g_wifi_on_at = millis() + 3000; }
        }
        break;
    default:
        break;
    }
}

static void gap_cb(esp_bt_gap_cb_event_t ev, esp_bt_gap_cb_param_t *p)
{
    switch (ev) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        uint32_t cod = 0; int rssi = 0; char name[64] = "";
        for (int i = 0; i < p->disc_res.num_prop; i++) {
            esp_bt_gap_dev_prop_t *pr = &p->disc_res.prop[i];
            if (pr->type == ESP_BT_GAP_DEV_PROP_COD)       cod  = *(uint32_t *)pr->val;
            else if (pr->type == ESP_BT_GAP_DEV_PROP_RSSI) rssi = *(int8_t *)pr->val;
            else if (pr->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
                int l = pr->len < (int)sizeof(name) - 1 ? pr->len : (int)sizeof(name) - 1;
                memcpy(name, pr->val, l); name[l] = 0;
            }
        }
        // 같은 MAC 이 여러 번 리포트된다. 고유 MAC 으로 묶어야 개수가 맞다
        // (예전 프로브가 "기기 80개" 라고 찍은 건 리포트 건수였다).
        for (auto &d : g_bt_devs) {
            if (!memcmp(d.bda, p->disc_res.bda, sizeof(esp_bd_addr_t))) {
                d.rssi = rssi;
                if (name[0]) strncpy(d.name, name, sizeof d.name - 1);
                return;
            }
        }
        BtDev d = {};
        memcpy(d.bda, p->disc_res.bda, sizeof(esp_bd_addr_t));
        d.rssi = rssi;
        d.major = (uint8_t)((cod >> 8) & 0x1F);
        strncpy(d.name, name[0] ? name : "(이름 없음)", sizeof d.name - 1);
        g_bt_devs.push_back(d);
        // 일반 조회만으로는 이름이 안 오는 기기가 많다 — 따로 물어본다.
        if (!name[0]) esp_bt_gap_read_remote_name(p->disc_res.bda);
        break;
    }
    case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
        if (p->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS) {
            for (auto &d : g_bt_devs)
                if (!memcmp(d.bda, p->read_rmt_name.bda, sizeof(esp_bd_addr_t)))
                    strncpy(d.name, (const char *)p->read_rmt_name.rmt_name, sizeof d.name - 1);
        }
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        g_bt_scanning = (p->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED);
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {
        // 구형 기기는 SSP 대신 레거시 PIN 을 요구한다. 0000 으로 답한다.
        esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
        esp_bt_gap_pin_reply(p->pin_req.bda, true, 4, pin);
        break;
    }
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        Serial.printf("[gap] 인증 %s: %s\n",
                      p->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS ? "성공" : "실패",
                      (const char *)p->auth_cmpl.device_name);
        break;
    default:
        break;
    }
}

static bool g_bt_up = false;

static void bt_setup()
{
    // 잭으로만 들을 거면 블루투스를 아예 올리지 않는다.
    //
    // Bluedroid + 클래식 BT 는 내부 DRAM 을 60~70KB 쥔다. SD 를 마운트한 플레이어
    // 모드에서 BT 까지 올리면 남는 힙이 11KB 까지 떨어지는데, 이 보드에서 그 수준은
    // 실제로 죽는다(파일 열기에서 할당 실패). 필요할 때만 올린다 —
    // 출력을 BT 로 바꾸면 cmd_out() 이 재부팅한다.
    if (g_role == ROLE_PLAYER && g_out != OUT_BT) {
        Serial.println("[bt] 잭 모드 — 블루투스를 올리지 않는다(내부 DRAM 60~70KB 절약)");
        return;
    }
    // BLE 몫의 컨트롤러 메모리를 반납한다. 이 펌웨어는 클래식 BT(A2DP)만 쓰는데
    // 안 반납하면 내부 DRAM 을 30~40KB 쥐고 앉아 있다. 실측으로 이게 결정적이었다 —
    // 반납 전에는 초기화가 끝난 뒤 남는 힙이 14KB 였고 재생을 걸면 죽었다.
    // **btStart() 보다 먼저** 불러야 한다(컨트롤러가 올라간 뒤에는 안 먹는다).
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    Serial.printf("[bt] 컨트롤러 상태 %d → btStart()\n", (int)esp_bt_controller_get_status());
    // btStart() 를 써야 한다. esp_bt_controller_init() 직접 호출은 상태가 IDLE 로
    // 읽히는데도 ESP_ERR_INVALID_STATE 를 돌려준다(모드 지정·mem_release 제거 다 실패).
    if (!btStart()) { Serial.println("[bt] btStart() 실패 — BT 없이 계속한다"); return; }

    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED)
        if (esp_bluedroid_init() != ESP_OK) { Serial.println("[bt] bluedroid_init 실패"); return; }
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED)
        if (esp_bluedroid_enable() != ESP_OK) { Serial.println("[bt] bluedroid_enable 실패"); return; }

    esp_bt_gap_set_device_name(BT_NAME);
    esp_bt_gap_register_callback(gap_cb);

    // IO 능력 없음 = just-works 페어링. 화면도 키패드도 없는 보드에 맞는 설정.
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap));

    esp_a2d_register_callback(a2dp_cb);

    if (g_role == ROLE_SINK) {
        // BT 스피커 역할. 맥·폰이 스캔해서 붙으면 그 소리가 잭으로 나온다.
        esp_a2d_sink_register_data_callback(a2dp_sink_data_cb);
        if (esp_a2d_sink_init() != ESP_OK) { Serial.println("[a2dp] 싱크 init 실패"); return; }

        // 기기 종류(CoD)를 오디오 렌더링으로 알린다. 이걸 안 하면 맥이 목록에
        // 띄워도 "스피커" 로 인식하지 않아 오디오 출력 장치로 안 잡힌다.
        esp_bt_cod_t cod = {};
        cod.major = ESP_BT_COD_MAJOR_DEV_AV;
        cod.minor = 0x06;                      // Headphones (맥에서 헤드폰 아이콘으로 보인다)
        cod.service = ESP_BT_COD_SRVC_AUDIO | ESP_BT_COD_SRVC_RENDERING;
        esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_ALL);

        // AVRCP 컨트롤러도 올린다 — 이걸로 보드의 버튼이 맥의 재생을 조작한다.
        esp_avrc_ct_init();
        esp_avrc_ct_register_callback(avrc_ct_cb);

        g_out = OUT_JACK;                      // 싱크의 소리는 잭으로 낸다
        Serial.println("[a2dp] **싱크(BT 스피커)** 모드 — 맥·폰에서 스캔해 연결하라");
    } else {
        esp_a2d_source_register_data_callback(a2dp_data_cb);
        if (esp_a2d_source_init() != ESP_OK) { Serial.println("[a2dp] 소스 init 실패"); return; }
        Serial.println("[a2dp] 소스 모드 — 보드가 이어폰으로 보낸다");
    }
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    g_bt_up = true;
    const uint8_t *mac = esp_bt_dev_get_address();
    Serial.printf("[bt] 준비됨, MAC %02x:%02x:%02x:%02x:%02x:%02x, 이름 %s\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], BT_NAME);

    // 지난번 붙었던 기기로 자동 재연결 (소스 역할일 때만. 싱크는 상대가 붙어 온다)
    if (g_role == ROLE_SINK) return;
    String last = g_prefs.getString("bt", "");
    if (last.length() && addr_parse(last, g_bt_target)) {
        g_bt_have_target = true;
        Serial.printf("[a2dp] 자동 재연결 시도 %s\n", last.c_str());
        esp_a2d_source_connect(g_bt_target);
    }
}

static void bt_scan_start()
{
    g_bt_devs.clear();
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10 /*×1.28초 ≈ 12.8초*/, 0);
}

// ─────────────────────────────────────────────────────────────── 잭 출력 (AC101 + I2S)
//
// 잭이 살아 있는지는 이 저장소에 기록이 없다. 판정 근거였던 음향 폐루프는 **입력이
// 죽으면 출력이 살아 있어도 똑같이 실패**하고, 보드에 스피커가 안 붙어 있어서
// 아무도 이어폰을 꽂고 들어본 적이 없다. 확정된 건 입력(ADC)이 정확히 0 이라는 것뿐.
//
// 그래서 잭을 **정식 출력 경로로 넣어 둔다.** 살아 있으면 그 자리에서 되고, 죽어 있어도
// 블루투스 경로가 그대로다. 어느 쪽이든 이 펌웨어는 완성품이다.
//
// "둘 다 동시" 는 일부러 넣지 않았다. I2S 와 A2DP 는 클록 도메인이 달라서 한쪽이
// 천천히 밀리고, 링버퍼 소비자가 둘이 되면 드리프트를 흡수할 곳이 없다.

class AC101Ex : public AC101 {
public:
    uint16_t rd(uint8_t r) { return ReadReg(r); }
    bool wr(uint8_t r, uint16_t v) { return WriteReg(r, v); }
};

static AC101Ex g_codec;
static bool     g_codec_ok = false;
static int16_t *g_i2s_buf = nullptr;
static volatile uint32_t g_tone_until = 0;     // 이 시각까지 테스트음을 낸다
static volatile uint64_t g_i2s_frames = 0;     // I2S 로 실제로 밀어낸 프레임 수
static float g_tone_phase = 0.0f;

static void jack_setup()
{
    // AC101 은 I2C 로는 늘 완벽하게 대답한다 — 지금까지 한 번도 실패한 적 없다.
    g_codec_ok = g_codec.begin(PIN_SDA, PIN_SCL);
    if (!g_codec_ok) {
        Serial.println("[잭] AC101 begin() 실패 — 잭 출력을 쓸 수 없다");
        return;
    }
    g_codec.SetMode(AC101::MODE_DAC);           // 출력만 연다. 죽은 ADC 는 건드리지 않는다
    g_codec.SetVolumeHeadphone(50);
    g_codec.SetVolumeSpeaker(50);

    i2s_config_t cfg = {};
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate = OUT_HZ;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    // 싱크 모드는 SD/LittleFS 를 안 올려 내부 DRAM 여유가 있으니 DMA 를 넉넉히 준다.
    // 23ms(4개)면 웹서버·BT 태스크에 밀릴 때 DMA 가 굶어 딱딱거린다.
    cfg.dma_buf_count = (g_role == ROLE_SINK) ? 8 : 4;   // 8×256프레임 ≈ 46ms
    cfg.dma_buf_len = I2S_BLOCK;
    cfg.use_apll = false;
    cfg.tx_desc_auto_clear = true;
    if (i2s_driver_install(I2S_PORT_JACK, &cfg, 0, nullptr) != ESP_OK) {
        Serial.println("[잭] I2S 설치 실패");
        g_codec_ok = false;
        return;
    }
    i2s_pin_config_t pins = {};
    pins.mck_io_num = PIN_MCLK;
    pins.bck_io_num = PIN_BCLK;
    pins.ws_io_num = PIN_LRCK;
    pins.data_out_num = PIN_DOUT;
    pins.data_in_num = I2S_PIN_NO_CHANGE;       // 입력을 아예 만들지 않는다
    i2s_set_pin(I2S_PORT_JACK, &pins);

    // IRAM 은 16비트 접근이 불법이다(LoadStoreError). DMA 가능 메모리로 잡는다.
    g_i2s_buf = (int16_t *)heap_caps_malloc((size_t)I2S_BLOCK * 2 * sizeof(int16_t), MALLOC_CAP_DMA);
    if (!g_i2s_buf) { Serial.println("[잭] DMA 버퍼 실패"); g_codec_ok = false; return; }

    {   // 저장된 음량을 코덱 볼륨으로 반영한다
        const uint8_t hv = (uint8_t)((g_vol * 63) / 100);
        g_codec.SetVolumeHeadphone(hv);
        g_codec.SetVolumeSpeaker(hv);
    }
    Serial.printf("[잭] 준비됨 — HPOUT=0x%04X SPKOUT=0x%04X DAC_VOL=0x%04X\n",
                  g_codec.rd(0x56), g_codec.rd(0x58), g_codec.rd(0x49));
}

// 링버퍼(또는 테스트음)를 I2S 로 계속 밀어낸다. 모드가 BT 면 무음을 밀어
// DMA 를 굶기지 않는다 — 굶으면 다음 전환에서 딸꾹질이 난다.
static void jack_task(void *)
{
    for (;;) {
        if (!g_codec_ok) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }

        const bool tone = (int32_t)(g_tone_until - millis()) > 0;
        // ★ 잭 경로에서는 디지털로 음량을 깎지 않는다. 0.36배 곱하면 정수 절단으로
        //   해상도가 1.4비트 날아가고 그게 노이즈로 들린다. 음량은 AC101 의 아날로그
        //   볼륨(SetVolumeHeadphone)으로 내린다 — cmd_vol() 이 같이 반영한다.

        if (tone) {
            // 스윕. 고정음보다 "정말 나오는 소리" 인지 알아듣기 쉽다.
            for (int i = 0; i < I2S_BLOCK; i++) {
                const float t = ((millis() % 2000) / 2000.0f);
                g_tone_phase += (300.0f + 1500.0f * t) / OUT_HZ;
                if (g_tone_phase >= 1.0f) g_tone_phase -= 1.0f;
                const int16_t v = (int16_t)(9000.0f * sinf(2.0f * PI * g_tone_phase));
                g_i2s_buf[2 * i] = v;
                g_i2s_buf[2 * i + 1] = v;
            }
        } else if (g_out == OUT_JACK) {
            const size_t want = (size_t)I2S_BLOCK * 4;

            // 선버퍼링. 링이 어느 정도 찰 때까지 무음을 내고 기다린다.
            //
            // 이걸 안 하면 데이터가 조금 모자랄 때마다 그 자리에 무음을 끼워 넣게 되고,
            // 그게 정확히 "지지직" 으로 들린다. 특히 A2DP 싱크는 상대(맥) 클럭으로
            // 들어오고 I2S 는 우리 클럭으로 나가므로 둘이 서로 밀린다 — 흡수할 여유가
            // 있어야 한다. RING_SZ/8 = 약 185ms.
            if (!g_prebuf) {
                if (ring_used() < RING_SZ / 8) {
                    memset(g_i2s_buf, 0, want);
                    size_t w0 = 0;
                    i2s_write(I2S_PORT_JACK, g_i2s_buf, want, &w0, portMAX_DELAY);
                    g_i2s_frames += w0 / 4;
                    continue;
                }
                g_prebuf = true;
            }
            size_t have = ring_used();
            have -= have % 4;
            if (have < want) {                  // 굶었다 — 무음을 내고 다시 채운다
                g_starves++;
                if (have == 0) g_prebuf = false;
                memset(g_i2s_buf, 0, want);
                size_t w0 = 0;
                i2s_write(I2S_PORT_JACK, g_i2s_buf, want, &w0, portMAX_DELAY);
                g_i2s_frames += w0 / 4;
                continue;
            }
            size_t rd = g_rd;
            for (size_t b = 0; b < want; b += 2) {           // 풀스케일로 그대로 넘긴다
                memcpy((uint8_t *)g_i2s_buf + b, &g_ring[rd & RING_MASK], 2);
                rd += 2;
            }
            g_rd = rd & RING_MASK;
        } else {
            memset(g_i2s_buf, 0, (size_t)I2S_BLOCK * 4);
        }

        size_t wrote = 0;
        i2s_write(I2S_PORT_JACK, g_i2s_buf, (size_t)I2S_BLOCK * 4, &wrote, portMAX_DELAY);
        g_i2s_frames += wrote / 4;
    }
}

// ─────────────────────────────────────────────────────────────── 저장소 · 곡목록

static fs::FS *fs_of(const Track &t) { return t.sd ? (fs::FS *)&SD_MMC : (fs::FS *)&LittleFS; }

static bool ends_with_mp3(const String &n)
{
    if (n.length() < 5) return false;
    String e = n.substring(n.length() - 4); e.toLowerCase();
    return e == ".mp3";
}

// 루트와 한 단계 아래 디렉터리에서 .mp3 를 모은다. 깊게 파지 않는다 —
// SD 에 대시캠 영상 수천 개가 있을 수 있고, 그걸 다 훑으면 부팅이 느려진다.
static void scan_dir(fs::FS &fsys, const char *dir, bool sd, int depth)
{
    File d = fsys.open(dir);
    if (!d || !d.isDirectory()) return;
    for (File f = d.openNextFile(); f; f = d.openNextFile()) {
        String p = f.path();
        if (f.isDirectory()) {
            if (depth > 0) scan_dir(fsys, p.c_str(), sd, depth - 1);
        } else if (ends_with_mp3(p)) {
            Track t;
            t.path = p;
            t.name = p.substring(p.lastIndexOf('/') + 1);
            t.size = f.size();
            t.sd   = sd;
            g_lib.push_back(t);
        }
        f.close();
        if (g_lib.size() >= 200) break;   // 목록 UI 와 RAM 을 지킨다
    }
    d.close();
}

static void lib_rescan()
{
    xSemaphoreTake(g_fsmux, portMAX_DELAY);
    g_lib.clear();
    // SD 는 /MP3 만 본다. 루트를 훑으면 DCIM 의 영상 수천 개를 헛돈다.
    if (g_have_sd && SD_MMC.exists(SD_MUSIC_DIR)) scan_dir(SD_MMC, SD_MUSIC_DIR, true, 1);
    scan_dir(LittleFS, "/", false, 1);
    xSemaphoreGive(g_fsmux);
    Serial.printf("[lib] 곡 %d개 (SD %s)\n", (int)g_lib.size(), g_have_sd ? "있음" : "없음");
}

static void storage_setup()
{
    if (!LittleFS.begin(true, "/littlefs", 10, "spiffs"))
        Serial.println("[fs] LittleFS 마운트 실패");
    else
        Serial.printf("[fs] LittleFS %u / %u KB\n",
                      (unsigned)(LittleFS.usedBytes() / 1024), (unsigned)(LittleFS.totalBytes() / 1024));

    // SD 는 1비트 모드로 붙인다(슬롯1 CLK14/CMD15/D0=2). 4비트는 GPIO13 을 먹어
    // KEY2 와 부딪힌다. format_if_empty 는 반드시 false — 카드에 남의 데이터가 있을 수 있다.
    // maxOpenFiles 를 3 으로 줄인다 — 기본 5 는 쓰지도 않는 FAT 캐시를 내부 DRAM 에 잡는다.
    if (SD_MMC.begin("/sdcard", true /*1bit*/, false /*format_if_empty=false*/,
                     BOARD_MAX_SDMMC_FREQ, 3)) {
        g_have_sd = true;
        g_store_name = "SD";
        Serial.printf("[sd] 마운트됨, %llu MB 중 %llu MB 사용\n",
                      SD_MMC.totalBytes() / (1024ULL * 1024), SD_MMC.usedBytes() / (1024ULL * 1024));
        Serial.println("[sd] 읽기 전용으로만 쓴다 — 이 펌웨어는 SD 에 쓰지 않는다");
    } else {
        Serial.println("[sd] 마운트 실패 — 카드 없음 또는 가운데 DIP 스위치가 SD 로 안 붙어 있다");
    }
}

// ─────────────────────────────────────────────────────────────── 디코더 태스크

static mp3dec_t *g_dec = nullptr;
static uint8_t  *g_inbuf = nullptr;           // 파일에서 읽은 원시 MP3
static const size_t IN_SZ = 16 * 1024;
static int16_t  *g_pcm = nullptr;             // 프레임 하나 분량의 디코드 결과

// 푸시형 선형 리샘플러 상태
static float   g_rs_pos = 0.0f;
static int16_t g_rs_prev_l = 0, g_rs_prev_r = 0;

static void ring_put_frame(int16_t l, int16_t r)
{
    while (ring_free() < 4) {
        if (g_req_stop || g_state != ST_PLAY) return;
        vTaskDelay(pdMS_TO_TICKS(4));
    }
    size_t w = g_wr;
    memcpy(&g_ring[w & RING_MASK], &l, 2); w += 2;
    memcpy(&g_ring[w & RING_MASK], &r, 2); w += 2;
    g_wr = w & RING_MASK;
}

// 디코드된 n 프레임을 44.1kHz 로 맞춰 링에 넣는다.
// 개념상 입력열은 [prev, in0, in1, ...] 이고 t=0 이 prev, t=1 이 in0 이다.
static void push_resampled(const int16_t *pcm, int n, int ch, int hz)
{
    if (n <= 0) return;

    if (hz == OUT_HZ) {                       // 같은 레이트면 그냥 통과
        for (int i = 0; i < n; i++) {
            int16_t l = pcm[ch == 2 ? 2 * i : i];
            int16_t r = (ch == 2) ? pcm[2 * i + 1] : l;
            ring_put_frame(l, r);
        }
        if (ch == 2) { g_rs_prev_l = pcm[2 * (n - 1)]; g_rs_prev_r = pcm[2 * (n - 1) + 1]; }
        else         { g_rs_prev_l = g_rs_prev_r = pcm[n - 1]; }
        return;
    }

    const float step = (float)hz / (float)OUT_HZ;
    float pos = g_rs_pos;
    while (pos < (float)n) {
        const int i = (int)pos;
        const float f = pos - (float)i;
        int16_t al, ar, bl, br;
        if (i == 0) { al = g_rs_prev_l; ar = g_rs_prev_r; }
        else if (ch == 2) { al = pcm[2 * (i - 1)]; ar = pcm[2 * (i - 1) + 1]; }
        else { al = ar = pcm[i - 1]; }
        if (ch == 2) { bl = pcm[2 * i]; br = pcm[2 * i + 1]; }
        else { bl = br = pcm[i]; }
        ring_put_frame((int16_t)(al + (bl - al) * f), (int16_t)(ar + (br - ar) * f));
        pos += step;
        if (g_req_stop || g_state != ST_PLAY) break;
    }
    g_rs_pos = pos - (float)n;
    if (ch == 2) { g_rs_prev_l = pcm[2 * (n - 1)]; g_rs_prev_r = pcm[2 * (n - 1) + 1]; }
    else         { g_rs_prev_l = g_rs_prev_r = pcm[n - 1]; }
}

static void play_file(const Track &t)
{
    xSemaphoreTake(g_fsmux, portMAX_DELAY);
    File f = fs_of(t)->open(t.path.c_str(), FILE_READ);
    xSemaphoreGive(g_fsmux);
    if (!f) { Serial.printf("[dec] 열기 실패 %s\n", t.path.c_str()); return; }

    Serial.printf("[dec] ▶ %s (%u KB)\n", t.name.c_str(), (unsigned)(t.size / 1024));
    mp3dec_init(g_dec);
    ring_reset();
    g_rs_pos = 0; g_rs_prev_l = g_rs_prev_r = 0;
    g_played_frames = 0; g_file_len = t.size; g_file_pos = 0;
    g_underruns = 0;

    size_t fill = 0;
    bool started = false;

    while (!g_req_stop) {
        if (g_state == ST_PAUSE || g_busy_fs) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
        if (g_state != ST_PLAY) break;

        if (fill < 2 * 1440 && f.available()) {           // 프레임 하나는 최대 1440바이트
            xSemaphoreTake(g_fsmux, portMAX_DELAY);
            int got = f.read(g_inbuf + fill, IN_SZ - fill);
            xSemaphoreGive(g_fsmux);
            if (got > 0) { fill += got; g_file_pos = f.position(); }
        }
        if (fill == 0) break;                              // 파일 끝

        mp3dec_frame_info_t info = {};
        const int samples = mp3dec_decode_frame(g_dec, g_inbuf, fill, g_pcm, &info);

        if (info.frame_bytes == 0) break;                  // 더 진행할 수 없다(깨진 파일 등)
        if (samples > 0) {
            g_src_hz = info.hz; g_src_ch = info.channels; g_bitrate = info.bitrate_kbps;
            push_resampled(g_pcm, samples, info.channels, info.hz);
            g_played_frames += (uint32_t)((uint64_t)samples * OUT_HZ / (info.hz ? info.hz : OUT_HZ));

            // 링이 절반쯤 찬 뒤에 스트림을 시작한다 — 시작 직후 언더런을 피한다.
            // 잭 모드에는 이 절차가 없다(jack_task 가 그냥 링을 빨아간다).
            if (!started && g_out == OUT_BT && g_bt_conn && ring_used() > RING_SZ / 2) {
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
                vTaskDelay(pdMS_TO_TICKS(50));
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
                started = true;
            }
        }
        memmove(g_inbuf, g_inbuf + info.frame_bytes, fill - info.frame_bytes);
        fill -= info.frame_bytes;
    }

    // 남은 링을 비우고 나서 멈춘다(꼬리가 잘리지 않게)
    if (started) {
        const uint32_t t0 = millis();
        while (ring_used() > 1024 && millis() - t0 < 3000 && !g_req_stop) vTaskDelay(pdMS_TO_TICKS(20));
        esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);
    }
    f.close();
    Serial.printf("[dec] ⏹ %s (언더런 %u회)\n", t.name.c_str(), (unsigned)g_underruns);
}

static void decode_task(void *)
{
    for (;;) {
        if (g_req_track >= 0) {
            const int want = g_req_track;
            g_req_track = -1;
            g_req_stop = false;
            if (want < (int)g_lib.size()) {
                g_idx = want;
                g_state = ST_PLAY;
                play_file(g_lib[want]);
                if (!g_req_stop && g_req_track < 0 && g_state == ST_PLAY) {
                    // 자연 종료 → 다음 곡
                    if (g_lib.size() > 1) g_req_track = (want + 1) % (int)g_lib.size();
                    else { g_state = ST_IDLE; }
                } else if (g_req_track < 0 && g_state != ST_PAUSE) {
                    g_state = ST_IDLE;
                }
            }
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// ─────────────────────────────────────────────────────────────── 조작

static void cmd_play(int i)
{
    // 싱크 모드에서는 링버퍼를 맥·폰이 쓴다. 디코더가 같이 쓰면 서로 뭉갠다.
    if (g_role == ROLE_SINK) return;
    if (i < 0 || i >= (int)g_lib.size()) return;
    g_req_stop = true;
    g_state = ST_PLAY;
    g_req_track = i;
}

static void cmd_stop()
{
    g_req_stop = true;
    g_state = ST_IDLE;
    g_req_track = -1;
    ring_reset();
}

static void cmd_toggle()
{
    if (g_state == ST_PLAY) g_state = ST_PAUSE;
    else if (g_state == ST_PAUSE) g_state = ST_PLAY;
    else cmd_play(g_idx >= 0 ? g_idx : 0);
}

static void cmd_step(int d)
{
    if (g_lib.empty()) return;
    int n = (int)g_lib.size();
    int i = ((g_idx < 0 ? 0 : g_idx) + d % n + n) % n;
    cmd_play(i);
}

static void cmd_vol(int v)
{
    g_vol = constrain(v, 0, 100);
    g_prefs.putInt("vol", g_vol);
    // 잭은 코덱의 아날로그 볼륨으로 내린다(0~63). 디지털로 깎으면 노이즈가 된다.
    if (g_codec_ok) {
        const uint8_t hv = (uint8_t)((g_vol * 63) / 100);
        g_codec.SetVolumeHeadphone(hv);
        g_codec.SetVolumeSpeaker(hv);
    }
}

static void cmd_out(OutMode m)
{
    if (m == g_out) return;
    // BT 로 내보내려는데 스택이 안 올라와 있으면(잭 모드로 부팅했을 때) 재부팅한다.
    if (m == OUT_BT && !g_bt_up) {
        g_prefs.putInt("out", (int)OUT_BT);
        Serial.println("[출력] 블루투스로 전환 — 스택을 올리려면 재부팅한다");
        delay(200);
        ESP.restart();
    }
    // A2DP 스트림을 먼저 접는다. 안 접으면 이어폰 쪽이 무음 스트림을 계속 받는다.
    if (g_out == OUT_BT && g_bt_stream) esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);
    g_out = m;
    g_prefs.putInt("out", (int)m);
    Serial.printf("[출력] %s\n", m == OUT_JACK ? "잭 (AC101/I2S)" : "블루투스 (A2DP)");
    // 재생 중이었다면 같은 곡을 새 경로로 다시 시작한다 — 링을 비우고 가는 게 안전하다.
    if (g_state == ST_PLAY && g_idx >= 0) cmd_play(g_idx);
}

// ─────────────────────────────────────────────────────────────── 웹 API

static const char *state_name()
{
    if (g_state == ST_PLAY)  return "재생 중";
    if (g_state == ST_PAUSE) return "일시정지";
    return "정지";
}

static String json_escape(const String &s)
{
    String o;
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if ((uint8_t)c < 0x20) { char b[8]; snprintf(b, sizeof b, "\\u%04x", c); o += b; }
        else o += c;
    }
    return o;
}

static void api_status()
{
    const uint32_t ela = g_played_frames / OUT_HZ;
    uint32_t dur = 0;
    if (g_bitrate > 0 && g_file_len > 0) dur = (uint32_t)((uint64_t)g_file_len * 8 / (g_bitrate * 1000));
    const int pct = g_file_len ? (int)((uint64_t)g_file_pos * 100 / g_file_len) : 0;

    String j = "{";
    j += "\"state\":\""; j += state_name(); j += "\",";
    j += "\"playing\":"; j += (g_state == ST_PLAY) ? "true," : "false,";
    j += "\"idx\":"; j += String((int)g_idx); j += ",";
    j += "\"track\":\""; j += (g_idx >= 0 && g_idx < (int)g_lib.size()) ? json_escape(g_lib[g_idx].name) : ""; j += "\",";
    j += "\"elapsed\":"; j += String(ela); j += ",";
    j += "\"duration\":"; j += String(dur); j += ",";
    j += "\"pct\":"; j += String(pct); j += ",";
    j += "\"vol\":"; j += String((int)g_vol); j += ",";
    j += "\"src\":\""; j += String(g_src_hz) + "Hz/" + String(g_src_ch) + "ch/" + String(g_bitrate) + "k"; j += "\",";
    j += "\"under\":"; j += String((unsigned)g_underruns); j += ",";
    j += "\"heap\":"; j += String((unsigned)ESP.getFreeHeap()); j += ",";
    j += "\"role\":\""; j += (g_role == ROLE_SINK ? "sink" : "player"); j += "\",";
    j += "\"sinkhz\":"; j += String((int)g_sink_hz); j += ",";
    j += "\"sinkkb\":"; j += String((unsigned)(g_sink_bytes / 1024)); j += ",";
    j += "\"starve\":"; j += String((unsigned)g_starves); j += ",";
    j += "\"sinkdrop\":"; j += String((unsigned)g_sink_drops); j += ",";
    j += "\"out\":\""; j += (g_out == OUT_JACK ? "jack" : "bt"); j += "\",";
    j += "\"hp\":"; j += (digitalRead(PIN_HP_DETECT) == LOW) ? "true," : "false,";
    j += "\"jackok\":"; j += g_codec_ok ? "true," : "false,";
    j += "\"tone\":"; j += ((int32_t)(g_tone_until - millis()) > 0) ? "true," : "false,";
    j += "\"i2sframes\":"; j += String((unsigned long)(g_i2s_frames & 0xFFFFFFFF)); j += ",";
    j += "\"storage\":\""; j += g_store_name; j += "\",";

    uint64_t used = 0, total = 0;
    if (g_have_sd) { used = SD_MMC.usedBytes(); total = SD_MMC.totalBytes(); }
    else           { used = LittleFS.usedBytes(); total = LittleFS.totalBytes(); }
    j += "\"used\":"; j += String((unsigned long long)used); j += ",";
    j += "\"total\":"; j += String((unsigned long long)total); j += ",";
    j += "\"sd\":"; j += g_have_sd ? "true," : "false,";

    j += "\"wifi\":\"";
    if (WiFi.getMode() & WIFI_MODE_AP) j += "AP " + WiFi.softAPIP().toString();
    else j += WiFi.SSID() + " " + WiFi.localIP().toString();
    j += "\",";

    j += "\"bt\":{";
    j += "\"state\":\"";
    j += !g_bt_up ? "미사용(잭 모드)"
       : g_bt_stream ? "스트리밍" : (g_bt_conn ? "연결됨(대기)" : (g_bt_scanning ? "탐색 중" : "미연결"));
    j += "\",";
    j += "\"connected\":"; j += g_bt_conn ? "true," : "false,";
    j += "\"streaming\":"; j += g_bt_stream ? "true," : "false,";
    j += "\"scanning\":"; j += g_bt_scanning ? "true," : "false,";
    j += "\"addr\":\""; j += g_bt_have_target ? addr_str(g_bt_target) : ""; j += "\",";
    j += "\"devices\":[";
    for (size_t i = 0; i < g_bt_devs.size(); i++) {
        if (i) j += ",";
        j += "{\"addr\":\"" + addr_str(g_bt_devs[i].bda) + "\",";
        j += "\"name\":\"" + json_escape(String(g_bt_devs[i].name)) + "\",";
        j += "\"rssi\":" + String(g_bt_devs[i].rssi) + ",";
        j += "\"cls\":\"" + String(cod_major_name(g_bt_devs[i].major)) + "\"}";
    }
    j += "]},";

    j += "\"files\":[";
    for (size_t i = 0; i < g_lib.size(); i++) {
        if (i) j += ",";
        j += "{\"n\":\"" + json_escape(g_lib[i].name) + "\",";
        j += "\"s\":" + String((unsigned long)g_lib[i].size) + ",";
        j += "\"sd\":" + String(g_lib[i].sd ? "true" : "false") + "}";
    }
    j += "]}";

    g_http.send(200, "application/json; charset=utf-8", j);
}

// ── SD 백업: 목록 + 원본 그대로 내려주기. 쓰기 기능은 일부러 없다.
static void api_sd_list()
{
    if (!g_have_sd) { g_http.send(404, "text/plain", "SD 없음"); return; }
    String p = g_http.hasArg("p") ? g_http.arg("p") : "/";
    if (!p.startsWith("/")) p = "/" + p;

    File d = SD_MMC.open(p.c_str());
    if (!d || !d.isDirectory()) { g_http.send(404, "text/plain", "디렉터리 아님"); return; }

    String j = "{\"path\":\"" + json_escape(p) + "\",\"entries\":[";
    bool first = true;
    for (File f = d.openNextFile(); f; f = d.openNextFile()) {
        if (!first) j += ",";
        first = false;
        j += "{\"n\":\"" + json_escape(String(f.name())) + "\",";
        j += "\"p\":\"" + json_escape(String(f.path())) + "\",";
        j += "\"d\":" + String(f.isDirectory() ? "true" : "false") + ",";
        j += "\"s\":" + String((unsigned long)f.size()) + "}";
        f.close();
    }
    d.close();
    j += "]}";
    g_http.send(200, "application/json; charset=utf-8", j);
}

// SD 파일을 내려준다. Range 를 직접 처리한다 — WebServer::streamFile 은 Range 를
// 무시하고 항상 전체를 보낸다. 1.4GB 를 50분 받다가 한 번 끊기면 처음부터 다시
// 받아야 하는데, 그게 이 경로에서 제일 큰 위험이었다.
static void api_sd_get()
{
    if (!g_have_sd) { g_http.send(404, "text/plain", "SD 없음"); return; }
    String p = g_http.urlDecode(g_http.uri().substring(3));   // "/sd" 를 떼면 앞에 / 가 남는다
    if (p.length() == 0) p = "/";

    File f = SD_MMC.open(p.c_str(), FILE_READ);
    if (!f || f.isDirectory()) { g_http.send(404, "text/plain", "파일 없음"); return; }

    const size_t len = f.size();
    size_t start = 0, end = len ? len - 1 : 0;
    bool partial = false;

    String r = g_http.header("Range");
    if (r.startsWith("bytes=")) {
        r = r.substring(6);
        const int dash = r.indexOf('-');
        if (dash >= 0) {
            if (dash > 0) start = (size_t)strtoull(r.substring(0, dash).c_str(), nullptr, 10);
            const String tail = r.substring(dash + 1);
            if (tail.length()) end = (size_t)strtoull(tail.c_str(), nullptr, 10);
            if (end >= len) end = len ? len - 1 : 0;
            if (start <= end) partial = true;
        }
    }
    if (!partial) { start = 0; end = len ? len - 1 : 0; }
    const size_t want = (len == 0) ? 0 : (end - start + 1);

    // 큰 파일을 내려주는 동안 디코더가 SD 를 같이 읽으면 둘 다 느려진다. 재생을 양보한다.
    const bool was = (g_state == ST_PLAY);
    g_busy_fs = true;

    WiFiClient c = g_http.client();
    String h = partial ? "HTTP/1.1 206 Partial Content\r\n" : "HTTP/1.1 200 OK\r\n";
    h += "Content-Type: application/octet-stream\r\n";
    h += "Content-Length: " + String((unsigned long)want) + "\r\n";
    h += "Accept-Ranges: bytes\r\n";
    if (partial)
        h += "Content-Range: bytes " + String((unsigned long)start) + "-" +
             String((unsigned long)end) + "/" + String((unsigned long)len) + "\r\n";
    h += "Content-Disposition: attachment\r\n";
    h += "Connection: close\r\n\r\n";
    c.print(h);

    if (want) {
        f.seek(start);
        // 4KB 씩 민다. 더 작게 쓰면 TCP 헤더 몫이 커지고, 더 크게 잡으면 내부 DRAM 이 아깝다.
        static uint8_t buf[4096];
        size_t left = want;
        while (left && c.connected()) {
            const size_t n = f.read(buf, left < sizeof buf ? left : sizeof buf);
            if (n == 0) break;
            if (c.write(buf, n) != n) break;
            left -= n;
        }
    }
    c.stop();
    f.close();
    g_busy_fs = false;
    if (was) g_state = ST_PLAY;
}

static File g_up;
static String g_up_err;

static bool sd_music_path(const String &name, String &out)
{
    String n = name.substring(name.lastIndexOf('/') + 1);   // 디렉터리 성분 제거
    if (n.length() == 0 || n.indexOf("..") >= 0) return false;
    out = String(SD_MUSIC_DIR) + "/" + n;
    return true;
}

static void api_upload()
{
    HTTPUpload &u = g_http.upload();
    if (u.status == UPLOAD_FILE_START) {
        g_up_err = "";
        g_busy_fs = true;
        cmd_stop();                            // 업로드 중에는 재생을 접는다(글리치 방지)
        String n = u.filename;
        n = n.substring(n.lastIndexOf('/') + 1);
        if (!ends_with_mp3(n)) { g_up_err = "mp3 만 받는다"; return; }

        // 대상: SD 가 있으면 SD 의 /MP3, 없으면 내장 플래시. ?to=flash 로 강제 가능.
        const bool to_sd = g_have_sd && g_http.arg("to") != "flash";
        if (to_sd) {
            String path;
            if (!sd_music_path(n, path)) { g_up_err = "파일 이름이 이상하다"; return; }
            if (!SD_MMC.exists(SD_MUSIC_DIR)) SD_MMC.mkdir(SD_MUSIC_DIR);
            if (SD_MMC.exists(path)) SD_MMC.remove(path);
            g_up = SD_MMC.open(path, FILE_WRITE);
            if (!g_up) g_up_err = "SD 열기 실패";
            else Serial.printf("[up] SD %s 수신 시작\n", path.c_str());
        } else {
            String path = "/" + n;
            if (LittleFS.exists(path)) LittleFS.remove(path);
            g_up = LittleFS.open(path, FILE_WRITE);
            if (!g_up) g_up_err = "플래시 열기 실패";
            else Serial.printf("[up] 플래시 %s 수신 시작\n", path.c_str());
        }
    } else if (u.status == UPLOAD_FILE_WRITE) {
        if (g_up && g_up_err.isEmpty()) {
            if (g_up.write(u.buf, u.currentSize) != u.currentSize) {
                g_up_err = "저장소가 꽉 찼다";
                g_up.close();
            }
        }
    } else if (u.status == UPLOAD_FILE_END || u.status == UPLOAD_FILE_ABORTED) {
        if (g_up) g_up.close();
        g_busy_fs = false;
        if (u.status == UPLOAD_FILE_ABORTED) g_up_err = "중단됨";
        lib_rescan();
        Serial.printf("[up] 종료 (%u 바이트) %s\n", (unsigned)u.totalSize,
                      g_up_err.isEmpty() ? "" : g_up_err.c_str());
    }
}

static void wifi_setup()
{
    String ssid = g_prefs.getString("ssid", "");
    String pass = g_prefs.getString("pass", "");
    WiFi.setHostname("esp32-mp3");

    if (ssid.length()) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid.c_str(), pass.c_str());
        Serial.printf("[wifi] %s 접속 시도\n", ssid.c_str());
        const uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) { delay(250); Serial.print('.'); }
        Serial.println();
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("[wifi] 접속됨 http://%s/  (mDNS: http://esp32-mp3.local/)\n",
                          WiFi.localIP().toString().c_str());
            MDNS.begin("esp32-mp3");
            return;
        }
        Serial.println("[wifi] 실패 → AP 모드로 뜬다");
    }
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[wifi] AP %s / %s → http://%s/\n",
                  AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
}

static void http_setup()
{
    g_http.on("/", HTTP_GET, [] {
        g_http.sendHeader("Cache-Control", "no-store");
        g_http.send_P(200, "text/html; charset=utf-8", UI_HTML);
    });
    g_http.on("/api/status", HTTP_GET, api_status);
    g_http.on("/api/play",   HTTP_POST, [] { cmd_play(g_http.arg("i").toInt()); g_http.send(200, "text/plain", "ok"); });
    g_http.on("/api/toggle", HTTP_POST, [] { cmd_toggle(); g_http.send(200, "text/plain", "ok"); });
    g_http.on("/api/stop",   HTTP_POST, [] { cmd_stop();   g_http.send(200, "text/plain", "ok"); });
    g_http.on("/api/next",   HTTP_POST, [] { cmd_step(+1); g_http.send(200, "text/plain", "ok"); });
    g_http.on("/api/prev",   HTTP_POST, [] { cmd_step(-1); g_http.send(200, "text/plain", "ok"); });
    g_http.on("/api/vol",    HTTP_POST, [] { cmd_vol(g_http.arg("v").toInt()); g_http.send(200, "text/plain", "ok"); });
    g_http.on("/api/rescan", HTTP_POST, [] { lib_rescan(); g_http.send(200, "text/plain", "ok"); });
    // 역할 전환. Bluedroid 는 소스와 싱크를 동시에 못 들고, 런타임 전환도 상태가
    // 꼬이기 쉽다 — NVS 에 적고 재부팅하는 쪽이 확실하다.
    g_http.on("/api/role", HTTP_POST, [] {
        const bool sink = (g_http.arg("r") == "sink");
        g_prefs.putInt("role", sink ? (int)ROLE_SINK : (int)ROLE_PLAYER);
        g_http.send(200, "text/plain", sink ? "싱크로 전환, 재부팅" : "플레이어로 전환, 재부팅");
        delay(300);
        ESP.restart();
    });
    g_http.on("/api/out",    HTTP_POST, [] {
        cmd_out(g_http.arg("m") == "jack" ? OUT_JACK : OUT_BT);
        g_http.send(200, "text/plain", "ok");
    });
    g_http.on("/api/avrc", HTTP_POST, [] {
        avrc_cmd(g_http.arg("c"));
        g_http.send(200, "text/plain", g_role == ROLE_SINK ? "ok" : "싱크 모드에서만 동작한다");
    });
    g_http.on("/api/pa", HTTP_POST, [] {
        const bool on = (g_http.arg("on") == "1");
        digitalWrite(PIN_PA, on ? HIGH : LOW);
        g_http.send(200, "text/plain", on ? "앰프 ON(스피커용)" : "앰프 OFF");
    });
    g_http.on("/api/tone",   HTTP_POST, [] {
        // 잭이 살아 있는지 다시 굽지 않고 확인하는 버튼. 재생 경로와 무관하게 동작한다.
        const int s = g_http.hasArg("s") ? g_http.arg("s").toInt() : 5;
        g_tone_until = millis() + (uint32_t)constrain(s, 1, 60) * 1000;
        g_http.send(200, "text/plain", g_codec_ok ? "ok" : "AC101 이 없다");
    });

    g_http.on("/api/delete", HTTP_POST, [] {
        String n = g_http.arg("f");
        n = n.substring(n.lastIndexOf('/') + 1);
        bool ok = false;
        String path = "/" + n;
        if (LittleFS.exists(path)) ok = LittleFS.remove(path);
        // SD 는 **/MP3 안에서만** 지운다. DCIM 등 원래 내용은 어떤 경로로도 못 지운다.
        if (!ok && g_have_sd) {
            String sp;
            if (sd_music_path(n, sp) && SD_MMC.exists(sp)) ok = SD_MMC.remove(sp);
        }
        if (ok) { cmd_stop(); lib_rescan(); }
        g_http.send(ok ? 200 : 404, "text/plain",
                    ok ? "ok" : "없는 파일 (SD 는 /MP3 안에서만 지운다)");
    });

    g_http.on("/api/upload", HTTP_POST,
              [] { g_http.send(g_up_err.isEmpty() ? 200 : 500, "text/plain",
                               g_up_err.isEmpty() ? "ok" : g_up_err.c_str()); },
              api_upload);

    g_http.on("/api/bt/scan",       HTTP_POST, [] { bt_scan_start(); g_http.send(200, "text/plain", "ok"); });
    g_http.on("/api/bt/disconnect", HTTP_POST, [] {
        if (g_bt_have_target) esp_a2d_source_disconnect(g_bt_target);
        g_http.send(200, "text/plain", "ok");
    });
    g_http.on("/api/bt/connect", HTTP_POST, [] {
        esp_bd_addr_t a;
        if (!addr_parse(g_http.arg("a"), a)) { g_http.send(400, "text/plain", "주소 형식"); return; }
        esp_bt_gap_cancel_discovery();
        memcpy(g_bt_target, a, sizeof a);
        g_bt_have_target = true;
        g_http.send(esp_a2d_source_connect(a) == ESP_OK ? 200 : 500, "text/plain", "ok");
    });

    // AC101 레지스터 덤프. 다시 굽지 않고 코덱 상태를 보려면 이게 필요하다 —
    // 무언가를 물리적으로 바꿔 보며 앞뒤를 비교할 때 쓴다.
    g_http.on("/api/codec", HTTP_GET, [] {
        if (!g_codec_ok) { g_http.send(503, "text/plain", "AC101 없음"); return; }
        static const uint8_t regs[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0C, 0x10, 0x11,
                                       0x40, 0x41, 0x44, 0x45, 0x46, 0x48, 0x49, 0x4C, 0x4D,
                                       0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x58};
        String j = "{\"pa22\":" + String(digitalRead(PIN_LED_G)) +
                   ",\"pa21\":" + String(digitalRead(PIN_PA)) + ",\"regs\":{";
        for (unsigned i = 0; i < sizeof regs; i++) {
            if (i) j += ",";
            char b[24];
            snprintf(b, sizeof b, "\"0x%02X\":\"0x%04X\"", regs[i], g_codec.rd(regs[i]));
            j += b;
        }
        j += "}}";
        g_http.send(200, "application/json", j);
    });

    g_http.on("/api/sd/list", HTTP_GET, api_sd_list);
    g_http.onNotFound([] {
        if (g_http.uri().startsWith("/sd/")) api_sd_get();
        else g_http.send(404, "text/plain", "없음");
    });

    g_http.on("/api/wifi", HTTP_POST, [] {
        g_prefs.putString("ssid", g_http.arg("ssid"));
        g_prefs.putString("pass", g_http.arg("pass"));
        g_http.send(200, "text/plain", "ok");
        delay(300);
        ESP.restart();
    });

    // Range 헤더는 기본으로 수집되지 않는다. 명시해야 header("Range") 가 값을 준다.
    const char *want_headers[] = {"Range"};
    g_http.collectHeaders(want_headers, 1);

    g_http.begin();
}

// ─────────────────────────────────────────────────────────────── 버튼

static Key g_keys[] = {
    {KEY_PLAY, 0, true, false}, {KEY_NEXT, 0, true, false}, {KEY_PREV, 0, true, false},
    {KEY_VUP,  0, true, false}, {KEY_VDN,  0, true, false}, {KEY_BT,   0, true, false},
};

static void keys_setup()
{
    // 앰프와 LED 를 키보다 먼저 구동한다. GPIO13(KEY2)이 GPIO22 에 끌리는 문제도
    // 이걸로 같이 해결된다(핀을 먼저 구동해 두면 읽기가 안정된다).
    pinMode(PIN_PA, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
    // ★ 파워앰프는 기본 OFF. 스피커 JST 에 아무것도 안 붙은 상태에서 앰프를 돌리면
    //   아날로그 전원에 노이즈를 얹어 헤드폰 잭 쪽 잡음이 커진다. 스피커를 쓸 때만
    //   /api/pa?on=1 로 켠다.
    digitalWrite(PIN_PA, LOW);
    digitalWrite(PIN_LED_G, HIGH);   // 초록 LED. 상태 표시로 쓴다(led_status() 참고)
    for (auto &k : g_keys) pinMode(k.pin, k.pin == 36 ? INPUT : INPUT_PULLUP);   // 36 은 입력 전용(내부 풀업 없음)
}

static void keys_poll()
{
    const uint32_t now = millis();
    for (size_t i = 0; i < sizeof g_keys / sizeof g_keys[0]; i++) {
        Key &k = g_keys[i];
        const bool lvl = digitalRead(k.pin);            // 눌리면 LOW
        if (k.prev && !lvl) { k.down_at = now; k.longsent = false; }
        else if (!k.prev && lvl && now - k.down_at > 30 && !k.longsent) {
            // 싱크 모드에서는 같은 버튼이 **맥의 재생**을 움직인다(AVRCP).
            if (g_role == ROLE_SINK) {
                switch (k.pin) {
                case KEY_PLAY: avrc_cmd("toggle"); break;
                case KEY_NEXT: avrc_cmd("next");   break;
                case KEY_PREV: avrc_cmd("prev");   break;
                case KEY_VUP:  cmd_vol(g_vol + 5); break;
                case KEY_VDN:  cmd_vol(g_vol - 5); break;
                }
                Serial.printf("[key] GPIO%d\n", k.pin);
                k.prev = lvl;
                continue;
            }
            switch (k.pin) {                            // 짧게 눌렀을 때
            case KEY_PLAY: cmd_toggle(); break;
            case KEY_NEXT: cmd_step(+1); break;
            case KEY_PREV: cmd_step(-1); break;
            case KEY_VUP:  cmd_vol(g_vol + 5); break;
            case KEY_VDN:  cmd_vol(g_vol - 5); break;
            case KEY_BT:   if (g_bt_have_target) esp_a2d_source_connect(g_bt_target); break;
            }
            Serial.printf("[key] GPIO%d\n", k.pin);
        } else if (!lvl && !k.longsent && now - k.down_at > 1200) {
            k.longsent = true;
            // KEY6(GPIO5)은 헤드폰 검출 겸용이라 이어폰을 꽂으면 계속 LOW 다.
            // 길게 누르기로 BT 탐색을 걸면 꽂는 순간 오발화한다 — 웹 UI 로만 둔다.
            if (k.pin == KEY_PLAY) { cmd_stop(); }
            else if (k.pin == KEY_VUP) {                  // 테스트음 5초 — 잭이 사는지 귀로 본다
                g_tone_until = millis() + 5000;
                Serial.println("[key] 잭 테스트음 5초");
            } else if (k.pin == KEY_VDN) {                // 출력 경로 전환
                cmd_out(g_out == OUT_BT ? OUT_JACK : OUT_BT);
            }
        }
        k.prev = lvl;
    }
}

// 초록 LED(GPIO22)로 상태를 보여준다.
//
// 한때 이 핀을 PA_ENABLE 로 의심해 깜빡임을 다 뺐는데, 실측으로 판정됐다 —
// loopback_test 에서 GPIO22 를 LOW/HIGH 로 바꿔도 출력 검파값이 그대로였다(비 0.85).
// 앰프는 GPIO21 이고 22 는 LED 다. 그래서 다시 상태 표시로 쓴다.
//
// 파란 LED(GPIO19)는 **KEY3 과 핀을 공유한다** — LED 로 쓰면 이전곡 버튼이 죽는다.
// 그래서 건드리지 않는다.
static void led_status()
{
    //  켜짐        = 소리가 실제로 흐르는 중 (스트리밍 또는 잭 재생)
    //  느린 깜빡임 = 연결됨/대기
    //  빠른 깜빡임 = 미연결
    const bool flowing = g_bt_stream || (g_out == OUT_JACK && g_state == ST_PLAY);
    const uint32_t p = flowing ? 0 : (g_bt_conn ? 1000 : 250);
    digitalWrite(PIN_LED_G, p == 0 ? HIGH : ((millis() / p) & 1));
}

// ─────────────────────────────────────────────────────────────── setup / loop

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== ESP32-Audio-Kit 웹 MP3 플레이어 ===");
    Serial.println("출력은 두 갈래다: 블루투스(A2DP) 와 3.5mm 잭(AC101/I2S).");
    Serial.println("잭 출력은 실기 확인됐다(2026-09-08). 8월의 '오디오 죽음' 은");
    Serial.println("board_pins.h 의 LRCK/DOUT 이 25/26 으로 뒤바뀐 탓이었다.");

    keys_setup();
    g_fsmux = xSemaphoreCreateMutex();
    g_prefs.begin("mp3", false);
    g_vol = g_prefs.getInt("vol", 60);
    g_out = (OutMode)g_prefs.getInt("out", (int)OUT_BT);
    g_role = (Role)g_prefs.getInt("role", (int)ROLE_PLAYER);

    // 큰 버퍼는 전부 PSRAM 으로. 내부 DRAM 은 BT+WiFi 스택에 남겨야 한다.
    if (!psramFound()) Serial.println("[mem] PSRAM 이 안 보인다 — PSRAM=enabled 로 빌드했는지 확인");
    g_ring  = (uint8_t *)ps_malloc(RING_SZ);
    g_inbuf = (uint8_t *)ps_malloc(IN_SZ);
    g_pcm   = (int16_t *)ps_malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(int16_t));
    g_dec   = (mp3dec_t *)ps_malloc(sizeof(mp3dec_t));

    // minimp3 의 프레임 스크래치도 PSRAM 으로 뺀다. 원본은 이걸 스택에 잡는데 16KB 가
    // 넘어서 6KB 스택 태스크가 "Stack canary watchpoint triggered (dec)" 로 즉사했다.
    // 내부 DRAM 이 27KB 뿐이라 스택을 키우는 건 답이 아니었다.
    void *scratch = ps_malloc(mp3dec_scratch_size());
    mp3dec_set_scratch(scratch);
    Serial.printf("[mem] minimp3 스크래치 %u KB 를 PSRAM 에 잡았다\n",
                  (unsigned)(mp3dec_scratch_size() / 1024));

    if (!g_ring || !g_inbuf || !g_pcm || !g_dec || !scratch) {
        Serial.println("[mem] 버퍼 할당 실패 — 중단");
        while (1) delay(1000);
    }
    Serial.printf("[mem] PSRAM 여유 %u KB, 내부 힙 %u KB\n",
                  (unsigned)(ESP.getFreePsram() / 1024), (unsigned)(ESP.getFreeHeap() / 1024));

    // 싱크(BT 스피커) 모드에서는 파일을 안 쓴다. SD/LittleFS 마운트는 내부 DRAM 을
    // 상당히 먹는데 BT 싱크 + WiFi + 웹서버가 이미 빡빡하다(싱크에서 힙 10KB 까지
    // 떨어지는 걸 봤다). 안 올려서 돌려준다.
    if (g_role == ROLE_PLAYER) {
        storage_setup();
        lib_rescan();
    } else {
        g_store_name = "쓰지 않음(싱크)";
        Serial.println("[fs] 싱크 모드 — SD/플래시를 마운트하지 않는다(메모리 확보)");
    }
    jack_setup();          // I2S/AC101 은 모드와 무관하게 올려 둔다 — 테스트음이 항상 되도록
    wifi_setup();
    http_setup();
    bt_setup();

    Serial.printf("[출력] %s\n", g_out == OUT_JACK ? "잭 (AC101/I2S)" : "블루투스 (A2DP)");

    // 디코더와 잭 송출은 코어 1 에. WiFi/BT 스택은 코어 0 에서 돈다.
    xTaskCreatePinnedToCore(decode_task, "dec", 6144, nullptr, 5, nullptr, 1);
    if (g_codec_ok) xTaskCreatePinnedToCore(jack_task, "jack", 4096, nullptr, 6, nullptr, 1);

    // 초기화가 끝난 뒤의 내부 힙이 진짜 여유다. 여기가 20KB 밑으로 떨어지면
    // 재생을 걸었을 때 파일 열기에서 죽는다 — 실제로 한 번 그렇게 죽었다.
    Serial.printf("[mem] 초기화 후 내부 힙 %u KB (최대 연속 블록 %u KB)\n",
                  (unsigned)(ESP.getFreeHeap() / 1024),
                  (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));
    Serial.println("[준비] 브라우저로 접속해 MP3 를 올리고, 이어폰을 페어링 모드로 두고 탐색하면 된다.");
}

void loop()
{
    // WiFi 켜기/끄기는 여기서 처리한다(BT 콜백에서 직접 만지면 위험하다).
    if (g_wifi_req == 1) {
        g_wifi_req = 0;
        Serial.println("[wifi] 스트리밍 중 — WiFi 를 끈다(2.4GHz 를 BT 에 양보)");
        WiFi.mode(WIFI_OFF);
    } else if (g_wifi_on_at && (int32_t)(millis() - g_wifi_on_at) > 0 && !g_bt_stream) {
        g_wifi_on_at = 0;
        Serial.println("[wifi] 스트리밍 멈춤 — WiFi 를 되살린다");
        wifi_setup();
        g_http.begin();
    }

    // 싱크 상태를 시리얼로 주기적으로 찍는다 — WiFi 가 꺼져 있어도 이걸로 계측한다.
    if (g_role == ROLE_SINK) {
        static uint32_t last_stat = 0;
        if (g_bt_stream && millis() - last_stat > 3000) {
            last_stat = millis();
            Serial.printf("[싱크] 수신 %u KB · 버림 %u · 굶음 %u · 링 %u%% · 힙 %u\n",
                          (unsigned)(g_sink_bytes / 1024), (unsigned)g_sink_drops,
                          (unsigned)g_starves, (unsigned)(ring_used() * 100 / RING_SZ),
                          (unsigned)ESP.getFreeHeap());
        }
    }

    g_http.handleClient();
    keys_poll();
    led_status();
    delay(2);
}
