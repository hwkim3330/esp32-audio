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
#include <esp_heap_caps.h>

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
static const int PIN_LED  = 22;   // 상태 LED. 키를 읽기 전에 먼저 설정해야 GPIO13 이 안정된다.

static const int SD_CLK = 14, SD_CMD = 15, SD_D0 = 2;   // SDMMC 슬롯1, 1비트 모드

static const char *AP_SSID = "esp32-mp3";
static const char *AP_PASS = "12345678";
static const char *BT_NAME = "KETI-MP3";

static const int OUT_HZ = 44100;   // A2DP 표준. 여기로 맞춰 리샘플한다.

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
static volatile int    g_vol = 60;              // 0~100, A2DP 콜백에서 적용(즉시 반응)
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

static void a2dp_cb(esp_a2d_cb_event_t ev, esp_a2d_cb_param_t *p)
{
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

static void bt_setup()
{
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
    esp_a2d_source_register_data_callback(a2dp_data_cb);
    if (esp_a2d_source_init() != ESP_OK) { Serial.println("[a2dp] 소스 init 실패"); return; }
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    const uint8_t *mac = esp_bt_dev_get_address();
    Serial.printf("[bt] 준비됨, MAC %02x:%02x:%02x:%02x:%02x:%02x, 이름 %s\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], BT_NAME);

    // 지난번 붙었던 기기로 자동 재연결
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
    if (g_have_sd) scan_dir(SD_MMC, "/", true, 1);
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
    if (SD_MMC.begin("/sdcard", true /*1bit*/, false /*format_if_empty=false*/)) {
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
            if (!started && g_bt_conn && ring_used() > RING_SZ / 2) {
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

static void cmd_vol(int v) { g_vol = constrain(v, 0, 100); g_prefs.putInt("vol", g_vol); }

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
    j += g_bt_stream ? "스트리밍" : (g_bt_conn ? "연결됨(대기)" : (g_bt_scanning ? "탐색 중" : "미연결"));
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

static void api_sd_get()
{
    if (!g_have_sd) { g_http.send(404, "text/plain", "SD 없음"); return; }
    String uri = g_http.uri();                 // /sd/<경로>
    String p = uri.substring(3);               // "/sd" 를 떼면 앞에 / 가 남는다
    if (p.length() == 0) p = "/";

    File f = SD_MMC.open(p.c_str(), FILE_READ);
    if (!f || f.isDirectory()) { g_http.send(404, "text/plain", "파일 없음"); return; }

    // 큰 파일을 내려주는 동안 디코더가 SD 를 같이 읽으면 둘 다 느려진다. 재생을 양보한다.
    const bool was = (g_state == ST_PLAY);
    g_busy_fs = true;
    g_http.sendHeader("Content-Disposition", "attachment");
    g_http.streamFile(f, "application/octet-stream");
    f.close();
    g_busy_fs = false;
    if (was) g_state = ST_PLAY;
}

static File g_up;
static String g_up_err;

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
        // 업로드는 언제나 내장 플래시로. SD 에는 절대 쓰지 않는다.
        String path = "/" + n;
        if (LittleFS.exists(path)) LittleFS.remove(path);
        g_up = LittleFS.open(path, FILE_WRITE);
        if (!g_up) g_up_err = "플래시 열기 실패";
        else Serial.printf("[up] %s 수신 시작\n", path.c_str());
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

    g_http.on("/api/delete", HTTP_POST, [] {
        String n = g_http.arg("f");
        n = n.substring(n.lastIndexOf('/') + 1);
        // 지우는 건 내장 플래시에 올린 것만. SD 는 손대지 않는다.
        String path = "/" + n;
        bool ok = LittleFS.exists(path) && LittleFS.remove(path);
        if (ok) { cmd_stop(); lib_rescan(); }
        g_http.send(ok ? 200 : 404, "text/plain", ok ? "ok" : "플래시에 없는 파일(SD 는 지우지 않는다)");
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

    g_http.begin();
}

// ─────────────────────────────────────────────────────────────── 버튼

static Key g_keys[] = {
    {KEY_PLAY, 0, true, false}, {KEY_NEXT, 0, true, false}, {KEY_PREV, 0, true, false},
    {KEY_VUP,  0, true, false}, {KEY_VDN,  0, true, false}, {KEY_BT,   0, true, false},
};

static void keys_setup()
{
    // LED 핀을 먼저 잡는다. 이걸 안 하면 GPIO13(KEY2)이 GPIO22 상태에 끌려 LOW 로 읽힌다.
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
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
            if (k.pin == KEY_BT) { Serial.println("[key] BT 탐색"); bt_scan_start(); }
            else if (k.pin == KEY_PLAY) { cmd_stop(); }
        }
        k.prev = lvl;
    }
}

static void led_poll()
{
    // 켜짐 = 스트리밍, 느린 깜빡임 = BT 연결만, 빠른 깜빡임 = BT 미연결
    const uint32_t p = g_bt_stream ? 0 : (g_bt_conn ? 1000 : 250);
    digitalWrite(PIN_LED, p == 0 ? HIGH : ((millis() / p) & 1));
}

// ─────────────────────────────────────────────────────────────── setup / loop

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== ESP32-Audio-Kit 웹 MP3 플레이어 ===");
    Serial.println("아날로그 출력은 죽어 있다(I2S 0). 소리는 A2DP 로 나간다.");

    keys_setup();
    g_fsmux = xSemaphoreCreateMutex();
    g_prefs.begin("mp3", false);
    g_vol = g_prefs.getInt("vol", 60);

    // 큰 버퍼는 전부 PSRAM 으로. 내부 DRAM 은 BT+WiFi 스택에 남겨야 한다.
    if (!psramFound()) Serial.println("[mem] PSRAM 이 안 보인다 — PSRAM=enabled 로 빌드했는지 확인");
    g_ring  = (uint8_t *)ps_malloc(RING_SZ);
    g_inbuf = (uint8_t *)ps_malloc(IN_SZ);
    g_pcm   = (int16_t *)ps_malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(int16_t));
    g_dec   = (mp3dec_t *)ps_malloc(sizeof(mp3dec_t));
    if (!g_ring || !g_inbuf || !g_pcm || !g_dec) {
        Serial.println("[mem] 버퍼 할당 실패 — 중단");
        while (1) delay(1000);
    }
    Serial.printf("[mem] PSRAM 여유 %u KB, 내부 힙 %u KB\n",
                  (unsigned)(ESP.getFreePsram() / 1024), (unsigned)(ESP.getFreeHeap() / 1024));

    storage_setup();
    lib_rescan();
    wifi_setup();
    http_setup();
    bt_setup();

    // 디코더는 코어 1 에. WiFi/BT 스택은 코어 0 에서 돈다.
    xTaskCreatePinnedToCore(decode_task, "dec", 6144, nullptr, 5, nullptr, 1);

    Serial.println("[준비] 브라우저로 접속해 MP3 를 올리고, 이어폰을 페어링 모드로 두고 탐색하면 된다.");
}

void loop()
{
    g_http.handleClient();
    keys_poll();
    led_poll();
    delay(2);
}
