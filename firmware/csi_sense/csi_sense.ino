// WiFi CSI 감지 — 오디오가 죽은 보드로 하는 온디바이스 감지.
//
// 왜 이걸 하는가:
//   이 보드의 오디오(마이크·스피커)는 실기에서 안 살아났다(firmware/loopback_test 참조).
//   그런데 WiFi 는 멀쩡하고, ESP32 는 CSI(Channel State Information)를 뽑을 수 있다.
//   CSI 는 전파가 사람 몸에 반사·회절되며 변하는 양이라, 이걸로 재실·움직임을 읽는다.
//   카메라도 마이크도 필요 없다 — 추가 부품 0.
//
//   차량 실내 재실 감지(아이 방치 감지)가 실제 자동차 안전 기능이고, 카메라·마이크
//   없이 된다는 게 그 용도의 장점이다.
//
// PSRAM 이 여기서 제 몫을 한다:
//   CSI 프레임을 시계열로 쌓아야 "정적 기준선" 과 "변화" 를 구분할 수 있다.
//   내부 DRAM 으로는 몇 초가 한계인데 PSRAM 4MB 면 수 분을 담는다.
//
// AP 접속이 필요 없다. 프로미스큐어스 모드로 주변 AP 의 비콘(약 100ms 주기)을 받으면
// CSI 가 생성된다. 자격증명 없이 어디서나 동작한다.

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <math.h>

// ── CSI 파라미터
#define MAX_SC        128        // 서브캐리어 상한 (실제는 런타임에 정해진다)
#define HIST_LEN      600        // 시계열 길이. 비콘 10Hz 기준 약 60초.
#define WARMUP_N      120        // 기준선 학습에 쓸 프레임 수

// PSRAM: 서브캐리어별 진폭 시계열
static float   *hist;            // HIST_LEN x MAX_SC
static uint16_t hist_w = 0;
static uint32_t n_frames = 0;
static uint8_t  n_sc = 0;

// 기준선 (정적 환경의 평균/표준편차)
static float base_mu[MAX_SC];
static float base_sd[MAX_SC];
static bool  base_ready = false;

// 최근 프레임 (콜백 → 메인 루프 전달)
static volatile float last_dev = 0.0f;     // 기준선 대비 편차 (z-score 평균)
static volatile float last_flux = 0.0f;    // 직전 프레임 대비 변화량
static float prev_amp[MAX_SC];
static bool  prev_valid = false;

static volatile uint32_t motion_events = 0;
static volatile uint32_t pkt_count = 0;
static int8_t  cur_channel = 1;

// ── CSI 콜백. WiFi 태스크에서 돌므로 짧아야 한다. 출력 금지.
static void IRAM_ATTR csi_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf || info->len < 8) return;
    pkt_count++;

    // CSI 는 서브캐리어당 (虚, 实) int8 쌍이다. 진폭 = sqrt(i^2+q^2).
    const int sc = info->len / 2;
    const int n = (sc > MAX_SC) ? MAX_SC : sc;
    if (!n_sc) n_sc = (uint8_t)n;

    float amp[MAX_SC];
    // 첫 4바이트는 하드웨어 제약으로 무효일 수 있다 — 그 경우 앞 2개 서브캐리어를 건너뛴다.
    const int start = info->first_word_invalid ? 2 : 0;
    for (int i = 0; i < n; i++) {
        if (i < start) { amp[i] = 0.0f; continue; }
        const float im = (float)info->buf[2 * i];
        const float re = (float)info->buf[2 * i + 1];
        amp[i] = sqrtf(im * im + re * re);
    }

    // 시계열에 적재 (PSRAM)
    float *row = hist + (size_t)hist_w * MAX_SC;
    for (int i = 0; i < n; i++) row[i] = amp[i];
    hist_w = (uint16_t)((hist_w + 1) % HIST_LEN);
    n_frames++;

    // 직전 프레임 대비 변화량 — 움직임에 가장 민감한 지표
    if (prev_valid) {
        float s = 0.0f; int c = 0;
        for (int i = start; i < n; i++) {
            s += fabsf(amp[i] - prev_amp[i]);
            c++;
        }
        last_flux = c ? (s / (float)c) : 0.0f;
    }
    for (int i = 0; i < n; i++) prev_amp[i] = amp[i];
    prev_valid = true;

    // 기준선 대비 z-score. 정적이면 0 근처, 사람이 지나가면 크게 튄다.
    if (base_ready) {
        float z = 0.0f; int c = 0;
        for (int i = start; i < n; i++) {
            if (base_sd[i] > 0.5f) { z += fabsf(amp[i] - base_mu[i]) / base_sd[i]; c++; }
        }
        last_dev = c ? (z / (float)c) : 0.0f;
        if (last_dev > 3.0f) motion_events++;
    }
}

// 워밍업 구간의 시계열로 기준선을 만든다.
static void build_baseline()
{
    const int use = (n_frames < HIST_LEN) ? (int)n_frames : HIST_LEN;
    if (use < 20) return;
    for (int i = 0; i < n_sc; i++) {
        double s = 0, s2 = 0;
        for (int t = 0; t < use; t++) {
            const float v = hist[(size_t)t * MAX_SC + i];
            s += v; s2 += (double)v * v;
        }
        const double mu = s / use;
        const double var = s2 / use - mu * mu;
        base_mu[i] = (float)mu;
        base_sd[i] = (float)sqrt(var > 0 ? var : 0.0);
    }
    base_ready = true;
}

// 채널별 패킷량을 재서 가장 붐비는 채널을 고른다. CSI 는 받은 패킷이 있어야 생긴다.
static int8_t pick_busiest_channel()
{
    const int8_t ch[3] = { 1, 6, 11 };
    int8_t best = 1; uint32_t best_n = 0;
    for (int i = 0; i < 3; i++) {
        esp_wifi_set_channel((uint8_t)ch[i], WIFI_SECOND_CHAN_NONE);
        pkt_count = 0;
        delay(1200);
        const uint32_t n = pkt_count;
        Serial.printf("  채널 %2d → %lu 프레임/1.2초\n", ch[i], (unsigned long)n);
        if (n > best_n) { best_n = n; best = ch[i]; }
    }
    return best;
}

void setup()
{
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== WiFi CSI 감지 ===");
    Serial.println("전파 변화로 움직임을 읽는다. 카메라·마이크 없음, 추가 부품 0.");

    if (!psramFound()) { Serial.println("PSRAM 없음. 중단."); while (1) delay(1000); }
    hist = (float *)heap_caps_calloc((size_t)HIST_LEN * MAX_SC, sizeof(float),
                                     MALLOC_CAP_SPIRAM);
    if (!hist) { Serial.println("PSRAM 할당 실패. 중단."); while (1) delay(1000); }
    Serial.printf("PSRAM 시계열 버퍼 %.2f MB (%d 프레임 × %d 서브캐리어)\n",
                  (double)HIST_LEN * MAX_SC * sizeof(float) / 1048576.0,
                  HIST_LEN, MAX_SC);

    // AP 접속 없이 프로미스큐어스로 주변 트래픽을 받는다.
    WiFi.mode(WIFI_MODE_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_filter(nullptr);

    wifi_csi_config_t cfg = {};
    cfg.lltf_en          = true;
    cfg.htltf_en         = true;
    cfg.stbc_htltf2_en   = true;
    cfg.ltf_merge_en     = true;
    cfg.channel_filter_en = true;   // 인접 서브캐리어 스무딩 — 노이즈가 줄어든다
    cfg.manu_scale       = false;
    cfg.shift            = 0;
    cfg.dump_ack_en      = false;
    if (esp_wifi_set_csi_config(&cfg) != ESP_OK) {
        Serial.println("CSI 설정 실패. 중단."); while (1) delay(1000);
    }
    esp_wifi_set_csi_rx_cb(csi_cb, nullptr);
    if (esp_wifi_set_csi(true) != ESP_OK) {
        Serial.println("CSI 활성화 실패. 중단."); while (1) delay(1000);
    }
    Serial.println("CSI 활성화 완료");

    Serial.println("\n[1] 붐비는 채널 찾기 (CSI 는 받은 패킷이 있어야 생긴다)");
    cur_channel = pick_busiest_channel();
    esp_wifi_set_channel((uint8_t)cur_channel, WIFI_SECOND_CHAN_NONE);
    Serial.printf("  → 채널 %d 선택\n", cur_channel);

    Serial.printf("\n[2] 기준선 학습 (%d 프레임). 이 동안 보드 주변에서 움직이지 마세요.\n",
                  WARMUP_N);
    pkt_count = 0; n_frames = 0; hist_w = 0;
    const uint32_t t0 = millis();
    while (n_frames < WARMUP_N && millis() - t0 < 25000) delay(100);
    build_baseline();
    if (!base_ready) {
        Serial.printf("  프레임이 부족하다 (%lu개). 주변에 WiFi 트래픽이 거의 없다.\n",
                      (unsigned long)n_frames);
        Serial.println("  그래도 변화량(flux) 지표는 동작한다.");
    } else {
        float mu = 0, sd = 0;
        for (int i = 0; i < n_sc; i++) { mu += base_mu[i]; sd += base_sd[i]; }
        Serial.printf("  기준선 완료: 서브캐리어 %d개, 평균진폭 %.1f, 평균표준편차 %.2f\n",
                      n_sc, mu / n_sc, sd / n_sc);
    }

    Serial.println("\n[3] 감지 시작. 보드 앞을 지나가면 dev 와 flux 가 튑니다.");
    Serial.println("    dev = 기준선 대비 z-score 평균, flux = 직전 프레임 대비 변화량");
    Serial.println("    motion 은 누적 카운터 — 나중에 읽어도 지나간 흔적이 남습니다.\n");
}

void loop()
{
    static uint32_t last = 0, last_pkt = 0;
    if (millis() - last < 500) { delay(20); return; }
    last = millis();

    const uint32_t p = pkt_count;
    const float rate = (float)(p - last_pkt) * 2.0f;   // 0.5초 간격 → Hz
    last_pkt = p;

    Serial.printf("sc %2d  %5.1f Hz  dev %6.2f  flux %6.2f  motion %lu%s\n",
                  n_sc, rate, last_dev, last_flux,
                  (unsigned long)motion_events,
                  (last_dev > 3.0f) ? "   <<< 움직임" : "");
}
