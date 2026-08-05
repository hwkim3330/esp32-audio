// 캐빈 노드 — 번역 무전기 / 음성 명령 노드
//
// 보드 하나로 완결된다: PC 없음, 클라우드 없음, SD 없음.
//   눌러 말한다 → 온디바이스 인코더가 한국어 문장을 알아맞힌다
//   → 플래시의 대상 언어 음성(ADPCM)을 꺼내 컬러링해서 스피커로 낸다
// 태블릿을 붙이면 문장을 늘리고 화면을 얻는다. 안 붙여도 4개 언어가 동작한다.
//
// 자원을 어디에 쓰는지:
//   코어 0    I2S 스테레오 캡처 + 링버퍼 + VAD (상시)
//   코어 1    추론 + 매칭 + 재생 (발화 종료 시)
//   PSRAM     스테레오 링 640KB + 추론 스크래치 964KB + 구문 캐시
//   플래시    인코더 268KB + 프로토타입 46KB + 구문 음성 약 2.2MB
//   마이크 2  방향 추정(좌/중/우) 과 채널 선택
//   터치키 6  PTT · 언어 전환 · 볼륨 · 뮤트
//
// ★ board_pins.h 의 핀 번호는 실크스크린으로 확인되지 않은 가정이다.
//   setup() 이 AC101 을 I2C 로 찔러보고, 응답이 없으면 그 자리에서 멈춘다.

#include <AC101.h>
#include <WiFi.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>

#include "board_pins.h"
#include "cn_audio.h"
#include "cn_infer.h"
#include "model_data.h"
#include "model_weights.h"
#include "phrasepack.h"
#include "prototypes.h"
#include "selftest.h"

// ── 설정
static const i2s_port_t I2S_PORT = I2S_NUM_0;
static const int  DMA_FRAMES     = 256;
static const int  MAX_UTTER_SEC  = 3;      // 인식에 쓸 최대 발화 길이
static const uint16_t TABLET_PORT = 8770;  // 태블릿이 여는 TCP 포트

// ── 상태
static AC101       codec;
static cn_ctx_t    infer;
static cn_ring_t   ring;
static cn_vad_t    vad;
static uint8_t     cur_lang   = 0;         // cn_pp_langs[] 인덱스
static bool        muted      = false;
static uint8_t     spk_volume = 42;        // AC101 스케일 0..63
static WiFiClient  tablet;

// PTT: 눌린 동안의 절대 샘플 구간을 기억한다.
static volatile bool     ptt_down   = false;
static volatile uint32_t ptt_start  = 0;
static volatile uint32_t ptt_end    = 0;
static volatile bool     ptt_ready  = false;

// 큰 버퍼는 전부 PSRAM 이다. 내부 DRAM 은 약 320KB 뿐이라 여기 두면 링크가 깨진다
// (실측: 339KB 초과). 이게 PSRAM 을 쓰는 실제 이유다.
static float   *mel_buf;    // CN_N_FRAMES*CN_N_MELS
static int16_t *utter;      // MAX_UTTER_SEC*CN_A_SR
static int16_t *ch_l, *ch_r;   // 발화 구간 좌/우 (방향 추정용)
static int16_t *dec_pcm;    // ADPCM 디코딩 1초 조각
static int16_t *dec_col;    // 컬러링 출력
static int16_t *dec_out;    // I2S 스테레오 인터리브

// ───────────────────────────────────────── 보드 프로브
//
// 핀 가정이 맞는지 런타임에 증명한다. 조용히 오작동하는 것보다 여기서 멈추는 게 낫다.
static bool probe_codec()
{
    Wire.begin(CN_PIN_I2C_SDA, CN_PIN_I2C_SCL, CN_I2C_FREQ_HZ);
    Wire.beginTransmission(CN_AC101_ADDR);
    const uint8_t err = Wire.endTransmission();
    Serial.printf("[probe] I2C SDA=%d SCL=%d addr=0x%02X → %s\n",
                  CN_PIN_I2C_SDA, CN_PIN_I2C_SCL, CN_AC101_ADDR,
                  err == 0 ? "응답 있음" : "응답 없음");
    if (err != 0) {
        Serial.println("[probe] AC101 이 응답하지 않는다. 확인할 것:");
        Serial.println("        1) board_pins.h 의 CN_PIN_I2C_SDA / SCL");
        Serial.println("        2) 보드 리비전 (v2.2 / A247 / B 계열이 다르다)");
        Serial.println("        3) 가운데 DIP 스위치 위치");
        Serial.println("        진행하지 않는다 — 잘못된 핀으로 I2S 를 켜면 위험하다.");
        return false;
    }
    return true;
}

// ───────────────────────────────────────── I2S 전이중
static bool init_i2s()
{
    i2s_config_t cfg = {};
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
    cfg.sample_rate = CN_A_SR;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;   // AC101 최대 24비트 → 16비트로 통일
    cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;   // 스테레오 — 마이크 2개를 살린다
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count = 8;
    cfg.dma_buf_len = DMA_FRAMES;
    cfg.use_apll = true;                                // 오디오용 정확한 클럭
    cfg.tx_desc_auto_clear = true;

    if (i2s_driver_install(I2S_PORT, &cfg, 0, nullptr) != ESP_OK) return false;

    i2s_pin_config_t pins = {};
    pins.mck_io_num   = CN_PIN_I2S_MCLK;
    pins.bck_io_num   = CN_PIN_I2S_BCLK;
    pins.ws_io_num    = CN_PIN_I2S_LRCK;
    pins.data_out_num = CN_PIN_I2S_DOUT;
    pins.data_in_num  = CN_PIN_I2S_DIN;
    return i2s_set_pin(I2S_PORT, &pins) == ESP_OK;
}

// ───────────────────────────────────────── 터치키
static void keys_init()
{
    // GPIO36 은 입력 전용이고 GPIO5 는 부팅 스트래핑 핀이다 — 둘 다 풀업을 걸지 않는다.
    pinMode(CN_PIN_KEY1, INPUT_PULLUP);
    pinMode(CN_PIN_KEY2, INPUT_PULLUP);
    pinMode(CN_PIN_KEY3, INPUT_PULLUP);
    pinMode(CN_PIN_KEY4, INPUT);
    pinMode(CN_PIN_KEY5, INPUT);
    pinMode(CN_PIN_KEY6, INPUT_PULLUP);
}

static void keys_poll()
{
    static uint32_t last = 0;
    if (millis() - last < 30) return;      // 디바운스
    last = millis();

    // KEY1 = PTT. 누르는 순간의 링 위치를 잡아 두면 앞말이 안 잘린다.
    const bool down = (digitalRead(CN_PIN_KEY1) == LOW);
    if (down && !ptt_down) {
        // 200ms 프리롤 — 버튼보다 말이 먼저 시작되는 경우를 흡수한다.
        const uint32_t pre = CN_A_SR / 5;
        ptt_start = (ring.w > pre) ? (ring.w - pre) : 0;
        ptt_down = true;
        Serial.println("[ptt] 눌림");
    } else if (!down && ptt_down) {
        ptt_end = ring.w;
        ptt_down = false;
        ptt_ready = true;
        Serial.printf("[ptt] 놓임 (%.2fs)\n",
                      (float)(ptt_end - ptt_start) / CN_A_SR);
    }

    static bool k2 = false, k6 = false;
    const bool d2 = (digitalRead(CN_PIN_KEY2) == LOW);
    if (d2 && !k2) {
        cur_lang = (uint8_t)((cur_lang + 1) % CN_PP_N_LANG);
        Serial.printf("[lang] → %s\n", cn_pp_langs[cur_lang]);
    }
    k2 = d2;

    const bool d6 = (digitalRead(CN_PIN_KEY6) == LOW);
    if (d6 && !k6) {
        muted = !muted;
        Serial.printf("[mute] %s\n", muted ? "on" : "off");
    }
    k6 = d6;
}

// ───────────────────────────────────────── 재생
static void play_entry(const cn_pp_entry_t *e, float pitch_semi, float tilt_db)
{
    if (muted) return;
    const int n = (int)e->frames16 * 16;
    cn_adpcm_t st;
    cn_adpcm_reset(&st);

    cn_color_t color;
    cn_color_init(&color, pitch_semi, tilt_db, 0.0f);

    digitalWrite(CN_PIN_PA_ENABLE, HIGH);
    int done = 0;
    while (done < n) {
        const int chunk = (n - done > CN_A_SR) ? CN_A_SR : (n - done);
        cn_adpcm_decode(&st, cn_pp_audio + e->off + (done >> 1), chunk, dec_pcm);
        const int m = cn_color_apply(&color, dec_pcm, chunk, dec_col, CN_A_SR * 2);
        // 모노 → 스테레오 16비트
        for (int i = 0; i < m; i++) {
            dec_out[2 * i] = dec_col[i];
            dec_out[2 * i + 1] = dec_col[i];
        }
        size_t wrote = 0;
        i2s_write(I2S_PORT, dec_out, (size_t)m * 2 * sizeof(int16_t), &wrote,
                  portMAX_DELAY);
        done += chunk;
    }
    digitalWrite(CN_PIN_PA_ENABLE, LOW);
}

static const cn_pp_entry_t *find_entry(uint8_t phrase, uint8_t lang)
{
    for (int i = 0; i < CN_PP_N_FLASH; i++)
        if (cn_pp_index[i].phrase == phrase && cn_pp_index[i].lang == lang)
            return &cn_pp_index[i];
    return nullptr;    // PSRAM 캐시(태블릿이 보낸 것)를 볼 자리
}

// ───────────────────────────────────────── 태스크
static void audio_task(void *)
{
    static int16_t dma[DMA_FRAMES * 2];
    static int16_t frame[CN_A_FRAME];
    for (;;) {
        size_t got = 0;
        if (i2s_read(I2S_PORT, dma, sizeof(dma), &got, portMAX_DELAY) != ESP_OK)
            continue;
        const int frames = (int)(got / (2 * sizeof(int16_t)));
        cn_ring_push(&ring, dma, frames);

        // VAD 는 참고용이다 — 최종 구간은 PTT 가 정한다.
        // (상시 인식은 OOD 오수락이 커서 PTT 를 1차 게이트로 쓴다.)
        for (int o = 0; o + CN_A_FRAME <= frames; o += CN_A_FRAME) {
            cn_ring_read_mono(&ring, ring.w - frames + o, CN_A_FRAME, 2, frame);
            cn_vad_push(&vad, frame, CN_A_FRAME);
        }
    }
}

static void brain_task(void *)
{
    for (;;) {
        if (!ptt_ready) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        ptt_ready = false;

        uint32_t n = ptt_end - ptt_start;
        const uint32_t cap = (uint32_t)MAX_UTTER_SEC * CN_A_SR;
        if (n < CN_A_SR / 4) { Serial.println("[brain] 너무 짧다"); continue; }
        if (n > cap) n = cap;

        // 마이크 2개 → 방향. 누가 말했는지 태블릿에 같이 보낸다.
        cn_ring_read_mono(&ring, ptt_start, (int)n, 0, ch_l);
        cn_ring_read_mono(&ring, ptt_start, (int)n, 1, ch_r);
        float doa_conf = 0.0f;
        const int dir = cn_doa(ch_l, ch_r, (int)n, &doa_conf);

        for (uint32_t i = 0; i < n; i++)
            utter[i] = (int16_t)(((int)ch_l[i] + (int)ch_r[i]) / 2);

        const uint32_t t0 = millis();
        cn_logmel(&infer, utter, (int)n, mel_buf);
        const uint32_t t1 = millis();
        float emb[CN_EMB_DIM];
        cn_encode(&infer, mel_buf, emb);
        const uint32_t t2 = millis();

        float score = 0.0f;
        const int row = cn_match(emb, cn_protos, CN_N_PROTO, &score);

        Serial.printf("[brain] mel %lums  추론 %lums  → \"%s\" (%.3f) dir=%+d(%.2f)\n",
                      (unsigned long)(t1 - t0), (unsigned long)(t2 - t1),
                      row >= 0 ? cn_proto_text[row] : "?", score, dir, doa_conf);

        if (score < CN_REJECT_THR) {
            Serial.println("[brain] 임계값 미달 — 모르는 말로 처리");
            continue;
        }

        const uint8_t intent = cn_proto_intent[row];
        // 인텐트 인덱스는 커맨드 사전 기준이다. 구문 팩과 이름으로 맞춘다.
        int phrase = -1;
        for (int i = 0; i < CN_PP_N_PHRASE; i++)
            if (!strcmp(cn_pp_phrase_ids[i], cn_intent_ids[intent])) { phrase = i; break; }

        if (phrase < 0) {
            Serial.println("[brain] 구문 팩에 없는 인텐트");
        } else {
            const cn_pp_entry_t *e = find_entry((uint8_t)phrase, cur_lang);
            if (e) play_entry(e, 0.0f, 0.0f);
            else Serial.println("[brain] 해당 언어 음성이 플래시에 없다");
        }

        if (tablet.connected())
            tablet.printf("{\"row\":%d,\"score\":%.3f,\"dir\":%d,\"lang\":\"%s\"}\n",
                          row, score, dir, cn_pp_langs[cur_lang]);
    }
}

// ───────────────────────────────────────── setup / loop
void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== 캐빈 노드 ===");

    if (!psramFound()) {
        Serial.println("PSRAM 이 없다. 이 펌웨어는 PSRAM 을 전제로 한다. 중단.");
        while (true) delay(1000);
    }
    Serial.printf("PSRAM %u KB 사용 가능\n",
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

    if (!probe_codec()) { while (true) delay(1000); }

    pinMode(CN_PIN_PA_ENABLE, OUTPUT);
    digitalWrite(CN_PIN_PA_ENABLE, LOW);
    keys_init();

    if (!codec.begin(CN_PIN_I2C_SDA, CN_PIN_I2C_SCL, CN_I2C_FREQ_HZ)) {
        Serial.println("AC101 초기화 실패. 중단.");
        while (true) delay(1000);
    }
    codec.SetI2sSampleRate(AC101::SAMPLE_RATE_16000);
    codec.SetI2sWordSize(AC101::WORD_SIZE_16_BITS);
    codec.SetI2sMode(AC101::MODE_SLAVE);
    codec.SetI2sFormat(AC101::DATA_FORMAT_I2S);
    codec.SetMode(AC101::MODE_ADC_DAC);          // 전이중: 재생 중에도 듣는다
    codec.SetVolumeSpeaker(spk_volume);
    codec.SetVolumeHeadphone(spk_volume);
    Serial.println("AC101 초기화 완료");

    if (!init_i2s()) { Serial.println("I2S 실패. 중단."); while (true) delay(1000); }

    // ── PSRAM 배치
    void *ring_mem  = heap_caps_malloc(cn_ring_bytes(), MALLOC_CAP_SPIRAM);
    void *scratch   = heap_caps_malloc(cn_scratch_bytes(), MALLOC_CAP_SPIRAM);
    mel_buf = (float *)heap_caps_malloc(
        (size_t)CN_N_FRAMES * CN_N_MELS * sizeof(float), MALLOC_CAP_SPIRAM);
    const size_t U = (size_t)MAX_UTTER_SEC * CN_A_SR * sizeof(int16_t);
    utter   = (int16_t *)heap_caps_malloc(U, MALLOC_CAP_SPIRAM);
    ch_l    = (int16_t *)heap_caps_malloc(U, MALLOC_CAP_SPIRAM);
    ch_r    = (int16_t *)heap_caps_malloc(U, MALLOC_CAP_SPIRAM);
    dec_pcm = (int16_t *)heap_caps_malloc(CN_A_SR * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    dec_col = (int16_t *)heap_caps_malloc(CN_A_SR * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    dec_out = (int16_t *)heap_caps_malloc(CN_A_SR * 4 * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!ring_mem || !scratch || !mel_buf || !utter || !ch_l || !ch_r ||
        !dec_pcm || !dec_col || !dec_out) {
        Serial.println("PSRAM 할당 실패. 중단."); while (true) delay(1000);
    }
    Serial.printf("PSRAM 배치: 링 %uKB, 추론 %uKB, 발화 %uKB, 재생 %uKB, 남음 %uKB\n",
                  (unsigned)(cn_ring_bytes() / 1024),
                  (unsigned)(cn_scratch_bytes() / 1024),
                  (unsigned)(3 * U / 1024),
                  (unsigned)(CN_A_SR * 7 * sizeof(int16_t) / 1024),
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

    cn_ring_init(&ring, ring_mem);
    cn_vad_init(&vad);
    cn_ctx_init(&infer, cn_weights, scratch);

    // ── 자기검증. 여기서 걸러야 나중에 "인식률 문제" 로 위장하지 않는다.
    float err = 0.0f;
    const uint32_t ts = millis();
    const int rc = cn_selftest(&infer, &err);
    Serial.printf("자기검증: %s (최대오차 %.2e, 허용 %.1e, %lums)\n",
                  rc == 0 ? "통과" : "실패", err, CN_SELFTEST_TOL,
                  (unsigned long)(millis() - ts));
    if (rc != 0) {
        Serial.println("추론 경로가 깨졌다. 중단.");
        while (true) delay(1000);
    }

    Serial.printf("모델 %u 파라미터, 프로토타입 %d개, 구문 %d문장 × %d언어\n",
                  (unsigned)(sizeof(cn_weights) / sizeof(float)),
                  CN_N_PROTO, CN_PP_N_PHRASE, CN_PP_N_LANG);
    Serial.printf("언어: ");
    for (int i = 0; i < CN_PP_N_LANG; i++) Serial.printf("%s ", cn_pp_langs[i]);
    Serial.printf("(현재 %s)\n", cn_pp_langs[cur_lang]);

    xTaskCreatePinnedToCore(audio_task, "audio", 4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(brain_task, "brain", 8192, nullptr, 4, nullptr, 1);

    Serial.println("준비 완료. KEY1 을 누른 채 말하세요. KEY2 = 언어 전환.");
}

void loop()
{
    keys_poll();

    // 태블릿 연결 유지 (옵션). 없어도 단독으로 동작한다.
    static uint32_t last_try = 0;
    if (!tablet.connected() && millis() - last_try > 5000) {
        last_try = millis();
        if (WiFi.status() == WL_CONNECTED)
            tablet.connect(WiFi.gatewayIP(), TABLET_PORT);
    }
    delay(5);
}
