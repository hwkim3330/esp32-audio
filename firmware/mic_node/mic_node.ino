// 마이크 노드 — 스피커가 없는 보드를 센서 + 컨트롤러로 쓴다.
//
// 이 보드에는 스피커가 붙어 있지 않다. 그래서 소리를 내는 쪽을 밖으로 뺐다:
// 보드는 마이크 2개로 감지만 하고, 소리와 화면은 태블릿이 맡는다.
// 스피커가 없다는 게 제약이 아니라 역할 분담이 된다.
//
// 마이크 2개에서 뽑는 것 (PC 에서 합성 신호로 검증한 값):
//   방향  부화소 TDOA, ±4 샘플 범위에서 최대오차 0.011 샘플
//   세기  dBFS, 좌우 레벨차
//   타격  온셋 감지 (손뼉 2회 → 정확히 2회)
//
// I2S 를 수신 전용으로 연다 — 출력이 없으니 TX 를 켤 이유가 없고,
// DMA 와 핀이 줄어 문제 지점도 줄어든다.
//
// 부팅하면 먼저 마이크 생존 확인을 1초 돌린다. GPIO35(I2S DIN)도 아직 가정이라
// 마이크가 안 들어오면 나머지가 전부 무의미하다 — 그걸 제일 먼저 알려준다.

#include <AC101.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>

#include "board_pins.h"
#include "mn_sense.h"

static const i2s_port_t I2S_PORT = I2S_NUM_0;
static const int N_KEYS = 6;

static const int KEY_PIN[N_KEYS] = {
    CN_PIN_KEY1, CN_PIN_KEY2, CN_PIN_KEY3, CN_PIN_KEY4, CN_PIN_KEY5, CN_PIN_KEY6
};
static bool key_ok[N_KEYS] = { false };
static bool key_down[N_KEYS] = { false };

// AC101 레지스터 주소 (라이브러리가 헤더로 노출하지 않아 여기 둔다).
// 출처: 라이브러리 AC101.cpp 의 #define 들.
#define AC101_MOD_CLK_ENA      0x04
#define AC101_MOD_RST_CTRL     0x05
#define AC101_ADC_DIG_CTRL     0x40
#define AC101_ADC_APC_CTRL     0x50
#define AC101_ADC_SRC          0x51
#define AC101_ADC_SRCBST_CTRL  0x52

// 라이브러리가 ReadReg/WriteReg 를 protected 로 두었다. 레지스터를 직접 만져야
// 하므로(MODE_ADC 가 미완성이다) 서브클래스로 노출한다.
class AC101Ex : public AC101 {
public:
    uint16_t rd(uint8_t r)             { return ReadReg(r); }
    bool     wr(uint8_t r, uint16_t v) { return WriteReg(r, v); }
};

static AC101Ex    codec;

static void dump_adc_regs(const char *tag)
{
    Serial.printf("[reg] %s  DIG(0x40)=0x%04X APC(0x50)=0x%04X "
                  "SRC(0x51)=0x%04X BST(0x52)=0x%04X CLK(0x04)=0x%04X\n",
                  tag,
                  codec.rd(AC101_ADC_DIG_CTRL),
                  codec.rd(AC101_ADC_APC_CTRL),
                  codec.rd(AC101_ADC_SRC),
                  codec.rd(AC101_ADC_SRCBST_CTRL),
                  codec.rd(AC101_MOD_CLK_ENA));
}

// 0.25초간 최대 진폭을 잰다. 스캔 판정에 쓴다.
static float peek_peak(int16_t *b);
static mn_state_t sense;
static int16_t   *blk;          // DMA 가능 메모리여야 한다 (IRAM 16비트 접근 금지)

// ───────────────────────────────────────── 마이크 생존 확인
//
// 채널별로 따로 본다. 한쪽만 죽은 경우가 가장 헷갈리는 고장이다.
static void mic_liveness()
{
    Serial.println("[mic] 생존 확인 1초...");
    float pk_l = 0, pk_r = 0, sum_l = 0, sum_r = 0;
    uint32_t n_tot = 0;
    const uint32_t t_end = millis() + 1000;
    while (millis() < t_end) {
        size_t got = 0;
        if (i2s_read(I2S_PORT, blk, (size_t)MN_BLOCK * 2 * sizeof(int16_t),
                     &got, pdMS_TO_TICKS(100)) != ESP_OK) continue;
        const int n = (int)(got / (2 * sizeof(int16_t)));
        for (int i = 0; i < n; i++) {
            const float a = fabsf((float)blk[2 * i]);
            const float b = fabsf((float)blk[2 * i + 1]);
            if (a > pk_l) pk_l = a;
            if (b > pk_r) pk_r = b;
            sum_l += a; sum_r += b;
        }
        n_tot += n;
    }
    if (!n_tot) {
        Serial.println("[mic] I2S 에서 아무 데이터도 안 온다 — "
                       "board_pins.h 의 I2S 핀(BCLK/LRCK/DIN)을 확인할 것.");
        return;
    }
    Serial.printf("[mic] %lu 샘플  L: 평균 %.0f 피크 %.0f   R: 평균 %.0f 피크 %.0f\n",
                  (unsigned long)n_tot, sum_l / n_tot, pk_l, sum_r / n_tot, pk_r);
    const bool l_ok = pk_l > 30.0f, r_ok = pk_r > 30.0f;
    Serial.printf("[mic] 판정 L=%s R=%s%s\n",
                  l_ok ? "살아있음" : "무신호", r_ok ? "살아있음" : "무신호",
                  (l_ok && r_ok) ? "" : "  ← 무신호면 마이크 배선/DIN 핀 확인");
}

// ───────────────────────────────────────── 키
static void keys_setup()
{
    for (int k = 0; k < N_KEYS; k++)
        pinMode(KEY_PIN[k], (KEY_PIN[k] == 36 || KEY_PIN[k] == 5)
                            ? INPUT : INPUT_PULLUP);
    Serial.print("[keys] 부팅 스캔: ");
    for (int k = 0; k < N_KEYS; k++) {
        const int v = digitalRead(KEY_PIN[k]);
        key_ok[k] = (v == HIGH);   // 안 눌렀으면 HIGH 여야 한다
        Serial.printf("K%d(GPIO%d)=%d%s ", k + 1, KEY_PIN[k], v,
                      key_ok[k] ? "" : "[비활성]");
    }
    Serial.println();
}

// 키 변화를 이벤트로 뿌린다 — 태블릿이 그걸 받아 음을 낸다.
static void keys_poll()
{
    for (int k = 0; k < N_KEYS; k++) {
        if (!key_ok[k]) continue;
        const bool d = (digitalRead(KEY_PIN[k]) == LOW);
        if (d != key_down[k]) {
            key_down[k] = d;
            Serial.printf("KEY %d %s\n", k + 1, d ? "down" : "up");
        }
    }
}

static float peek_peak(int16_t *b)
{
    float pk = 0.0f;
    const uint32_t t_end = millis() + 250;
    while (millis() < t_end) {
        size_t got = 0;
        if (i2s_read(I2S_PORT, b, (size_t)MN_BLOCK * 2 * sizeof(int16_t),
                     &got, pdMS_TO_TICKS(60)) != ESP_OK) continue;
        const int n = (int)(got / (2 * sizeof(int16_t)));
        for (int i = 0; i < 2 * n; i++) {
            const float a = fabsf((float)b[i]);
            if (a > pk) pk = a;
        }
    }
    return pk;
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== 마이크 노드 ===");
    Serial.println("보드는 감지만 한다. 소리와 화면은 태블릿이 맡는다.");

    Wire.begin(CN_PIN_I2C_SDA, CN_PIN_I2C_SCL, CN_I2C_FREQ_HZ);
    Wire.beginTransmission(CN_AC101_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[probe] AC101 응답 없음 — board_pins.h 확인. 중단.");
        while (1) delay(1000);
    }
    Serial.println("[probe] AC101 응답 있음");

    keys_setup();

    if (!codec.begin(CN_PIN_I2C_SDA, CN_PIN_I2C_SCL, CN_I2C_FREQ_HZ)) {
        Serial.println("AC101 초기화 실패. 중단."); while (1) delay(1000);
    }
    codec.SetI2sSampleRate(AC101::SAMPLE_RATE_32000);
    codec.SetI2sWordSize(AC101::WORD_SIZE_16_BITS);
    codec.SetI2sMode(AC101::MODE_SLAVE);
    codec.SetI2sFormat(AC101::DATA_FORMAT_I2S);
    // MODE_ADC 만으로는 전원/경로가 다 안 올라올 수 있어 ADC_DAC 로 켠다.
    codec.SetMode(AC101::MODE_ADC_DAC);

    // ★ 라이브러리 MODE_ADC 는 미완성이다.
    //   SetMode() 를 보면 ADC_SRC / ADC_DIG_CTRL / ADC_APC_CTRL 을 MODE_LINE 에서만
    //   쓰고, MODE_ADC 는 MOD_CLK_ENA/MOD_RST_CTRL 만 건드린다. 즉 ADC 클럭은 켜지만
    //   디지털·아날로그 경로를 열지 않아 I2S 로 0 만 나온다 (실측: 32256 샘플 전부 0).
    //   begin() 이 이미 ADC_SRC=0x2020(마이크)와 ADC_SRCBST_CTRL=0xccc4(부스트)를
    //   써 두었으므로, 빠진 것은 경로 활성화 두 개다.
    codec.wr(AC101_ADC_DIG_CTRL, 0x8000);   // ADC 디지털 인에이블
    codec.wr(AC101_ADC_APC_CTRL, 0x3bc0);   // 마이크 PGA / 바이어스 / 아날로그 경로
    Serial.println("AC101 초기화 완료 (ADC 전용 + 경로 활성화 보정)");
    dump_adc_regs("보정 후");

    // I2S 수신 전용
    i2s_config_t cfg = {};
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    cfg.sample_rate = MN_SR;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;   // 마이크 2개
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count = 8;
    cfg.dma_buf_len = MN_BLOCK;
    // APLL 로 MCLK 를 만들면 코덱이 락하지 못하는 보드가 있다. false 로 시험.
    cfg.use_apll = false;
    if (i2s_driver_install(I2S_PORT, &cfg, 0, nullptr) != ESP_OK) {
        Serial.println("I2S 실패. 중단."); while (1) delay(1000);
    }
    i2s_pin_config_t pins = {};
    pins.mck_io_num   = CN_PIN_I2S_MCLK;
    pins.bck_io_num   = CN_PIN_I2S_BCLK;
    pins.ws_io_num    = CN_PIN_I2S_LRCK;
    pins.data_out_num = I2S_PIN_NO_CHANGE;   // 출력 없음
    pins.data_in_num  = CN_PIN_I2S_DIN;
    i2s_set_pin(I2S_PORT, &pins);

    // MALLOC_CAP_INTERNAL 은 IRAM 을 줄 수 있고 IRAM 은 16비트 접근이 불법이다
    // (신스 노드에서 LoadStoreError 로 확인). I2S 버퍼는 DMA 가능 메모리로.
    blk = (int16_t *)heap_caps_malloc(
        (size_t)MN_BLOCK * 2 * sizeof(int16_t), MALLOC_CAP_DMA);
    if (!blk) { Serial.println("버퍼 실패. 중단."); while (1) delay(1000); }

    mn_init(&sense);
    mic_liveness();

    // ── 채널별 정밀 측정. L 만 살아있는 이유를 좁힌다.
    //   ADC_APC_CTRL(0x50) 이 좌우 PGA/ADC 인에이블을 담는다. 후보를 순회하며
    //   R 채널이 깨어나는 값을 찾는다. 데이터시트 비트맵 추측보다 실측이 확실하다.
    {
        Serial.println("\n[ch] ADC_APC_CTRL 순회 — R 채널을 깨우는 값을 찾는다");
        static const uint16_t apc[] = {
            0x3bc0, 0xbbc0, 0xfbc0, 0x7bc0, 0x3fc0, 0xffc0, 0x3bff, 0xf7c0, 0xcfc0
        };
        uint16_t best = 0; float best_min = 0.0f;
        for (unsigned i = 0; i < sizeof(apc) / sizeof(*apc); i++) {
            codec.wr(AC101_ADC_APC_CTRL, apc[i]);
            delay(80);
            float pl = 0, pr = 0;
            const uint32_t te = millis() + 400;
            while (millis() < te) {
                size_t got = 0;
                if (i2s_read(I2S_PORT, blk, (size_t)MN_BLOCK * 2 * sizeof(int16_t),
                             &got, pdMS_TO_TICKS(60)) != ESP_OK) continue;
                const int n = (int)(got / (2 * sizeof(int16_t)));
                for (int k = 0; k < n; k++) {
                    const float a = fabsf((float)blk[2 * k]);
                    const float b = fabsf((float)blk[2 * k + 1]);
                    if (a > pl) pl = a;
                    if (b > pr) pr = b;
                }
            }
            const float mn = pl < pr ? pl : pr;
            Serial.printf("  APC=0x%04X → L %6.0f  R %6.0f%s\n",
                          apc[i], pl, pr, (mn > 20.0f) ? "  ← 양쪽 살아있음" : "");
            if (mn > best_min) { best_min = mn; best = apc[i]; }
        }
        if (best_min > 20.0f) {
            codec.wr(AC101_ADC_APC_CTRL, best);
            Serial.printf("[ch] 채택 APC=0x%04X (약한쪽 %.0f)\n", best, best_min);
        } else {
            codec.wr(AC101_ADC_APC_CTRL, 0x3bc0);
            Serial.println("[ch] 어느 값에서도 R 이 깨어나지 않는다 — "
                           "마이크 1개만 실장됐을 가능성.");
        }
    }

    dump_adc_regs("최종");

    Serial.println("\n텔레메트리 시작 (10Hz). 손뼉을 좌/우로 옮겨보세요.");
    Serial.println("lag 은 샘플 단위 지연입니다 — +는 왼쪽, −는 오른쪽에서 온 소리.");
    Serial.println("coh 가 0.3 미만이면 방향을 믿을 수 없습니다(잡음).");
}

void loop()
{
    static uint32_t last_print = 0;
    static int onsets = 0;

    size_t got = 0;
    if (i2s_read(I2S_PORT, blk, (size_t)MN_BLOCK * 2 * sizeof(int16_t),
                 &got, pdMS_TO_TICKS(50)) == ESP_OK) {
        const int n = (int)(got / (2 * sizeof(int16_t)));
        mn_process(&sense, blk, n);
        if (sense.onset) {
            onsets++;
            // 온셋은 즉시 뿌린다 — 리듬 트리거라 10Hz 를 기다리면 늦다.
            Serial.printf("ONSET  강도 %.1f  lag %+.2f  coh %.2f\n",
                          sense.onset_strength, sense.lag, sense.coh);
        }
    }

    keys_poll();

    if (millis() - last_print >= 100) {
        last_print = millis();
        Serial.printf("L %6.1fdB  R %6.1fdB  ILD %+5.2f  lag %+6.2f  coh %.2f  "
                      "DC %+.3f/%+.3f  onsets %d\n",
                      sense.db_l, sense.db_r, sense.ild_db,
                      sense.lag, sense.coh, sense.dc_l, sense.dc_r, onsets);
    }
}
