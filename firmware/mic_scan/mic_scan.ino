// 마이크 입력 소스를 쓸어서 "무엇이 실제로 배선돼 있는가" 를 판정한다.
//
// 왜: 4핀(TRRS) 이어폰의 마이크를 쓸 수 있는지 알고 싶다. 이 보드는 출력 잭
// (EARPHONE)과 입력 잭(LINEIN)이 따로 있어서, 출력 잭의 4번째 링(마이크)이 코덱의
// HPMIC 핀에 붙어 있는지는 보드 리비전에 달렸다. 추측 대신 재서 답한다.
//
// 방법 두 가지를 같이 쓴다:
//   1. ADC_SRC(0x51) 를 **비트별로** 켜 보며 각 소스의 RMS 를 잰다.
//      온보드 마이크 2개는 반드시 신호를 낸다(2026-09-08 실측 RMS L 7800 / R 3300).
//      그 두 비트 말고 신호가 나오는 비트가 있으면 그게 다른 입력(LINEIN/HPMIC)이다.
//      전부 0 이면 그 소스는 배선이 안 된 것이다.
//   2. **동기 검파** — 스피커로 1kHz 를 내면서 동시에 듣는다. 헤드셋 마이크는 이어폰
//      하우징 **안**에 있으므로, 배선돼 있다면 온보드 마이크보다 압도적으로 크게 잡힌다.
//      이 대비가 "배선됨/안 됨" 을 가른다.
//   3. AC101 의 헤드셋 마이크 검출 레지스터(HMIC_CTRL1/2, HMIC_STATUS)도 켜서 읽는다.
//
// 전제: 이어폰을 **EARPHONE 잭**에 꽂고 돌려라. 조용한 곳일 필요는 없다(동기 검파).
//
// 핀맵 주의: LRCK 26 / DOUT 25 다. 이게 25/26 으로 뒤바뀐 채로 8월 내내
// "오디오가 죽었다" 는 결론이 나왔다.

#include <Arduino.h>
#include <Wire.h>
#include <AC101.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include <math.h>

static const int SR = 44100;                 // AC101 PLL 이 256×44.1kHz 를 전제한다
static const i2s_port_t PORT = I2S_NUM_0;
static const int BLOCK = 256;

static const int PIN_MCLK = 0, PIN_BCLK = 27, PIN_LRCK = 26, PIN_DOUT = 25, PIN_DIN = 35;
static const int PIN_SDA = 33, PIN_SCL = 32;
static const int PIN_PA = 21;                // 파워앰프 (스피커용)

static const uint8_t R_ADC_DIG   = 0x40, R_ADC_VOL  = 0x41;
static const uint8_t R_HMIC_CTL1 = 0x44, R_HMIC_CTL2 = 0x45, R_HMIC_STAT = 0x46;
static const uint8_t R_ADC_APC   = 0x50, R_ADC_SRC  = 0x51, R_ADC_BST = 0x52;

class AC101Ex : public AC101 {
public:
    uint16_t rd(uint8_t r) { return ReadReg(r); }
    bool wr(uint8_t r, uint16_t v) { return WriteReg(r, v); }
};

static AC101Ex codec;
static int16_t *tx = nullptr, *rx = nullptr;
static float g_phase = 0.0f;

typedef struct { double det_l, det_r, rms_l, rms_r; } meas_t;

// hz 를 amp 로 dur_ms 동안 내면서 동시에 받아, 그 주파수 성분을 동기 검파한다.
// 그냥 RMS 만 보면 "조용해서 안 들림" 과 "경로가 죽음" 이 구분되지 않는다.
static meas_t measure(float hz, float amp, int dur_ms)
{
    meas_t m = {0, 0, 0, 0};
    const float w = 2.0f * (float)M_PI * hz / SR;
    double il = 0, ql = 0, ir = 0, qr = 0, el = 0, er = 0;
    uint32_t n = 0, ph = 0;

    i2s_zero_dma_buffer(PORT);
    const uint32_t t_end = millis() + dur_ms;
    while (millis() < t_end) {
        for (int i = 0; i < BLOCK; i++) {
            g_phase += hz / SR;
            if (g_phase >= 1.0f) g_phase -= 1.0f;
            const int16_t v = (int16_t)(amp * sinf(2.0f * (float)M_PI * g_phase));
            tx[2 * i] = v;
            tx[2 * i + 1] = v;
        }
        size_t wr_b = 0, rd_b = 0;
        i2s_write(PORT, tx, (size_t)BLOCK * 4, &wr_b, portMAX_DELAY);
        i2s_read(PORT, rx, (size_t)BLOCK * 4, &rd_b, portMAX_DELAY);
        const int frames = (int)(rd_b / 4);
        for (int i = 0; i < frames; i++, ph++) {
            const double c = cos(w * ph), s = sin(w * ph);
            const double l = rx[2 * i], r = rx[2 * i + 1];
            il += l * c; ql += l * s; el += l * l;
            ir += r * c; qr += r * s; er += r * r;
            n++;
        }
    }
    if (!n) return m;
    m.det_l = 2.0 * sqrt(il * il + ql * ql) / n;
    m.det_r = 2.0 * sqrt(ir * ir + qr * qr) / n;
    m.rms_l = sqrt(el / n);
    m.rms_r = sqrt(er / n);
    return m;
}

static void i2s_setup()
{
    i2s_config_t cfg = {};
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);   // 전이중
    cfg.sample_rate = SR;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count = 6;
    cfg.dma_buf_len = BLOCK;
    cfg.use_apll = false;
    cfg.tx_desc_auto_clear = true;
    if (i2s_driver_install(PORT, &cfg, 0, nullptr) != ESP_OK) {
        Serial.println("I2S 실패. 중단."); while (1) delay(1000);
    }
    i2s_pin_config_t pins = {};
    pins.mck_io_num = PIN_MCLK;
    pins.bck_io_num = PIN_BCLK;
    pins.ws_io_num = PIN_LRCK;
    pins.data_out_num = PIN_DOUT;
    pins.data_in_num = PIN_DIN;
    i2s_set_pin(PORT, &pins);

    // IRAM 은 16비트 접근이 불법이다(LoadStoreError). DMA 가능 메모리로 잡는다.
    tx = (int16_t *)heap_caps_malloc((size_t)BLOCK * 4, MALLOC_CAP_DMA);
    rx = (int16_t *)heap_caps_malloc((size_t)BLOCK * 4, MALLOC_CAP_DMA);
    if (!tx || !rx) { Serial.println("버퍼 실패. 중단."); while (1) delay(1000); }
}

void setup()
{
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== 마이크 입력 소스 스캔 ===");
    Serial.println("무엇이 실제로 배선돼 있는지 재서 판정한다. 이어폰은 EARPHONE 잭에.");

    pinMode(PIN_PA, OUTPUT);
    digitalWrite(PIN_PA, LOW);            // 스피커 앰프는 끈다(노이즈만 얹는다)

    if (!codec.begin(PIN_SDA, PIN_SCL)) {
        Serial.println("AC101 begin() 실패 — 중단"); while (1) delay(1000);
    }
    codec.SetMode(AC101::MODE_ADC_DAC);   // 듣고 동시에 내야 한다
    codec.SetVolumeHeadphone(45);
    i2s_setup();

    Serial.printf("I2S: MCLK%d BCLK%d LRCK%d DOUT%d DIN%d @ %dHz\n",
                  PIN_MCLK, PIN_BCLK, PIN_LRCK, PIN_DOUT, PIN_DIN, SR);
    Serial.printf("기본값: ADC_SRC=0x%04X BST=0x%04X APC=0x%04X VOL=0x%04X\n\n",
                  codec.rd(R_ADC_SRC), codec.rd(R_ADC_BST),
                  codec.rd(R_ADC_APC), codec.rd(R_ADC_VOL));

    // ── 헤드셋 마이크 검출을 켜고 상태를 읽는다.
    //    이 레지스터가 삽입을 보고하면 잭의 마이크 링이 코덱까지 배선돼 있다는 뜻이다.
    Serial.println("[HMIC] 헤드셋 마이크 검출 활성화 후 상태");
    Serial.printf("  켜기 전:  CTL1=0x%04X CTL2=0x%04X STAT=0x%04X\n",
                  codec.rd(R_HMIC_CTL1), codec.rd(R_HMIC_CTL2), codec.rd(R_HMIC_STAT));
    codec.wr(R_HMIC_CTL1, 0x0f0f);
    codec.wr(R_HMIC_CTL2, 0x4f00);
    delay(300);
    Serial.printf("  켠 뒤:    CTL1=0x%04X CTL2=0x%04X STAT=0x%04X\n\n",
                  codec.rd(R_HMIC_CTL1), codec.rd(R_HMIC_CTL2), codec.rd(R_HMIC_STAT));

    // ── ADC_SRC 를 비트별로 켜 본다.
    //
    //    이 레지스터는 좌/우 ADC 믹서에 어느 아날로그 소스를 넣을지 고른다. 비트별
    //    의미는 공개 데이터시트가 없어 모르지만, **어느 비트가 신호를 물고 있는지는
    //    측정으로 알 수 있다.** 온보드 마이크에 해당하는 비트는 반드시 신호를 낸다.
    Serial.println("[스캔] ADC_SRC 비트별 — 1kHz 를 내면서 동시에 검파");
    Serial.println("  값       검파 L     검파 R     RMS L     RMS R");

    const uint16_t cands[] = {
        0x2020,   // 라이브러리 기본값 (온보드 마이크로 알려진 설정)
        0x0101, 0x0202, 0x0404, 0x0808, 0x1010, 0x4040,
        0x3f3f,   // 전부 켜기 — 배선된 소스가 있으면 무조건 여기서 잡힌다
        0x0000,   // 전부 끄기 — 바닥값 (여기서도 신호가 나오면 경로가 다른 데 있다)
    };
    meas_t base = {0, 0, 0, 0};
    for (unsigned i = 0; i < sizeof cands / sizeof *cands; i++) {
        codec.wr(R_ADC_SRC, cands[i]);
        delay(120);
        const meas_t m = measure(1000.0f, 9000.0f, 350);
        if (cands[i] == 0x0000) base = m;
        Serial.printf("  0x%04X  %9.2f  %9.2f  %8.1f  %8.1f%s\n",
                      cands[i], m.det_l, m.det_r, m.rms_l, m.rms_r,
                      (m.rms_l > 50 || m.rms_r > 50) ? "   <<< 신호 있음" : "");
    }

    Serial.println("\n[스캔] 마이크 부스트(ADC_SRCBST_CTRL) — 전부 켜기 상태에서");
    codec.wr(R_ADC_SRC, 0x3f3f);
    const uint16_t bsts[] = {0xccc4, 0xffc4, 0xcccc, 0xffff, 0x0000};
    for (unsigned i = 0; i < sizeof bsts / sizeof *bsts; i++) {
        codec.wr(R_ADC_BST, bsts[i]);
        delay(120);
        const meas_t m = measure(1000.0f, 9000.0f, 350);
        Serial.printf("  BST=0x%04X  검파 L %9.2f R %9.2f  RMS L %8.1f R %8.1f\n",
                      bsts[i], m.det_l, m.det_r, m.rms_l, m.rms_r);
    }

    // ── 판정
    Serial.println("\n=== 판정 ===");
    codec.wr(R_ADC_SRC, 0x2020);
    codec.wr(R_ADC_BST, 0xccc4);
    delay(150);
    const meas_t on = measure(1000.0f, 9000.0f, 500);
    const meas_t off = measure(1000.0f, 0.0f, 500);
    Serial.printf("온보드 기본설정(0x2020): 음 출력 검파 L %.2f R %.2f / 무음 L %.2f R %.2f\n",
                  on.det_l, on.det_r, off.det_l, off.det_r);
    Serial.printf("바닥값(ADC_SRC=0): RMS L %.1f R %.1f\n", base.rms_l, base.rms_r);
    Serial.println("");
    Serial.println("읽는 법:");
    Serial.println(" - 0x2020 만 신호가 있고 다른 비트는 바닥이면 → 쓸 수 있는 입력은 온보드 마이크뿐.");
    Serial.println("   4핀 이어폰의 마이크는 이 보드에서 배선되지 않았다는 뜻이다.");
    Serial.println(" - 0x2020 보다 훨씬 큰 검파값을 내는 비트가 있으면 → 그게 헤드셋 마이크다");
    Serial.println("   (이어폰 하우징 안에 있으니 자기 소리를 압도적으로 크게 듣는다).");
    scan2d();
    Serial.println("완료. 이후 무음 상태 소스별 RMS 를 반복해 찍는다.");
}

// ── 2차 스캔: 입력 소스 × 아날로그 경로(바이어스)를 2차원으로 쓴다.
//
// 1차 스캔에 구멍이 두 개 있었다:
//   (1) ADC_SRC 비트 7(0x8080)을 재보지 않았다.
//   (2) ADC_APC_CTRL(0x50)을 전혀 건드리지 않았다. 헤드셋 마이크는 전용 바이어스를
//       받아야 소리를 내는데, 꺼져 있으면 배선돼 있어도 바닥으로 읽힌다.
//
// 8월에 이 레지스터를 9가지 쓸어본 기록이 있지만 그건 LRCK/DOUT 이 뒤바뀐 상태에서
// 나온 것이라 전부 무효다. 다시 쓴다.
//
// 판정 기준은 무음에서의 RMS — 방의 소리를 듣는가. 톤을 내면서 재면 코덱 내부
// 루프백(0x0101/0x0202)에 속는다.
static void scan2d()
{
    static const uint16_t apcs[] = {
        0xbbc3, 0xffc3, 0xbbcf, 0xffcf, 0xbbf3, 0xfff3, 0xffff,
        0xbbe3, 0xbfc3, 0x3bc3, 0x7bc3,
    };
    // 온보드로 확정된 비트(0x4040 좌 / 0x1010 우 / 0x2020 둘)는 빼고 나머지만 본다.
    static const uint16_t srcs[] = {0x8080, 0x0808, 0x0404, 0x0101, 0x0202};

    Serial.println("\n[2차 스캔] 무음 RMS (L/R) — 소스 x 아날로그 경로");
    Serial.print("  APC        ");
    for (unsigned j = 0; j < sizeof srcs / sizeof *srcs; j++) Serial.printf("   %04X     ", srcs[j]);
    Serial.println();

    double best = 0; uint16_t best_apc = 0, best_src = 0;
    for (unsigned i = 0; i < sizeof apcs / sizeof *apcs; i++) {
        codec.wr(R_ADC_APC, apcs[i]);
        Serial.printf("  0x%04X  ", apcs[i]);
        double row_min = 1e9, row_max = 0;
        for (unsigned j = 0; j < sizeof srcs / sizeof *srcs; j++) {
            codec.wr(R_ADC_SRC, srcs[j]);
            delay(90);
            const meas_t m = measure(1000.0f, 0.0f, 260);
            Serial.printf(" %5.0f/%-5.0f", m.rms_l, m.rms_r);
            const double v = m.rms_l > m.rms_r ? m.rms_l : m.rms_r;
            if (v < row_min) row_min = v;
            if (v > row_max) row_max = v;
            // 후보 판정은 아래에서 행 단위로 걸러 넣는다
            if (v > best && v < 25000) { best = v; best_apc = apcs[i]; best_src = srcs[j]; }
        }
        // **포화(레일) 걸러내기.** 실측에서 APC=0x3BC3/0x7BC3 이 소스와 무관하게 R 을
        // 30000 으로 railed 시켰고, 처음 판정 로직이 그걸 "마이크" 로 잡아 오탐했다.
        // 마이크라면 소스를 바꾸면 값이 달라진다 — 행 안에서 변화가 없고 값이 크면
        // 그건 포화다.
        if (row_max > 25000 && row_min > 25000 * 0.8) Serial.print("   <- 포화(레일), 무시");
        Serial.println();
    }
    codec.wr(R_ADC_APC, 0xbbc3);
    codec.wr(R_ADC_SRC, 0x2020);

    Serial.println("\n  == 2차 판정 ==");
    Serial.printf("  온보드 아닌 소스 중 최고: APC=0x%04X SRC=0x%04X -> RMS %.0f\n",
                  best_apc, best_src, best);
    // 온보드 마이크의 무음 RMS 가 500~700 이다. 다른 입력이 배선돼 있다면 같은
    // 자릿수여야 한다. 25000 이상은 포화라 후보에서 이미 뺐다.
    if (best > 150)
        Serial.println("  <- 방의 소리를 듣는 다른 입력이 있다. 이 조합을 쓰면 된다.");
    else
        Serial.println("  <- 전부 바닥이다(온보드 마이크는 500~700). 배선된 입력은 온보드 2개뿐.");
}

// ── 결정적 판정: **무음으로 두고** 각 소스의 RMS 를 잰다.
//
// 톤을 내면서 잰 값만으로는 "마이크가 그 소리를 들었다" 와 "코덱 내부에서 출력이
// 입력으로 되먹임됐다(전기적 루프백)" 를 구분할 수 없다. 둘 다 큰 검파값을 낸다.
//
// 무음으로 두면 갈린다:
//   진짜 마이크   → 방의 소리를 듣는다. RMS 가 바닥보다 확실히 높다.
//   전기적 루프백 → 낼 소리가 없으니 바닥에 붙는다.
void loop()
{
    static const uint16_t srcs[] = {0x2020, 0x0101, 0x0202, 0x1010, 0x4040, 0x0000};
    static uint32_t k = 0;
    Serial.printf("\n[%u] 무음 상태에서 각 소스가 방의 소리를 듣는가\n", ++k);
    for (unsigned i = 0; i < sizeof srcs / sizeof *srcs; i++) {
        codec.wr(R_ADC_SRC, srcs[i]);
        delay(120);
        const meas_t m = measure(1000.0f, 0.0f, 400);   // 무음
        Serial.printf("  0x%04X  RMS L %8.1f  R %8.1f  %s\n", srcs[i], m.rms_l, m.rms_r,
                      (m.rms_l > 100 || m.rms_r > 100) ? "← 방의 소리를 듣는다(진짜 마이크)"
                                                       : "← 바닥 (마이크 아님)");
    }
    delay(500);
}
