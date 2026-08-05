// 음향 폐루프 자기진단 — 사람이 보거나 듣지 않아도 판정한다.
//
// 이 보드에는 출력(코덱 DAC + 앰프)과 입력(마이크)이 같이 있다. 그래서 보드가
// 스스로 음을 내고 자기 마이크로 들으면, 사람의 관찰 없이 다음이 판정된다:
//
//   1. 출력 경로가 사는가        — 마이크가 그 음을 들으면 산다
//   2. GPIO22 가 앰프 인에이블인가 — HIGH/LOW 를 바꿔 레벨 차이를 본다
//   3. R 채널이 정말 죽었나       — 이제 들을 소리가 실제로 있으니 확정된다
//
// 핵심은 동기 검파다. 낸 주파수의 sin/cos 과 곱해 적분하면 노이즈 플로어보다
// 훨씬 아래의 신호도 잡힌다 — 주변이 조용하기를 기다릴 필요가 없다.
// 그냥 RMS 만 보면 "조용해서 안 들림" 과 "경로가 죽음" 이 구분되지 않는다.

#include <AC101.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include <math.h>

#include "board_pins.h"

static const i2s_port_t I2S_PORT = I2S_NUM_0;
static const int   SR     = 32000;
static const int   BLOCK  = 256;
static const float TONE_HZ[3] = { 500.0f, 1000.0f, 2000.0f };

#define AC101_ADC_DIG_CTRL   0x40
#define AC101_ADC_APC_CTRL   0x50
#define AC101_ADC_SRC        0x51

class AC101Ex : public AC101 {
public:
    uint16_t rd(uint8_t r)             { return ReadReg(r); }
    bool     wr(uint8_t r, uint16_t v) { return WriteReg(r, v); }
};

static AC101Ex codec;
static int16_t *tx, *rx;

// 동기 검파 결과 (I/Q 크기). 낸 주파수 성분의 진폭에 비례한다.
typedef struct { float l, r, rms_l, rms_r; } det_t;

// hz 를 amp 로 dur_ms 동안 내면서 동시에 받아, 그 주파수 성분을 검파한다.
static det_t run_tone(float hz, float amp, int dur_ms)
{
    det_t d = { 0, 0, 0, 0 };
    const float w = 2.0f * (float)M_PI * hz / SR;
    double il = 0, ql = 0, ir = 0, qr = 0, el = 0, er = 0;
    uint32_t n_tot = 0;
    uint32_t phase_n = 0;

    i2s_zero_dma_buffer(I2S_PORT);
    const uint32_t t_end = millis() + dur_ms;
    while (millis() < t_end) {
        // 송신 블록 생성
        for (int i = 0; i < BLOCK; i++) {
            const float s = sinf(w * (float)(phase_n + i));
            const int16_t v = (int16_t)(s * amp * 32000.0f);
            tx[2 * i] = v;
            tx[2 * i + 1] = v;
        }
        size_t wrote = 0;
        i2s_write(I2S_PORT, tx, (size_t)BLOCK * 2 * sizeof(int16_t), &wrote,
                  pdMS_TO_TICKS(50));

        size_t got = 0;
        if (i2s_read(I2S_PORT, rx, (size_t)BLOCK * 2 * sizeof(int16_t), &got,
                     pdMS_TO_TICKS(50)) == ESP_OK) {
            const int n = (int)(got / (2 * sizeof(int16_t)));
            for (int i = 0; i < n; i++) {
                // 위상은 절대 시간 기준으로 둔다. 송수신 지연이 있어도 I/Q 크기는
                // 보존되므로(위상만 돌아감) 검파에 문제가 없다.
                const float ph = w * (float)(phase_n + i);
                const float cs = cosf(ph), sn = sinf(ph);
                const float a = (float)rx[2 * i];
                const float b = (float)rx[2 * i + 1];
                il += a * cs; ql += a * sn;
                ir += b * cs; qr += b * sn;
                el += (double)a * a; er += (double)b * b;
            }
            n_tot += n;
        }
        phase_n += BLOCK;
    }
    if (!n_tot) return d;
    const double inv = 1.0 / (double)n_tot;
    d.l = (float)(2.0 * sqrt(il * il + ql * ql) * inv);
    d.r = (float)(2.0 * sqrt(ir * ir + qr * qr) * inv);
    d.rms_l = (float)sqrt(el * inv);
    d.rms_r = (float)sqrt(er * inv);
    return d;
}

static void report(const char *tag, det_t d)
{
    Serial.printf("  %-26s 검파 L %8.2f  R %8.2f   (RMS L %7.1f  R %7.1f)\n",
                  tag, d.l, d.r, d.rms_l, d.rms_r);
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== 음향 폐루프 자기진단 ===");
    Serial.println("보드가 스스로 음을 내고 자기 마이크로 듣는다.");
    Serial.println("동기 검파를 쓰므로 주변이 조용하지 않아도, 소리가 작아도 판정된다.\n");

    Wire.begin(CN_PIN_I2C_SDA, CN_PIN_I2C_SCL, CN_I2C_FREQ_HZ);
    Wire.beginTransmission(CN_AC101_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[probe] AC101 응답 없음. 중단."); while (1) delay(1000);
    }
    Serial.println("[probe] AC101 응답 있음");

    pinMode(CN_PIN_PA_ENABLE, OUTPUT);
    digitalWrite(CN_PIN_PA_ENABLE, LOW);

    if (!codec.begin(CN_PIN_I2C_SDA, CN_PIN_I2C_SCL, CN_I2C_FREQ_HZ)) {
        Serial.println("AC101 초기화 실패. 중단."); while (1) delay(1000);
    }
    codec.SetI2sSampleRate(AC101::SAMPLE_RATE_32000);
    codec.SetI2sWordSize(AC101::WORD_SIZE_16_BITS);
    codec.SetI2sMode(AC101::MODE_SLAVE);
    codec.SetI2sFormat(AC101::DATA_FORMAT_I2S);
    codec.SetMode(AC101::MODE_ADC_DAC);          // 전이중
    // 라이브러리 MODE_ADC 계열이 ADC 경로를 안 열어서 직접 켠다 (mic_node 참조)
    codec.wr(AC101_ADC_DIG_CTRL, 0x8000);
    codec.wr(AC101_ADC_APC_CTRL, 0x3bc0);
    codec.SetVolumeSpeaker(50);
    codec.SetVolumeHeadphone(50);
    Serial.printf("[reg] DIG=0x%04X APC=0x%04X SRC=0x%04X\n",
                  codec.rd(AC101_ADC_DIG_CTRL), codec.rd(AC101_ADC_APC_CTRL),
                  codec.rd(AC101_ADC_SRC));

    i2s_config_t cfg = {};
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
    cfg.sample_rate = SR;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count = 6;
    cfg.dma_buf_len = BLOCK;
    cfg.use_apll = false;
    cfg.tx_desc_auto_clear = true;
    if (i2s_driver_install(I2S_PORT, &cfg, 0, nullptr) != ESP_OK) {
        Serial.println("I2S 실패. 중단."); while (1) delay(1000);
    }
    i2s_pin_config_t pins = {};
    pins.mck_io_num   = CN_PIN_I2S_MCLK;
    pins.bck_io_num   = CN_PIN_I2S_BCLK;
    pins.ws_io_num    = CN_PIN_I2S_LRCK;
    pins.data_out_num = CN_PIN_I2S_DOUT;
    pins.data_in_num  = CN_PIN_I2S_DIN;      // 실측 정정된 GPIO34
    i2s_set_pin(I2S_PORT, &pins);

    // IRAM 은 16비트 접근이 불법이라 DMA 가능 메모리로 잡는다 (synth_node 참조)
    tx = (int16_t *)heap_caps_malloc((size_t)BLOCK * 2 * sizeof(int16_t), MALLOC_CAP_DMA);
    rx = (int16_t *)heap_caps_malloc((size_t)BLOCK * 2 * sizeof(int16_t), MALLOC_CAP_DMA);
    if (!tx || !rx) { Serial.println("버퍼 실패. 중단."); while (1) delay(1000); }

    Serial.printf("I2S: MCLK%d BCLK%d LRCK%d DOUT%d DIN%d @ %dHz\n\n",
                  CN_PIN_I2S_MCLK, CN_PIN_I2S_BCLK, CN_PIN_I2S_LRCK,
                  CN_PIN_I2S_DOUT, CN_PIN_I2S_DIN, SR);

    // ── 1) 기준선: 음을 내지 않고 검파. 이 값이 "없음" 의 기준이다.
    Serial.println("[1] 기준선 (무음)");
    det_t base[3];
    for (int i = 0; i < 3; i++) {
        base[i] = run_tone(TONE_HZ[i], 0.0f, 700);
        char t[48]; snprintf(t, sizeof t, "%.0fHz 무음", TONE_HZ[i]);
        report(t, base[i]);
    }

    // ── 2) GPIO22 LOW 로 음 내기
    Serial.println("\n[2] 음 출력, GPIO22 = LOW");
    digitalWrite(CN_PIN_PA_ENABLE, LOW);
    delay(120);
    det_t lo[3];
    for (int i = 0; i < 3; i++) {
        lo[i] = run_tone(TONE_HZ[i], 0.6f, 700);
        char t[48]; snprintf(t, sizeof t, "%.0fHz PA=LOW", TONE_HZ[i]);
        report(t, lo[i]);
    }

    // ── 3) GPIO22 HIGH 로 음 내기
    Serial.println("\n[3] 음 출력, GPIO22 = HIGH");
    digitalWrite(CN_PIN_PA_ENABLE, HIGH);
    delay(120);
    det_t hi[3];
    for (int i = 0; i < 3; i++) {
        hi[i] = run_tone(TONE_HZ[i], 0.6f, 700);
        char t[48]; snprintf(t, sizeof t, "%.0fHz PA=HIGH", TONE_HZ[i]);
        report(t, hi[i]);
    }

    // ── 판정
    Serial.println("\n=== 판정 ===");
    float b = 0, l = 0, h = 0, br = 0, lr = 0, hr = 0;
    for (int i = 0; i < 3; i++) {
        b += base[i].l; l += lo[i].l; h += hi[i].l;
        br += base[i].r; lr += lo[i].r; hr += hi[i].r;
    }
    b /= 3; l /= 3; h /= 3; br /= 3; lr /= 3; hr /= 3;
    Serial.printf("L 평균 검파: 무음 %.2f  PA=LOW %.2f  PA=HIGH %.2f\n", b, l, h);
    Serial.printf("R 평균 검파: 무음 %.2f  PA=LOW %.2f  PA=HIGH %.2f\n", br, lr, hr);

    const float best_l = (h > l) ? h : l;
    const bool out_ok = best_l > b * 3.0f + 2.0f;
    Serial.printf("\n출력 경로: %s\n", out_ok
        ? "동작 — 마이크가 자기 소리를 들었다. 스피커/이어폰이 소리를 낸다."
        : "무응답 — 스피커·이어폰 미연결이거나 출력 경로가 죽었다.");

    if (out_ok) {
        const float ratio = (l > 0.01f) ? (h / l) : 999.0f;
        Serial.printf("GPIO22: %s (HIGH/LOW 비 %.2f)\n",
            (ratio > 2.0f) ? "앰프 인에이블로 보인다 — HIGH 에서 크게 커진다"
          : (ratio < 0.5f) ? "반대로 동작한다 — LOW 에서 소리가 난다(액티브 로우)"
          : "출력에 영향 없음 — 앰프 인에이블이 아닐 가능성(LED 등)", ratio);

        const float best_r = (hr > lr) ? hr : lr;
        Serial.printf("R 채널: %s\n", (best_r > br * 3.0f + 2.0f)
            ? "동작 — 두 마이크 모두 살아있다. 방향 추정 가능."
            : "무응답 — 소리가 실제로 났는데도 R 이 안 잡힌다. R 마이크는 죽었다고 확정.");
    } else {
        Serial.println("출력이 없으니 R 채널은 이 테스트로 판정할 수 없다.");
    }

    digitalWrite(CN_PIN_PA_ENABLE, LOW);
    Serial.println("\n완료.");
}

void loop()
{
    // ── 무한 감시.
    //
    // 소프트웨어로 할 수 있는 건 다 했다: DIN 핀 8개, ADC_SRC 9개, ADC_APC_CTRL 9개,
    // MODE_ADC/ADC_DAC, APLL on/off — 전부 무신호. 코덱 I2C 는 양방향으로 완벽하다.
    // 이 조합이 가리키는 것은 가운데 DIP 스위치다(공유 GPIO 라우팅). 물리 스위치라
    // 코드로 못 바꾼다.
    //
    // 그래서 계속 테스트하며 기다린다. 사용자가 언제 DIP 를 만져도 그 순간이 로그에
    // 남는다 — 지켜볼 필요가 없다.
    static uint32_t round_n = 0;
    static bool ever_alive = false;
    round_n++;

    digitalWrite(CN_PIN_PA_ENABLE, HIGH);
    delay(80);
    const det_t d = run_tone(1000.0f, 0.6f, 600);
    digitalWrite(CN_PIN_PA_ENABLE, LOW);

    const bool alive = (d.l > 3.0f) || (d.r > 3.0f) ||
                       (d.rms_l > 20.0f) || (d.rms_r > 20.0f);
    if (alive && !ever_alive) {
        ever_alive = true;
        Serial.println("\n***********************************************");
        Serial.println("*** 오디오가 살아났다! DIP 스위치가 원인이었다 ***");
        Serial.println("***********************************************");
    }
    Serial.printf("[%4lu] 검파 L %7.2f R %7.2f  RMS L %6.1f R %6.1f  %s\n",
                  (unsigned long)round_n, d.l, d.r, d.rms_l, d.rms_r,
                  alive ? "◀◀ 신호 있음" : "무신호 (DIP 스위치를 딸깍거려 보세요)");
    delay(400);
}
