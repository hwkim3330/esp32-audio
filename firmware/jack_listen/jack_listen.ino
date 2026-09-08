// 잭에 이어폰을 꽂고 **귀로** 판정한다 — 출력이 죽었는지 아닌지.
//
// 왜 이게 필요한가. 이 보드의 오디오가 "죽었다" 는 판정은 `loopback_test` 의 음향
// 폐루프(보드가 소리 내고 자기 마이크로 듣기)에서 나왔다. 그런데 **그 방법은 입력이
// 죽으면 출력이 살아 있어도 똑같이 실패한다.** 그리고 확정된 것은 입력 쪽이다 —
// DIN(GPIO35)이 구동되는데 내용이 정확히 0, 즉 코덱 ADC 가 무음을 보낸다.
//
// 저장소를 다 훑어도 **이어폰을 꽂고 들어본 기록이 없다**(보드에 스피커가 안 붙어 있다).
// 그래서 출력은 지금까지 미지다. 이 스케치가 그걸 가른다.
//
// 이 스케치는 I2S 를 **출력 전용**으로 연다. 입력을 아예 안 만들므로 죽은 ADC 가
// 판정을 오염시키지 않는다.
//
// 굽고 나서:
//   1. 이어폰(3핀이든 4핀이든)을 잭에 꽂는다. 4핀이어도 스테레오 출력은 그대로 들린다
//      (마이크 링은 슬리브에 붙어 무시된다).
//   2. 들리면 → **출력은 살아 있다.** MP3 를 잭으로도 들을 수 있다.
//   3. 안 들리면 → KEY6 으로 코덱 출력 경로 설정을 순회하며 계속 듣는다.
//   4. 그래도 없으면 → **가운데 DIP 스위치를 하나씩 토글한다.** 신호가 계속 나가고
//      있으므로 살아나는 순간 귀로 안다. 이게 이 스케치의 진짜 목적이다.

#include <Arduino.h>
#include <Wire.h>
#include <AC101.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include <math.h>

static const int SR = 44100;                 // 44100 고정. 바꾸면 AC101 PLL 이 락을 잃는다
static const i2s_port_t PORT = I2S_NUM_0;
static const int BLOCK = 256;                // 프레임

// ★ 정정(2026-09-08): LRCK/DOUT 이 25/26 으로 뒤바뀌어 있었다(arduino-audiokit 기준).
static const int PIN_MCLK = 0, PIN_BCLK = 27, PIN_LRCK = 26, PIN_DOUT = 25;
static const int PIN_SDA = 33, PIN_SCL = 32;

// 실기로 확정된 키 매핑. 한때 두 칸 밀려 있어서 "3번만 반응한다" 로 보였다.
static const int KEY1 = 36, KEY2 = 13, KEY3 = 19, KEY4 = 23, KEY5 = 18, KEY6 = 5;
// 앰프 인에이블 후보. 이 저장소는 22 를 PA_ENABLE(가정)로 적어 두고도 LED 로
// 깜빡여 왔다 — 22 가 정말 앰프 인에이블이면 청음 시험 내내 앰프를 끊는 짓이다.
// 리비전에 따라 21 을 쓰기도 하므로 둘 다 HIGH 로 못박고 깜빡이지 않는다.
static const int PIN_PA_A = 22, PIN_PA_B = 21;

// AC101 레지스터 상수 (라이브러리가 .cpp 안에 숨겨 놔서 여기 다시 적는다)
static const uint8_t R_DAC_DIG_CTRL = 0x48, R_DAC_VOL_CTRL = 0x49;
static const uint8_t R_DAC_MXR_SRC = 0x4C, R_DAC_MXR_GAIN = 0x4D;
static const uint8_t R_OMIXER_DACA = 0x53, R_OMIXER_SR = 0x54;
static const uint8_t R_HPOUT_CTRL = 0x56, R_SPKOUT_CTRL = 0x58;

class AC101Ex : public AC101 {
public:
    uint16_t rd(uint8_t r) { return ReadReg(r); }
    bool wr(uint8_t r, uint16_t v) { return WriteReg(r, v); }
};

static AC101Ex codec;
static int16_t *tx = nullptr;

// ── 신호 종류. 화이트 노이즈가 제일 잘 들리고, 좌우 교대가 채널을 가른다.
enum Sig { SIG_TONE, SIG_SWEEP, SIG_LR, SIG_NOISE, SIG_BEEP, SIG_SILENCE, SIG_N };
static const char *sig_name[SIG_N] = {
    "440Hz 연속음 (좌우 동시)",
    "200→2000Hz 스윕",
    "좌 440Hz / 우 880Hz 교대 (채널 판별)",
    "화이트 노이즈 (가장 잘 들린다)",
    "삐-삐 경보음 (1초 on/off)",
    "무음 (기준선 — 이어폰 잡음 바닥 확인)",
};

static int  g_sig = SIG_NOISE;      // 처음부터 제일 잘 들리는 걸로 시작한다
static int  g_vol = 45;             // AC101 볼륨 0~63
static int  g_cfg = 0;              // 출력 경로 설정 변형
static float g_phase = 0.0f;
static uint32_t g_rng = 0x12345678;

// ── 코덱 출력 경로 설정 변형.
//
// `begin()` 기본값으로 안 들릴 때 무엇을 의심할지의 목록이다. 각 변형이 무엇을
// 건드리는지 시리얼에 찍으므로, 어느 변형에서 소리가 나면 원인이 바로 특정된다.
static const int CFG_N = 6;
static const char *cfg_name[CFG_N] = {
    "0: begin() 기본값 그대로",
    "1: HPOUT_CTRL 강제 활성 (0xfbc0)",
    "2: OMIXER 를 DAC 로 강제 (DACA 0xff80 / SR 0x0808)",
    "3: SPKOUT_CTRL 활성 (스피커 JST 단자용, 0xeabd)",
    "4: DAC 디지털·볼륨 최대 (DIG 0x8000 / VOL 0xa0a0 / MXR 0x2200)",
    "5: 위 전부 한꺼번에",
};

static void apply_cfg(int c)
{
    codec.SetMode(AC101::MODE_DAC);          // 출력 경로만 연다
    if (c == 1 || c == 5) codec.wr(R_HPOUT_CTRL, 0xfbc0);
    if (c == 2 || c == 5) { codec.wr(R_OMIXER_DACA, 0xff80); codec.wr(R_OMIXER_SR, 0x0808); }
    if (c == 3 || c == 5) codec.wr(R_SPKOUT_CTRL, 0xeabd);
    if (c == 4 || c == 5) {
        codec.wr(R_DAC_DIG_CTRL, 0x8000);
        codec.wr(R_DAC_VOL_CTRL, 0xa0a0);
        codec.wr(R_DAC_MXR_SRC,  0x2200);
        codec.wr(R_DAC_MXR_GAIN, 0x0000);
    }
    codec.SetVolumeHeadphone(g_vol);
    codec.SetVolumeSpeaker(g_vol);
    Serial.printf("\n[설정] %s\n", cfg_name[c]);
    Serial.printf("  HPOUT=0x%04X SPKOUT=0x%04X OMIXER_DACA=0x%04X SR=0x%04X\n",
                  codec.rd(R_HPOUT_CTRL), codec.rd(R_SPKOUT_CTRL),
                  codec.rd(R_OMIXER_DACA), codec.rd(R_OMIXER_SR));
    Serial.printf("  DAC_DIG=0x%04X DAC_VOL=0x%04X MXR_SRC=0x%04X\n",
                  codec.rd(R_DAC_DIG_CTRL), codec.rd(R_DAC_VOL_CTRL), codec.rd(R_DAC_MXR_SRC));
}

static inline int16_t noise()
{
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
    return (int16_t)((int32_t)(g_rng & 0xFFFF) - 32768) / 4;
}

// 한 블록을 채운다. 진폭은 넉넉히 — 이어폰에서 확실히 들려야 판정이 된다.
static void fill(int16_t *b, int n)
{
    const float A = 9000.0f;
    const uint32_t ms = millis();
    for (int i = 0; i < n; i++) {
        int16_t l = 0, r = 0;
        switch (g_sig) {
        case SIG_TONE:
            g_phase += 440.0f / SR;
            if (g_phase >= 1) g_phase -= 1;
            l = r = (int16_t)(A * sinf(2 * PI * g_phase));
            break;
        case SIG_SWEEP: {
            const float t = (ms % 2000) / 2000.0f;
            const float hz = 200.0f + 1800.0f * t;
            g_phase += hz / SR;
            if (g_phase >= 1) g_phase -= 1;
            l = r = (int16_t)(A * sinf(2 * PI * g_phase));
            break;
        }
        case SIG_LR: {
            const bool left = ((ms / 1000) & 1) == 0;
            g_phase += (left ? 440.0f : 880.0f) / SR;
            if (g_phase >= 1) g_phase -= 1;
            const int16_t v = (int16_t)(A * sinf(2 * PI * g_phase));
            l = left ? v : 0;
            r = left ? 0 : v;
            break;
        }
        case SIG_NOISE:
            l = noise(); r = noise();
            break;
        case SIG_BEEP:
            if (((ms / 1000) & 1) == 0) {
                g_phase += 880.0f / SR;
                if (g_phase >= 1) g_phase -= 1;
                l = r = (int16_t)(A * sinf(2 * PI * g_phase));
            }
            break;
        default:
            break;
        }
        b[2 * i] = l;
        b[2 * i + 1] = r;
    }
}

static void i2s_setup()
{
    i2s_config_t cfg = {};
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);   // 출력 전용 — 죽은 ADC 를 아예 배제한다
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
    pins.data_in_num = I2S_PIN_NO_CHANGE;
    i2s_set_pin(PORT, &pins);

    // IRAM 은 16비트 접근이 불법이라(LoadStoreError) DMA 가능 메모리로 잡는다.
    tx = (int16_t *)heap_caps_malloc((size_t)BLOCK * 2 * sizeof(int16_t), MALLOC_CAP_DMA);
    if (!tx) { Serial.println("버퍼 실패. 중단."); while (1) delay(1000); }
}

static void help()
{
    Serial.println("\n──────── 이어폰을 꽂고 들어라 ────────");
    Serial.printf("신호: %s   볼륨 %d/63   %s\n", sig_name[g_sig], g_vol, cfg_name[g_cfg]);
    Serial.println("KEY1 신호 바꾸기 | KEY2 볼륨+ | KEY3 볼륨- | KEY4 레지스터 덤프");
    Serial.println("KEY5 볼륨 최대 | KEY6 출력경로 설정 순회");
    Serial.println("아무 것도 안 들리면 **가운데 DIP 스위치를 하나씩 토글**하라 —");
    Serial.println("신호는 계속 나가고 있으니 살아나는 순간 귀로 알 수 있다.");
    Serial.println("──────────────────────────────────────\n");
}

static void dump()
{
    Serial.println("\n[AC101 덤프]");
    static const uint8_t regs[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0C, 0x10, 0x11,
                                   0x40, 0x41, 0x48, 0x49, 0x4C, 0x4D, 0x50, 0x51, 0x52,
                                   0x53, 0x54, 0x55, 0x56, 0x58};
    for (unsigned i = 0; i < sizeof regs; i++)
        Serial.printf("  0x%02X = 0x%04X\n", regs[i], codec.rd(regs[i]));
}

struct Btn { int pin; bool prev; };
static Btn btn[] = {{KEY1, true}, {KEY2, true}, {KEY3, true},
                    {KEY4, true}, {KEY5, true}, {KEY6, true}};

void setup()
{
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== 잭 청음 시험 — 출력이 죽었는지 귀로 가른다 ===");
    Serial.println("확정된 사실: 입력(ADC)은 정확히 0 을 보낸다. 출력은 아무도 들어본 적 없다.");

    // 앰프 인에이블 후보를 키보다 먼저 HIGH 로 잡는다 — GPIO13(KEY2)이 GPIO22 에
    // 끌리는 문제도 같이 해결된다.
    pinMode(PIN_PA_A, OUTPUT); digitalWrite(PIN_PA_A, HIGH);
    pinMode(PIN_PA_B, OUTPUT); digitalWrite(PIN_PA_B, HIGH);
    for (auto &b : btn) pinMode(b.pin, b.pin == 36 ? INPUT : INPUT_PULLUP);

    if (!codec.begin(PIN_SDA, PIN_SCL)) {
        Serial.println("AC101 begin() 실패 — I2C 를 확인하라. (지금까지는 늘 성공했다)");
    } else {
        Serial.println("AC101 begin() 성공 — I2C 는 양방향으로 정상이다");
    }
    i2s_setup();
    apply_cfg(g_cfg);
    help();
}

void loop()
{
    fill(tx, BLOCK);
    size_t wrote = 0;
    i2s_write(PORT, tx, (size_t)BLOCK * 2 * sizeof(int16_t), &wrote, portMAX_DELAY);

    static uint32_t last = 0;
    if (millis() - last > 60) {                 // 버튼은 60ms 마다만 본다(오디오가 우선)
        last = millis();
        for (auto &b : btn) {
            const bool lvl = digitalRead(b.pin);
            if (b.prev && !lvl) {               // 눌림
                switch (b.pin) {
                case KEY1: g_sig = (g_sig + 1) % SIG_N;
                           Serial.printf("[신호] %s\n", sig_name[g_sig]); break;
                case KEY2: g_vol = min(63, g_vol + 4);
                           codec.SetVolumeHeadphone(g_vol); codec.SetVolumeSpeaker(g_vol);
                           Serial.printf("[볼륨] %d/63\n", g_vol); break;
                case KEY3: g_vol = max(0, g_vol - 4);
                           codec.SetVolumeHeadphone(g_vol); codec.SetVolumeSpeaker(g_vol);
                           Serial.printf("[볼륨] %d/63\n", g_vol); break;
                case KEY4: dump(); break;
                case KEY5: g_vol = 63;
                           codec.SetVolumeHeadphone(63); codec.SetVolumeSpeaker(63);
                           Serial.println("[볼륨] 63/63 (최대)"); break;
                case KEY6: g_cfg = (g_cfg + 1) % CFG_N; apply_cfg(g_cfg); break;
                }
            }
            b.prev = lvl;
        }
    }
}
