// 신스 노드 — ESP32-Audio-Kit 을 악기로 쓴다.
//
// 화면이 없다는 문제를 소리로 푼다. 부팅하면 아르페지오가 나오고(들리면 코덱·I2S·앰프·
// 스피커가 전부 정상), 모드나 파형을 바꿀 때마다 확인음이 다르게 난다. 그래서 지금
// 무슨 상태인지 귀로 안다.
//
// 캐빈 노드에서 얻은 교훈을 그대로 쓴다: 추론이 2455ms 나온 이유는 PSRAM 랜덤 접근이
// 캐시를 갈아냈기 때문이었다. 여기서는 DSP 상태를 전부 내부 DRAM 에 두고, PSRAM 은
// 딜레이/루퍼처럼 순차 접근하는 것에만 쓴다.
//
// 버튼 6개를 전부 쓴다. 짧게 = 연주, 길게(600ms) = 조작.
//   KEY1 도  / 모드 순환 (보코더→신스→딜레이)
//   KEY2 레  / 파형 순환 (톱니→사각→펄스→삼각)
//   KEY3 미  / 옥타브 −
//   KEY4 솔  / 옥타브 +
//   KEY5 라  / 루프 녹음 토글
//   KEY6 도′ / 루프 지우기

#include <AC101.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>

#include "board_pins.h"
#include "sn_dsp.h"

static const i2s_port_t I2S_PORT = I2S_NUM_0;

// 5음계(펜타토닉). 화면이 없으니 아무 조합을 눌러도 불협이 안 나는 음계가 안전하다.
static const float SCALE_HZ[SN_VOICES] = {
    261.63f, 293.66f, 329.63f, 392.00f, 440.00f, 523.25f   // C D E G A C'
};
static const int KEY_PIN[SN_VOICES] = {
    CN_PIN_KEY1, CN_PIN_KEY2, CN_PIN_KEY3, CN_PIN_KEY4, CN_PIN_KEY5, CN_PIN_KEY6
};
static bool key_ok[SN_VOICES] = { false };   // 부팅 스캔에서 정상으로 판정된 키
static const char *const WAVE_NAME[4] = { "톱니", "사각", "펄스", "삼각" };
static const char *const MODE_NAME[3] = { "보코더", "신스", "딜레이" };

// ── DSP 상태는 전부 내부 DRAM (전역). PSRAM 에 두면 캐시를 갈아낸다.
static AC101         codec;
static sn_voc_t      voc;
static sn_osc_t      osc;
static sn_fx_state_t fx;

static volatile int   mode = 0;          // 0=보코더 1=신스 2=딜레이
static volatile int   octave = 0;        // -2..+2
static volatile bool  note_on[SN_VOICES] = { false };

// 확인음을 오디오 태스크가 섞어서 내보낸다 (별도 재생 경로를 만들지 않는다).
static volatile int   beep_hz = 0;
static volatile int   beep_left = 0;
static float          beep_phase = 0.0f;

static int16_t *io_buf;      // I2S 블록 (스테레오 인터리브)

// ───────────────────────────────────────── 확인음
static void beep(int hz, int ms)
{
    beep_hz = hz;
    beep_left = ms * SN_SR / 1000;
}

// ───────────────────────────────────────── 키
struct KeyState { bool down; uint32_t t_down; bool long_fired; bool xlong_fired; };
static KeyState keys[SN_VOICES];

static void key_long(int k)
{
    switch (k) {
    case 0:
        mode = (mode + 1) % 3;
        fx.mode = (mode == 2) ? SN_FX_DELAY
                : (fx.mode == SN_FX_LOOP ? SN_FX_LOOP : SN_FX_OFF);
        Serial.printf("[모드] %s\n", MODE_NAME[mode]);
        // 모드마다 다른 음 — 귀로 구분된다
        beep(mode == 0 ? 523 : (mode == 1 ? 659 : 784), 120);
        break;
    case 1:
        osc.wave = (sn_wave_t)((osc.wave + 1) % 4);
        Serial.printf("[파형] %s\n", WAVE_NAME[osc.wave]);
        beep(880 + 110 * (int)osc.wave, 90);
        break;
    case 2:
        if (octave > -2) octave--;
        Serial.printf("[옥타브] %+d\n", octave);
        beep(330, 90);
        break;
    case 3:
        if (octave < 2) octave++;
        Serial.printf("[옥타브] %+d\n", octave);
        beep(1047, 90);
        break;
    case 4:
        if (fx.mode != SN_FX_LOOP) {
            fx.mode = SN_FX_LOOP;
            fx.loop_len = 0;
            fx.w = 0;
            fx.recording = 1;
            Serial.println("[루퍼] 녹음 시작");
            beep(659, 150);
        } else if (fx.recording) {
            fx.loop_len = fx.w ? fx.w : fx.len;   // 놓은 지점이 루프 길이
            fx.w = 0;
            fx.recording = 0;
            Serial.printf("[루퍼] 길이 확정 %.2fs, 재생\n",
                          (float)fx.loop_len / SN_SR);
            beep(784, 150);
        } else {
            fx.recording = 1;
            Serial.println("[루퍼] 오버덥");
            beep(988, 150);
        }
        break;
    case 5:
        fx.mode = (mode == 2) ? SN_FX_DELAY : SN_FX_OFF;
        fx.loop_len = 0;
        fx.w = 0;
        fx.recording = 0;
        if (fx.buf) memset(fx.buf, 0, (size_t)fx.len * sizeof(int16_t));
        Serial.println("[루퍼] 지움");
        beep(220, 200);
        break;
    default: break;
    }
}

// 아주 길게 누름(2초). 배선이 죽은 키의 기능을 살아있는 키로 옮기는 자리다.
static void key_xlong(int k)
{
    if (k == 4) {                 // K5 2초 = 루프 지우기 (원래 K6)
        fx.mode = (mode == 2) ? SN_FX_DELAY : SN_FX_OFF;
        fx.loop_len = 0;
        fx.w = 0;
        fx.recording = 0;
        if (fx.buf) memset(fx.buf, 0, (size_t)fx.len * sizeof(int16_t));
        Serial.println("[루퍼] 지움 (K5 2초)");
        beep(220, 250);
    }
}

static void keys_poll()
{
    const uint32_t now = millis();
    for (int k = 0; k < SN_VOICES; k++) {
        if (!key_ok[k]) continue;            // 배선이 의심스러운 키는 무시
        const bool down = (digitalRead(KEY_PIN[k]) == LOW);
        KeyState &s = keys[k];
        if (down && !s.down) {
            s.down = true;
            s.t_down = now;
            s.long_fired = false;
            s.xlong_fired = false;
            note_on[k] = true;
            sn_osc_note(&osc, k, SCALE_HZ[k] * powf(2.0f, (float)octave), 1);
        } else if (down && s.down && !s.long_fired && now - s.t_down > 600) {
            // 길게 누름 — 음을 끊고 조작으로 전환한다
            s.long_fired = true;
            note_on[k] = false;
            sn_osc_note(&osc, k, SCALE_HZ[k], 0);
            key_long(k);
        } else if (down && s.down && s.long_fired && !s.xlong_fired &&
                   now - s.t_down > 2000) {
            // 아주 길게(2초) — 두 번째 기능. K6(GPIO13)이 배선 불량으로 비활성이라
            // '루프 지우기' 를 여기로 옮겼다. 키가 하나 죽어도 기능은 안 잃는다.
            s.xlong_fired = true;
            key_xlong(k);
        } else if (!down && s.down) {
            s.down = false;
            if (!s.long_fired) {
                note_on[k] = false;
                sn_osc_note(&osc, k, SCALE_HZ[k], 0);
            }
        }
    }
}

// ───────────────────────────────────────── 오디오
static void audio_task(void *)
{
    for (;;) {
        size_t got = 0;
        if (i2s_read(I2S_PORT, io_buf, (size_t)SN_BLOCK * 2 * sizeof(int16_t),
                     &got, portMAX_DELAY) != ESP_OK) continue;
        const int n = (int)(got / (2 * sizeof(int16_t)));

        for (int i = 0; i < n; i++) {
            // 마이크 2개를 합쳐 모듈레이터로 쓴다 (한쪽만 쓰면 3dB 손해다)
            const float mic = (((float)io_buf[2 * i] + (float)io_buf[2 * i + 1])
                               * 0.5f) * (1.0f / 32768.0f);
            const float car = sn_osc_run(&osc);

            float y;
            switch (mode) {
            case 0: y = sn_voc_run(&voc, mic, car); break;   // 보코더
            case 1: y = car * 0.8f; break;                    // 순수 신스
            default: y = mic * 0.9f; break;                   // 마이크 통과 + FX
            }
            y = sn_fx_run(&fx, y);

            // 확인음 섞기
            if (beep_left > 0) {
                beep_phase += (float)beep_hz / SN_SR;
                if (beep_phase >= 1.0f) beep_phase -= 1.0f;
                y += ((beep_phase < 0.5f) ? 0.25f : -0.25f);
                beep_left--;
            }

            if (y > 1.0f) y = 1.0f;
            if (y < -1.0f) y = -1.0f;
            const int16_t s = (int16_t)(y * 30000.0f);
            io_buf[2 * i] = s;
            io_buf[2 * i + 1] = s;
        }

        size_t wrote = 0;
        i2s_write(I2S_PORT, io_buf, (size_t)n * 2 * sizeof(int16_t), &wrote,
                  portMAX_DELAY);
    }
}

// ───────────────────────────────────────── setup
void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== 신스 노드 ===");

    if (!psramFound()) { Serial.println("PSRAM 없음. 중단."); while (1) delay(1000); }

    Wire.begin(CN_PIN_I2C_SDA, CN_PIN_I2C_SCL, CN_I2C_FREQ_HZ);
    Wire.beginTransmission(CN_AC101_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[probe] AC101 응답 없음 — board_pins.h 확인. 중단.");
        while (1) delay(1000);
    }
    Serial.println("[probe] AC101 응답 있음");

    pinMode(CN_PIN_PA_ENABLE, OUTPUT);
    digitalWrite(CN_PIN_PA_ENABLE, HIGH);       // 악기는 계속 소리를 내므로 켜 둔다
    for (int k = 0; k < SN_VOICES; k++)
        pinMode(KEY_PIN[k], (KEY_PIN[k] == 36 || KEY_PIN[k] == 5)
                            ? INPUT : INPUT_PULLUP);

    // 키 배선을 스캔한다. 부팅 시점엔 아무도 안 눌렀을 것이므로 HIGH 여야 한다.
    // LOW 로 읽히면 핀 가정이 틀렸거나 다른 게 물려 있다 — 그 키는 비활성화한다.
    // (실측: K6=GPIO13 이 LOW. 이대로 두면 '루퍼 지우기'가 계속 발동한다.)
    Serial.print("[keys] 부팅 스캔: ");
    int n_ok = 0;
    for (int k = 0; k < SN_VOICES; k++) {
        const int v = digitalRead(KEY_PIN[k]);
        key_ok[k] = (v == HIGH);
        if (key_ok[k]) n_ok++;
        Serial.printf("K%d(GPIO%d)=%d%s ", k + 1, KEY_PIN[k], v,
                      key_ok[k] ? "" : "[비활성]");
    }
    Serial.printf("\n[keys] 사용 가능 %d/%d\n", n_ok, SN_VOICES);

    if (!codec.begin(CN_PIN_I2C_SDA, CN_PIN_I2C_SCL, CN_I2C_FREQ_HZ)) {
        Serial.println("AC101 초기화 실패. 중단."); while (1) delay(1000);
    }
    codec.SetI2sSampleRate(AC101::SAMPLE_RATE_32000);
    codec.SetI2sWordSize(AC101::WORD_SIZE_16_BITS);
    codec.SetI2sMode(AC101::MODE_SLAVE);
    codec.SetI2sFormat(AC101::DATA_FORMAT_I2S);
    codec.SetMode(AC101::MODE_ADC_DAC);         // 전이중 — 마이크와 스피커 동시
    codec.SetVolumeSpeaker(45);
    codec.SetVolumeHeadphone(45);

    i2s_config_t cfg = {};
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
    cfg.sample_rate = SN_SR;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count = 6;
    cfg.dma_buf_len = SN_BLOCK;
    cfg.use_apll = true;
    cfg.tx_desc_auto_clear = true;
    if (i2s_driver_install(I2S_PORT, &cfg, 0, nullptr) != ESP_OK) {
        Serial.println("I2S 실패. 중단."); while (1) delay(1000);
    }
    i2s_pin_config_t pins = {};
    pins.mck_io_num   = CN_PIN_I2S_MCLK;
    pins.bck_io_num   = CN_PIN_I2S_BCLK;
    pins.ws_io_num    = CN_PIN_I2S_LRCK;
    pins.data_out_num = CN_PIN_I2S_DOUT;
    pins.data_in_num  = CN_PIN_I2S_DIN;
    i2s_set_pin(I2S_PORT, &pins);

    // ── DSP. 상태는 내부 DRAM, 딜레이/루퍼 버퍼만 PSRAM (순차 접근).
    sn_voc_init(&voc, SN_SR);
    sn_osc_init(&osc, SN_SR);

    const size_t fx_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    // 여유의 대부분을 딜레이 라인에 준다. 이게 루퍼 길이를 정한다.
    uint32_t fx_len = (uint32_t)((fx_free - 128 * 1024) / sizeof(int16_t));
    int16_t *fx_buf = (int16_t *)heap_caps_malloc(
        (size_t)fx_len * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!fx_buf) { Serial.println("PSRAM 할당 실패. 중단."); while (1) delay(1000); }
    sn_fx_init(&fx, fx_buf, fx_len);
    Serial.printf("루퍼/딜레이 버퍼 %.1f MB = %.1f초\n",
                  fx_len * 2.0 / 1048576.0, (double)fx_len / SN_SR);

    // MALLOC_CAP_INTERNAL 은 IRAM 을 돌려줄 수 있고, IRAM 은 16비트 접근이 불법이다
    // (실측: LoadStoreError, EXCVADDR 0x400918e4). I2S 버퍼는 DMA 가능 메모리여야
    // 하므로 MALLOC_CAP_DMA 가 정답이다 — 8비트 접근이 보장되고 DMA 도 닿는다.
    io_buf = (int16_t *)heap_caps_malloc(
        (size_t)SN_BLOCK * 2 * sizeof(int16_t), MALLOC_CAP_DMA);
    if (!io_buf) { Serial.println("io_buf 할당 실패. 중단."); while (1) delay(1000); }

    // ── 부팅 자기진단 음. 화면이 없으니 이게 "살아있다" 의 유일한 증거다.
    {
        const int max_pairs = SN_SR / 2;
        int16_t *ch = (int16_t *)heap_caps_malloc(
            (size_t)max_pairs * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM);
        if (ch) {
            const int k = sn_boot_chime(ch, max_pairs, SN_SR);
            size_t wrote = 0;
            i2s_write(I2S_PORT, ch, (size_t)k * 2 * sizeof(int16_t), &wrote,
                      portMAX_DELAY);
            heap_caps_free(ch);
            Serial.printf("부팅 진단음 재생 (%d 샘플쌍). 들렸다면 "
                          "코덱·I2S·앰프·스피커 정상.\n", k);
        }
    }

    xTaskCreatePinnedToCore(audio_task, "audio", 4096, nullptr, 6, nullptr, 1);

    Serial.printf("\n모드 %s, 파형 %s, 옥타브 %+d\n",
                  MODE_NAME[mode], WAVE_NAME[osc.wave], octave);
    Serial.println("짧게 = 연주(펜타토닉 6음), 길게 = 조작");
    Serial.println("  K1 모드  K2 파형  K3 옥타브−  K4 옥타브+  K5 루프  K6 지우기");
    if (!key_ok[5])
        Serial.println("  ※ K6 배선 불량 — '지우기' 는 K5 를 2초 누르세요");
    Serial.println("보코더 모드에서 마이크에 말하면 신스가 말합니다.");
}

void loop()
{
    keys_poll();
    delay(4);
}
