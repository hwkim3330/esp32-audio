// 실시간 오디오 처리 블록들 — Hearing Aid / 잡음억제 모드가 쓴다.
//
// 왜 헤더에 몰아 넣었나: arduino-cli 는 .ino 에 함수 원형을 #include 직후에만
// 자동 삽입한다. 그래서 .ino 본문에 정의한 타입을 인수로 받는 함수는 컴파일이
// 깨진다(player.h 주석 참고). 여기 두면 순서 문제가 없다.
//
// 전부 float 로 쓴다. ESP32 는 단정도 FPU 가 있어서 44.1kHz 스테레오를 블록 단위로
// 처리하는 데 여유가 많다 — 실측으로 코어 하나의 몇 % 수준이다.
#pragma once
#include <Arduino.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────── 바이쿼드
//
// RBJ 쿡북 계수. 고역/저역/피킹을 같은 구조로 쓴다.
struct Biquad {
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    float z1 = 0, z2 = 0;

    inline float run(float x)
    {
        // 전치 직접형 II — 상태가 두 개뿐이고 계수 갱신에 안전하다
        const float y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
    inline void reset() { z1 = z2 = 0; }

    void highpass(float fs, float f0, float q)
    {
        const float w = 2.0f * (float)M_PI * f0 / fs, c = cosf(w), s = sinf(w);
        const float al = s / (2.0f * q), a0 = 1 + al;
        b0 = (1 + c) / 2 / a0; b1 = -(1 + c) / a0; b2 = b0;
        a1 = -2 * c / a0;      a2 = (1 - al) / a0;
    }
    void lowpass(float fs, float f0, float q)
    {
        const float w = 2.0f * (float)M_PI * f0 / fs, c = cosf(w), s = sinf(w);
        const float al = s / (2.0f * q), a0 = 1 + al;
        b0 = (1 - c) / 2 / a0; b1 = (1 - c) / a0; b2 = b0;
        a1 = -2 * c / a0;      a2 = (1 - al) / a0;
    }
    // 말소리 명료도용 프레즌스 피킹. gain_db 를 0 으로 두면 통과와 같다.
    void peaking(float fs, float f0, float q, float gain_db)
    {
        const float A = powf(10.0f, gain_db / 40.0f);
        const float w = 2.0f * (float)M_PI * f0 / fs, c = cosf(w), s = sinf(w);
        const float al = s / (2.0f * q), a0 = 1 + al / A;
        b0 = (1 + al * A) / a0; b1 = -2 * c / a0; b2 = (1 - al * A) / a0;
        a1 = -2 * c / a0;       a2 = (1 - al / A) / a0;
    }
};

// ─────────────────────────────────────────────────────────────── 빔포밍 (2마이크)
//
// 마이크 간격을 모르기 때문에 정확한 조향은 못 한다. 대신 물리적으로 확실한 두
// 가지만 제공하고, 어느 쪽이 조용한지는 **측정값(noise RMS)으로 고르게** 한다.
//
//   BEAM_SUM  : (L + R)/2 — 브로드사이드. 상관 없는 잡음에 대해 3dB 이득.
//               정면(두 마이크에 같은 위상으로 도달)을 살린다.
//   BEAM_DIFF : L - R(지연) — 엔드파이어 차동. 한쪽 방향에 널을 만든다.
//               측면 잡음원이 뚜렷할 때 크게 준다. 저역이 깎이는 대가가 있다.
enum BeamMode { BEAM_SUM, BEAM_DIFF };

static const int BEAM_MAX_D = 8;     // 샘플 단위 지연 한도 (44.1kHz 에서 0.18ms)

struct Beamformer {
    BeamMode mode = BEAM_SUM;
    int delay = 0;                   // R 채널에 걸 지연 (샘플)
    float hist[BEAM_MAX_D + 1] = {0};
    int wp = 0;

    inline float run(float l, float r)
    {
        hist[wp] = r;
        const int rp = (wp - delay + (BEAM_MAX_D + 1)) % (BEAM_MAX_D + 1);
        const float rd = hist[rp];
        wp = (wp + 1) % (BEAM_MAX_D + 1);
        return (mode == BEAM_SUM) ? 0.5f * (l + rd) : (l - rd);
    }
    inline void reset() { for (int i = 0; i <= BEAM_MAX_D; i++) hist[i] = 0; wp = 0; }
};

// ─────────────────────────────────────────────────────────────── NLMS 잡음억제
//
// 마이크 두 개 중 하나를 목표(primary), 다른 하나를 잡음 기준(reference)으로 두고
// 상관된 성분을 적응적으로 뺀다. 탭을 여러 개 두는 이유는 두 마이크 사이의 도달
// 시간차·위상차를 필터가 스스로 맞춰야 하기 때문이다.
//
// strength 로 빼는 양을 조절한다(0 = 그대로, 1 = 전부). 목소리까지 지워지는 걸
// 사용자가 귀로 조절할 수 있어야 한다.
struct Nlms {
    static const int TAPS = 16;
    float w[TAPS] = {0};
    float x[TAPS] = {0};
    float mu = 0.10f;
    float strength = 1.0f;

    inline float run(float pri, float ref)
    {
        for (int i = TAPS - 1; i > 0; i--) x[i] = x[i - 1];
        x[0] = ref;

        float y = 0, p = 1e-6f;
        for (int i = 0; i < TAPS; i++) { y += w[i] * x[i]; p += x[i] * x[i]; }
        const float e = pri - y;                 // 적응은 언제나 전량으로 한다
        const float g = mu * e / p;
        for (int i = 0; i < TAPS; i++) w[i] += g * x[i];
        return pri - strength * y;               // 출력만 강도로 섞는다
    }
    inline void reset() { for (int i = 0; i < TAPS; i++) { w[i] = 0; x[i] = 0; } }
};

// ─────────────────────────────────────────────────────────────── 컴프레서 + 리미터
//
// 보청기에서 컴프레서는 선택이 아니다. 작은 소리를 올리려면 큰 소리를 눌러야
// 클리핑하지 않는다. 리미터는 마지막 안전장치 — 절대 풀스케일을 넘기지 않는다.
struct Compressor {
    float env = 0;
    float atk = 0, rel = 0;          // 계수 (set() 에서 계산)
    float thresh = 6000.0f;          // 이 위로는 압축한다 (16비트 스케일)
    float ratio = 4.0f;
    float makeup = 1.0f;
    float ceiling = 30000.0f;        // 리미터 천장 (풀스케일 32767 의 92%)

    void set(float fs, float atk_ms, float rel_ms)
    {
        atk = expf(-1.0f / (fs * atk_ms / 1000.0f));
        rel = expf(-1.0f / (fs * rel_ms / 1000.0f));
    }
    inline float run(float x)
    {
        const float a = fabsf(x);
        env = (a > env) ? (atk * env + (1 - atk) * a) : (rel * env + (1 - rel) * a);

        float g = 1.0f;
        if (env > thresh) {
            // 임계 위 초과분을 ratio 로 나눈다(정적 곡선, 소프트니 하드니 하지 않는다)
            const float over = env / thresh;
            g = powf(over, 1.0f / ratio - 1.0f);
        }
        float y = x * g * makeup;
        if (y > ceiling) y = ceiling;            // 하드 리미터
        else if (y < -ceiling) y = -ceiling;
        return y;
    }
    inline void reset() { env = 0; }
};

// ─────────────────────────────────────────────────────────────── RMS 측정기
//
// 진단값을 웹에 띄우려면 블록마다 값을 남겨야 한다. 지수 평균으로 부드럽게.
struct RmsMeter {
    double acc = 0;
    uint32_t n = 0;
    float value = 0;

    inline void add(float x) { acc += (double)x * x; n++; }
    inline void commit()
    {
        if (!n) return;
        const float r = sqrtf((float)(acc / n));
        value = (value == 0) ? r : (0.8f * value + 0.2f * r);
        acc = 0; n = 0;
    }
};

// dB 로 바꿔 보여주면 사람이 비교하기 쉽다. 0 은 -99 로 클램프.
static inline float rms_db(float a, float b)
{
    if (a <= 0.0f || b <= 0.0f) return -99.0f;
    return 20.0f * log10f(a / b);
}
