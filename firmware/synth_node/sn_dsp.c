// sn_dsp.h 의 구현. 설계 메모는 헤더 참조.
#include "sn_dsp.h"

#include <math.h>
#include <string.h>

#define SN_PI 3.14159265358979323846f

// ───────────────────────────────────────────── 바이쿼드
//
// RBJ 쿡북 형태. a0 로 미리 나눠 두어 런타임에 나눗셈이 없다.

static void norm(sn_biq_t *f, float b0, float b1, float b2,
                 float a0, float a1, float a2)
{
    const float ia = 1.0f / a0;
    f->b0 = b0 * ia; f->b1 = b1 * ia; f->b2 = b2 * ia;
    f->a1 = a1 * ia; f->a2 = a2 * ia;
}

void sn_biq_bandpass(sn_biq_t *f, float fc, float q, float sr)
{
    if (fc > sr * 0.45f) fc = sr * 0.45f;      // 나이퀴스트 근처에서 발산 방지
    const float w = 2.0f * SN_PI * fc / sr;
    const float cw = cosf(w), sw = sinf(w);
    const float al = sw / (2.0f * q);
    // 상수 피크 이득(0dB) 밴드패스
    norm(f, al, 0.0f, -al, 1.0f + al, -2.0f * cw, 1.0f - al);
}

void sn_biq_lowpass(sn_biq_t *f, float fc, float q, float sr)
{
    if (fc > sr * 0.45f) fc = sr * 0.45f;
    const float w = 2.0f * SN_PI * fc / sr;
    const float cw = cosf(w), sw = sinf(w);
    const float al = sw / (2.0f * q);
    norm(f, (1.0f - cw) * 0.5f, 1.0f - cw, (1.0f - cw) * 0.5f,
         1.0f + al, -2.0f * cw, 1.0f - al);
}

static void sn_biq_highpass(sn_biq_t *f, float fc, float q, float sr)
{
    if (fc > sr * 0.45f) fc = sr * 0.45f;
    const float w = 2.0f * SN_PI * fc / sr;
    const float cw = cosf(w), sw = sinf(w);
    const float al = sw / (2.0f * q);
    norm(f, (1.0f + cw) * 0.5f, -(1.0f + cw), (1.0f + cw) * 0.5f,
         1.0f + al, -2.0f * cw, 1.0f - al);
}

float sn_biq_run(const sn_biq_t *f, sn_biq_st_t *s, float x)
{
    const float y = f->b0 * x + s->z1;
    s->z1 = f->b1 * x - f->a1 * y + s->z2;
    s->z2 = f->b2 * x - f->a2 * y;
    return y;
}

// ───────────────────────────────────────────── 보코더

void sn_voc_init(sn_voc_t *v, float sr)
{
    memset(v, 0, sizeof(*v));
    // 밴드 중심을 로그 간격으로 배치한다. 200Hz~6.4kHz 가 말소리 대역이다.
    const float f_lo = 200.0f, f_hi = 6400.0f;
    const float ratio = powf(f_hi / f_lo, 1.0f / (float)(SN_BANDS - 1));
    float fc = f_lo;
    for (int b = 0; b < SN_BANDS; b++) {
        // Q 를 밴드 간격에 맞춘다 — 너무 낮으면 뭉개지고 높으면 구멍이 생긴다.
        sn_biq_bandpass(&v->band[b], fc, 3.0f, sr);
        fc *= ratio;
    }
    // 포락선 시간상수 약 12ms. 더 빠르면 거칠고 느리면 발음이 뭉갠다.
    v->env_a = 1.0f - expf(-1.0f / (0.012f * sr));
    v->drive = 3.0f;
    v->sibilance = 0.35f;
    sn_biq_highpass(&v->hp, 5000.0f, 0.7f, sr);
}

float sn_voc_run(sn_voc_t *v, float mod, float car)
{
    float out = 0.0f;
    for (int b = 0; b < SN_BANDS; b++) {
        // 모듈레이터 밴드 → 정류 → 포락선
        const float m = sn_biq_run(&v->band[b], &v->mod_st[b], mod);
        const float a = m < 0.0f ? -m : m;
        v->env[b] += v->env_a * (a - v->env[b]);
        // 같은 밴드로 캐리어를 자르고 포락선으로 곱한다
        const float c = sn_biq_run(&v->band[b], &v->car_st[b], car);
        out += c * v->env[b];
    }
    // 치찰음(ㅅ, ㅊ)은 밴드 에너지가 낮아 보코더만 쓰면 발음이 흐려진다.
    // 마이크의 고역을 그대로 조금 섞어 자음 명료도를 살린다.
    out = out * v->drive + sn_biq_run(&v->hp, &v->hp_st, mod) * v->sibilance;
    if (out > 1.0f) out = 1.0f;
    if (out < -1.0f) out = -1.0f;
    return out;
}

// ───────────────────────────────────────────── 오실레이터

void sn_osc_init(sn_osc_t *o, float sr)
{
    memset(o, 0, sizeof(*o));
    o->sr = sr;
    o->wave = SN_SAW;
    o->detune = 0.004f;
}

void sn_osc_note(sn_osc_t *o, int voice, float hz, int on)
{
    if (voice < 0 || voice >= SN_VOICES) return;
    // 보이스마다 살짝 디튠해서 유니즌 두께를 만든다.
    const float d = 1.0f + o->detune * (float)(voice - SN_VOICES / 2);
    o->inc[voice] = hz * d / o->sr;
    o->on[voice] = on ? 1 : 0;
}

static float poly_blep(float t, float dt)
{
    // 톱니/사각의 계단 불연속을 부드럽게 — 없으면 에일리어싱이 심하다.
    if (t < dt)          { const float x = t / dt - 1.0f; return -x * x; }
    if (t > 1.0f - dt)   { const float x = (t - 1.0f) / dt + 1.0f; return x * x; }
    return 0.0f;
}

float sn_osc_run(sn_osc_t *o)
{
    float sum = 0.0f;
    for (int i = 0; i < SN_VOICES; i++) {
        // 어택/릴리스. 클릭이 나지 않을 정도로만 짧게.
        const float target = o->on[i] ? 1.0f : 0.0f;
        o->amp[i] += (target - o->amp[i]) * (o->on[i] ? 0.004f : 0.0015f);
        if (o->amp[i] < 1e-4f) continue;

        const float dt = o->inc[i];
        o->phase[i] += dt;
        if (o->phase[i] >= 1.0f) o->phase[i] -= 1.0f;
        const float t = o->phase[i];

        float s;
        switch (o->wave) {
        case SN_SQUARE:
            s = (t < 0.5f) ? 1.0f : -1.0f;
            s -= poly_blep(t, dt);
            s += poly_blep(fmodf(t + 0.5f, 1.0f), dt);
            break;
        case SN_PULSE:                       // 듀티 0.25 — 더 얇고 리드 같다
            s = (t < 0.25f) ? 1.0f : -1.0f;
            s -= poly_blep(t, dt);
            s += poly_blep(fmodf(t + 0.75f, 1.0f), dt);
            break;
        case SN_TRI:
            s = (t < 0.5f) ? (4.0f * t - 1.0f) : (3.0f - 4.0f * t);
            break;
        case SN_SAW:
        default:
            s = 2.0f * t - 1.0f;
            s -= poly_blep(t, dt);
            break;
        }
        sum += s * o->amp[i];
    }
    return sum * (1.0f / (float)SN_VOICES);
}

// ───────────────────────────────────────────── 딜레이 / 루퍼

void sn_fx_init(sn_fx_state_t *f, int16_t *psram, uint32_t len)
{
    memset(f, 0, sizeof(*f));
    f->buf = psram;
    f->len = len;
    f->delay_n = len / 4 < (uint32_t)(SN_SR / 3) ? len / 4 : (uint32_t)(SN_SR / 3);
    f->feedback = 0.45f;
    f->mix = 0.35f;
    f->mode = SN_FX_OFF;
    if (psram && len) memset(psram, 0, (size_t)len * sizeof(int16_t));
}

float sn_fx_run(sn_fx_state_t *f, float x)
{
    if (f->mode == SN_FX_OFF || !f->buf) return x;

    if (f->mode == SN_FX_DELAY) {
        const uint32_t r = (f->w + f->len - f->delay_n) % f->len;
        const float d = (float)f->buf[r] * (1.0f / 32768.0f);
        float wr = x + d * f->feedback;
        if (wr > 1.0f) wr = 1.0f;
        if (wr < -1.0f) wr = -1.0f;
        f->buf[f->w] = (int16_t)(wr * 32767.0f);
        f->w = (f->w + 1) % f->len;
        return x * (1.0f - f->mix) + d * f->mix;
    }

    // 루퍼: loop_len 이 정해지기 전까지는 녹음하며 길이를 늘린다.
    const uint32_t L = f->loop_len ? f->loop_len : f->len;
    const float d = (float)f->buf[f->w] * (1.0f / 32768.0f);
    if (f->recording) {
        float wr = d + x;                    // 오버덥 (누적)
        if (wr > 1.0f) wr = 1.0f;
        if (wr < -1.0f) wr = -1.0f;
        f->buf[f->w] = (int16_t)(wr * 32767.0f);
    }
    f->w++;
    if (f->w >= L) f->w = 0;
    return x + d;
}

// ───────────────────────────────────────────── 부팅 자기진단 음

int sn_boot_chime(int16_t *out, int max_pairs, float sr)
{
    // 상승 4음. 각 120ms, 짧은 페이드로 클릭 제거.
    static const float hz[4] = { 523.25f, 659.25f, 783.99f, 1046.50f };  // C E G C'
    const int n_note = (int)(0.12f * sr);
    int k = 0;
    for (int i = 0; i < 4; i++) {
        float ph = 0.0f;
        const float inc = hz[i] / sr;
        for (int j = 0; j < n_note && k < max_pairs; j++, k++) {
            ph += inc;
            if (ph >= 1.0f) ph -= 1.0f;
            // 삼각파 — 사인보다 싸고 작은 스피커에서 더 잘 들린다
            float s = (ph < 0.5f) ? (4.0f * ph - 1.0f) : (3.0f - 4.0f * ph);
            const float fade = (float)j / (float)n_note;
            const float env = fade < 0.05f ? fade / 0.05f
                            : (fade > 0.8f ? (1.0f - fade) / 0.2f : 1.0f);
            const int16_t v = (int16_t)(s * env * 9000.0f);
            out[2 * k] = v;
            out[2 * k + 1] = v;
        }
    }
    return k;
}
