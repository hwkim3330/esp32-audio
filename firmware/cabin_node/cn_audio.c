// cn_audio.h 의 구현. 설계 메모는 헤더 참조.
#include "cn_audio.h"

#include <math.h>
#include <string.h>

// ───────────────────────────────────────────── 링버퍼

size_t cn_ring_bytes(void) { return (size_t)CN_A_RING_LEN * 2u * sizeof(int16_t); }

void cn_ring_init(cn_ring_t *rb, void *psram)
{
    rb->l = (int16_t *)psram;
    rb->r = rb->l + CN_A_RING_LEN;
    rb->w = 0;
    memset(rb->l, 0, cn_ring_bytes());
}

void cn_ring_push(cn_ring_t *rb, const int16_t *interleaved, int n_frames)
{
    // AC101 이 최대 24비트라 I2S 를 16비트로 통일했다. 그대로 받는다.
    for (int i = 0; i < n_frames; i++) {
        const uint32_t p = (rb->w + (uint32_t)i) % CN_A_RING_LEN;
        rb->l[p] = interleaved[2 * i];
        rb->r[p] = interleaved[2 * i + 1];
    }
    rb->w += (uint32_t)n_frames;
}

void cn_ring_read_mono(const cn_ring_t *rb, uint32_t from, int n, int ch, int16_t *out)
{
    for (int i = 0; i < n; i++) {
        const uint32_t abs = from + (uint32_t)i;
        // 아직 안 쓰인 미래거나, 링을 한 바퀴 넘겨 잃어버린 과거면 0.
        if (abs >= rb->w || (rb->w - abs) > CN_A_RING_LEN) { out[i] = 0; continue; }
        const uint32_t p = abs % CN_A_RING_LEN;
        if (ch == 0)      out[i] = rb->l[p];
        else if (ch == 1) out[i] = rb->r[p];
        else              out[i] = (int16_t)(((int)rb->l[p] + (int)rb->r[p]) / 2);
    }
}

// ───────────────────────────────────────────── VAD

void cn_vad_init(cn_vad_t *v) { v->noise = 1e6f; v->hang = 0; v->active = 0; }

int cn_vad_push(cn_vad_t *v, const int16_t *frame, int n)
{
    float e = 0.0f;
    int zc = 0;
    for (int i = 0; i < n; i++) {
        e += (float)frame[i] * (float)frame[i];
        if (i && ((frame[i] < 0) != (frame[i - 1] < 0))) zc++;
    }
    e /= (float)n;

    // 잡음 바닥은 조용할 때만 빠르게, 말할 때는 아주 느리게 따라간다.
    // (말소리로 바닥을 올려버리면 발화 중간이 잘린다)
    const float a = (e < v->noise * 3.0f) ? 0.05f : 0.001f;
    v->noise += a * (e - v->noise);

    // 순수 톤이나 험(zc 가 아주 낮음)은 말이 아니다. 자잘한 잡음(zc 과다)도 뺀다.
    const int voiced = (e > v->noise * 8.0f) &&
                       (zc > n / 40) && (zc < n * 3 / 4);
    if (voiced) {
        v->active = 1;
        v->hang = 25;                 // 250ms 유지 — 어절 사이 공백에 안 끊긴다
    } else if (v->hang > 0) {
        v->hang--;
    } else {
        v->active = 0;
    }
    return v->active;
}

// ───────────────────────────────────────────── 방향 추정

int cn_doa(const int16_t *l, const int16_t *r, int n, float *conf)
{
    // 마이크 간격이 좁아 최대 지연이 ±3 샘플 수준이다. 그 범위만 상관을 본다.
    const int MAXLAG = 3;
    double best = -1e30; int bestlag = 0, sum2 = 0;
    for (int lag = -MAXLAG; lag <= MAXLAG; lag++) {
        double acc = 0.0;
        for (int i = MAXLAG; i < n - MAXLAG; i++)
            acc += (double)l[i] * (double)r[i + lag];
        if (acc > best) { best = acc; bestlag = lag; }
    }
    double el = 0.0, er = 0.0;
    for (int i = 0; i < n; i++) { el += (double)l[i]*l[i]; er += (double)r[i]*r[i]; }
    (void)sum2;
    const double denom = sqrt(el * er) + 1e-9;
    const float c = (float)(best / denom);
    if (conf) *conf = c < 0.0f ? 0.0f : (c > 1.0f ? 1.0f : c);
    if (bestlag > 0) return +1;
    if (bestlag < 0) return -1;
    return 0;
}

// ───────────────────────────────────────────── IMA ADPCM

static const int16_t A_STEP[89] = {
    7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,
    88,97,107,118,130,143,157,173,190,209,230,253,279,307,337,371,408,449,494,
    544,598,658,724,796,876,963,1060,1166,1282,1411,1552,1707,1878,2066,2272,
    2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,
    10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,27086,29794,32767
};
static const int8_t A_IDX[8] = { -1,-1,-1,-1,2,4,6,8 };

void cn_adpcm_reset(cn_adpcm_t *st) { st->pred = 0; st->idx = 0; }

void cn_adpcm_decode(cn_adpcm_t *st, const uint8_t *data, int n, int16_t *out)
{
    for (int i = 0; i < n; i++) {
        const uint8_t byte = data[i >> 1];
        const int code = (i & 1) ? (byte >> 4) : (byte & 0x0F);
        const int step = A_STEP[st->idx];
        int d = step >> 3;
        if (code & 4) d += step;
        if (code & 2) d += step >> 1;
        if (code & 1) d += step >> 2;
        st->pred = (code & 8) ? st->pred - d : st->pred + d;
        if (st->pred < -32768) st->pred = -32768;
        if (st->pred >  32767) st->pred =  32767;
        st->idx += A_IDX[code & 7];
        if (st->idx < 0)  st->idx = 0;
        if (st->idx > 88) st->idx = 88;
        out[i] = (int16_t)st->pred;
    }
}

// ───────────────────────────────────────────── 재생 컬러링

void cn_color_init(cn_color_t *c, float pitch_semi, float tilt_db, float gain_db)
{
    c->pitch_semi = pitch_semi;
    c->tilt_db = tilt_db;
    c->gain_db = gain_db;
    c->hp_z = 0.0f;
    c->phase = 0.0f;
}

int cn_color_apply(cn_color_t *c, const int16_t *in, int n,
                   int16_t *out, int max_out)
{
    // 피치는 재샘플로 옮긴다. 포먼트까지 같이 움직여서 "다른 사람" 처럼 들린다 —
    // 위상 보코더가 정석이지만 ESP32 에서 실시간으로 돌리기엔 비싸고,
    // 경보음 톤 변화 용도로는 이게 오히려 더 확실하게 들린다.
    const float ratio = powf(2.0f, c->pitch_semi / 12.0f);
    const float g = powf(10.0f, c->gain_db / 20.0f);
    // 1차 고역 강조. tilt_db 만큼 고역을 올린다.
    const float tilt = powf(10.0f, c->tilt_db / 20.0f) - 1.0f;

    int k = 0;
    while (k < max_out) {
        const float src = c->phase;
        const int i0 = (int)src;
        if (i0 + 1 >= n) break;
        const float f = src - (float)i0;
        float s = (1.0f - f) * (float)in[i0] + f * (float)in[i0 + 1];

        // 고역 성분 = 원신호 − 1차 저역. 그걸 tilt 만큼 더한다.
        c->hp_z += 0.25f * (s - c->hp_z);
        s += tilt * (s - c->hp_z);

        s *= g;
        if (s >  32767.0f) s =  32767.0f;
        if (s < -32768.0f) s = -32768.0f;
        out[k++] = (int16_t)s;
        c->phase += ratio;
    }
    // 다음 호출을 위해 위상을 소비한 만큼 되돌린다.
    c->phase -= (float)n;
    if (c->phase < 0.0f) c->phase = 0.0f;
    return k;
}
