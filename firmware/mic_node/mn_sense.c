// mn_sense.h 의 구현.
#include "mn_sense.h"

#include <math.h>
#include <string.h>

void mn_init(mn_state_t *s)
{
    memset(s, 0, sizeof(*s));
    s->coh = 0.0f;
    s->db_l = s->db_r = -120.0f;
}

void mn_process(mn_state_t *s, const int16_t *in, int n)
{
    if (n < 2 * MN_MAX_LAG + 8) return;

    // ── 1) DC 제거 + 레벨.
    //    DC 오프셋을 따로 보고하는 이유: 마이크가 안 붙어 있거나 배선이 이상하면
    //    신호는 0 인데 DC 만 크게 나오는 경우가 있다. RMS 만 보면 그걸 놓친다.
    float sl = 0.0f, sr = 0.0f;
    for (int i = 0; i < n; i++) {
        sl += (float)in[2 * i];
        sr += (float)in[2 * i + 1];
    }
    const float ml = sl / (float)n, mr = sr / (float)n;
    s->dc_l = ml * (1.0f / 32768.0f);
    s->dc_r = mr * (1.0f / 32768.0f);

    float el = 0.0f, er = 0.0f;
    for (int i = 0; i < n; i++) {
        const float a = (float)in[2 * i] - ml;
        const float b = (float)in[2 * i + 1] - mr;
        el += a * a;
        er += b * b;
    }
    s->rms_l = sqrtf(el / (float)n) * (1.0f / 32768.0f);
    s->rms_r = sqrtf(er / (float)n) * (1.0f / 32768.0f);
    s->db_l = 20.0f * log10f(s->rms_l + 1e-9f);
    s->db_r = 20.0f * log10f(s->rms_r + 1e-9f);
    s->ild_db = s->db_l - s->db_r;

    // ── 2) 방향: 정규화 교차상관 + 포물선 보간.
    //    보간을 넣는 이유는 해상도다. 마이크 간격이 좁아 최대 지연이 몇 샘플뿐이라
    //    정수 lag 만 쓰면 좌/중/우 3단이 끝이다. 부화소 보간으로 연속값이 된다.
    const int lo = MN_MAX_LAG, hi = n - MN_MAX_LAG;
    float best = -1e30f;
    int   best_k = 0;
    float corr[2 * MN_MAX_LAG + 1];
    for (int k = -MN_MAX_LAG; k <= MN_MAX_LAG; k++) {
        float acc = 0.0f;
        for (int i = lo; i < hi; i++) {
            const float a = (float)in[2 * i] - ml;
            const float b = (float)in[2 * (i + k) + 1] - mr;
            acc += a * b;
        }
        corr[k + MN_MAX_LAG] = acc;
        if (acc > best) { best = acc; best_k = k; }
    }
    const float denom = sqrtf(el * er) + 1e-9f;
    s->coh = best / denom;
    if (s->coh < 0.0f) s->coh = 0.0f;
    if (s->coh > 1.0f) s->coh = 1.0f;

    // 피크 주변 3점 포물선 보간
    float frac = 0.0f;
    const int bi = best_k + MN_MAX_LAG;
    if (bi > 0 && bi < 2 * MN_MAX_LAG) {
        const float y0 = corr[bi - 1], y1 = corr[bi], y2 = corr[bi + 1];
        const float d = 2.0f * (2.0f * y1 - y0 - y2);
        if (fabsf(d) > 1e-12f) {
            frac = (y2 - y0) / d;
            if (frac > 1.0f) frac = 1.0f;
            if (frac < -1.0f) frac = -1.0f;
        }
    }
    s->lag = (float)best_k + frac;

    // ── 3) 온셋: 빠른/느린 포락선 비.
    //    절대 임계값을 쓰지 않는다 — 주변 소음이 커지면 임계값이 같이 올라야 한다.
    const float e = (s->rms_l + s->rms_r) * 0.5f;
    s->env_fast += 0.5f  * (e - s->env_fast);
    s->env_slow += 0.02f * (e - s->env_slow);
    s->onset_strength = s->env_fast / (s->env_slow + 1e-6f);
    s->onset = 0;
    if (s->onset_hold > 0) {
        s->onset_hold--;
    } else if (s->onset_strength > 2.2f && e > 0.004f) {
        s->onset = 1;
        s->onset_hold = 6;        // 약 100ms 재트리거 금지
    }
}
