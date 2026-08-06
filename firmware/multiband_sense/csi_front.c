// csi_front.h 의 구현. csi_features.py 와 같은 값을 내야 한다.
#include "csi_front.h"

#include <math.h>
#include <string.h>

// 중앙값. 서브캐리어당 한 번씩 부르므로 n_frames 가 작아 삽입정렬로 충분하다.
// (128프레임 × 64서브캐리어 기준, 퀵셀렉트를 쓸 이유가 없다)
static float median_of(float *tmp, int n)
{
    for (int i = 1; i < n; i++) {
        const float v = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = v;
    }
    // 파이썬 np.median 과 같은 규칙: 짝수면 가운데 두 값의 평균
    return (n & 1) ? tmp[n / 2] : 0.5f * (tmp[n / 2 - 1] + tmp[n / 2]);
}

int cf_window(const int8_t *iq, const uint32_t *ms, int n_in, int n_sc,
              int n_frames, float clip, float eps, float *scratch, float *out)
{
    if (n_in < 2 || n_sc < 1 || n_sc > CF_MAX_SC || n_frames < 2) return -1;
    if (n_in > CF_MAX_IN) n_in = CF_MAX_IN;

    // ── 1) 로그 진폭. 스크래치는 호출자가 준다(PSRAM).
    if (!scratch || !out) return -1;
    float *la = scratch;
    for (int t = 0; t < n_in; t++) {
        const int8_t *row = iq + (size_t)t * 2 * n_sc;
        float *o = la + (size_t)t * n_sc;
        for (int f = 0; f < n_sc; f++) {
            const float im = (float)row[2 * f], re = (float)row[2 * f + 1];
            const float a = sqrtf(im * im + re * re);
            o[f] = logf(a > eps ? a : eps);
        }
    }

    // ── 2) 시간 축 선형 보간. 파이썬 np.interp 와 같은 규칙:
    //      범위를 벗어나면 양 끝 값으로 고정(clamp), 격자는 [t0, t_last] 균일.
    const double t0 = (double)ms[0] / 1000.0;
    const double t1 = (double)ms[n_in - 1] / 1000.0;
    if (!(t1 > t0)) return -1;
    const double step = (t1 - t0) / (double)(n_frames - 1);

    // 구간 찾기. 격자가 단조라 이전 위치에서 이어서 찾으면 전체가 O(n) 이다.
    // lo 를 static 으로 두면 두 태스크가 동시에 부를 때 깨진다 — 평범한 지역변수로.
    int lo = 0;
    for (int k = 0; k < n_frames; k++) {
        const double tg = t0 + step * (double)k;
        while (lo < n_in - 2 && (double)ms[lo + 1] / 1000.0 < tg) lo++;
        const double ta = (double)ms[lo] / 1000.0;
        const double tb = (double)ms[lo + 1] / 1000.0;
        const float w = (tb > ta) ? (float)((tg - ta) / (tb - ta)) : 0.0f;
        const float *A = la + (size_t)lo * n_sc;
        const float *B = la + (size_t)(lo + 1) * n_sc;
        float *O = out + (size_t)k * n_sc;
        for (int f = 0; f < n_sc; f++) O[f] = A[f] + w * (B[f] - A[f]);
    }

    // ── 3) 서브캐리어별 중앙값 차감
    float tmp[CF_MAX_IN];
    for (int f = 0; f < n_sc; f++) {
        for (int k = 0; k < n_frames; k++) tmp[k] = out[(size_t)k * n_sc + f];
        const float med = median_of(tmp, n_frames);
        for (int k = 0; k < n_frames; k++) out[(size_t)k * n_sc + f] -= med;
    }

    // ── 4) 전체 표준편차로 정규화 + 클리핑
    double s = 0.0, s2 = 0.0;
    const int N = n_frames * n_sc;
    for (int i = 0; i < N; i++) { s += out[i]; s2 += (double)out[i] * out[i]; }
    const double mu = s / N;
    const double var = s2 / N - mu * mu;
    const float sd = (float)sqrt(var > 0 ? var : 0);
    if (sd > 1e-6f) {
        const float inv = 1.0f / sd;
        for (int i = 0; i < N; i++) out[i] *= inv;
    }
    for (int i = 0; i < N; i++) {
        if (out[i] >  clip) out[i] =  clip;
        if (out[i] < -clip) out[i] = -clip;
    }
    return 0;
}
