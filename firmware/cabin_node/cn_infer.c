// 캐빈 노드 추론 엔진 구현. cn_infer.h 의 설계 메모 참조.
#include "cn_infer.h"

#include <math.h>
#include <string.h>

#include "selftest.h"

// ───────────────────────────────────────────── FFT
//
// numpy.fft.rfft 와 같은 값을 내야 한다. 그래서 표준적인 radix-2 반복 FFT 를
// 직접 쓴다 (외부 의존 없음 = PC 패리티 테스트 가능). CN_N_FFT 는 2의 거듭제곱.

// M_PI 는 C99 표준이 아니다 (POSIX 확장). 직접 둬서 툴체인 차이를 없앤다.
#define CN_PI 3.14159265358979323846f

static void fft_inplace(float *re, float *im, int n)
{
    // 비트 반전 정렬
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        // 전방 변환: exp(-2*pi*i*k/len)
        const float ang = -2.0f * CN_PI / (float)len;
        const float wr = cosf(ang), wi = sinf(ang);
        for (int i = 0; i < n; i += len) {
            float cr = 1.0f, ci = 0.0f;
            for (int k = 0; k < len / 2; k++) {
                const int a = i + k, b = i + k + len / 2;
                const float xr = re[b] * cr - im[b] * ci;
                const float xi = re[b] * ci + im[b] * cr;
                re[b] = re[a] - xr;  im[b] = im[a] - xi;
                re[a] += xr;         im[a] += xi;
                const float ncr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = ncr;
            }
        }
    }
}

// ───────────────────────────────────────────── 스크래치

size_t cn_scratch_bytes(void)
{
    return (size_t)(2u * CN_ACT_MAX + 2u * CN_N_FFT) * sizeof(float);
}

void cn_ctx_init(cn_ctx_t *ctx, const float *blob, void *scratch)
{
    float *p = (float *)scratch;
    ctx->w = blob;
    ctx->act_a = p; p += CN_ACT_MAX;
    ctx->act_b = p; p += CN_ACT_MAX;
    ctx->fft_re = p; p += CN_N_FFT;
    ctx->fft_im = p;
}

// ───────────────────────────────────────────── 로그멜

void cn_logmel(cn_ctx_t *ctx, const int16_t *pcm, int n, float *mel_out)
{
    // 프레임 수를 세고, 창보다 길면 가운데를 쓴다.
    int avail = (n >= CN_WIN) ? 1 + (n - CN_WIN) / CN_HOP : 0;
    int off = 0;
    int nf = avail;
    if (avail > CN_N_FRAMES) {
        off = ((avail - CN_N_FRAMES) / 2) * CN_HOP;
        nf = CN_N_FRAMES;
    }

    const float log_floor = logf(CN_LOG_FLOOR);
    // 프레임이 부족한 구간은 바닥값으로 채운다 (파이썬 쪽과 같은 규칙: 가운데 정렬)
    int pad_lo = (nf < CN_N_FRAMES) ? (CN_N_FRAMES - nf) / 2 : 0;
    for (int t = 0; t < CN_N_FRAMES; t++)
        if (t < pad_lo || t >= pad_lo + nf)
            for (int m = 0; m < CN_N_MELS; m++)
                mel_out[t * CN_N_MELS + m] = log_floor;

    for (int f = 0; f < nf; f++) {
        const int16_t *src = pcm + off + (size_t)f * CN_HOP;
        for (int i = 0; i < CN_WIN; i++) {
            ctx->fft_re[i] = ((float)src[i] * (1.0f / 32768.0f)) * cn_hann[i];
            ctx->fft_im[i] = 0.0f;
        }
        for (int i = CN_WIN; i < CN_N_FFT; i++) {
            ctx->fft_re[i] = 0.0f;
            ctx->fft_im[i] = 0.0f;
        }
        fft_inplace(ctx->fft_re, ctx->fft_im, CN_N_FFT);

        // 파워 스펙트럼은 rfft 구간(0..N/2)만 쓴다.
        // 멜 투영은 희소하다: 필터별로 0 아닌 구간만 돈다 (10280 → 468 MAC).
        float *row = mel_out + (size_t)(pad_lo + f) * CN_N_MELS;
        const float *w = cn_mel_w;
        for (int m = 0; m < CN_N_MELS; m++) {
            const int s = cn_mel_start[m];
            const int L = cn_mel_len[m];
            float acc = 0.0f;
            for (int k = 0; k < L; k++) {
                const float re = ctx->fft_re[s + k], im = ctx->fft_im[s + k];
                acc += (re * re + im * im) * w[k];
            }
            w += L;
            row[m] = logf(acc > CN_LOG_FLOOR ? acc : CN_LOG_FLOOR);
        }
    }

    // 발화 단위 평균 차감(주파수 빈별). 마이크 감도·거리 차이를 상당히 지운다.
    for (int m = 0; m < CN_N_MELS; m++) {
        float s = 0.0f;
        for (int t = 0; t < CN_N_FRAMES; t++) s += mel_out[t * CN_N_MELS + m];
        const float mu = s / (float)CN_N_FRAMES;
        for (int t = 0; t < CN_N_FRAMES; t++) mel_out[t * CN_N_MELS + m] -= mu;
    }
}

// ───────────────────────────────────────────── 컨볼루션
//
// 활성값 레이아웃은 CHW: [채널][높이=시간][너비=멜]. export.py 의 numpy 참조와 같다.

static void conv_run(const cn_layer_t *L, const float *w, const float *b,
                     const float *in, int cin, int H, int W,
                     float *out, int *oH, int *oW)
{
    const int kh = L->kh, kw = L->kw;
    const int sh = L->sh, sw = L->sw;
    const int pad = (kh == 3) ? 1 : 0;
    const int Ho = (H + 2 * pad - kh) / sh + 1;
    const int Wo = (W + 2 * pad - kw) / sw + 1;
    const int cout = L->cout;
    const int groups = (L->kind == 1) ? cin : 1;   // dwconv 는 채널별
    const int gin = cin / groups, gout = cout / groups;

    for (int oc = 0; oc < cout; oc++) {
        const int g = oc / gout;
        float *o = out + (size_t)oc * Ho * Wo;
        const float bias = b[oc];
        for (int i = 0; i < Ho * Wo; i++) o[i] = bias;

        for (int ic = 0; ic < gin; ic++) {
            const float *src = in + (size_t)(g * gin + ic) * H * W;
            const float *ker = w + ((size_t)oc * gin + ic) * kh * kw;
            for (int ky = 0; ky < kh; ky++) {
                for (int kx = 0; kx < kw; kx++) {
                    const float wv = ker[ky * kw + kx];
                    if (wv == 0.0f) continue;
                    for (int oy = 0; oy < Ho; oy++) {
                        const int iy = oy * sh + ky - pad;
                        if (iy < 0 || iy >= H) continue;
                        const float *s = src + (size_t)iy * W;
                        float *d = o + (size_t)oy * Wo;
                        for (int ox = 0; ox < Wo; ox++) {
                            const int ix = ox * sw + kx - pad;
                            if (ix < 0 || ix >= W) continue;
                            d[ox] += wv * s[ix];
                        }
                    }
                }
            }
        }
        if (L->relu)
            for (int i = 0; i < Ho * Wo; i++)
                if (o[i] < 0.0f) o[i] = 0.0f;
    }
    *oH = Ho; *oW = Wo;
}

void cn_encode(cn_ctx_t *ctx, const float *mel, float *emb_out)
{
    float *cur = ctx->act_a, *nxt = ctx->act_b;
    int c = 1, H = CN_N_FRAMES, W = CN_N_MELS;

    memcpy(cur, mel, (size_t)CN_N_FRAMES * CN_N_MELS * sizeof(float));

    for (int li = 0; li < CN_N_LAYERS; li++) {
        const cn_layer_t *L = &cn_layers[li];
        switch (L->kind) {
        case 0:   // conv
        case 1:   // dwconv
        case 2: { // pwconv
            const float *w = ctx->w + L->w_off / sizeof(float);
            const float *b = ctx->w + L->b_off / sizeof(float);
            int oH, oW;
            conv_run(L, w, b, cur, c, H, W, nxt, &oH, &oW);
            c = L->cout; H = oH; W = oW;
            float *t = cur; cur = nxt; nxt = t;
            break;
        }
        case 3: { // global average pool → (c,1,1)
            const float inv = 1.0f / (float)(H * W);
            for (int ch = 0; ch < c; ch++) {
                const float *s = cur + (size_t)ch * H * W;
                float acc = 0.0f;
                for (int i = 0; i < H * W; i++) acc += s[i];
                nxt[ch] = acc * inv;
            }
            H = W = 1;
            float *t = cur; cur = nxt; nxt = t;
            break;
        }
        case 4: { // fc
            const float *w = ctx->w + L->w_off / sizeof(float);
            const float *b = ctx->w + L->b_off / sizeof(float);
            const int cout = L->cout, cin = L->cin;
            for (int o = 0; o < cout; o++) {
                float acc = b[o];
                const float *wr = w + (size_t)o * cin;
                for (int i = 0; i < cin; i++) acc += wr[i] * cur[i];
                nxt[o] = acc;
            }
            c = cout;
            float *t = cur; cur = nxt; nxt = t;
            break;
        }
        case 5: { // L2 정규화
            float ss = 0.0f;
            for (int i = 0; i < c; i++) ss += cur[i] * cur[i];
            const float inv = 1.0f / (sqrtf(ss) + 1e-12f);
            for (int i = 0; i < c; i++) cur[i] *= inv;
            break;
        }
        default: break;
        }
    }
    memcpy(emb_out, cur, (size_t)CN_EMB_DIM * sizeof(float));
}

// ───────────────────────────────────────────── 매칭 / 자기검증

int cn_match(const float *emb, const float *protos, int n_proto, float *score)
{
    int best = -1;
    float bs = -2.0f;
    for (int p = 0; p < n_proto; p++) {
        const float *pr = protos + (size_t)p * CN_EMB_DIM;
        float d = 0.0f;
        for (int i = 0; i < CN_EMB_DIM; i++) d += emb[i] * pr[i];
        if (d > bs) { bs = d; best = p; }
    }
    if (score) *score = bs;
    return best;
}

int cn_selftest(cn_ctx_t *ctx, float *max_err)
{
    float emb[CN_EMB_DIM];
    cn_encode(ctx, cn_selftest_mel, emb);
    float e = 0.0f;
    for (int i = 0; i < CN_EMB_DIM; i++) {
        const float d = fabsf(emb[i] - cn_selftest_emb[i]);
        if (d > e) e = d;
    }
    if (max_err) *max_err = e;
    return (e <= CN_SELFTEST_TOL) ? 0 : -1;
}
