// PC 패리티 테스트: ESP32 에 올릴 C 추론 엔진을 gcc 로 돌려서
// numpy 참조 구현과 같은 값이 나오는지 확인한다.
//
// 보드에 올리기 전에 이 테스트를 통과해야 한다. 통과하지 못하면
// 나중에 "인식률이 낮다" 로 위장한 수치 버그를 쫓게 된다.
//
//   gcc -O2 -I firmware/cabin_node model/scripts/parity_test.c \
//       firmware/cabin_node/cn_infer.c -lm -o /tmp/parity
//   /tmp/parity model/out/encoder.bin [input.wav]
//
// 출력: selftest 결과 + (wav 가 주어지면) mel/emb 를 바이너리로 덤프.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cn_infer.h"
#include "selftest.h"   // CN_SELFTEST_TOL

static float *load_blob(const char *path, size_t *n_float)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return NULL; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint32_t hdr[4];
    if (fread(hdr, sizeof(hdr), 1, f) != 1) { fclose(f); return NULL; }
    if (hdr[0] != 0x4E424143u) {
        fprintf(stderr, "매직 불일치: 0x%08x\n", hdr[0]);
        fclose(f); return NULL;
    }
    size_t nf = (size_t)(sz - (long)sizeof(hdr)) / sizeof(float);
    float *w = malloc(nf * sizeof(float));
    if (fread(w, sizeof(float), nf, f) != nf) { fclose(f); free(w); return NULL; }
    fclose(f);
    fprintf(stderr, "encoder.bin: dim=%u tensors=%u params=%zu\n",
            hdr[2], hdr[3], nf);
    *n_float = nf;
    return w;
}

// 최소 WAV 리더: 16bit mono PCM 만 받는다 (gen_data.py 가 그것만 낸다).
static int16_t *load_wav16(const char *path, int *n)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return NULL; }
    char id[4]; uint32_t sz;
    // RIFF / size / WAVE 를 건너뛴다. 짧은 파일이면 아래 루프가 그냥 끝난다.
    if (fread(id, 4, 1, f) != 1 || fread(&sz, 4, 1, f) != 1 ||
        fread(id, 4, 1, f) != 1) { fclose(f); return NULL; }
    int16_t *pcm = NULL; *n = 0;
    while (fread(id, 4, 1, f) == 1 && fread(&sz, 4, 1, f) == 1) {
        if (!memcmp(id, "data", 4)) {
            *n = (int)(sz / 2);
            pcm = malloc(sz);
            if (fread(pcm, 1, sz, f) != sz) { free(pcm); pcm = NULL; *n = 0; }
            break;
        }
        fseek(f, (long)((sz + 1) & ~1u), SEEK_CUR);
    }
    fclose(f);
    return pcm;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "사용법: %s encoder.bin [input.wav]\n", argv[0]);
        return 2;
    }
    size_t nf = 0;
    float *blob = load_blob(argv[1], &nf);
    if (!blob) return 1;

    void *scratch = malloc(cn_scratch_bytes());
    if (!scratch) { fprintf(stderr, "스크래치 할당 실패\n"); return 1; }
    fprintf(stderr, "스크래치 %.1f KB\n", cn_scratch_bytes() / 1024.0);

    cn_ctx_t ctx;
    cn_ctx_init(&ctx, blob, scratch);

    float err = 0.0f;
    int rc = cn_selftest(&ctx, &err);
    fprintf(stderr, "selftest: %s (최대오차 %.3e, 허용 %.1e)\n",
            rc == 0 ? "통과" : "실패", err, CN_SELFTEST_TOL);

    if (argc >= 3) {
        int n = 0;
        int16_t *pcm = load_wav16(argv[2], &n);
        if (!pcm) return 1;
        fprintf(stderr, "wav: %d 샘플 (%.2fs)\n", n, (double)n / CN_SR);
        float *mel = malloc((size_t)CN_N_FRAMES * CN_N_MELS * sizeof(float));
        float emb[CN_EMB_DIM];
        cn_logmel(&ctx, pcm, n, mel);
        cn_encode(&ctx, mel, emb);
        // stdout 으로 mel 과 emb 를 그대로 흘린다 (파이썬이 읽어 비교)
        fwrite(mel, sizeof(float), (size_t)CN_N_FRAMES * CN_N_MELS, stdout);
        fwrite(emb, sizeof(float), CN_EMB_DIM, stdout);
        free(mel); free(pcm);
    }
    return rc;
}
