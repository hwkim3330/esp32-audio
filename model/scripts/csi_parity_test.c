// CSI 모델 PC 패리티 테스트.
#include <math.h>
//
// 음성 모델과 같은 추론 엔진(cn_infer.c)을 쓰는지, 그리고 numpy 참조와 같은 값을
// 내는지 확인한다. 로그멜은 조건 컴파일로 빠지므로 스크래치도 그만큼 작아진다.
//
//   gcc -O2 -std=c99 -I firmware/csi_infer model/scripts/csi_parity_test.c \
//       firmware/csi_infer/cn_infer.c -lm -o /tmp/csi_parity
//   /tmp/csi_parity model/out_csi/csi_encoder.bin
#include <stdio.h>
#include <stdlib.h>

#include "cn_infer.h"
#include "prototypes.h"
#include "selftest.h"

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "사용법: %s csi_encoder.bin\n", argv[0]);
        return 2;
    }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 1; }
    fseek(f, 0, SEEK_END);
    const long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint32_t hdr[4];
    if (fread(hdr, sizeof hdr, 1, f) != 1) { fclose(f); return 1; }
    const size_t nf = (size_t)(sz - (long)sizeof hdr) / sizeof(float);
    float *w = (float *)malloc(nf * sizeof(float));
    if (!w || fread(w, sizeof(float), nf, f) != nf) { fclose(f); return 1; }
    fclose(f);
    printf("csi_encoder.bin: dim=%u tensors=%u params=%zu\n", hdr[2], hdr[3], nf);

    void *sc = malloc(cn_scratch_bytes());
    if (!sc) { fprintf(stderr, "스크래치 할당 실패\n"); return 1; }
    printf("스크래치 %.1f KB", cn_scratch_bytes() / 1024.0);
#ifdef CN_HAS_LOGMEL
    printf(" (로그멜 포함)\n");
#else
    printf(" (로그멜 없음 — CSI 는 프런트엔드가 다르다)\n");
#endif

    cn_ctx_t ctx;
    cn_ctx_init(&ctx, w, sc);

    float err = 0.0f;
    const int rc = cn_selftest(&ctx, &err);
    printf("selftest: %s (최대오차 %.3e, 허용 %.1e)\n",
           rc == 0 ? "통과" : "실패", err, CN_SELFTEST_TOL);

    // 임베딩이 정규화됐는지, 프로토타입 매칭이 도는지
    float emb[CN_EMB_DIM];
    cn_encode(&ctx, cn_selftest_mel, emb);
    float ss = 0.0f;
    for (int i = 0; i < CN_EMB_DIM; i++) ss += emb[i] * emb[i];
    printf("임베딩 L2 노름 %.6f (1.0 이어야 한다)\n", (double)sqrtf(ss));

    float score = 0.0f;
    const int best = cn_match(emb, cn_protos, CN_N_PROTO, &score);
    printf("프로토타입 매칭: %d개 중 %d번 (클래스 %d), 코사인 %.4f\n",
           CN_N_PROTO, best, best >= 0 ? cn_proto_class[best] : -1, (double)score);

    printf("입력 %d 프레임 × %d 서브캐리어, 레이어 %d개\n",
           CN_N_FRAMES, CN_N_MELS, CN_N_LAYERS);
    return rc;
}
