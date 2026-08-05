// 캐빈 노드 추론 엔진 — 로그멜 프런트엔드 + float32 CNN 인코더.
//
// ESP-IDF 에 의존하지 않는 순수 C99 다. 이유가 있다: 같은 파일을 gcc 로 컴파일해
// PC 에서 numpy 참조 구현과 출력을 비교할 수 있다. 임베디드 추론에서 "돌긴 도는데
// 값이 다름" 은 가장 잡기 어려운 버그이고, 보드에 올리기 전에 걸러야 한다.
//
// ESP32 는 단정밀도 FPU 가 있고 SIMD 는 없다. 그래서 int8 로 내려도 속도 이득이
// 거의 없고 양자화 오차만 생긴다. PSRAM 4MB 로 float32 가중치(약 268KB)와
// 활성값(약 490KB)이 넉넉히 들어가므로 float32 를 쓴다.
#pragma once

#include <stdint.h>
#include <stddef.h>

#include "model_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// 활성값 버퍼 하나의 최대 크기(float 개수). stem 출력이 가장 크다:
//   (CN_N_MELS/2 채널) x (CN_N_FRAMES/2) x (CN_N_MELS/2)
// 여유를 두고 잡는다. 두 개를 핑퐁으로 쓴다.
#define CN_ACT_MAX  (64u * 96u * 20u)

typedef struct {
    const float *w;      // encoder.bin 의 텐서 블롭 (float32, PSRAM)
    float *act_a;        // 활성값 버퍼 A (CN_ACT_MAX float)
    float *act_b;        // 활성값 버퍼 B (CN_ACT_MAX float)
    float *fft_re;       // CN_N_FFT float
    float *fft_im;       // CN_N_FFT float
} cn_ctx_t;

// 필요한 스크래치 총량(바이트). 호출자가 PSRAM 에서 한 번에 잡으면 편하다.
size_t cn_scratch_bytes(void);

// ctx 의 포인터들을 하나의 스크래치 블록 위에 배치한다.
// blob 은 encoder.bin 의 헤더(16바이트)를 건너뛴 텐서 시작 주소.
void cn_ctx_init(cn_ctx_t *ctx, const float *blob, void *scratch);

// int16 PCM(16kHz mono) → (CN_N_FRAMES x CN_N_MELS) 로그멜, 발화 단위 평균 차감까지.
// n 이 창보다 길면 가운데를 남긴다 (앞뒤 무음보다 가운데가 정보가 많다).
// mel_out 은 CN_N_FRAMES*CN_N_MELS float. 행 우선(프레임, 멜).
void cn_logmel(cn_ctx_t *ctx, const int16_t *pcm, int n, float *mel_out);

// 로그멜 → L2 정규화 임베딩 (CN_EMB_DIM float).
void cn_encode(cn_ctx_t *ctx, const float *mel, float *emb_out);

// 임베딩과 등록된 프로토타입들 중 최근접을 찾는다.
// protos: n_proto x CN_EMB_DIM (각 행은 L2 정규화되어 있어야 한다)
// 반환: 최근접 인덱스. *score 에 코사인 유사도.
int cn_match(const float *emb, const float *protos, int n_proto, float *score);

// 부팅 자기검증. selftest.h 의 알려진 입력으로 알려진 출력이 나오는지 본다.
// 반환 0 = 통과. *max_err 에 최대 절대오차.
int cn_selftest(cn_ctx_t *ctx, float *max_err);

#ifdef __cplusplus
}
#endif
