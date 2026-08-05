// 캐빈 노드 오디오 경로. 보드 자원을 실제로 쓰는 곳이다.
//
//   PSRAM 4MB  — 스테레오 링버퍼(프리롤 포함) + 구문 팩 캐시
//   마이크 2개 — 방향 추정(운전석/조수석) 과 채널 선택
//   스피커     — ADPCM 디코딩 + 실시간 피치/톤 컬러링
//
// cn_infer.c 와 마찬가지로 ESP-IDF 에 의존하지 않는 C99 다. PC 에서 검증 가능해야 한다.
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CN_A_SR          16000
#define CN_A_RING_SEC    10          // 프리롤 10초. PTT 를 늦게 눌러도 앞말이 안 잘린다.
#define CN_A_RING_LEN    (CN_A_SR * CN_A_RING_SEC)
#define CN_A_FRAME       160         // 10ms. VAD 판정 단위.

// ── 스테레오 링버퍼 (PSRAM)
typedef struct {
    int16_t *l, *r;          // 각 CN_A_RING_LEN 샘플
    uint32_t w;              // 총 기록 샘플 수 (모듈로 안 함 — 절대 시각으로 쓴다)
} cn_ring_t;

size_t cn_ring_bytes(void);
void cn_ring_init(cn_ring_t *rb, void *psram);
// 인터리브된 스테레오 프레임을 밀어넣는다 (I2S 에서 온 그대로, 16비트).
void cn_ring_push(cn_ring_t *rb, const int16_t *interleaved, int n_frames);
// 절대 위치 [from, from+n) 을 모노로 뽑는다. ch: 0=L 1=R 2=합. 부족하면 0 을 채운다.
void cn_ring_read_mono(const cn_ring_t *rb, uint32_t from, int n, int ch, int16_t *out);

// ── VAD (에너지 + 영교차). 신경망 없이 충분하고, 상시 돌아야 하므로 싸야 한다.
typedef struct {
    float noise;             // 적응 잡음 바닥
    int   hang;              // 발화 종료 후 유지 프레임
    int   active;
} cn_vad_t;

void cn_vad_init(cn_vad_t *v);
// 10ms 프레임 하나를 먹인다. 반환 1 = 발화 중. 상태 전이는 호출자가 본다.
int  cn_vad_push(cn_vad_t *v, const int16_t *frame, int n);

// ── 방향 추정. 마이크가 2개라 좌/중앙/우 정도는 나온다.
// 반환: -1=왼쪽 0=가운데 +1=오른쪽, conf 에 신뢰도(0..1).
// 각도는 못 낸다 — 마이크 간격이 좁아 16kHz 에서 최대 지연이 몇 샘플뿐이다.
int  cn_doa(const int16_t *l, const int16_t *r, int n, float *conf);

// ── IMA ADPCM 디코더. gen_phrasepack.py 의 인코더와 같은 테이블을 쓴다.
typedef struct { int pred, idx; } cn_adpcm_t;
void cn_adpcm_reset(cn_adpcm_t *st);
// n 샘플을 디코딩한다. data 는 (n+1)/2 바이트를 담고 있어야 한다.
void cn_adpcm_decode(cn_adpcm_t *st, const uint8_t *data, int n, int16_t *out);

// ── 재생 컬러링. 같은 음성을 상황에 따라 다르게 들리게 한다.
// pitch_semi: 반음 단위 피치 이동, tilt_db: 고역 강조, gain_db: 음량.
// 피치는 재샘플 방식이라 길이가 바뀐다 — 경보용으로는 그게 오히려 자연스럽다.
typedef struct {
    float pitch_semi, tilt_db, gain_db;
    float hp_z;              // 틸트 필터 상태
    float phase;             // 재샘플 위상
} cn_color_t;

void cn_color_init(cn_color_t *c, float pitch_semi, float tilt_db, float gain_db);
// in 에서 n 샘플을 읽어 out 에 최대 max_out 샘플을 쓴다. 반환: 쓴 샘플 수.
int  cn_color_apply(cn_color_t *c, const int16_t *in, int n,
                    int16_t *out, int max_out);

#ifdef __cplusplus
}
#endif
