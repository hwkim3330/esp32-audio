// 신스 노드 DSP — 보코더 · 오실레이터 · 루퍼 · 딜레이.
//
// 설계 결정 두 개를 먼저 적는다.
//
// 1) 보코더를 FFT 가 아니라 시간영역 필터뱅크로 만든다.
//    FFT 보코더는 윈도잉·오버랩애드·위상 처리가 붙고 지연이 프레임 단위로 생긴다.
//    16밴드 바이쿼드 뱅크는 샘플당 약 64 MAC 이라 32kHz 에서 2 MMAC/s 뿐이고,
//    지연이 사실상 0 이며, 소리는 오히려 더 클래식하다.
//
// 2) 긴 딜레이/루퍼는 PSRAM 에 둔다. PSRAM 은 랜덤 접근이 느리지만(캐빈 노드에서
//    추론이 2455ms 나온 이유가 그것) 딜레이 라인은 순차 접근이라 문제가 없다.
//    4MB 면 32kHz 모노로 65초, 이게 루퍼를 성립시킨다.
//
// cn_infer.c / cn_audio.c 와 마찬가지로 ESP-IDF 에 의존하지 않는 C99 다 —
// PC 에서 WAV 를 뽑아 검증할 수 있어야 한다.
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SN_SR         32000      // AC101 지원. 16k 보다 대역이 넓고 CPU 는 충분하다.
#define SN_BANDS      16         // 보코더 밴드 수
#define SN_VOICES     6          // 동시 발음 수 (터치키 6개와 맞춘다)
#define SN_BLOCK      256        // 오디오 블록 (8ms)

// ── 바이쿼드 (직접형 II 전치). 계수는 float, 상태는 채널별로 따로 둔다.
typedef struct { float b0, b1, b2, a1, a2; } sn_biq_t;
typedef struct { float z1, z2; } sn_biq_st_t;

void  sn_biq_bandpass(sn_biq_t *f, float fc, float q, float sr);
void  sn_biq_lowpass(sn_biq_t *f, float fc, float q, float sr);
float sn_biq_run(const sn_biq_t *f, sn_biq_st_t *s, float x);

// ── 보코더
// 마이크(모듈레이터)의 밴드별 에너지로 캐리어의 같은 밴드를 조절한다.
// 사람 목소리가 신스를 "말하게" 만드는 고전 회로다.
typedef struct {
    sn_biq_t    band[SN_BANDS];        // 밴드패스 (분석·합성 공용 계수)
    sn_biq_st_t mod_st[SN_BANDS];      // 모듈레이터 상태
    sn_biq_st_t car_st[SN_BANDS];      // 캐리어 상태
    float       env[SN_BANDS];         // 밴드별 포락선
    float       env_a;                 // 포락선 어택/릴리스 계수
    float       drive;                 // 출력 게인
    float       sibilance;             // 무성음(치찰음) 통과량 — 없으면 발음이 흐려진다
    sn_biq_t    hp;                    // 치찰음 추출용 고역
    sn_biq_st_t hp_st;
} sn_voc_t;

void  sn_voc_init(sn_voc_t *v, float sr);
// mod: 마이크 한 샘플, car: 캐리어 한 샘플 → 보코딩된 한 샘플
float sn_voc_run(sn_voc_t *v, float mod, float car);

// ── 오실레이터 뱅크 (캐리어). 폴리 6음, 톱니/사각/펄스.
typedef enum { SN_SAW = 0, SN_SQUARE, SN_PULSE, SN_TRI } sn_wave_t;

typedef struct {
    float    phase[SN_VOICES];
    float    inc[SN_VOICES];
    float    amp[SN_VOICES];       // ADSR 대신 단순 어택/릴리스 포락선
    uint8_t  on[SN_VOICES];
    sn_wave_t wave;
    float    detune;               // 보이스 간 디튠 (두께)
    float    sr;
} sn_osc_t;

void  sn_osc_init(sn_osc_t *o, float sr);
void  sn_osc_note(sn_osc_t *o, int voice, float hz, int on);
float sn_osc_run(sn_osc_t *o);

// ── 딜레이 / 루퍼 (PSRAM). 같은 버퍼를 모드로 나눠 쓴다.
typedef enum { SN_FX_OFF = 0, SN_FX_DELAY, SN_FX_LOOP } sn_fx_t;

typedef struct {
    int16_t *buf;          // PSRAM
    uint32_t len;          // 샘플 수
    uint32_t w;            // 쓰기 위치
    uint32_t loop_len;     // 루프 길이 (0 = 아직 정하지 않음)
    uint32_t delay_n;      // 딜레이 탭
    float    feedback;     // 0..0.95
    float    mix;          // 0..1
    sn_fx_t  mode;
    uint8_t  recording;    // 루퍼 오버덥 중
} sn_fx_state_t;

void  sn_fx_init(sn_fx_state_t *f, int16_t *psram, uint32_t len);
float sn_fx_run(sn_fx_state_t *f, float x);

// ── 부팅 자기진단 음. 화면이 없으니 소리가 유일한 "살아있다" 신호다.
// 짧은 상승 아르페지오를 낸다. 들리면 코덱·I2S·앰프·스피커가 전부 정상이다.
// out 에 스테레오 인터리브 16비트로 최대 max 샘플쌍을 쓴다. 반환: 쓴 샘플쌍 수.
int sn_boot_chime(int16_t *out, int max_pairs, float sr);

#ifdef __cplusplus
}
#endif
