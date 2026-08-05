// 마이크 노드 센싱 — 스피커가 없는 보드로 할 수 있는 것.
//
// 보드는 소리를 내지 않는다. 2마이크 센서 + 컨트롤러가 되고, 소리와 화면은 태블릿이
// 맡는다. 그래서 마이크 2개에서 뽑아낼 수 있는 것을 최대한 뽑는 것이 이 파일의 일이다.
//
//   방향  좌우 위상차(TDOA). 손뼉 위치를 옮기면 연속 파라미터가 된다.
//   세기  dBFS 레벨. 숨·목소리 크기로 표현력 제어.
//   타격  온셋 감지. 손뼉·두드림으로 리듬 트리거.
//
// 마이크 간격을 모른다. 그래서 각도로 환산하지 않고 **지연(샘플)** 을 그대로 보고한다.
// 각도 매핑은 실측 캘리브레이션으로 정한다 — 모르는 값을 가정해서 각도를 만들면
// 틀린 숫자가 맞는 것처럼 보인다.
//
// ESP-IDF 에 의존하지 않는 C99 — PC 에서 합성 신호로 검증할 수 있어야 한다.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MN_SR        32000
#define MN_BLOCK     512      // 16ms. 상관 창.
#define MN_MAX_LAG   8        // ±8 샘플. 간격 5cm 면 최대 약 4.7 샘플이라 여유가 있다.

typedef struct {
    // ── 레벨
    float rms_l, rms_r;       // 선형 0..1
    float db_l, db_r;         // dBFS
    float dc_l, dc_r;         // DC 오프셋 (마이크 배선 이상을 잡는다)

    // ── 방향
    float lag;                // 샘플 단위 지연. + 면 R 이 늦음(= 왼쪽에서 온 소리)
    float coh;                // 정규화 상관 피크 0..1. 낮으면 방향을 믿을 수 없다.
    float ild_db;             // 좌우 레벨차 (또 하나의 방향 단서)

    // ── 타격
    int   onset;              // 이 블록에서 온셋 발생
    float onset_strength;     // 빠른/느린 포락선 비

    // 내부 상태
    float env_fast, env_slow;
    int   onset_hold;
} mn_state_t;

void mn_init(mn_state_t *s);

// 인터리브 스테레오 16비트 블록 하나를 처리한다. n = 프레임 수 (≤ MN_BLOCK).
void mn_process(mn_state_t *s, const int16_t *interleaved, int n);

#ifdef __cplusplus
}
#endif
