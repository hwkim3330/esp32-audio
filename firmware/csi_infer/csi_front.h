// CSI 창 프런트엔드 (C) — csi_features.py 와 같은 값을 내야 한다.
//
// 음성에서 로그멜이 그랬듯, 여기가 학습과 추론이 어긋나면 "인식률 문제" 로 위장해서
// 나타나는 지점이다. 그래서 파이썬 참조와 PC 에서 대조한 뒤에 보드에 올린다.
//
// 파이썬(csi_features.py)이 하는 일을 그대로 옮긴다:
//   1. 진폭 = sqrt(I²+Q²), 로그
//   2. 불균일 시각 → 균일 n_frames 로 선형 보간 (창을 시간으로 자르므로 필수)
//   3. 서브캐리어별 **중앙값** 차감 (평균이 아니다 — 평균은 큰 변화에 끌려간다)
//   4. 전체 표준편차로 나누고 ±CLIP 으로 클리핑
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CF_MAX_SC     128
// 창 안에 담을 수 있는 최대 프레임 수. 2초 창에 20Hz 면 40프레임이라 128은 충분하고,
// 512 로 두면 내부 버퍼가 262KB 가 되어 내부 DRAM(320KB)을 넘긴다(실측 469KB 초과).
#define CF_MAX_IN     128

// (imag, real) int8 쌍 시퀀스 + 각 프레임의 시각(ms) → (n_frames, n_sc) 정규화 창.
//
// iq   : n_in x (2*n_sc) int8
// ms   : n_in uint32 (단조 증가)
// out  : n_frames x n_sc float
// scratch : n_in*n_sc float 이상. 호출자가 PSRAM 에 잡아 넘긴다 —
//            내부에 static 으로 두면 내부 DRAM 을 넘긴다.
// 반환 : 0 성공, -1 입력 부족
int cf_window(const int8_t *iq, const uint32_t *ms, int n_in, int n_sc,
              int n_frames, float clip, float eps, float *scratch, float *out);

#ifdef __cplusplus
}
#endif
