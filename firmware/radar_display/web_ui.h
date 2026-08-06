// 브라우저로 보는 라이브 레이더.
//
// 왜 3D 사람 모형을 안 그리는가:
//   WiFi 로 사람 자세를 뽑는 논문들(DensePose From WiFi, Person-in-WiFi, WiPose)은
//   전부 **3×3 MIMO**(Intel 5300, 링크 9개)를 쓴다. ESP32 는 **안테나가 하나**다.
//   링크 하나는 방을 1차원으로 투영한 것이고, 거기서 3D 골격을 복원하는 것은 모델
//   문제가 아니라 정보이론적으로 불가능하다. 그려주면 그건 데이터가 아니라 애니메이션이다.
//
// 대신 데이터가 실제로 지지하는 것을 그린다:
//   1. CSI 워터폴 — 서브캐리어 64개 × 시간. 이건 측정값 그대로다.
//   2. 방 도면 — 보드와 앵커 4개, 각 링크의 편차(z)를 굵기·색으로. **진짜 공간 정보다.**
//      논문들은 안테나가 한 곳에 모여 있지만 우리 앵커는 방 네 지점에 퍼져 있다.
//   3. 추세 그래프 4개 시간 축 + 모델 출력.
//
// 부수 효과: 브라우저가 폴링하면 그 트래픽이 곧 CSI 소스가 된다. 그동안 프레임률이
// 10~20Hz 밖에 안 나와 창의 절반이 버려졌는데(능동 프로빙으로도 1.2배가 한계였다),
// 붙은 폰이 초당 수십 번 요청하면 그게 해결된다.
//
// 대가: SoftAP 는 채널이 고정되므로 채널 순환(주파수 다이버시티)을 포기한다.
// 그래서 기본이 아니라 **K6 길게 누르기로 켜는 모드**다.
#pragma once

#include <stdint.h>

// SoftAP + 웹서버를 켠다. 채널 순환이 멈추고 ap_ch 로 고정된다.
void web_start(uint8_t ap_ch);
void web_stop(void);
bool web_running(void);
void web_poll(void);

// 센싱 쪽이 매 프레임 채워주는 스냅샷. 웹은 이것만 읽는다 —
// 웹 코드가 센싱 내부를 직접 들여다보면 둘을 따로 고칠 수 없다.
struct WebSnap {
    uint8_t  n_sc;
    int8_t   amp[128];        // 서브캐리어별 진폭(dB 스케일, -128..127)
    float    band, thresh;
    float    anchor_z[8];
    uint8_t  anchor_last[8];  // MAC 마지막 바이트 — 화면에서 링크를 구분한다
    int8_t   anchor_rssi[8];
    uint8_t  n_anchor;
    uint32_t infer_ms, cls_hit, cls_tot;
    int      last_cls;
    float    last_score;
    int      n_ble;
    uint32_t ble_adv;
    uint32_t csi_hz;
    uint32_t uptime_s, boot_n;
    uint32_t mark_n, unmark_n;
    float    cohen_d;
};
extern WebSnap g_snap;
