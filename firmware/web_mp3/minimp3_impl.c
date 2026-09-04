// minimp3 구현부 분리 컴파일 단위.
//
// minimp3.h 는 헤더 하나에 선언과 구현이 같이 들어 있고, 구현은
// MINIMP3_IMPLEMENTATION 가 정의된 번역 단위에서만 펼쳐진다. .ino 는 여러 파일이
// 이어붙여지므로 구현을 여기 한 곳에 못박아 중복 정의를 막는다.
//
// MINIMP3_ONLY_MP3 : Layer 1/2 디코더를 뺀다 (플래시 ~10KB 절약, mp3 만 쓰므로)
// MINIMP3_NO_SIMD  : SSE/NEON 경로 제거 — ESP32(Xtensa LX6)엔 해당 SIMD 가 없다

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_SIMD
#include "minimp3.h"
