// GPIO16/17 이 PSRAM 에 점유돼 있는가 — 확정한다.
//
// 왜 중요한가: PLEOS path_display_node.ino 가 ST7789 의 DC 핀을 GPIO16 에 쓴다.
// A1S 모듈이 PSRAM 배선에 GPIO16/17 을 쓰는 계열이면 그 코드를 이 보드에 못 올린다.
// docs/HARDWARE.md 에 미확인으로 남겨둔 항목이고, 이제 실측으로 답한다.
//
// 방법: PSRAM 에 알려진 패턴을 쓰고, GPIO16/17 을 출력으로 마구 토글한 뒤,
// PSRAM 을 다시 읽어 무결성을 본다. 깨지면 그 핀은 PSRAM 것이다.

#include <Arduino.h>
#include <esp_heap_caps.h>

static const size_t N = 64 * 1024;   // 64K 워드 = 256KB

static uint32_t lcg(uint32_t x) { return x * 1664525u + 1013904223u; }

static bool verify(uint32_t *buf, uint32_t seed, const char *tag)
{
    uint32_t x = seed;
    size_t bad = 0, first = 0;
    for (size_t i = 0; i < N; i++) {
        x = lcg(x);
        if (buf[i] != x) { if (!bad) first = i; bad++; }
    }
    Serial.printf("  %-22s 불일치 %u/%u%s\n", tag, (unsigned)bad, (unsigned)N,
                  bad ? "" : "  (무결)");
    if (bad) Serial.printf("     첫 불일치 인덱스 %u\n", (unsigned)first);
    return bad == 0;
}

void setup()
{
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== GPIO16/17 이 PSRAM 것인가 ===");

    if (!psramFound()) { Serial.println("PSRAM 없음. 중단."); while (1) delay(1000); }
    Serial.printf("PSRAM 여유 %u KB\n",
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

    uint32_t *buf = (uint32_t *)heap_caps_malloc(N * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
    if (!buf) { Serial.println("PSRAM 할당 실패. 중단."); while (1) delay(1000); }

    const uint32_t SEED = 0xC0FFEE01;
    uint32_t x = SEED;
    for (size_t i = 0; i < N; i++) { x = lcg(x); buf[i] = x; }
    Serial.println("\n[1] 기준 (아무것도 건드리지 않음)");
    const bool base_ok = verify(buf, SEED, "256KB 패턴");
    if (!base_ok) {
        Serial.println("기준부터 깨진다 — PSRAM 자체가 불안정. 이후 결과는 무의미.");
        while (1) delay(1000);
    }

    // GPIO16 만
    Serial.println("\n[2] GPIO16 을 출력으로 1000회 토글");
    pinMode(16, OUTPUT);
    for (int i = 0; i < 1000; i++) { digitalWrite(16, i & 1); }
    const bool g16 = verify(buf, SEED, "토글 후");

    // 패턴 복구
    x = SEED; for (size_t i = 0; i < N; i++) { x = lcg(x); buf[i] = x; }

    // GPIO17 만
    Serial.println("\n[3] GPIO17 을 출력으로 1000회 토글");
    pinMode(17, OUTPUT);
    for (int i = 0; i < 1000; i++) { digitalWrite(17, i & 1); }
    const bool g17 = verify(buf, SEED, "토글 후");

    x = SEED; for (size_t i = 0; i < N; i++) { x = lcg(x); buf[i] = x; }

    // 둘 다 + 큰 PSRAM 접근을 섞는다 (실사용에 가깝게)
    Serial.println("\n[4] GPIO16+17 토글 + 동시에 PSRAM 읽기/쓰기");
    volatile uint32_t acc = 0;
    for (int i = 0; i < 1000; i++) {
        digitalWrite(16, i & 1);
        digitalWrite(17, (i >> 1) & 1);
        acc += buf[(i * 977) % N];
        buf[(i * 613) % N] ^= 0;          // 값은 그대로, 쓰기 트래픽만 발생
    }
    const bool both = verify(buf, SEED, "토글+접근 후");
    (void)acc;

    Serial.println("\n=== 판정 ===");
    if (g16 && g17 && both) {
        Serial.println("GPIO16/17 은 PSRAM 과 무관하다 — 자유롭게 쓸 수 있다.");
        Serial.println("→ PLEOS path_display_node 의 ST7789(kTftDc=16)를 이 보드에 올릴 수 있다.");
    } else {
        Serial.printf("GPIO%s 이 PSRAM 을 깨뜨린다 — 쓰면 안 된다.\n",
                      (!g16 && !g17) ? "16/17 둘 다" : (!g16 ? "16" : "17"));
        Serial.println("→ ST7789 의 DC 핀을 다른 GPIO 로 옮겨야 한다.");
    }
}

void loop() { delay(1000); }
