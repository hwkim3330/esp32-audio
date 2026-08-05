// LED 탐색 — 어느 GPIO 가 보드의 LED(D4/D5)를 켜는가.
//
// 왜 필요한가:
//
//  1. GPIO22 를 앰프 인에이블로 가정했는데, 근거가 공장 펌웨어 부팅 로그의
//     "GPIO[22]| OutputEn: 1" 한 줄뿐이었다. 출력으로 쓴다는 것만 알고 용도는 추측이다.
//     이게 LED 라면 스피커가 안 울리는 이유가 설명된다.
//
//  2. 이 보드에는 화면이 없어서 동작 여부를 알 방법이 없다는 게 실제 문제였다.
//     LED 를 찾으면 그게 상태 표시기가 된다.
//
// 사용법: 굽고 보드를 보면서, 빨간불이 깜빡이는 것이 몇 번째 단계인지 세면 된다.
// 각 단계는 3초이고 5Hz 로 깜빡인다. 전체 순환을 반복하므로 놓쳐도 다시 볼 수 있다.
// 단계 사이에 1초 전체 소등이 있어서 경계가 구분된다.

#include <Arduino.h>

// 후보 핀. 안전한 것만 넣었다:
//   - GPIO34~39 는 입력 전용이라 제외
//   - GPIO32/33 은 I2C(AC101) 라 제외 — 코덱 통신을 깨뜨리면 안 된다
//   - GPIO0/25/26/27 은 I2S 라 제외
//   - GPIO6~11 은 플래시라 제외 (건드리면 즉시 죽는다)
//
// GPIO18/19/23/13 은 버튼 후보이기도 하다. 버튼이라면 열린 상태에서 출력으로
// 구동해도 문제없지만, 그 버튼을 누른 채로 이 테스트를 돌리면 GND 로 단락된다.
// 테스트 중에는 버튼을 누르지 말 것.
static const int CAND[] = { 22, 21, 19, 18, 23, 13, 2, 4 };
static const int N_CAND = sizeof(CAND) / sizeof(CAND[0]);

static const uint32_t STEP_MS  = 3000;   // 단계 길이
static const uint32_t GAP_MS   = 1000;   // 단계 사이 소등
static const uint32_t BLINK_MS = 100;    // 5Hz (100ms on, 100ms off)

static void all_off()
{
    for (int i = 0; i < N_CAND; i++) {
        pinMode(CAND[i], OUTPUT);
        digitalWrite(CAND[i], LOW);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== LED 탐색 ===");
    Serial.printf("후보 %d개를 3초씩 5Hz 로 깜빡입니다. 단계 사이 1초 소등.\n", N_CAND);
    Serial.println("빨간불(D5)이 깜빡이는 단계 번호를 세어 주세요.");
    Serial.println("※ 테스트 중에는 버튼을 누르지 마세요.");
    Serial.println("※ 액티브 로우 LED 는 '소등 구간에 켜지는' 것으로 보입니다 — 그것도 알려주세요.\n");
    all_off();
}

void loop()
{
    static uint32_t cycle = 0;
    cycle++;
    Serial.printf("─── 순환 %lu ───\n", (unsigned long)cycle);

    for (int i = 0; i < N_CAND; i++) {
        // 소등 구간. 액티브 로우 LED 는 여기서 켜져 보인다 —
        // 그것도 정보이므로 사용자에게 알려 달라고 했다.
        all_off();
        Serial.printf("  (소등 %lums)\n", (unsigned long)GAP_MS);
        delay(GAP_MS);

        Serial.printf("  단계 %d/%d : GPIO%d 깜빡임 %lums\n",
                      i + 1, N_CAND, CAND[i], (unsigned long)STEP_MS);
        const uint32_t t_end = millis() + STEP_MS;
        while (millis() < t_end) {
            digitalWrite(CAND[i], HIGH);
            delay(BLINK_MS);
            digitalWrite(CAND[i], LOW);
            delay(BLINK_MS);
        }
    }
}
