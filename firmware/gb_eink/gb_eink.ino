// 게임보이를 ESP32 에서 돌리고 전자종이 한 장에 그린다.
//
// 이 조합의 제약을 먼저 못 박아야 설계가 선다.
//
// **깜박임은 우리가 없앨 수 없다.** 태그에 픽셀을 넘기면 패널 구동은 태그의 MCU 가
// 자기 웨이브폼으로 한다(잔상을 지우려고 화면을 여러 번 반전시킨다). BLE 프로토콜은
// 이미지를 건네는 것뿐이라 부분 갱신을 고를 수 없다. 그래서 줄일 수 있는 것은 **횟수**다.
//
// **전송도 느리다.** 실측 3.0~5.5초, 그중 병목이 바이트가 아니라 연결 수립과 패널
// 리프레시다(페이로드를 절반으로 줄였을 때 오히려 느렸다: BWR 4074ms 대 흑백만 4327ms).
//
// 그래서 프레임을 보내지 않는다. **장면이 바뀔 때만** 보낸다. 그리고 그 사이의 움직임을
// 버리지 않으려고 프레임을 누적해서 **궤적(잔상)을 비트맵 안에** 만든다 — 패널의 잔상에
// 기대는 것이 아니라 우리가 그리는 것이므로 재현 가능하고 조절 가능하다.
//
// 롬은 `tools/gb_eink/embed_rom.py` 가 만든 rom_data.h 에 있고 **커밋되지 않는다.**
//
// 빌드:
//   python3 tools/gb_eink/embed_rom.py ~/Downloads/"Pokemon - Red Version (K).gb"
//   arduino-cli compile --fqbn \
//     'esp32:esp32:esp32:PSRAM=enabled,FlashSize=4M,PartitionScheme=huge_app,CPUFreq=240' \
//     firmware/gb_eink
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <BLEDevice.h>

#include "esl_bwr.h"
#include "rom_data.h"

#define ENABLE_LCD 1
#define ENABLE_SOUND 0
#include "peanut_gb.h"

// ── 대상 태그. 여러 장을 돌리지 않는다 — 한 장이 화면이고, 그 한 장의 갱신 예산을 쓴다.
// 앵커 측정에서 가장 조용하고 가까웠던 태그다(-50.4dBm, 편차 2.29dB).
static const uint8_t TAG_SUFFIX = 0x78;

// ── 버튼 6개 → 게임보이 8개. 두 개가 모자라므로 길게 누르기로 겹친다.
#define N_KEYS 6
static const int KEY_PIN[N_KEYS] = { 36, 13, 19, 23, 18, 5 };
//   K1 위 / K2 아래 / K3 왼 / K4 오 / K5 = A / K6 = B
//
// START·SELECT 는 **동시 누르기**로 낸다. 길게 누르기로 겹치면 안 된다 — A/B 는 게임에서
// 가장 많이 쓰는 키이고 누르고 있어야 하는 경우도 있는데(달리기, 연타), 그때마다
// START 가 튀어나간다. 코드는 단발 입력을 방해하지 않는다.
//
//   K5+K6 = START      K3+K4 = SELECT (왼+오는 게임에서 같이 눌릴 일이 없다)
//   K1+K2 = 강제 갱신  (위+아래도 같이 눌릴 일이 없다)
//
// 코드 판정에는 짧은 유예를 둔다. 두 키를 물리적으로 정확히 같은 순간에 누를 수 없으므로,
// 한 키가 먼저 들어와도 CHORD_MS 안에 짝이 오면 코드로 본다.
#define CHORD_MS 60

// ── 갱신 정책. 전자종이 한 장을 태우지 않으려면 여기가 전부다.
#define PUSH_MIN_GAP_MS 8000     // 이보다 자주는 절대 안 보낸다
#define PUSH_DIFF_PCT   12       // 보낼 만한 변화(누적 이미지의 픽셀 %)
#define TRAIL_DECAY     24       // 궤적이 사라지는 속도 (0=영구, 255=즉시)

static struct gb_s gb;
static uint8_t *rom_ram = nullptr;      // MBC5+RAM+BAT — 세이브 램
static uint8_t *fb = nullptr;           // 160x144, 값 0~3
static uint8_t *trail = nullptr;        // 160x144 누적 밝기(궤적)
static GFXcanvas1 *cbw = nullptr, *cred = nullptr;
static U8G2_FOR_ADAFRUIT_GFX u8g2;
static uint8_t esl_buf[ESL_BYTES];
static EslTag tag;
static bool have_tag = false;

static uint32_t frames = 0, pushes = 0, last_push = 0;
static uint8_t *last_sent = nullptr;    // 마지막으로 보낸 1비트 이미지(142x128 팩)

// ─────────────────────────────────────────── 에뮬레이터 연결

static uint8_t rom_read(struct gb_s *g, const uint_fast32_t addr)
{
    (void)g;
    return pgm_read_byte(&GB_ROM[addr]);       // 롬은 플래시에 있다(메모리 매핑)
}
static uint8_t ram_read(struct gb_s *g, const uint_fast32_t addr)
{
    (void)g;
    return rom_ram ? rom_ram[addr] : 0xFF;
}
static void ram_write(struct gb_s *g, const uint_fast32_t addr, const uint8_t v)
{
    (void)g;
    if (rom_ram) rom_ram[addr] = v;
}
static void on_error(struct gb_s *g, const enum gb_error_e e, const uint16_t addr)
{
    (void)g;
    Serial.printf("[gb] 오류 %d @ %04x\n", (int)e, addr);
}
static void draw_line(struct gb_s *g, const uint8_t *px, const uint_fast8_t line)
{
    (void)g;
    uint8_t *row = fb + (size_t)line * 160;
    for (int x = 0; x < 160; x++) row[x] = px[x] & 3;
}

// ─────────────────────────────────────────── 궤적 누적
//
// 프레임을 보낼 수 없으니 그 사이의 움직임을 버리게 된다. 그래서 어두운 픽셀이
// 지나간 자리를 누적해 두고, 보낼 때 그것을 함께 그린다. 결과는 "여러 순간이 겹친
// 한 장" 이다 — 장노출 사진과 같은 원리이고, 8초에 한 장이라는 제약에서 오히려
// 정보가 늘어난다(어디를 지나왔는지가 보인다).
static void trail_accum(void)
{
    for (size_t i = 0; i < 160u * 144u; i++) {
        const uint8_t dark = (uint8_t)(fb[i] * 85);       // 0~3 → 0~255
        if (dark > trail[i]) trail[i] = dark;             // 어두운 쪽을 남긴다
        else trail[i] = (trail[i] > TRAIL_DECAY) ? (uint8_t)(trail[i] - TRAIL_DECAY) : 0;
    }
}

// ─────────────────────────────────────────── 전자종이 합성
//
// 게임보이 160x144 는 296x128 에 그대로 안 들어간다(높이가 넘는다). 최근접으로
// 142x128 로 줄여 왼쪽에 두고, 남는 154x128 을 상태판으로 쓴다. 확대·보간은 하지 않는다 —
// 1비트 화면에서 보간은 격자무늬만 만든다.
#define GX 142
#define GY 128

static inline uint8_t gb_at(int gx, int gy)
{
    const int sx = gx * 160 / GX, sy = gy * 144 / GY;
    return trail[(size_t)sy * 160 + sx];
}

// 1비트로 내리는 방법: 순서 디더(4x4 베이어). 오차확산은 프레임마다 무늬가 흔들려서
// 전자종이에서 "지지직" 거린다 — 정지 화면에는 순서 디더가 맞다.
static const uint8_t BAYER4[4][4] = {
    {  15, 135,  45, 165 },
    { 195,  75, 225, 105 },
    {  60, 180,  30, 150 },
    { 240, 120, 210,  90 },
};

static void compose(void)
{
    cbw->fillScreen(1);          // 1 = 흰색
    cred->fillScreen(1);
    for (int y = 0; y < GY; y++)
        for (int x = 0; x < GX; x++)
            if (gb_at(x, y) > BAYER4[y & 3][x & 3]) cbw->drawPixel(x, y, 0);

    cbw->drawFastVLine(GX + 3, 0, GY, 0);
    u8g2.begin(*cbw);
    u8g2.setFontMode(1);
    u8g2.setForegroundColor(0);
    u8g2.setBackgroundColor(1);
    u8g2.setFont(u8g2_font_helvB10_tf);
    u8g2.setCursor(GX + 10, 16);
    u8g2.print(GB_ROM_TITLE);
    u8g2.setFont(u8g2_font_5x7_tf);
    int y = 34;
    u8g2.setCursor(GX + 10, y); u8g2.printf("frames %lu", (unsigned long)frames); y += 10;
    u8g2.setCursor(GX + 10, y); u8g2.printf("pushes %lu", (unsigned long)pushes); y += 10;
    u8g2.setCursor(GX + 10, y);
    u8g2.printf("%.2f V", tag.have_mfg ? tag.volts : 0.0f); y += 14;
    u8g2.setCursor(GX + 10, y); u8g2.print("K1-4 dpad  K5 A  K6 B"); y += 10;
    u8g2.setCursor(GX + 10, y); u8g2.print("K5+K6 START"); y += 10;
    u8g2.setCursor(GX + 10, y); u8g2.print("K3+K4 SELECT"); y += 10;
    u8g2.setCursor(GX + 10, y); u8g2.print("K1+K2 push now"); y += 14;
    u8g2.setCursor(GX + 10, y);
    u8g2.printf("gap>=%ds diff>=%d%%", PUSH_MIN_GAP_MS / 1000, PUSH_DIFF_PCT);
}

// 보낼 만한 변화인가. 합성된 흑백 평면을 마지막으로 보낸 것과 비교한다 —
// 게임 프레임이 아니라 **실제로 태그에 나갈 픽셀**을 비교해야 의미가 있다.
static int diff_pct(void)
{
    const uint8_t *cur = cbw->getBuffer();
    const size_t n = (size_t)ESL_W * (ESL_H / 8);
    if (!last_sent) return 100;
    size_t bits = 0;
    for (size_t i = 0; i < n; i++) {
        uint8_t x = (uint8_t)(cur[i] ^ last_sent[i]);
        while (x) { bits += x & 1; x >>= 1; }
    }
    return (int)(bits * 100 / (ESL_W * ESL_H));
}

static bool push_now(const char *why)
{
    if (!have_tag) return false;
    const size_t len = esl_pack(tag, *cbw, *cred, esl_buf, false);
    uint32_t ms = 0, parts = 0;
    // 링크를 붙들고 쓴다. 같은 태그에 계속 보내므로 연결 수립을 매번 할 이유가 없다.
    const EslResult r = esl_upload(tag.addr, esl_buf, len, &ms, &parts, true);
    Serial.printf("[eink] %s → %s  %lums  %lu파트\n", why, esl_result_name(r),
                  (unsigned long)ms, (unsigned long)parts);
    if (r != ESL_OK) return false;
    pushes++;
    last_push = millis();
    if (!last_sent) last_sent = (uint8_t *)malloc((size_t)ESL_W * (ESL_H / 8));
    if (last_sent) memcpy(last_sent, cbw->getBuffer(), (size_t)ESL_W * (ESL_H / 8));
    return true;
}

// ─────────────────────────────────────────── 버튼
static bool down[N_KEYS] = { false };
static bool force_push = false;

static void keys_poll(void)
{
    static uint32_t last = 0;
    if (millis() - last < 10) return;
    last = millis();

    bool d[N_KEYS];
    for (int k = 0; k < N_KEYS; k++) d[k] = (digitalRead(KEY_PIN[k]) == LOW);

    // 코드가 성립한 뒤에는 그 두 키의 단발 입력을 내지 않는다. 안 그러면 START 를
    // 누를 때 A 와 B 가 같이 들어간다.
    static uint32_t first_ms[N_KEYS] = { 0 };
    for (int k = 0; k < N_KEYS; k++) {
        if (d[k] && !down[k]) first_ms[k] = millis();
        down[k] = d[k];
    }
    auto chord = [&](int a, int b) {
        if (!d[a] || !d[b]) return false;
        const uint32_t ga = millis() - first_ms[a], gb2 = millis() - first_ms[b];
        return (ga < CHORD_MS + 400) && (gb2 < CHORD_MS + 400);
    };
    const bool c_start  = chord(4, 5);
    const bool c_select = chord(2, 3);
    const bool c_push   = chord(0, 1);

    static bool push_latch = false;
    if (c_push && !push_latch) { push_latch = true; force_push = true; }
    if (!c_push) push_latch = false;

    gb.direct.joypad = 0xFF;                    // 1 = 안 눌림
    if (c_start)  { gb.direct.joypad_bits.start = 0;  return; }
    if (c_select) { gb.direct.joypad_bits.select = 0; return; }
    if (c_push)   return;                       // 갱신용 코드는 게임에 넣지 않는다

    if (d[0]) gb.direct.joypad_bits.up = 0;
    if (d[1]) gb.direct.joypad_bits.down = 0;
    if (d[2]) gb.direct.joypad_bits.left = 0;
    if (d[3]) gb.direct.joypad_bits.right = 0;
    if (d[4]) gb.direct.joypad_bits.a = 0;
    if (d[5]) gb.direct.joypad_bits.b = 0;
}

// ─────────────────────────────────────────── 부팅

void setup(void)
{
    Serial.begin(115200);
    delay(300);
    Serial.printf("\n=== 게임보이 + 전자종이 한 장 ===\n%s  롬 %dKB  세이브램 %dKB\n",
                  GB_ROM_TITLE, GB_ROM_LEN / 1024, GB_CART_RAM_LEN / 1024);

    for (int k = 0; k < N_KEYS; k++) pinMode(KEY_PIN[k], INPUT_PULLUP);

    // **프레임 버퍼와 궤적은 내부 DRAM 에 둔다.** 처음에 둘 다 PSRAM 에 뒀는데 그게
    // 가장 뜨거운 경로다 — draw_line 이 스캔라인마다 쓰고 trail_accum 이 프레임마다
    // 23K 픽셀을 읽고 쓴다(60fps 면 2.7MB/s). PSRAM 은 내부 SRAM 보다 한참 느려서
    // 에뮬 속도를 그대로 깎는다. 둘 다 23KB 씩이고 내부에 260KB 가 남아 있다.
    // 세이브램(32KB)만 PSRAM 으로 보낸다 — 접근이 드물다.
    fb = (uint8_t *)heap_caps_calloc(160 * 144, 1, MALLOC_CAP_INTERNAL);
    trail = (uint8_t *)heap_caps_calloc(160 * 144, 1, MALLOC_CAP_INTERNAL);
    rom_ram = (uint8_t *)ps_calloc(GB_CART_RAM_LEN ? GB_CART_RAM_LEN : 1, 1);
    cbw = new GFXcanvas1(ESL_W, ESL_H);
    cred = new GFXcanvas1(ESL_W, ESL_H);
    if (!fb || !trail || !rom_ram || !cbw || !cred) {
        Serial.println("메모리 할당 실패 — PSRAM 이 꺼져 있나");
        return;
    }

    const enum gb_init_error_e ie =
        gb_init(&gb, rom_read, ram_read, ram_write, on_error, nullptr);
    if (ie != GB_INIT_NO_ERROR) { Serial.printf("gb_init 실패 %d\n", (int)ie); return; }
    gb_init_lcd(&gb, draw_line);
    Serial.printf("gb_init 통과. 프레임/궤적 내부 DRAM, 세이브램 %dKB PSRAM\n",
                  GB_CART_RAM_LEN / 1024);

    BLEDevice::init("");
    EslTag found[ESL_MAX_TAG];
    const int n = esl_scan(found, ESL_MAX_TAG, 6);
    Serial.printf("[eink] 태그 %d대\n", n);
    for (int i = 0; i < n; i++) {
        Serial.printf("  :%02X %s %.2fV\n", found[i].addr[5], found[i].name,
                      found[i].volts);
        if (found[i].addr[5] == TAG_SUFFIX) { tag = found[i]; have_tag = true; }
    }
    if (!have_tag && n > 0) { tag = found[0]; have_tag = true;
        Serial.printf("[eink] :%02X 를 못 찾아 :%02X 로 대체\n", TAG_SUFFIX, tag.addr[5]); }
    Serial.println(have_tag ? "[eink] 준비됨" : "[eink] 태그 없음 — 화면 없이 돈다");
}

void loop(void)
{
    keys_poll();
    gb_run_frame(&gb);
    frames++;
    trail_accum();

    // 갱신 판정. 하한을 먼저 보는 이유: 하한을 안 넘겼으면 합성 비용조차 쓸 이유가 없다.
    const bool due = (millis() - last_push) >= PUSH_MIN_GAP_MS;
    if (force_push || due) {
        compose();
        const int d = diff_pct();
        if (force_push) { push_now("강제(K6 2초)"); force_push = false; }
        else if (d >= PUSH_DIFF_PCT) {
            char why[40];
            snprintf(why, sizeof why, "변화 %d%%", d);
            push_now(why);
        } else {
            // 보내지 않는다. 하한을 다시 세지 않으면 매 프레임 합성하게 된다.
            last_push = millis() - PUSH_MIN_GAP_MS / 2;
        }
    }

    if (frames % 600 == 0)
        Serial.printf("[gb] %lu 프레임, 갱신 %lu회, 여유 DRAM %uKB\n",
                      (unsigned long)frames, (unsigned long)pushes,
                      (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
}
