// 게임보이 프레임을 뽑아 stdout 으로 흘린다. 전자종이에 몇 번 갱신해야 하는지를
// 재기 위한 것이고, 판정 규칙은 파이썬 쪽에서 튜닝한다(컴파일 없이 바꿀 수 있게).
//
// 롬은 인자로 받는다. **레포에 넣지 않는다** — 상용 롬을 공개 저장소에 올리면 배포다.
// pokedex 쪽에 이미 세운 BYOR 방식과 같다.
//
//   gcc -O2 -std=c99 gb_dump.c -o gb_dump -lm
//   ./gb_dump "롬.gb" 600 6 > frames.bin
//     600 = 뽑을 프레임 수, 6 = 몇 프레임마다 하나씩 (6이면 10Hz)
//
// 출력: 프레임마다 160*144 바이트, 값 0~3 (게임보이 4단계 명암)
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ENABLE_LCD 1
#define ENABLE_SOUND 0
#define PEANUT_GB_HEADER_ONLY 0
#include "peanut_gb.h"

struct priv_t {
    uint8_t *rom;
    uint8_t *cart_ram;
};

static uint8_t fb[144][160];

static uint8_t rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
    return ((struct priv_t *)gb->direct.priv)->rom[addr];
}
static uint8_t ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
    return ((struct priv_t *)gb->direct.priv)->cart_ram[addr];
}
static void ram_write(struct gb_s *gb, const uint_fast32_t addr, const uint8_t v)
{
    ((struct priv_t *)gb->direct.priv)->cart_ram[addr] = v;
}
static void on_error(struct gb_s *gb, const enum gb_error_e e, const uint16_t addr)
{
    (void)gb;
    fprintf(stderr, "gb error %d @ %04x\n", (int)e, addr);
}
static void draw_line(struct gb_s *gb, const uint8_t *px, const uint_fast8_t line)
{
    (void)gb;
    for (int x = 0; x < 160; x++) fb[line][x] = px[x] & 3;
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: gb_dump rom.gb [n_out] [every]\n"); return 2; }
    const long n_out = (argc > 2) ? atol(argv[2]) : 600;
    const long every = (argc > 3) ? atol(argv[3]) : 6;

    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror("rom"); return 1; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    struct priv_t priv;
    priv.rom = malloc(sz);
    if (fread(priv.rom, 1, sz, f) != (size_t)sz) { fprintf(stderr, "short read\n"); return 1; }
    fclose(f);

    struct gb_s gb;
    enum gb_init_error_e ie = gb_init(&gb, rom_read, ram_read, ram_write, on_error, &priv);
    if (ie != GB_INIT_NO_ERROR) { fprintf(stderr, "gb_init %d\n", (int)ie); return 1; }
    const size_t save = gb_get_save_size(&gb);
    priv.cart_ram = calloc(1, save ? save : 1);
    gb_init_lcd(&gb, draw_line);
    fprintf(stderr, "rom %ldKB  save %zuB  mbc ok\n", sz / 1024, save);

    // 인트로를 넘기려면 입력이 필요하다. 사람이 누르는 리듬을 흉내내 START/A 를
    // 주기적으로 눌러준다 — 이 하네스의 목적은 "플레이 중 화면이 얼마나 자주 바뀌나" 라서
    // 완벽한 조작은 필요 없다.
    long frame = 0;
    for (long out = 0; out < n_out; ) {
        gb.direct.joypad = 0xFF;                       // 1 = 안 눌림
        const long t = frame % 120;
        if (t < 4)        gb.direct.joypad_bits.start = 0;
        else if (t < 8)   gb.direct.joypad_bits.a = 0;
        else if (t < 40)  gb.direct.joypad_bits.down = 0;   // 걸어다니게
        else if (t < 44)  gb.direct.joypad_bits.a = 0;
        else if (t < 76)  gb.direct.joypad_bits.right = 0;

        gb_run_frame(&gb);
        frame++;
        if (frame % every == 0) {
            fwrite(fb, 1, sizeof fb, stdout);
            out++;
        }
    }
    fprintf(stderr, "%ld 프레임 실행, %ld개 출력 (매 %ld프레임)\n", frame, n_out, every);
    return 0;
}
