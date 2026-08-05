// CSI 로거 — 학습 데이터를 모은다.
//
// 음성 데이터는 Supertonic 으로 공짜로 합성했지만 전파는 실측이 필요하다. 그래서
// 사람이 여러 상태로 있는 동안 CSI 를 모아야 하고, 그 라벨을 누가 찍어야 한다.
//
// 라벨을 **보드의 버튼**으로 찍는다. 사람이 PC 앞에 앉아 있을 필요가 없다 —
// 보드 옆에서 상태를 만들면서 그 상태에 해당하는 버튼을 누르면 된다.
//   KEY1 = 빈 방   KEY2 = 왼쪽   KEY3 = 가운데   KEY4 = 오른쪽
//   KEY5 = 이동 중  KEY6 = 라벨 없음(폐기)
//
// 왜 라벨이 '위치' 인가:
//   CSI 는 AP↔보드 사이 전파 경로를 보므로 사람이 어디 서 있느냐에 따라 패턴이
//   완전히 달라진다. 그게 약점이면서 기능이다 — 기하가 바뀌면 모델이 안 통하지만,
//   같은 기하 안에서는 위치를 구분할 수 있다. 마이크 2개가 죽어서 포기했던 좌/우
//   구분을 이걸로 되찾는다.
//
//   단, ESP32 는 안테나가 하나라 진짜 도래각(AoA)은 못 낸다. 위치별 지문이고
//   학습한 위치·기하에서만 통한다. 보드를 옮기면 다시 모아야 한다.
//
// 출력은 바이너리 프레임이다. 텍스트로 흘리면 대역폭이 3배가 되고 파싱이 느려진다.
// 921600 baud 에서 프레임 136바이트 × 270Hz = 37KB/s 로 여유가 있다.
//
// 프레임 형식 (little-endian):
//   uint16 magic 0xC511
//   uint16 seq
//   uint8  label      0..5, 0xFF = 라벨 없음
//   int8   rssi
//   uint8  n_sc       서브캐리어 수
//   uint8  flags      bit0 = first_word_invalid
//   uint32 millis
//   int8   data[2*n_sc]   (imag, real) 쌍

#include <WiFi.h>
#include <esp_wifi.h>

#define MAX_SC   128
#define MAGIC    0xC511

static const int N_KEYS = 6;
static const int KEY_PIN[N_KEYS] = { 19, 23, 18, 5, 36, 13 };
static bool key_ok[N_KEYS] = { false };
static volatile uint8_t cur_label = 0xFF;      // 시작은 라벨 없음
static const char *LABEL_NAME[N_KEYS] = {
    "빈 방", "왼쪽", "가운데", "오른쪽", "이동 중", "라벨 없음"
};

// 콜백에서 채우고 메인 루프에서 내보낸다. 콜백은 WiFi 태스크라 길면 안 된다.
typedef struct {
    uint16_t seq;
    uint8_t  label, n_sc, flags;
    int8_t   rssi;
    uint32_t ms;
    int8_t   data[2 * MAX_SC];
    uint16_t bytes;
} frame_t;

// 큐 깊이. 8 로는 CSI 버스트와 채널 탐색의 delay() 동안 넘친다(실측 dropped=259).
// frame_t 가 약 270B 라 32개도 8.6KB 뿐이다.
#define QN 32
static frame_t   q[QN];
static volatile uint8_t q_w = 0, q_r = 0;
static volatile uint32_t dropped = 0, total = 0;
static uint16_t seq_ctr = 0;

static void IRAM_ATTR csi_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf || info->len < 8) return;
    // total 은 큐 검사보다 먼저 센다. 뒤에 두면 큐가 찬 동안 total 이 안 늘어나서
    // 채널 탐색이 '트래픽' 대신 '큐 용량' 을 재게 된다 — 실측에서 세 채널이 모두
    // 똑같이 14 로 나와 채널 선택이 무의미해졌다.
    total++;
    const uint8_t nxt = (uint8_t)((q_w + 1) % QN);
    if (nxt == q_r) { dropped++; return; }      // 큐가 찼다 — 버린다

    frame_t *f = &q[q_w];
    int nb = info->len;
    if (nb > 2 * MAX_SC) nb = 2 * MAX_SC;
    f->seq   = seq_ctr++;
    f->label = cur_label;
    f->n_sc  = (uint8_t)(nb / 2);
    f->flags = info->first_word_invalid ? 1 : 0;
    f->rssi  = info->rx_ctrl.rssi;
    f->ms    = millis();
    f->bytes = (uint16_t)nb;
    for (int i = 0; i < nb; i++) f->data[i] = info->buf[i];
    q_w = nxt;
}

static void keys_setup()
{
    for (int k = 0; k < N_KEYS; k++)
        pinMode(KEY_PIN[k], (KEY_PIN[k] == 36 || KEY_PIN[k] == 5) ? INPUT : INPUT_PULLUP);
    // 부팅 시 LOW 면 배선이 의심스럽다 — 그 키는 쓰지 않는다 (mic_node 에서 겪었다)
    for (int k = 0; k < N_KEYS; k++) key_ok[k] = (digitalRead(KEY_PIN[k]) == HIGH);
}

static void keys_poll()
{
    static uint32_t last = 0;
    static bool down[N_KEYS] = { false };
    if (millis() - last < 40) return;
    last = millis();
    for (int k = 0; k < N_KEYS; k++) {
        if (!key_ok[k]) continue;
        const bool d = (digitalRead(KEY_PIN[k]) == LOW);
        if (d && !down[k]) cur_label = (k == 5) ? 0xFF : (uint8_t)k;
        down[k] = d;
    }
}

// 채널을 2초씩 2회 훑어 합계로 고른다. 1초 1회는 트래픽 버스트에 속는다 —
// 실측에서 같은 환경인데 한 번은 채널 6(59프레임/1.2초), 다음 부팅엔 채널 1이
// 뽑혀 수집률이 13Hz 로 떨어졌다.
static int8_t pick_busiest_channel()
{
    const int8_t ch[3] = { 1, 6, 11 };
    uint32_t acc[3] = { 0, 0, 0 };
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < 3; i++) {
            esp_wifi_set_channel((uint8_t)ch[i], WIFI_SECOND_CHAN_NONE);
            const uint32_t t0 = total;
            delay(2000);
            acc[i] += total - t0;
            q_r = q_w;                          // 탐색 중 프레임은 버린다
        }
    }
    int bi = 0;
    for (int i = 1; i < 3; i++) if (acc[i] > acc[bi]) bi = i;
    Serial.printf("#SCAN ch1=%lu ch6=%lu ch11=%lu\n",
                  (unsigned long)acc[0], (unsigned long)acc[1], (unsigned long)acc[2]);
    return ch[bi];
}

void setup()
{
    // 바이너리를 흘리므로 빠른 baud 가 필요하다. 136B × 270Hz = 37KB/s.
    Serial.begin(921600);
    delay(400);
    keys_setup();

    WiFi.mode(WIFI_MODE_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_filter(nullptr);

    wifi_csi_config_t cfg = {};
    cfg.lltf_en = true; cfg.htltf_en = true; cfg.stbc_htltf2_en = true;
    cfg.ltf_merge_en = true; cfg.channel_filter_en = true;
    cfg.manu_scale = false; cfg.shift = 0; cfg.dump_ack_en = false;
    esp_wifi_set_csi_config(&cfg);
    esp_wifi_set_csi_rx_cb(csi_cb, nullptr);
    esp_wifi_set_csi(true);

    const int8_t chan = pick_busiest_channel();
    esp_wifi_set_channel((uint8_t)chan, WIFI_SECOND_CHAN_NONE);

    // 헤더는 텍스트로 한 번만 (수집 스크립트가 이 줄을 찾아 동기를 잡는다)
    Serial.printf("#CSI_LOGGER v1 chan=%d keys_ok=", chan);
    for (int k = 0; k < N_KEYS; k++) Serial.print(key_ok[k] ? '1' : '0');
    Serial.println();
    Serial.println("#LABELS K1=빈방 K2=왼쪽 K3=가운데 K4=오른쪽 K5=이동중 K6=라벨없음");
    Serial.flush();
}

void loop()
{
    keys_poll();

    // 큐를 비운다
    while (q_r != q_w) {
        frame_t *f = &q[q_r];
        uint8_t hdr[12];
        hdr[0] = MAGIC & 0xFF;   hdr[1] = MAGIC >> 8;
        hdr[2] = f->seq & 0xFF;  hdr[3] = f->seq >> 8;
        hdr[4] = f->label;
        hdr[5] = (uint8_t)f->rssi;
        hdr[6] = f->n_sc;
        hdr[7] = f->flags;
        hdr[8]  = f->ms & 0xFF;        hdr[9]  = (f->ms >> 8) & 0xFF;
        hdr[10] = (f->ms >> 16) & 0xFF; hdr[11] = (f->ms >> 24) & 0xFF;
        Serial.write(hdr, sizeof hdr);
        Serial.write((const uint8_t *)f->data, f->bytes);
        q_r = (uint8_t)((q_r + 1) % QN);
    }

    // 상태를 주기적으로 텍스트로 (수집 스크립트가 '#' 줄은 건너뛴다)
    static uint32_t last = 0;
    if (millis() - last > 2000) {
        last = millis();
        Serial.printf("#STAT total=%lu dropped=%lu label=%s\n",
                      (unsigned long)total, (unsigned long)dropped,
                      (cur_label == 0xFF) ? "라벨없음" : LABEL_NAME[cur_label]);
    }
}
