// ESL 태그 탐색 — 이 보드에 없는 "화면" 을 밖에서 구한다.
//
// 이 프로젝트 내내 발목을 잡은 것은 화면이 없다는 점이었다. CSI 채널 일치율도,
// 추론 지연도, 대역 점수도 전부 시리얼로만 나왔다. 시리얼은 PC 가 붙어 있어야 하고
// 그러면 "PC 에서는 하지 마라" 는 전제와 어긋난다.
//
// Gicisky/PICKSMART BLE 전자선반라벨(ESL)이 그 구멍을 메운다. 296x128 BWR 이고
// BLE 로 이미지를 받는다. 프로토콜은 atc1441 이 리버스엔지니어링해 공개했다:
//   서비스 0xFEF0 / CMD 0xFEF1 / IMG 0xFEF2
//
// 여기서 확인하는 것은 단 하나다 — **그 태그가 실제로 전파에 잡히는가.**
// 잡히지 않으면 뒤의 모든 계획이 무의미하다. 그래서 드라이버보다 스캐너가 먼저다.
//
// 같이 확인하는 것:
//   - 0xFEF0 을 광고하는 기기 (= 확실한 ESL)
//   - FF:FF 로 시작하는 MAC (사용자 레포의 태그가 FF:FF:92:95:75:78 이었다)
//   - 이름에 ESL/PICKSMART/Gicisky 가 들어간 기기
//   - 그 외 전부 (다른 이잉크가 섞여 있을 수 있다)

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define MAX_DEV 96

struct Dev {
    uint8_t  addr[6];
    char     name[24];
    int8_t   rssi_min, rssi_max;
    uint16_t n_seen;
    bool     has_fef0;
    bool     ff_prefix;
    uint8_t  mfg[8];
    uint8_t  mfg_len;
    uint8_t  n_svc;
    uint16_t svc16[4];
};

static Dev devs[MAX_DEV];
static int n_dev = 0;
static uint32_t n_adv = 0;

static int find_or_add(const uint8_t *a)
{
    for (int i = 0; i < n_dev; i++)
        if (!memcmp(devs[i].addr, a, 6)) return i;
    if (n_dev >= MAX_DEV) return -1;
    Dev &d = devs[n_dev];
    memset(&d, 0, sizeof d);
    memcpy(d.addr, a, 6);
    d.rssi_min = 127; d.rssi_max = -127;
    // 사용자 레포의 태그 MAC 이 FF:FF:92:95:75:78 이었다. FF:FF 는 이 태그 계열의
    // 표식으로 보이는데, 근거가 그 한 대뿐이므로 "표식" 이 아니라 "단서" 로만 쓴다.
    d.ff_prefix = (a[0] == 0xFF && a[1] == 0xFF);
    return n_dev++;
}

class Cb : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice ad) override {
        n_adv++;
        const uint8_t *a = ad.getAddress().getNative();
        const int i = find_or_add(a);
        if (i < 0) return;
        Dev &d = devs[i];
        d.n_seen++;
        const int8_t r = (int8_t)ad.getRSSI();
        if (r < d.rssi_min) d.rssi_min = r;
        if (r > d.rssi_max) d.rssi_max = r;

        if (ad.haveName() && !d.name[0]) {
            const String n = ad.getName();
            strncpy(d.name, n.c_str(), sizeof d.name - 1);
        }
        if (ad.haveManufacturerData() && !d.mfg_len) {
            const String m = ad.getManufacturerData();
            d.mfg_len = (uint8_t)((m.length() > sizeof d.mfg) ? sizeof d.mfg : m.length());
            memcpy(d.mfg, m.c_str(), d.mfg_len);
        }
        // 광고에 실린 16비트 서비스 UUID 들을 모은다. 0xFEF0 이 있으면 ESL 확정이다.
        for (int k = 0; k < ad.getServiceUUIDCount(); k++) {
            BLEUUID u = ad.getServiceUUID(k);
            if (u.bitSize() != 16) continue;
            const uint16_t v = u.getNative()->uuid.uuid16;
            if (v == 0xFEF0) d.has_fef0 = true;
            bool dup = false;
            for (int j = 0; j < d.n_svc; j++) if (d.svc16[j] == v) dup = true;
            if (!dup && d.n_svc < 4) d.svc16[d.n_svc++] = v;
        }
    }
};

static bool looks_esl(const Dev &d)
{
    if (d.has_fef0 || d.ff_prefix) return true;
    for (const char *k : { "ESL", "esl", "PICKSMART", "Picksmart", "Gicisky",
                           "GICISKY", "EPD", "epaper", "E-Paper", "Paper" })
        if (strstr(d.name, k)) return true;
    return false;
}

static void report(void)
{
    Serial.printf("\n=== 광고 %lu건, 기기 %d대 ===\n", (unsigned long)n_adv, n_dev);

    int n_esl = 0;
    Serial.println("\n[ESL 후보]");
    for (int i = 0; i < n_dev; i++) {
        const Dev &d = devs[i];
        if (!looks_esl(d)) continue;
        n_esl++;
        Serial.printf("  %02X:%02X:%02X:%02X:%02X:%02X  %4d..%4ddBm  %3u회  %-22s",
                      d.addr[0], d.addr[1], d.addr[2], d.addr[3], d.addr[4], d.addr[5],
                      d.rssi_max, d.rssi_min, d.n_seen, d.name[0] ? d.name : "(이름없음)");
        if (d.has_fef0)   Serial.print("  FEF0");
        if (d.ff_prefix)  Serial.print("  FF:FF");
        Serial.println();
    }
    if (!n_esl) Serial.println("  없음");

    Serial.println("\n[그 밖의 기기 — 이름이 있는 것만]");
    for (int i = 0; i < n_dev; i++) {
        const Dev &d = devs[i];
        if (looks_esl(d) || !d.name[0]) continue;
        Serial.printf("  %02X:%02X:%02X:%02X:%02X:%02X  %4ddBm  %-22s",
                      d.addr[0], d.addr[1], d.addr[2], d.addr[3], d.addr[4], d.addr[5],
                      d.rssi_max, d.name);
        for (int j = 0; j < d.n_svc; j++) Serial.printf(" %04X", d.svc16[j]);
        Serial.println();
    }

    // 이름도 서비스도 없는 기기는 대부분 랜덤화 MAC 을 쓰는 휴대폰이다.
    int anon = 0;
    for (int i = 0; i < n_dev; i++)
        if (!looks_esl(devs[i]) && !devs[i].name[0]) anon++;
    Serial.printf("\n이름 없는 기기 %d대 (대개 MAC 랜덤화한 휴대폰)\n", anon);

    Serial.printf("\n판정: ESL 후보 %d대. ", n_esl);
    if (n_esl >= 1)
        Serial.println("드라이버를 붙일 수 있다. 다음은 연결해서 0xFEF0 서비스 확인.");
    else
        Serial.println("태그가 광고를 안 한다 — 배터리를 넣거나 깨워야 할 수 있다.");
}

void setup()
{
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== ESL 태그 탐색 ===");
    Serial.println("Gicisky/PICKSMART 296x128 BWR: 서비스 0xFEF0, CMD 0xFEF1, IMG 0xFEF2");
    Serial.println("찾는 것: 0xFEF0 광고 / FF:FF MAC / 이름에 ESL·PICKSMART·EPD\n");

    BLEDevice::init("");
    BLEScan *sc = BLEDevice::getScan();
    sc->setAdvertisedDeviceCallbacks(new Cb(), true);
    // 능동 스캔. 수동이면 스캔 응답(이름이 거기 실린다)을 못 받는다.
    sc->setActiveScan(true);
    sc->setInterval(80);
    sc->setWindow(60);

    for (int round = 1; round <= 4; round++) {
        Serial.printf("[스캔 %d/4] 15초...\n", round);
        sc->start(15, false);
        sc->clearResults();
        report();
    }
    Serial.println("\n탐색 종료. 15초마다 다시 돈다.");
}

void loop()
{
    BLEScan *sc = BLEDevice::getScan();
    sc->start(15, false);
    sc->clearResults();
    report();
}
