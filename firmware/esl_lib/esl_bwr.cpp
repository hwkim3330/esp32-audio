#include "esl_bwr.h"

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <vector>

// ───────────────────────────────────────── 모델 표
// eigger/hass-gicisky 의 devices.py 를 옮겼다. 같은 296x128 이 다섯 변종이라
// 모델을 모르면 적색 평면 유무도, 페이로드 길이도 알 수 없다.
static const EslModel MODELS[] = {
  // id      model             w    h   red   4col  mx     my     rot cmp   cmp2  vmax
  { 0x00A0, "TFT 2.1\" BW",   250, 132, false,false,true,  false,  90, false,false, 2.9f },
  { 0x000B, "EPD 2.1\" BWR",  212, 104, true, false,true,  false, 270, false,false, 2.9f },
  { 0x010B, "EPD 2.1\" BWR",  250, 128, true, false,true,  false, 270, false,false, 2.9f },
  { 0x0028, "EPD 2.9\" BW",   296, 128, false,false,false, false,  90, false,false, 3.0f },
  { 0x0033, "EPD 2.9\" BWR",  296, 128, true, false,false, false,  90, false,false, 3.0f },
  { 0x002E, "EPD 2.9\" BWRY", 296, 128, true, true, false, false,  90, false,false, 3.0f },
  { 0x022B, "EPD 3.7\" BWR",  240, 416, true, false,true,  false, 180, true, false, 3.0f },
  { 0x004B, "EPD 4.2\" BWR",  400, 300, true, false,false, false,   0, false,false, 3.0f },
  { 0x004E, "EPD 4.2\" BWRY", 400, 300, true, true, false, false,   0, false,false, 3.0f },
  { 0x012B, "EPD 7.5\" BWR",  800, 480, true, false,false, true,    0, false,true,  3.0f },
  { 0x008B, "EPD 10.2\" BWR", 960, 640, true, false,false, false,   0, false,true,  3.2f },
};

const EslModel *esl_model(uint16_t id)
{
    for (const auto &m : MODELS) if (m.id == id) return &m;
    return nullptr;
}

static BLEUUID SVC_UUID((uint16_t)0xFEF0);
static BLEUUID CMD_UUID((uint16_t)0xFEF1);
static BLEUUID IMG_UUID((uint16_t)0xFEF2);

const char *esl_result_name(EslResult r)
{
    switch (r) {
    case ESL_OK:            return "성공";
    case ESL_ERR_CONNECT:   return "연결 실패";
    case ESL_ERR_SERVICE:   return "0xFEF0 없음(ESL 아님)";
    case ESL_ERR_CHAR:      return "FEF1/FEF2 없음";
    case ESL_ERR_NEGOTIATE: return "파트크기 협상 무응답";
    case ESL_ERR_TRANSFER:  return "전송 중 정지";
    case ESL_ERR_TIMEOUT:   return "시간 초과";
    case ESL_SKIPPED:       return "같은 내용 — 안 보냄";
    }
    return "알 수 없음";
}

// ───────────────────────────────────────── 탐색
static EslTag *s_out = nullptr;
static int s_max = 0, s_n = 0;

static bool is_candidate(BLEAdvertisedDevice &ad)
{
    const uint8_t *a = ad.getAddress().getNative();
    if (a[0] == 0xFF && a[1] == 0xFF) return true;
    for (int k = 0; k < ad.getServiceUUIDCount(); k++) {
        BLEUUID u = ad.getServiceUUID(k);
        if (u.bitSize() == 16 && u.getNative()->uuid.uuid16 == 0xFEF0) return true;
    }
    return false;
}

class ScanCb : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice ad) override {
        if (!s_out || s_n >= s_max) return;
        if (!is_candidate(ad)) return;
        const uint8_t *a = ad.getAddress().getNative();
        for (int i = 0; i < s_n; i++)
            if (!memcmp(s_out[i].addr, a, 6)) {   // 이미 본 태그면 RSSI 만 갱신
                if (ad.getRSSI() > s_out[i].rssi) s_out[i].rssi = (int8_t)ad.getRSSI();
                return;
            }
        EslTag &t = s_out[s_n];
        memset(&t, 0, sizeof t);
        memcpy(t.addr, a, 6);
        t.rssi = (int8_t)ad.getRSSI();
        if (ad.haveName()) {
            const String n = ad.getName();
            strncpy(t.name, n.c_str(), sizeof t.name - 1);
        }
        // 제조사 데이터: [회사ID LE16 = 0x5053][5바이트]
        //   device_id = ((d[4] << 8) | d[0]) & 0x3FFF
        //   battery   = d[1] / 10  (볼트)
        //   firmware  = (d[2] << 8) | d[3]
        // 표에 적힌 실측 예(2.9인치): 33 1D 81 01 40 → id 0x0033, 2.9V, fw 0x8101
        if (ad.haveManufacturerData()) {
            const String md = ad.getManufacturerData();
            const uint8_t *d = (const uint8_t *)md.c_str();
            if (md.length() >= 7 && d[0] == 0x53 && d[1] == 0x50) {
                const uint8_t *p = d + 2;
                t.have_mfg  = true;
                t.device_id = (uint16_t)(((p[4] << 8) | p[0]) & 0x3FFF);
                t.volts     = p[1] / 10.0f;
                t.firmware  = (uint16_t)((p[2] << 8) | p[3]);
                t.m         = esl_model(t.device_id);
            }
        }
        s_n++;
    }
};

int esl_scan(EslTag *out, int max_n, uint32_t secs)
{
    s_out = out; s_max = max_n; s_n = 0;
    BLEScan *sc = BLEDevice::getScan();
    sc->setAdvertisedDeviceCallbacks(new ScanCb(), true);
    sc->setActiveScan(true);              // 수동이면 이름이 안 온다
    sc->setInterval(80);
    sc->setWindow(60);
    sc->start(secs, false);
    sc->clearResults();
    sc->setAdvertisedDeviceCallbacks(nullptr);
    // 세기 순으로 정렬한다 — 가장 센 태그를 주 화면으로 쓴다.
    for (int i = 1; i < s_n; i++)
        for (int j = i; j > 0 && out[j].rssi > out[j - 1].rssi; j--) {
            const EslTag t = out[j]; out[j] = out[j - 1]; out[j - 1] = t;
        }
    return s_n;
}

// ───────────────────────────────────────── 픽셀 배치
static void esl_pack_planes(const GFXcanvas1 &bw, const GFXcanvas1 &red,
                            uint8_t *buf, int n_plane);

size_t esl_payload_len(const EslTag &t)
{
    // 모델을 모르면 BWR 로 가정한다. 이 방 태그의 다수가 그쪽이다.
    const bool has_red = t.m ? t.m->red : true;
    return (size_t)ESL_W * (ESL_H / 8) * (has_red ? 2 : 1);
}

size_t esl_pack(const EslTag &t, const GFXcanvas1 &bw, const GFXcanvas1 &red,
                uint8_t *buf, bool force_bw)
{
    // BW 모델에 두 평면을 보내면 태그가 기대량의 두 배를 받는다. 증상은
    // "화면이 나오긴 하는데 완벽하지 않다" 로만 보여 원인이 감춰진다(실기 관측).
    const int n_plane = (force_bw || (t.m && !t.m->red)) ? 1 : 2;
    esl_pack_planes(bw, red, buf, n_plane);
    return (size_t)ESL_W * (ESL_H / 8) * n_plane;
}

static void esl_pack_planes(const GFXcanvas1 &bw, const GFXcanvas1 &red,
                            uint8_t *buf, int n_plane)
{
    const uint8_t *pbw  = bw.getBuffer();
    const uint8_t *pred = red.getBuffer();
    const int bpr = (ESL_W + 7) / 8;
    size_t o = 0;

    // 참조 구현이 검증한 규칙 그대로 둔다. 값을 바꾸면 화면이 반전되거나 뒤집힌다.
    //   MIRROR_X=true, BW: 1=흰색, RED: 1=적색(반전)
    for (int pl = 0; pl < n_plane; pl++) {
        const uint8_t *src = pl ? pred : pbw;
        const bool one_is_white = (pl == 0);
        for (int xx = 0; xx < ESL_W; xx++) {
            const int x = ESL_W - 1 - xx;                 // MIRROR_X
            for (int bi = 0; bi < ESL_H / 8; bi++) {
                uint8_t b = 0;
                for (int bit = 0; bit < 8; bit++) {
                    const int y = bi * 8 + bit;
                    const bool one =
                        (src[y * bpr + (x >> 3)] & (0x80 >> (x & 7))) != 0;
                    const bool set = one_is_white ? one : !one;
                    if (set) b |= (1 << (7 - bit));
                }
                buf[o++] = b;
            }
        }
    }
}

// ───────────────────────────────────────── 전송
static volatile bool     g_got01 = false, g_got05 = false, g_hasAck = false;
static volatile uint8_t  g_st05 = 0;
static volatile uint32_t g_ack = 0;
static volatile uint16_t g_partMsgSize = 244;

static void on_notify(BLERemoteCharacteristic *, uint8_t *p, size_t n, bool)
{
    if (!p || !n) return;
    if (p[0] == 0x01 && n >= 3) {
        g_partMsgSize = (uint16_t)(p[1] | (p[2] << 8));
        g_got01 = true;
    } else if (p[0] == 0x05) {
        g_st05 = (n >= 2) ? p[1] : 0xFF;
        g_hasAck = (n >= 6);
        if (g_hasAck)
            g_ack = (uint32_t)p[2] | ((uint32_t)p[3] << 8)
                  | ((uint32_t)p[4] << 16) | ((uint32_t)p[5] << 24);
        g_got05 = true;
    }
}

// 클라이언트를 하나만 만들어 계속 재사용한다. **삭제하지 않는다.**
//
// 처음에는 전송마다 createClient/delete 를 했는데 매번 죽었다:
//   assert failed: xQueueGenericSend queue.c:937
//   BLEClient::gattClientEventHandler → ESP_GATTC_DISCONNECT_EVT
//     → m_semaphoreOpenEvt.give(ESP_GATT_IF_NONE)   (BLEClient.cpp:535)
// 끊김 이벤트가 delete 뒤에 도착해서 해제된 객체의 세마포어를 건드린다.
// Bluedroid 스택이 클라이언트 포인터를 계속 들고 있으므로 우리가 먼저 지우면 안 된다.
// 객체 하나가 새는 셈이지만 4대를 돌려 쓰는 데 문제가 없다.
static BLEClient *s_cli = nullptr;

// 태그별 "마지막에 보낸 내용" 의 해시. 전자종이는 이미지 유지에 전력이 0 이므로
// 같은 그림을 다시 굽는 것은 순수한 낭비다 — 배터리도, 3.5초도, 잔상도.
// 32비트 FNV-1a 를 쓴다. 충돌해서 갱신을 한 번 빠뜨릴 확률은 40억분의 1이고,
// 다음 사건에서 다시 그려지므로 대가가 없다.
#define ESL_MEMO 8
static struct { uint8_t addr[6]; uint32_t hash; bool used; } s_memo[ESL_MEMO];

static uint32_t fnv1a(const uint8_t *p, size_t n)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}

static int memo_find(const uint8_t *a)
{
    for (int i = 0; i < ESL_MEMO; i++)
        if (s_memo[i].used && !memcmp(s_memo[i].addr, a, 6)) return i;
    return -1;
}

void esl_forget(const uint8_t addr[6])
{
    if (!addr) { memset(s_memo, 0, sizeof s_memo); return; }
    const int i = memo_find(addr);
    if (i >= 0) s_memo[i].used = false;
}

bool esl_warmup(const uint8_t addr[6])
{
    // 아무 내용이나 보내는 것이 아니라 **연결과 협상만** 한다. 첫 세션의 나쁜
    // 파트 크기를 여기서 소진시키는 것이 목적이다.
    if (!s_cli) s_cli = BLEDevice::createClient();
    BLEAddress a(const_cast<uint8_t *>(addr));
    bool ok = s_cli->connect(a);
    if (ok) {
        delay(200);
        if (BLERemoteService *svc = s_cli->getService(SVC_UUID))
            if (BLERemoteCharacteristic *cmd = svc->getCharacteristic(CMD_UUID)) {
                cmd->registerForNotify(on_notify, true, true);
                if (BLERemoteDescriptor *d = cmd->getDescriptor(BLEUUID((uint16_t)0x2902))) {
                    uint8_t v[2] = { 0x01, 0x00 };
                    d->writeValue(v, 2, true);
                    delay(120);
                }
                g_got01 = false;
                uint8_t c1 = 0x01;
                cmd->writeValue(&c1, 1, true);
                const uint32_t t0 = millis();
                while (!g_got01 && millis() - t0 < 3000) delay(10);
            }
        s_cli->disconnect();
        delay(500);
    }
    return ok;
}

EslResult esl_upload(const uint8_t addr[6], const uint8_t *payload, size_t len,
                     uint32_t *ms_out, uint32_t *parts_out)
{
    const uint32_t t_start = millis();

    // 같은 내용이면 아예 연결하지 않는다. 가장 값싼 최적화이고 가장 크게 아낀다.
    const uint32_t hash = fnv1a(payload, len);
    const int mi = memo_find(addr);
    if (mi >= 0 && s_memo[mi].hash == hash) {
        if (ms_out) *ms_out = 0;
        if (parts_out) *parts_out = 0;
        return ESL_SKIPPED;
    }

    g_got01 = g_got05 = g_hasAck = false;
    g_partMsgSize = 244;

    if (!s_cli) s_cli = BLEDevice::createClient();
    BLEClient *cli = s_cli;
    // 이전 전송이 남긴 연결을 확실히 닫고, 스택이 끊김 이벤트를 처리할 틈을 준다.
    if (cli->isConnected()) { cli->disconnect(); delay(600); }

    BLEAddress a(const_cast<uint8_t *>(addr));
    bool ok = false;
    for (int r = 0; r < 3 && !ok; r++) {
        ok = cli->connect(a);
        if (!ok) delay(800);
    }
    if (!ok) return ESL_ERR_CONNECT;

    EslResult res = ESL_ERR_TRANSFER;
    delay(300);
    BLERemoteService *svc = cli->getService(SVC_UUID);
    if (!svc) { res = ESL_ERR_SERVICE; goto done; }
    {
        BLERemoteCharacteristic *cmd = svc->getCharacteristic(CMD_UUID);
        BLERemoteCharacteristic *img = svc->getCharacteristic(IMG_UUID);
        if (!cmd || !img) { res = ESL_ERR_CHAR; goto done; }

        // 어느 고리가 끊겼는지 눈으로 봐야 한다. 여기가 안 되면 태그는 영원히
        // 0x01 에 답하지 않고, 증상은 "협상 무응답" 하나로만 보인다.
        Serial.printf("\n      [특성] CMD notify=%d indicate=%d / IMG write=%d\n",
                      cmd->canNotify(), cmd->canIndicate(), img->canWrite());
        BLERemoteDescriptor *d = cmd->getDescriptor(BLEUUID((uint16_t)0x2902));
        Serial.printf("      [CCCD] %s\n", d ? "있음" : "없음(구독 불가)");

        // subscribe() 는 이 코어에서 NimBLE 백엔드 전용 선언이라 Bluedroid 에서는
        // 존재하지 않는다(컴파일 에러로 확인). Bluedroid 경로는 이것뿐이다.
        cmd->registerForNotify(on_notify, true, true);
        delay(150);
        if (d) {
            uint8_t v[2] = { 0x01, 0x00 };
            d->writeValue(v, 2, true);
            delay(150);
            // 태그가 실제로 구독 상태인지 CCCD 를 되읽어 확인한다. 쓰기가 조용히
            // 실패하면 증상은 "협상 무응답" 하나로만 보여 원인이 감춰진다.
            const String rb = d->readValue();
            Serial.printf("      [CCCD] 되읽기 %u바이트", (unsigned)rb.length());
            for (unsigned i = 0; i < rb.length(); i++) Serial.printf(" %02X", (uint8_t)rb[i]);
            Serial.println();
        }

        // 파트 크기 협상. 두 번까지 시도한다 —
        // 부팅 후 첫 전송에서 태그가 20 을 주는 일이 있다(MTU 교환이 끝나기 전의
        // 보수적인 값으로 보인다). 그러면 파트가 592개가 되고 21초가 걸린다.
        // 같은 태그가 두 번째 회차에서는 244 를 줘서 40파트/3.2초였다(실측).
        uint32_t t0 = millis();
        for (int attempt = 0; attempt < 2; attempt++) {
            g_got01 = false;
            uint8_t c1 = 0x01;
            cmd->writeValue(&c1, 1, true);
            const uint32_t ta = millis();
            while (!g_got01 && millis() - ta < 3000) {
                if (!cli->isConnected()) { res = ESL_ERR_CONNECT; goto done; }
                delay(10);
            }
            if (!g_got01) { res = ESL_ERR_NEGOTIATE; goto done; }
            if (g_partMsgSize >= 64) break;       // 쓸만한 값이면 그대로 간다
            Serial.printf("      [협상] %u 는 너무 작다 — 다시 물어본다\n",
                          (unsigned)g_partMsgSize);
            delay(400);
        }
        t0 = millis();

        // 태그가 알려준 파트 크기를 쓴다. 헤더 4바이트(파트번호)를 뺀 만큼이 데이터다.
        size_t dataSz = (g_partMsgSize >= 8) ? (size_t)(g_partMsgSize - 4) : 240;
        if (dataSz > 240) dataSz = 240;     // 참조 구현의 상한. 넘기면 태그가 끊는다.
        // 태그마다 다른 값을 준다. 실측으로 한 대가 20을 줘서 파트가 592개(21.9초)가
        // 됐고 다른 대는 244를 줘서 40개(3.8초)였다. 같은 모델인데 5배 차이다.
        Serial.printf("      [협상] 파트메시지 %u → 데이터 %u바이트\n",
                      (unsigned)g_partMsgSize, (unsigned)dataSz);
        const uint32_t parts = (uint32_t)((len + dataSz - 1) / dataSz);
        if (parts_out) *parts_out = parts;

        uint8_t c2[8] = { 0x02, (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF),
                          (uint8_t)((len >> 16) & 0xFF), (uint8_t)((len >> 24) & 0xFF),
                          0, 0, 0 };
        cmd->writeValue(c2, sizeof c2, false);
        delay(100);
        uint8_t c3 = 0x03;
        cmd->writeValue(&c3, 1, false);

        std::vector<uint8_t> chunk;
        uint32_t cur = UINT32_MAX, last_tx = 0, stalls = 0;
        auto send_part = [&](uint32_t part) {
            const size_t off = (size_t)part * dataSz;
            if (off >= len) return;
            const size_t n = (dataSz < len - off) ? dataSz : (len - off);
            chunk.resize(4 + n);
            chunk[0] = part & 0xFF; chunk[1] = (part >> 8) & 0xFF;
            chunk[2] = (part >> 16) & 0xFF; chunk[3] = (part >> 24) & 0xFF;
            memcpy(&chunk[4], payload + off, n);
            img->writeValue(chunk.data(), chunk.size(), false);
            last_tx = millis();
        };
        send_part(0);

        while (true) {
            if (!cli->isConnected()) { res = ESL_ERR_CONNECT; goto done; }
            if (g_got05) {
                g_got05 = false; stalls = 0;
                if (g_st05 == 0x08) { res = ESL_OK; goto done; }
                if (g_st05 != 0x00) { res = ESL_ERR_TRANSFER; goto done; }
                if (g_hasAck) {
                    const uint32_t ack = g_ack;
                    if (ack >= parts) { res = ESL_OK; goto done; }
                    // 같은 파트를 다시 요구하면 이미 보냈으므로 무시한다.
                    if (ack != cur) { cur = ack; send_part(ack); }
                }
            }
            // 태그가 조용하면 마지막 파트를 다시 보낸다. 20번 헛돌면 포기한다.
            if (!chunk.empty() && millis() - last_tx > 1500) {
                img->writeValue(chunk.data(), chunk.size(), false);
                last_tx = millis();
                if (++stalls > 20) { res = ESL_ERR_TRANSFER; goto done; }
            }
            if (millis() - t_start > 90000) { res = ESL_ERR_TIMEOUT; goto done; }
            delay(5);
        }
    }
done:
    // 성공했을 때만 기억한다. 실패한 내용을 기억하면 다음에 건너뛰어 영영 안 보낸다.
    if (res == ESL_OK) {
        int i = memo_find(addr);
        if (i < 0) for (int k = 0; k < ESL_MEMO; k++) if (!s_memo[k].used) { i = k; break; }
        if (i < 0) i = 0;          // 자리가 없으면 가장 앞을 밀어낸다
        memcpy(s_memo[i].addr, addr, 6);
        s_memo[i].hash = hash;
        s_memo[i].used = true;
    }
    if (cli->isConnected()) cli->disconnect();
    delay(600);   // 끊김 이벤트가 처리될 틈. 짧게 두면 다음 연결과 겹친다.
    if (ms_out) *ms_out = millis() - t_start;
    return res;
}
