#include "web_ui.h"

#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>

WebSnap g_snap;

static WebServer *srv = nullptr;
static bool running = false;

// 페이지는 플래시에 그대로 둔다. SPIFFS 를 쓰면 파티션을 바꿔야 하고, 이 정도
// 크기는 PROGMEM 이 더 간단하다. 외부 CDN 은 쓰지 않는다 — AP 모드에는 인터넷이 없다.
static const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>CABIN NODE — 2.4GHz radar</title>
<style>
:root{--bg:#0b0d10;--fg:#e6edf3;--dim:#8b949e;--hot:#ff5f56;--ok:#3fb950;--line:#21262d}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:13px/1.5 ui-monospace,Menlo,monospace}
header{padding:10px 14px;border-bottom:1px solid var(--line);display:flex;
  justify-content:space-between;align-items:baseline;gap:12px;flex-wrap:wrap}
h1{font-size:14px;margin:0;letter-spacing:.08em}
.sub{color:var(--dim);font-size:11px}
main{display:grid;grid-template-columns:1fr;gap:12px;padding:12px}
@media(min-width:900px){main{grid-template-columns:1fr 1fr}}
section{border:1px solid var(--line);border-radius:6px;padding:10px;min-width:0}
h2{font-size:11px;margin:0 0 8px;color:var(--dim);letter-spacing:.1em;text-transform:uppercase}
canvas{width:100%;display:block;background:#000;border-radius:3px}
table{width:100%;border-collapse:collapse;font-size:12px}
td{padding:2px 4px;border-bottom:1px solid var(--line)}
td:last-child{text-align:right;font-variant-numeric:tabular-nums}
.big{font-size:26px;font-variant-numeric:tabular-nums}
.hot{color:var(--hot)}.ok{color:var(--ok)}.dim{color:var(--dim)}
.note{color:var(--dim);font-size:11px;margin-top:8px;line-height:1.6}
</style>
<header>
  <h1>CABIN NODE — 2.4 GHz PASSIVE RADAR</h1>
  <div class=sub id=hdr>connecting…</div>
</header>
<main>
  <section>
    <h2>CSI waterfall — 64 subcarriers x time</h2>
    <canvas id=wf height=220></canvas>
    <div class=note>측정값 그대로다. 가로가 서브캐리어(주파수), 위로 갈수록 최근.
      밝은 줄이 위로 흐르면 채널 응답이 바뀐 것 — 그게 무언가 움직였다는 뜻이다.</div>
  </section>
  <section>
    <h2>Room — board + 4 fixed anchors</h2>
    <canvas id=room height=220></canvas>
    <div class=note>앵커는 벽에 붙은 전자종이 태그다. 안 움직이고 MAC 도 안 바꾼다.
      링크가 굵고 붉어지면 그 <b>선분을 가로지른 것</b>이 있다는 뜻이다.
      자세 추정 논문들은 안테나가 한 곳에 모여 있는데 이쪽은 방에 퍼져 있다.</div>
  </section>
  <section>
    <h2>Band score</h2>
    <div class=big id=band>—</div>
    <canvas id=tr height=90></canvas>
    <table id=tbl></table>
  </section>
  <section>
    <h2>왜 사람 모형을 안 그리는가</h2>
    <div class=note>
      WiFi 로 자세를 뽑는 연구(DensePose From WiFi, Person-in-WiFi, WiPose)는 전부
      <b>3x3 MIMO — 링크 9개</b>를 쓴다. 이 보드는 <b>안테나가 하나</b>다.
      링크 하나는 방을 1차원으로 투영한 것이고, 거기서 3D 골격을 되살리는 것은
      모델 문제가 아니라 <b>정보이론적으로 불가능</b>하다. 그려 보이면 그건 측정이
      아니라 애니메이션이다.<br><br>
      대신 여기 있는 것은 전부 실측이다 — 워터폴은 CSI 원값, 방 도면의 링크 굵기는
      각 앵커 RSSI 의 표준편차 대비 편차(z), 점수는 그 최댓값이다.<br><br>
      <span id=verdict></span>
    </div>
  </section>
</main>
<script>
const WF = document.getElementById('wf'), RM = document.getElementById('room'),
      TR = document.getElementById('tr');
function fit(c){ c.width = c.clientWidth * devicePixelRatio; c.height = c.height; }
[WF,RM,TR].forEach(fit); addEventListener('resize', ()=>[WF,RM,TR].forEach(fit));

const wfx = WF.getContext('2d'), rmx = RM.getContext('2d'), trx = TR.getContext('2d');
const trend = [];

// 워터폴: 매 프레임 한 줄을 맨 아래 그리고 전체를 1px 위로 올린다.
function waterfall(amp, n){
  const w = WF.width, h = WF.height;
  const img = wfx.getImageData(0, 1, w, h - 1);
  wfx.putImageData(img, 0, 0);
  for (let x = 0; x < w; x++){
    const i = Math.min(n - 1, Math.floor(x / w * n));
    // -128..127 → 0..255. 파랑→노랑 램프. 회색조보다 변화가 눈에 띈다.
    const v = Math.max(0, Math.min(255, amp[i] + 128));
    const r = v, g = Math.floor(v * 0.85), b = Math.floor(255 - v * 0.8);
    wfx.fillStyle = `rgb(${r},${g},${b})`;
    wfx.fillRect(x, h - 1, 1, 1);
  }
}

function room(s){
  const w = RM.width, h = RM.height;
  rmx.clearRect(0, 0, w, h);
  const cx = w / 2, cy = h / 2;
  // 앵커를 원주에 균등 배치한다. 실제 위치는 모르므로 **위치를 주장하지 않는다** —
  // 링크의 상대적 세기만 보여주는 도식이다.
  const n = s.n_anchor || 0, R = Math.min(w, h) * 0.36;
  for (let k = 0; k < n; k++){
    const a = -Math.PI / 2 + k * 2 * Math.PI / n;
    const x = cx + R * Math.cos(a), y = cy + R * Math.sin(a);
    const z = s.anchor_z[k] || 0;
    const t = Math.min(z / 3, 1);
    rmx.strokeStyle = `rgb(${Math.floor(80 + 175 * t)},${Math.floor(200 - 140 * t)},${Math.floor(160 - 100 * t)})`;
    rmx.lineWidth = (1 + 5 * t) * devicePixelRatio;
    rmx.beginPath(); rmx.moveTo(cx, cy); rmx.lineTo(x, y); rmx.stroke();
    rmx.fillStyle = '#e6edf3';
    rmx.beginPath(); rmx.arc(x, y, 5 * devicePixelRatio, 0, 7); rmx.fill();
    rmx.font = `${11 * devicePixelRatio}px ui-monospace`;
    rmx.textAlign = 'center';
    rmx.fillText(`:${s.anchor_last[k].toString(16).padStart(2,'0')}  z${z.toFixed(1)}`,
                 x, y - 10 * devicePixelRatio);
    rmx.fillStyle = '#8b949e';
    rmx.fillText(`${s.anchor_rssi[k]}dBm`, x, y + 20 * devicePixelRatio);
  }
  rmx.fillStyle = '#3fb950';
  rmx.beginPath(); rmx.arc(cx, cy, 8 * devicePixelRatio, 0, 7); rmx.fill();
  rmx.fillStyle = '#e6edf3'; rmx.textAlign = 'center';
  rmx.fillText('ESP32', cx, cy + 24 * devicePixelRatio);
}

function graph(){
  const w = TR.width, h = TR.height;
  trx.clearRect(0, 0, w, h);
  const N = 240; while (trend.length > N) trend.shift();
  trx.strokeStyle = '#21262d'; trx.lineWidth = devicePixelRatio;
  const ty = h - h * (thr / 6);
  trx.setLineDash([4 * devicePixelRatio, 4 * devicePixelRatio]);
  trx.beginPath(); trx.moveTo(0, ty); trx.lineTo(w, ty); trx.stroke();
  trx.setLineDash([]);
  trx.strokeStyle = '#58a6ff'; trx.lineWidth = 1.5 * devicePixelRatio;
  trx.beginPath();
  trend.forEach((v, i) => {
    const x = i / N * w, y = h - h * (Math.min(v, 6) / 6);
    i ? trx.lineTo(x, y) : trx.moveTo(x, y);
  });
  trx.stroke();
}

let thr = 3;
async function tick(){
  try{
    const s = await (await fetch('/s', {cache:'no-store'})).json();
    thr = s.thresh;
    waterfall(s.amp, s.n_sc);
    room(s);
    trend.push(s.band); graph();
    const hot = s.band >= s.thresh;
    document.getElementById('band').innerHTML =
      `<span class="${hot?'hot':'ok'}">${s.band.toFixed(2)}</span>` +
      `<span class=dim> / thr ${s.thresh.toFixed(1)}</span>`;
    document.getElementById('hdr').textContent =
      `csi ${s.csi_hz}Hz  sc${s.n_sc}  |  ble ${s.n_ble}  |  boot ${s.boot_n}  ` +
      `up ${Math.floor(s.uptime_s/3600)}h${String(Math.floor(s.uptime_s/60)%60).padStart(2,'0')}m`;
    document.getElementById('tbl').innerHTML = [
      ['on-device inference', s.infer_ms + ' ms'],
      ['channel match', `${s.cls_hit}/${s.cls_tot} = ` +
        (s.cls_tot ? (100*s.cls_hit/s.cls_tot).toFixed(0) : '0') + '%  (random 33%)'],
      ['last class / cosine', `${s.last_cls} / ${s.last_score.toFixed(2)}`],
      ['ble advertisements', s.ble_adv],
      ['validation samples', `mark ${s.mark_n} / idle ${s.unmark_n}`],
      ["Cohen's d", s.cohen_d.toFixed(2)],
    ].map(r => `<tr><td class=dim>${r[0]}</td><td>${r[1]}</td></tr>`).join('');
    document.getElementById('verdict').innerHTML = s.mark_n < 10
      ? '<b>사람 감지는 아직 미검증이다.</b> 보드의 K1 을 한 번 누르고 30초 동안 ' +
        '앞을 왕복하면 보드가 마크/비마크 분포를 스스로 비교해 판정한다.'
      : (s.cohen_d >= 0.8
        ? `<b class=ok>검증됨 — 분리도 d = ${s.cohen_d.toFixed(2)}.</b> 이 센서는 사람을 본다.`
        : `<b class=hot>분리도 d = ${s.cohen_d.toFixed(2)} — 부족하다.</b> ` +
          '표본을 더 모으거나 임계값을 손봐야 한다.');
  }catch(e){ document.getElementById('hdr').textContent = 'disconnected — ' + e; }
}
setInterval(tick, 200); tick();
</script>
)HTML";

static void h_root(void) { srv->send_P(200, "text/html; charset=utf-8", PAGE); }

static void h_snap(void)
{
    // 손으로 만든다. ArduinoJson 을 넣으면 플래시가 커지고, 이 정도 구조에는 과하다.
    String j;
    j.reserve(1400);
    j += "{\"n_sc\":" + String(g_snap.n_sc) + ",\"amp\":[";
    for (int i = 0; i < g_snap.n_sc; i++) {
        if (i) j += ',';
        j += String((int)g_snap.amp[i]);
    }
    j += "],\"band\":" + String(g_snap.band, 2);
    j += ",\"thresh\":" + String(g_snap.thresh, 1);
    j += ",\"n_anchor\":" + String(g_snap.n_anchor);
    j += ",\"anchor_z\":[";
    for (int i = 0; i < g_snap.n_anchor; i++) { if (i) j += ','; j += String(g_snap.anchor_z[i], 2); }
    j += "],\"anchor_last\":[";
    for (int i = 0; i < g_snap.n_anchor; i++) { if (i) j += ','; j += String(g_snap.anchor_last[i]); }
    j += "],\"anchor_rssi\":[";
    for (int i = 0; i < g_snap.n_anchor; i++) { if (i) j += ','; j += String((int)g_snap.anchor_rssi[i]); }
    j += "],\"infer_ms\":" + String(g_snap.infer_ms);
    j += ",\"cls_hit\":" + String(g_snap.cls_hit);
    j += ",\"cls_tot\":" + String(g_snap.cls_tot);
    j += ",\"last_cls\":" + String(g_snap.last_cls);
    j += ",\"last_score\":" + String(g_snap.last_score, 2);
    j += ",\"n_ble\":" + String(g_snap.n_ble);
    j += ",\"ble_adv\":" + String(g_snap.ble_adv);
    j += ",\"csi_hz\":" + String(g_snap.csi_hz);
    j += ",\"uptime_s\":" + String(g_snap.uptime_s);
    j += ",\"boot_n\":" + String(g_snap.boot_n);
    j += ",\"mark_n\":" + String(g_snap.mark_n);
    j += ",\"unmark_n\":" + String(g_snap.unmark_n);
    j += ",\"cohen_d\":" + String(g_snap.cohen_d, 2);
    j += "}";
    srv->sendHeader("Cache-Control", "no-store");
    srv->send(200, "application/json", j);
}

void web_start(uint8_t ap_ch)
{
    if (running) return;
    // 프로미스큐어스를 끄지 않는다. AP 모드에서도 같은 채널의 프레임은 계속 들어오고,
    // 붙은 폰이 초당 다섯 번 폴링하면 그 트래픽이 곧 CSI 소스가 된다 —
    // 그동안 능동 프로빙으로도 1.2배가 한계였던 프레임률 문제가 여기서 풀린다.
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("CABIN-NODE", "cabinnode", ap_ch);
    delay(300);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ap_ch, WIFI_SECOND_CHAN_NONE);

    srv = new WebServer(80);
    srv->on("/", h_root);
    srv->on("/s", h_snap);
    srv->onNotFound([]() { srv->send(404, "text/plain", "no"); });
    srv->begin();
    running = true;
    Serial.printf("\n[웹] SoftAP \"CABIN-NODE\" (암호 cabinnode) 채널 %u\n", ap_ch);
    Serial.printf("[웹] 폰을 붙이고 http://%s 로 접속\n",
                  WiFi.softAPIP().toString().c_str());
    Serial.println("[웹] 채널 순환은 멈춘다 — 주파수 다이버시티를 이 모드에서는 포기한다.");
}

void web_stop(void)
{
    if (!running) return;
    if (srv) { srv->stop(); delete srv; srv = nullptr; }
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_MODE_STA);
    esp_wifi_set_promiscuous(true);
    running = false;
    Serial.println("[웹] 종료 — 채널 순환 복귀");
}

bool web_running(void) { return running; }
void web_poll(void) { if (running && srv) srv->handleClient(); }
