#include "web_ui.h"

#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>

#include "web_assets.h"     // gzip 한 uPlot (tools/embed_web.py 가 생성)

WebSnap g_snap;

// 시각을 받는다. RTC 도 NTP 도 없으므로 **접속한 폰이 유일한 시계 출처**다.
void (*web_set_clock)(uint32_t epoch) = nullptr;

static WebServer *srv = nullptr;
static bool running = false;

// 페이지는 플래시에 그대로 둔다. SPIFFS 를 쓰면 파티션을 바꿔야 하고, 이 정도
// 크기는 PROGMEM 이 더 간단하다. 외부 CDN 은 쓰지 않는다 — AP 모드에는 인터넷이 없다.
// 차트는 uPlot(MIT)을 gzip 해서 같이 넣었다: /u.js, /u.css.
static const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>CABIN NODE — 2.4GHz radar</title>
<link rel=stylesheet href=/u.css>
<style>
:root{--bg:#0b0d10;--pan:#0f1216;--fg:#e6edf3;--dim:#8b949e;--hot:#ff5f56;--ok:#3fb950;
  --warn:#d29922;--line:#21262d;--acc:#58a6ff}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:13px/1.5 ui-monospace,Menlo,monospace}
header{padding:10px 14px;border-bottom:1px solid var(--line);display:flex;
  justify-content:space-between;align-items:baseline;gap:12px;flex-wrap:wrap;
  position:sticky;top:0;background:var(--bg);z-index:5}
h1{font-size:14px;margin:0;letter-spacing:.08em}
.sub{color:var(--dim);font-size:11px}
main{display:grid;grid-template-columns:1fr;gap:12px;padding:12px}
@media(min-width:980px){main{grid-template-columns:1fr 1fr}.wide{grid-column:1/-1}}
section{border:1px solid var(--line);border-radius:6px;padding:10px;min-width:0;
  background:var(--pan)}
h2{font-size:11px;margin:0 0 8px;color:var(--dim);letter-spacing:.1em;text-transform:uppercase}
canvas.raw{width:100%;display:block;background:#000;border-radius:3px}
table{width:100%;border-collapse:collapse;font-size:12px}
td,th{padding:2px 4px;border-bottom:1px solid var(--line);text-align:left}
th{color:var(--dim);font-weight:400;font-size:11px}
td.n,th.n{text-align:right;font-variant-numeric:tabular-nums}
.big{font-size:30px;font-variant-numeric:tabular-nums;line-height:1.1}
.hot{color:var(--hot)}.ok{color:var(--ok)}.dim{color:var(--dim)}.warn{color:var(--warn)}
.note{color:var(--dim);font-size:11px;margin-top:8px;line-height:1.6}
.badge{display:inline-block;padding:1px 6px;border:1px solid var(--line);border-radius:3px;
  font-size:10px;letter-spacing:.06em}
.b-un{color:var(--warn);border-color:var(--warn)}
.b-ok{color:var(--ok);border-color:var(--ok)}
.u-legend{font-size:11px}
.cb{display:flex;gap:2px;margin-top:6px;align-items:center}
.cb i{flex:1;height:8px;display:block}
</style>
<header>
  <h1>CABIN NODE — 2.4 GHz PASSIVE RADAR</h1>
  <div class=sub id=hdr>connecting…</div>
</header>
<main>
  <section class=wide>
    <h2>Now</h2>
    <div class=big id=band>—</div>
    <div id=verdict class=note></div>
    <div id=live style="height:150px;margin-top:8px"></div>
    <div class=note>초당 다섯 번 폴링한 값이다. 점선이 임계값.
      <b>폴링 자체가 CSI 소스</b>이기도 하다 — 이 창을 열어두면 프레임률이 올라간다.</div>
  </section>

  <section>
    <h2>CSI deviation waterfall — 64 subcarriers x time</h2>
    <canvas class=raw id=wf height=220></canvas>
    <div class=cb><span class=dim>z 0</span><i id=cb0></i><i id=cb1></i><i id=cb2></i>
      <i id=cb3></i><i id=cb4></i><span class=dim>8+</span></div>
    <div class=note>가로가 서브캐리어(주파수), 위로 갈수록 최근. 색은 <b>기준선 대비 편차
      z</b> — 원시 진폭이 아니라 <b>판정에 쓰는 값</b>이다. 눈에 보이는 것과 점수가 같은
      출처여야 한다. 밝은 줄이 위로 흐르면 채널 응답이 바뀐 것이다.</div>
  </section>

  <section>
    <h2>Per-channel deviation (ch1 / ch6 / ch11)</h2>
    <div id=chan style="height:170px"></div>
    <div class=note>band score 는 이 세 값의 최댓값이다. 널 지점 때문에 최댓값을 쓴다 —
      어떤 주파수에서 안 변해도 다른 데서 변하면 무언가 움직인 것이다.
      기준선을 아직 못 배운 채널은 선이 끊긴다.</div>
  </section>

  <section class=wide>
    <h2>Trends — 72s / 12min / 1hr / 6hr</h2>
    <div id=tr0 style="height:120px"></div>
    <div id=tr1 style="height:120px;margin-top:6px"></div>
    <div id=tr2 style="height:120px;margin-top:6px"></div>
    <div id=tr3 style="height:120px;margin-top:6px"></div>
    <div class=note>칸마다 <b>최댓값</b>을 쌓는다. 평균을 쓰면 3초 지나간 사람이
      300초 칸에서 사라진다. 전자종이에는 6시간짜리만 그렸다 — 15분마다 갱신되는 종이에
      72초 그래프를 올리면 15분 전의 72초를 보여주는 셈이라서다. 웹에는 넷 다 있다.</div>
  </section>

  <section>
    <h2>Events</h2>
    <table id=evt></table>
    <div class=note>켜짐은 임계값, 꺼짐은 그 70%, 2초 미만은 잡음으로 버리고 5초 안에
      다시 오르면 같은 사건으로 이어붙인다. 빈 방 555초 실측 오경보 0건.</div>
  </section>

  <section>
    <h2>Events per hour (24 h)</h2>
    <div id=hist style="height:170px"></div>
    <div class=note id=histnote></div>
  </section>

  <section>
    <h2>Room — board + fixed anchors</h2>
    <canvas class=raw id=room height=220></canvas>
    <div class=note>앵커는 벽에 붙은 전자종이 태그다. 안 움직이고 MAC 도 안 바꾼다 —
      그래서 태그 하나가 보드↔태그 <b>경로 하나</b>를 대표한다. 링크가 굵고 붉어지면
      그 선분을 가로지른 것이 있다는 뜻이다. <b>배치는 도식이다</b> — 실제 방위는 모르므로
      원주에 균등 배치했고, 위치를 주장하지 않는다.</div>
  </section>

  <section>
    <h2>Validation &amp; device</h2>
    <table id=tbl></table>
    <div class=note>
      <b>왜 사람 모형을 안 그리는가.</b> WiFi 로 자세를 뽑는 연구(DensePose From WiFi,
      Person-in-WiFi, WiPose)는 전부 <b>3×3 MIMO — 링크 9개</b>를 쓴다. 이 보드는
      <b>안테나가 하나</b>다. 링크 하나는 방을 1차원으로 투영한 것이고, 거기서 3D 골격을
      되살리는 것은 모델 문제가 아니라 <b>정보이론적으로 불가능</b>하다. 그려 보이면
      그건 측정이 아니라 애니메이션이다. 여기 있는 것은 전부 실측이다.
    </div>
  </section>
</main>
<script src=/u.js></script>
<script>
const $ = id => document.getElementById(id);
const AX = {stroke:'#8b949e', grid:{stroke:'#21262d'}, ticks:{stroke:'#21262d'}};
const mk = (el, opts) => {
  // 초기 데이터는 **시리즈 수만큼** 줘야 한다. [[0],[0]] 을 고정으로 주면 시리즈가
  // 3개 이상인 차트(채널별 편차)가 로드 시점에 길이 불일치로 죽는다.
  const n = (opts.series||[]).length || 2;
  return new uPlot(Object.assign({
    width: el.clientWidth, height: el.clientHeight, padding:[8,8,0,0],
    axes:[AX,AX], cursor:{y:false}, legend:{live:true},
  }, opts), Array.from({length:n}, ()=>[0]), el);
};

// 임계값 선은 uPlot 훅으로 그린다. 시리즈로 넣으면 범례가 지저분해진다.
function thrHook(getThr){
  return {draw:[u=>{
    const t=getThr(); if(!(t>0)) return;
    const y=u.valToPos(t,'y',true);
    u.ctx.save(); u.ctx.strokeStyle='#d29922'; u.ctx.setLineDash([4,4]);
    u.ctx.beginPath(); u.ctx.moveTo(u.bbox.left,y); u.ctx.lineTo(u.bbox.left+u.bbox.width,y);
    u.ctx.stroke(); u.ctx.restore();
  }]};
}
let THR = 3;
const S = (n,c) => ({label:n, stroke:c, width:1.5, spanGaps:false});

const live = mk($('live'), {series:[{label:'t-'},S('band','#58a6ff')],
  scales:{y:{range:(u,min,max)=>[0,Math.max(6,max*1.1)]}}, hooks:thrHook(()=>THR)});
const chan = mk($('chan'), {series:[{label:'t-'},S('ch1','#58a6ff'),S('ch6','#3fb950'),
  S('ch11','#d29922')], scales:{y:{range:(u,mn,mx)=>[0,Math.max(6,mx*1.1)]}},
  hooks:thrHook(()=>THR)});
const TRN=['72 s','12 min','1 hr','6 hr'];
const trs = TRN.map((nm,i)=>mk($('tr'+i), {series:[{label:nm},S(nm,'#58a6ff')],
  scales:{y:{range:(u,mn,mx)=>[0,Math.max(6,mx*1.1)]}}, hooks:thrHook(()=>THR)}));
const hist = mk($('hist'), {series:[{label:'hour'},{label:'events',stroke:'#58a6ff',
  fill:'#58a6ff55', paths:uPlot.paths.bars({size:[0.75]})}]});
addEventListener('resize',()=>{
  [live,chan,...trs,hist].forEach(u=>u.setSize({width:u.root.parentNode.clientWidth-20,
    height:u.height}));
});

// ── 워터폴. 편차 z 를 색으로. 파랑(0) → 청록 → 노랑 → 빨강(8+).
const WF=$('wf'), wfx=WF.getContext('2d'), RM=$('room'), rmx=RM.getContext('2d');
function fit(c){ c.width=c.clientWidth*devicePixelRatio; }
[WF,RM].forEach(fit); addEventListener('resize',()=>[WF,RM].forEach(fit));
function ramp(z){
  const t=Math.max(0,Math.min(1,z/8));
  const r=Math.round(255*Math.min(1,Math.max(0,t*2-0.6)));
  const g=Math.round(255*Math.min(1,Math.max(0,1.4-Math.abs(t-0.55)*2.6)));
  const b=Math.round(255*Math.max(0,1-t*2.2));
  return `rgb(${r},${g},${b})`;
}
[0,2,4,6,8].forEach((z,i)=>{ const e=$('cb'+i); if(e) e.style.background=ramp(z); });
function waterfall(z,n){
  const w=WF.width,h=WF.height;
  wfx.putImageData(wfx.getImageData(0,1,w,h-1),0,0);
  for(let x=0;x<w;x++){
    const i=Math.min(n-1,Math.floor(x/w*n));
    wfx.fillStyle=ramp((z[i]||0)/8);   // sc_z 는 z*8 로 실려 온다
    wfx.fillRect(x,h-1,1,1);
  }
}
function room(s){
  const w=RM.width,h=RM.height; rmx.clearRect(0,0,w,h);
  const cx=w/2,cy=h/2,n=s.n_anchor||0,R=Math.min(w,h)*0.36,dp=devicePixelRatio;
  for(let k=0;k<n;k++){
    const a=-Math.PI/2+k*2*Math.PI/n, x=cx+R*Math.cos(a), y=cy+R*Math.sin(a);
    const t=Math.min((s.anchor_z[k]||0)/3,1);
    rmx.strokeStyle=ramp(t*8); rmx.lineWidth=(1+5*t)*dp;
    rmx.beginPath(); rmx.moveTo(cx,cy); rmx.lineTo(x,y); rmx.stroke();
    rmx.fillStyle='#e6edf3'; rmx.beginPath(); rmx.arc(x,y,5*dp,0,7); rmx.fill();
    rmx.font=`${11*dp}px ui-monospace`; rmx.textAlign='center';
    rmx.fillText(`:${(s.anchor_last[k]||0).toString(16).padStart(2,'0')}  z${(s.anchor_z[k]||0).toFixed(1)}`,
                 x,y-10*dp);
    rmx.fillStyle='#8b949e';
    rmx.fillText(`${s.anchor_rssi[k]}dBm sd${(s.anchor_sd[k]||0).toFixed(1)}`,x,y+20*dp);
  }
  rmx.fillStyle='#3fb950'; rmx.beginPath(); rmx.arc(cx,cy,8*dp,0,7); rmx.fill();
  rmx.fillStyle='#e6edf3'; rmx.textAlign='center'; rmx.fillText('ESP32',cx,cy+24*dp);
}

// ── 폴링. 빠른 것과 느린 것을 나눈다 — 추세 1KB 를 초당 다섯 번 받을 이유가 없다.
const N=600, xs=[], bs=[], c1=[], c6=[], c11=[];
let verified=false;
async function fast(){
  try{
    const s=await (await fetch('/s',{cache:'no-store'})).json();
    THR=s.thresh;
    waterfall(s.sc_z,s.n_sc);
    xs.push(xs.length?xs[xs.length-1]+1:0); bs.push(s.band);
    const ok=b=>(s.base_ok>>b)&1;
    c1.push(ok(0)?s.w_dev[0]:null); c6.push(ok(1)?s.w_dev[1]:null);
    c11.push(ok(2)?s.w_dev[2]:null);
    while(xs.length>N){xs.shift();bs.shift();c1.shift();c6.shift();c11.shift();}
    live.setData([xs,bs]); chan.setData([xs,c1,c6,c11]);
    // 검증 전에는 단정하지 않는다 — 전자종이와 같은 규칙이다.
    const hot=s.band>=s.thresh;
    $('band').innerHTML =
      `<span class="${hot?'hot':'ok'}">${verified?(hot?'MOTION NOW':'QUIET'):(hot?'BAND HIGH':'BAND LOW')}</span>`+
      `<span class=dim style="font-size:16px"> ${s.band.toFixed(2)} / thr ${s.thresh.toFixed(1)}</span>`;
    $('hdr').innerHTML=`csi ${s.csi_hz}Hz sc${s.n_sc} · infer ${s.infer_ms}ms · `+
      `ch-match ${s.cls_tot?(100*s.cls_hit/s.cls_tot).toFixed(0):0}% `+
      `<span class=dim>(random 33%)</span>`;
  }catch(e){ $('hdr').textContent='disconnected — '+e; }
}
async function slow(){
  try{
    const s=await (await fetch('/h',{cache:'no-store'})).json();
    verified=!!s.verified;
    room(s);
    s.trend.forEach((arr,i)=>{
      const sec=s.scale_sec[i], x=arr.map((_,k)=>-(arr.length-1-k)*sec);
      trs[i].setData([x,arr.map(v=>v>0?v:null)]);
    });
    hist.setData([[...Array(24).keys()], s.hour_cnt]);
    $('histnote').innerHTML = s.have_clock
      ? '가로는 하루의 시각이다.'
      : '<b class=warn>시계 미설정</b> — 이 페이지를 열면 폰 시계를 보드에 준다. '+
        '그 전에는 가로축이 가동 시간 기준이다.';
    $('evt').innerHTML = '<tr><th>#</th><th class=n>ago</th><th class=n>dur</th>'+
      '<th class=n>peak</th><th class=n>link</th></tr>' + (s.events.length
      ? s.events.map((e,i)=>`<tr><td>${s.ev_total-i}</td>`+
          `<td class=n>${e.ago_s<3600?Math.round(e.ago_s/60)+'m':Math.round(e.ago_s/3600)+'h'}</td>`+
          `<td class=n>${e.dur_s}s</td><td class=n>${e.peak.toFixed(1)}</td>`+
          `<td class=n>:${e.hot_anchor.toString(16).padStart(2,'0')}</td></tr>`).join('')
      : '<tr><td colspan=5 class=dim>(none yet)</td></tr>');
    const badge = s.verified
      ? '<span class="badge b-ok">HUMAN-VERIFIED</span>'
      : '<span class="badge b-un">NOT HUMAN-VERIFIED</span>';
    $('verdict').innerHTML = badge + ' ' + (s.mark_n<10
      ? '<b>사람 감지는 아직 미검증이다.</b> 위 숫자는 전부 빈 방 기준선에서 나온 것이다. '+
        `보드의 K1 을 누르고 30초 왕복하면 보드가 스스로 판정한다 (표본 ${s.mark_n}/10).`
      : (s.verified
        ? `<b class=ok>검증됨 — 분리도 d = ${s.cohen_d.toFixed(2)}.</b> 이 센서는 사람을 본다.`
        : `<b class=hot>분리도 d = ${s.cohen_d.toFixed(2)} — 부족하다(0.8 필요).</b> `+
          '표본을 더 모으거나 임계값을 손봐야 한다.'));
    $('tbl').innerHTML=[
      ['on-device inference', s.infer_ms+' ms'],
      ['validation samples', `mark ${s.mark_n} / idle ${s.unmark_n}`],
      ["Cohen's d", s.cohen_d.toFixed(2)+(s.verified?' ✓':' (need 0.8)')],
      ['events total', s.ev_total],
      ['ble devices / adv', `${s.n_ble} / ${s.ble_adv}`],
      ['active probing', `${s.probe_tx} tx, ${s.probe_fail} fail`],
      ['uptime / boot', `${Math.floor(s.uptime_s/3600)}h`+
        `${String(Math.floor(s.uptime_s/60)%60).padStart(2,'0')}m / #${s.boot_n}`],
    ].map(r=>`<tr><td class=dim>${r[0]}</td><td class=n>${r[1]}</td></tr>`).join('');
  }catch(e){}
}
fetch('/t?e='+Math.floor(Date.now()/1000)).catch(()=>{});
setInterval(fast,200); setInterval(slow,5000); fast(); slow();
</script>
)HTML";

static void h_root(void) { srv->send_P(200, "text/html; charset=utf-8", PAGE); }

// gzip 자산. WebServer 가 바이트열을 그대로 흘려보내고 브라우저가 푼다.
static void h_gz(const uint8_t *gz, size_t len, const char *mime)
{
    srv->sendHeader("Content-Encoding", "gzip");
    srv->sendHeader("Cache-Control", "max-age=31536000, immutable");
    srv->send_P(200, mime, (PGM_P)gz, len);
}

// ── 빠른 스냅샷. 초당 다섯 번 받는다. 여기에 추세를 넣으면 그것도 초당 다섯 번 간다.
static void h_fast(void)
{
    String j;
    j.reserve(900);
    j = "{\"n_sc\":" + String(g_snap.n_sc) + ",\"sc_z\":[";
    for (int i = 0; i < g_snap.n_sc; i++) { if (i) j += ','; j += String((int)g_snap.sc_z[i]); }
    j += "],\"band\":" + String(g_snap.band, 2);
    j += ",\"thresh\":" + String(g_snap.thresh, 1);
    j += ",\"w_dev\":[";
    for (int i = 0; i < WS_N_WCH; i++) { if (i) j += ','; j += String(g_snap.w_dev[i], 2); }
    j += "],\"base_ok\":" + String(g_snap.base_ok);
    j += ",\"csi_hz\":" + String(g_snap.csi_hz);
    j += ",\"infer_ms\":" + String(g_snap.infer_ms);
    j += ",\"cls_hit\":" + String(g_snap.cls_hit);
    j += ",\"cls_tot\":" + String(g_snap.cls_tot);
    j += ",\"last_cls\":" + String(g_snap.last_cls);
    j += ",\"last_score\":" + String(g_snap.last_score, 2) + "}";
    srv->sendHeader("Cache-Control", "no-store");
    srv->send(200, "application/json", j);
}

// ── 느린 스냅샷. 5초마다. 전자종이 네 페이지가 갖고 있던 것 전부가 여기 있다.
static void h_hist(void)
{
    String j;
    j.reserve(5200);
    j = "{\"trend\":[";
    for (int s = 0; s < WS_N_SCALE; s++) {
        if (s) j += ',';
        j += '[';
        // 링 버퍼를 시간순(오래된 것 먼저)으로 펴서 보낸다 — 브라우저가 다시 풀 이유가 없다.
        const uint32_t tn = g_snap.trend_n[s];
        const int n = (tn < WS_TREND_N) ? (int)tn : WS_TREND_N;
        for (int i = 0; i < WS_TREND_N; i++) {
            if (i) j += ',';
            if (i < WS_TREND_N - n) { j += '0'; continue; }
            const int k = i - (WS_TREND_N - n);
            j += String(g_snap.trend[s][(tn - n + k) % WS_TREND_N], 2);
        }
        j += ']';
    }
    j += "],\"scale_sec\":[";
    for (int s = 0; s < WS_N_SCALE; s++) { if (s) j += ','; j += String(g_snap.scale_sec[s]); }
    j += "],\"hour_cnt\":[";
    for (int i = 0; i < 24; i++) { if (i) j += ','; j += String(g_snap.hour_cnt[i]); }
    j += "],\"events\":[";
    for (int i = 0; i < g_snap.n_events; i++) {
        if (i) j += ',';
        const WebEvent &e = g_snap.events[i];
        j += "{\"ago_s\":" + String(e.ago_s) + ",\"dur_s\":" + String(e.dur_s)
           + ",\"peak\":" + String(e.peak, 2)
           + ",\"hot_anchor\":" + String(e.hot_anchor) + "}";
    }
    j += "],\"ev_total\":" + String(g_snap.ev_total);
    j += ",\"n_anchor\":" + String(g_snap.n_anchor);
    j += ",\"anchor_z\":[";
    for (int i = 0; i < g_snap.n_anchor; i++) { if (i) j += ','; j += String(g_snap.anchor_z[i], 2); }
    j += "],\"anchor_sd\":[";
    for (int i = 0; i < g_snap.n_anchor; i++) { if (i) j += ','; j += String(g_snap.anchor_sd[i], 2); }
    j += "],\"anchor_last\":[";
    for (int i = 0; i < g_snap.n_anchor; i++) { if (i) j += ','; j += String(g_snap.anchor_last[i]); }
    j += "],\"anchor_rssi\":[";
    for (int i = 0; i < g_snap.n_anchor; i++) { if (i) j += ','; j += String((int)g_snap.anchor_rssi[i]); }
    j += "],\"have_clock\":" + String(g_snap.have_clock);
    j += ",\"mark_n\":" + String(g_snap.mark_n);
    j += ",\"unmark_n\":" + String(g_snap.unmark_n);
    j += ",\"cohen_d\":" + String(g_snap.cohen_d, 2);
    j += ",\"verified\":" + String(g_snap.verified);
    j += ",\"n_ble\":" + String(g_snap.n_ble);
    j += ",\"ble_adv\":" + String(g_snap.ble_adv);
    j += ",\"probe_tx\":" + String(g_snap.probe_tx);
    j += ",\"probe_fail\":" + String(g_snap.probe_fail);
    j += ",\"infer_ms\":" + String(g_snap.infer_ms);
    j += ",\"uptime_s\":" + String(g_snap.uptime_s);
    j += ",\"boot_n\":" + String(g_snap.boot_n) + "}";
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
    srv->on("/s", h_fast);
    srv->on("/h", h_hist);
    srv->on("/u.js", []() { h_gz(UPLOT_JS_GZ, UPLOT_JS_GZ_LEN, "application/javascript"); });
    srv->on("/u.css", []() { h_gz(UPLOT_CSS_GZ, UPLOT_CSS_GZ_LEN, "text/css"); });
    srv->on("/t", []() {
        // 폰이 자기 시계를 준다. 보드에 RTC 가 없으므로 이것이 실제 시각의 유일한 출처다.
        if (srv->hasArg("e") && web_set_clock) {
            const uint32_t e = (uint32_t)strtoul(srv->arg("e").c_str(), nullptr, 10);
            if (e > 1700000000UL) { web_set_clock(e); srv->send(200, "text/plain", "ok"); return; }
        }
        srv->send(400, "text/plain", "no");
    });
    srv->onNotFound([]() { srv->send(404, "text/plain", "no"); });
    srv->begin();
    running = true;
    Serial.printf("\n[웹] SoftAP \"CABIN-NODE\" (암호 cabinnode) 채널 %u\n", ap_ch);
    Serial.printf("[웹] 폰을 붙이고 http://%s 로 접속\n",
                  WiFi.softAPIP().toString().c_str());
    Serial.printf("[웹] uPlot %uKB(gzip)를 플래시에서 낸다 — 인터넷이 없어도 그려진다\n",
                  (unsigned)((UPLOT_JS_GZ_LEN + UPLOT_CSS_GZ_LEN) / 1024));
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
