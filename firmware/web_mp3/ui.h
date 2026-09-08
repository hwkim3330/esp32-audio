// 웹 UI — 플래시에 상수로 박아 넣는다. 외부 CDN 참조 없음(보드가 인터넷 없이도 뜨는 게 정상).
#pragma once
#include <pgmspace.h>

static const char UI_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 MP3</title>
<style>
:root{--bg:#0f1115;--card:#181b22;--line:#272b35;--fg:#e7e9ee;--dim:#8b93a3;--acc:#4ea1ff;--ok:#4ade80;--warn:#fbbf24;--bad:#f87171}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.5 system-ui,-apple-system,"Noto Sans KR",sans-serif}
.wrap{max-width:720px;margin:0 auto;padding:16px 14px 60px}
h1{font-size:18px;margin:0 0 4px;letter-spacing:-.01em}
.sub{color:var(--dim);font-size:12px;margin-bottom:14px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:12px}
.card h2{font-size:13px;text-transform:uppercase;letter-spacing:.06em;color:var(--dim);margin:0 0 10px;font-weight:600}
.row{display:flex;align-items:center;gap:8px;flex-wrap:wrap}
.spread{justify-content:space-between}
button{background:#232733;color:var(--fg);border:1px solid var(--line);border-radius:9px;padding:9px 13px;font:inherit;font-size:14px;cursor:pointer}
button:hover{border-color:#3a404e}button:active{transform:translateY(1px)}
button.p{background:var(--acc);border-color:var(--acc);color:#04121f;font-weight:600}
button.sm{padding:5px 9px;font-size:12px}
button.dz{background:transparent;border-color:transparent;color:var(--dim);padding:5px 7px}
button.dz:hover{color:var(--bad)}
input[type=text],input[type=password]{background:#0e1016;border:1px solid var(--line);border-radius:8px;color:var(--fg);padding:8px 10px;font:inherit;font-size:14px;min-width:0;flex:1}
input[type=range]{flex:1;accent-color:var(--acc)}
.now{font-size:16px;font-weight:600;margin:2px 0 8px;word-break:break-all}
.bar{height:5px;background:#0e1016;border-radius:3px;overflow:hidden}
.bar>i{display:block;height:100%;background:var(--acc);width:0}
.t{color:var(--dim);font-size:12px;margin-top:5px;display:flex;justify-content:space-between}
ul{list-style:none;margin:0;padding:0}
li{display:flex;align-items:center;gap:8px;padding:8px 0;border-top:1px solid var(--line)}
li:first-child{border-top:0}
li .nm{flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;cursor:pointer}
li.on .nm{color:var(--acc);font-weight:600}
li .sz{color:var(--dim);font-size:12px;font-variant-numeric:tabular-nums}
.pill{font-size:11px;padding:2px 8px;border-radius:99px;border:1px solid var(--line);color:var(--dim)}
.pill.ok{color:var(--ok);border-color:#255b3a}.pill.bad{color:var(--bad);border-color:#5b2525}.pill.warn{color:var(--warn);border-color:#5b4a25}
.note{color:var(--dim);font-size:12px;margin-top:8px}
.drop{border:1px dashed var(--line);border-radius:10px;padding:14px;text-align:center;color:var(--dim);font-size:13px}
.drop.hot{border-color:var(--acc);color:var(--acc)}
progress{width:100%;height:5px}
.keys{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;font-size:12px;color:var(--dim)}
.keys div{background:#0e1016;border:1px solid var(--line);border-radius:7px;padding:6px 8px}
.keys b{color:var(--fg);font-weight:600}
</style></head><body><div class="wrap">

<h1>ESP32-Audio-Kit · MP3</h1>
<div class="sub">3.5mm 잭과 블루투스 양쪽으로 소리를 낸다. 맥·폰이 이 보드에 연결하는 <b>BT 스피커</b> 역할도 된다.</div>

<div class="card">
  <div class="row spread" style="margin-bottom:8px">
    <h2 style="margin:0">재생</h2>
    <span id="st" class="pill">…</span>
  </div>
  <div class="now" id="now">—</div>
  <div class="bar"><i id="pb"></i></div>
  <div class="t"><span id="ela">0:00</span><span id="tot">0:00</span></div>
  <div class="row" style="margin-top:12px">
    <button onclick="api('/api/prev')">⏮</button>
    <button class="p" id="pp" onclick="api('/api/toggle')">▶︎</button>
    <button onclick="api('/api/next')">⏭</button>
    <button onclick="api('/api/stop')">⏹</button>
  </div>
  <div class="row" style="margin-top:12px">
    <span style="color:var(--dim);font-size:12px">음량</span>
    <input type="range" id="vol" min="0" max="100" oninput="vlab.textContent=this.value" onchange="api('/api/vol?v='+this.value)">
    <span id="vlab" style="width:2.4em;text-align:right;font-variant-numeric:tabular-nums">—</span>
  </div>
</div>

<div class="card">
  <div class="row spread" style="margin-bottom:8px">
    <h2 style="margin:0">역할</h2>
    <span id="rolest" class="pill">…</span>
  </div>
  <div class="row">
    <button id="rp" onclick="role('player')">플레이어 (파일 재생)</button>
    <button id="rs" onclick="role('sink')">BT 스피커 (맥·폰이 연결)</button>
  </div>
  <div class="note"><b>BT 스피커</b>로 두면 맥·폰에서 블루투스를 스캔해 <code>KETI-MP3</code>를
  고르면 된다 — 그 소리가 이 보드의 3.5mm 잭으로 나온다. 전환하면 <b>재부팅</b>한다
  (Bluedroid 는 소스와 싱크를 동시에 못 든다).</div>
</div>

<div class="card">
  <div class="row spread" style="margin-bottom:8px">
    <h2 style="margin:0">출력 경로</h2>
    <span id="outst" class="pill">…</span>
  </div>
  <div class="row">
    <button id="ob" onclick="api('/api/out?m=bt')">블루투스 (A2DP)</button>
    <button id="oj" onclick="api('/api/out?m=jack')">3.5mm 잭</button>
    <button onclick="api('/api/tone?s=5')">잭 테스트음 5초</button>
  </div>
  <div class="note"><b>잭 출력은 실기로 확인됐다</b>(2026-09-08). 8월에 "오디오가 죽었다" 고
  적힌 것은 <code>board_pins.h</code> 의 LRCK/DOUT 이 25/26 으로 뒤바뀌어 있었던 탓이다.
  이어폰은 <b>EARPHONE</b> 잭에 꽂아야 한다 — 옆의 <b>LINEIN</b> 은 입력이다.</div>
</div>

<div class="card">
  <div class="row spread" style="margin-bottom:8px">
    <h2 style="margin:0">블루투스 기기</h2>
    <span id="btst" class="pill">…</span>
  </div>
  <div class="row">
    <button onclick="api('/api/bt/scan')">기기 탐색 (12초)</button>
    <button onclick="api('/api/bt/disconnect')">연결 끊기</button>
  </div>
  <ul id="btlist" style="margin-top:10px"></ul>
  <div class="note">이어폰을 <b>페어링 모드</b>로 두고 탐색해야 목록에 뜬다(보통 전원 버튼 길게).
  이미 다른 기기에 붙어 있으면 검색되지 않는다.</div>
</div>

<div class="card">
  <div class="row spread" style="margin-bottom:8px">
    <h2 style="margin:0">곡 목록</h2>
    <span id="store" class="pill">…</span>
  </div>
  <ul id="files"></ul>
  <div class="drop" id="drop" style="margin-top:10px">
    여기에 MP3 를 끌어다 놓거나 <button class="sm" onclick="fi.click()">파일 선택</button>
    <input type="file" id="fi" accept=".mp3,audio/mpeg" multiple hidden onchange="up(this.files)">
  </div>
  <progress id="prg" hidden></progress>
  <div class="note" id="upmsg"></div>
</div>

<div class="card" id="reccard">
  <div class="row spread" style="margin-bottom:8px">
    <h2 style="margin:0">녹음 (온보드 마이크 2개)</h2>
    <span id="recst" class="pill">…</span>
  </div>
  <div class="row">
    <button class="p" id="recbtn" onclick="rec(0)">● 녹음 (스테레오)</button>
    <button id="recncbtn" onclick="rec(1)">● 잡음제거 녹음 (모노)</button>
    <span style="color:var(--dim);font-size:12px">레벨</span>
    <div class="bar" style="flex:1"><i id="lvl_l"></i></div>
    <div class="bar" style="flex:1"><i id="lvl_r"></i></div>
  </div>
  <div class="note"><b>잡음제거</b>는 마이크 두 개로 하는 적응 필터(NLMS 12탭)다 — 한쪽을 목표,
  다른 쪽을 잡음 기준으로 두고 상관된 성분을 뺀다. 결과는 모노(초당 88KB).
  재생용 ANC 는 이 하드웨어로 불가능하다(루프 지연이 1ms 미만이어야 한다).<br>
  44.1kHz 16비트로 SD 의 <code>/REC</code> 에 쓴다.
  녹음을 시작하면 재생은 멈춘다 — 마이크가 보드 위에 있어서 자기 소리를 다시 듣는다.
  받은 파일은 아래 SD 목록의 <code>/REC</code> 에서 내려받는다.</div>
</div>

<div class="card" id="sdcard" hidden>
  <div class="row spread" style="margin-bottom:8px">
    <h2 style="margin:0">SD 카드 (읽기 전용)</h2>
    <span id="sdpath" class="pill">/</span>
  </div>
  <ul id="sdlist"></ul>
  <div class="note">이 펌웨어는 SD 에 <b>쓰지 않는다</b> — 업로드·삭제는 내장 플래시에만 적용된다.
  카드를 통째로 내려받으려면 PC 에서 <code>tools/sd_backup.py</code> 를 돌려라.</div>
</div>

<div class="card">
  <h2>버튼 6개</h2>
  <div class="keys">
    <div><b>KEY1</b> 재생/정지</div><div><b>KEY2</b> 다음곡</div><div><b>KEY3</b> 이전곡</div>
    <div><b>KEY4</b> 음량+<br>(길게=테스트음)</div><div><b>KEY5</b> 음량−<br>(길게=출력전환)</div><div><b>KEY6</b> BT 재연결<br>(헤드폰검출 겸용)</div>
  </div>
</div>

<div class="card">
  <h2>WiFi</h2>
  <div class="row"><span id="wifi" class="pill">…</span></div>
  <div class="row" style="margin-top:10px">
    <input type="text" id="ssid" placeholder="SSID">
    <input type="password" id="pass" placeholder="비밀번호">
    <button onclick="wifi()">저장 후 재부팅</button>
  </div>
  <div class="note">저장한 AP 에 못 붙으면 스스로 <b>AP 모드</b>(SSID <code>esp32-mp3</code> / 비번 <code>12345678</code>,
  주소 <code>192.168.4.1</code>)로 뜬다.</div>
</div>

<script>
const $=id=>document.getElementById(id);
let dragVol=false;
$('vol').addEventListener('pointerdown',()=>dragVol=true);
$('vol').addEventListener('pointerup',()=>dragVol=false);
const mmss=s=>{s=Math.max(0,Math.round(s));return Math.floor(s/60)+':'+String(s%60).padStart(2,'0')};
const kb=n=>n>=1048576?(n/1048576).toFixed(1)+' MB':Math.round(n/1024)+' KB';

async function api(u){try{await fetch(u,{method:'POST'})}catch(e){} tick();}

async function tick(){
  let s; try{ s=await (await fetch('/api/status')).json() }catch(e){ $('st').textContent='연결 끊김'; $('st').className='pill bad'; return }
  $('st').textContent = s.state; $('st').className='pill '+(s.state=='재생 중'?'ok':s.state=='일시정지'?'warn':'');
  $('pp').textContent = s.playing?'⏸':'▶︎';
  $('now').textContent = s.track || '—';
  $('pb').style.width = (s.pct||0)+'%';
  $('ela').textContent = mmss(s.elapsed); $('tot').textContent = s.duration?mmss(s.duration):'—:—';
  if(!dragVol){ $('vol').value=s.vol; $('vlab').textContent=s.vol; }

  // 녹음
  $('reccard').hidden = (s.role=='sink');
  $('recst').textContent = s.rec ? ('● 녹음 중 '+s.recsec+'초 · '+s.recname) : (s.sd?'대기':'SD 없음');
  $('recst').className='pill '+(s.rec?'bad':s.sd?'':'warn');
  $('recbtn').textContent = s.rec ? '■ 정지' : '● 녹음 (스테레오)';
  $('recncbtn').hidden = s.rec;
  if (s.rec && s.recnc) $('recbtn').textContent = '■ 정지 (잡음제거 중)';
  $('lvl_l').style.width = Math.min(100,(s.reclvl[0]/300)*100)+'%';
  $('lvl_r').style.width = Math.min(100,(s.reclvl[1]/300)*100)+'%';
  window._rec = s.rec;

  const sink = s.role=='sink';
  $('rolest').textContent = sink ? ('BT 스피커 · '+s.sinkhz+'Hz · '+s.sinkkb+' KB 수신'+(s.sinkdrop?' · 버림 '+s.sinkdrop:'')) : '플레이어';
  $('rolest').className='pill '+(sink?'warn':'ok');
  $('rp').className = sink?'':'p'; $('rs').className = sink?'p':'';

  const jack = s.out=='jack';
  $('outst').textContent = (jack?'잭':'블루투스') + (s.tone?' · 테스트음 재생중':'') + (s.jackok?'':' · AC101 없음');
  $('outst').className='pill '+(s.tone?'warn':'ok');
  $('ob').className = jack?'':'p'; $('oj').className = jack?'p':'';

  $('btst').textContent = s.bt.state; $('btst').className='pill '+(s.bt.streaming?'ok':s.bt.connected?'warn':'bad');
  $('btlist').innerHTML = s.bt.devices.length? s.bt.devices.map(d=>
    `<li${d.addr==s.bt.addr?' class="on"':''}><span class="nm" onclick="api('/api/bt/connect?a=${d.addr}')">${esc(d.name)}</span>`+
    `<span class="sz">${d.rssi}dBm</span><span class="pill">${esc(d.cls)}</span></li>`).join('')
    : `<li><span class="nm" style="color:var(--dim);cursor:default">${s.bt.scanning?'탐색 중…':'탐색된 기기 없음'}</span></li>`;

  $('store').textContent = s.storage+' · '+kb(s.used)+' / '+kb(s.total);
  $('files').innerHTML = s.files.length? s.files.map((f,i)=>
    `<li${i==s.idx?' class="on"':''}><span class="nm" onclick="api('/api/play?i=${i}')">${esc(f.n)}</span>`+
    `<span class="sz">${kb(f.s)}</span><button class="dz" onclick="del('${encodeURIComponent(f.n)}')">✕</button></li>`).join('')
    : '<li><span class="nm" style="color:var(--dim);cursor:default">비어 있음 — MP3 를 올려라</span></li>';

  $('wifi').textContent = s.wifi; $('wifi').className='pill '+(s.wifi.startsWith('AP')?'warn':'ok');
  if(s.sd && $('sdcard').hidden){ $('sdcard').hidden=false; sd('/'); }
}

// SD 브라우저 — 목록과 원본 다운로드만. 쓰기 엔드포인트는 펌웨어에 아예 없다.
async function sd(p){
  let r; try{ r=await (await fetch('/api/sd/list?p='+encodeURIComponent(p))).json() }catch(e){ return }
  $('sdpath').textContent=r.path;
  const up = r.path=='/'? '' : `<li><span class="nm" onclick="sd('${r.path.replace(/\/[^/]*$/,'')||'/'}')">⬆ 상위</span></li>`;
  $('sdlist').innerHTML = up + (r.entries.length? r.entries.map(e=>e.d
    ? `<li><span class="nm" onclick="sd('${encodeURI(e.p)}')">📁 ${esc(e.n)}</span></li>`
    : `<li><a class="nm" href="/sd${encodeURI(e.p)}" download style="color:inherit;text-decoration:none">⬇ ${esc(e.n)}</a><span class="sz">${kb(e.s)}</span></li>`
    ).join('') : '<li><span class="nm" style="color:var(--dim)">비어 있음</span></li>');
}
const esc=s=>String(s).replace(/[<>&"]/g,c=>({'<':'&lt;','>':'&gt;','&':'&amp;','"':'&quot;'}[c]));
async function rec(nc){ await fetch('/api/rec?on='+(window._rec?'0':'1')+'&nc='+nc,{method:'POST'}); tick(); }
async function role(r){ if(!confirm(r=='sink'?'BT 스피커로 바꾸고 재부팅한다':'플레이어로 바꾸고 재부팅한다'))return;
  await fetch('/api/role?r='+r,{method:'POST'}); $('upmsg').textContent='역할 전환, 재부팅 중…'; }
async function del(n){ if(!confirm(decodeURIComponent(n)+' 삭제?'))return; await fetch('/api/delete?f='+n,{method:'POST'}); tick(); }
async function wifi(){ await fetch('/api/wifi?ssid='+encodeURIComponent($('ssid').value)+'&pass='+encodeURIComponent($('pass').value),{method:'POST'});
  $('upmsg').textContent='저장했다. 재부팅 중…'; }

function up(files){
  if(!files.length)return;
  const f=files[0], fd=new FormData(); fd.append('f',f,f.name);
  const x=new XMLHttpRequest(); x.open('POST','/api/upload');
  $('prg').hidden=false; $('upmsg').textContent=f.name+' 업로드 중… (재생은 자동으로 멈춘다)';
  x.upload.onprogress=e=>{ $('prg').max=e.total; $('prg').value=e.loaded; };
  x.onload=()=>{ $('prg').hidden=true; $('upmsg').textContent=x.status==200?(f.name+' 완료'):('실패: '+x.responseText);
    const rest=Array.from(files).slice(1); tick(); if(rest.length) up(rest); };
  x.onerror=()=>{ $('prg').hidden=true; $('upmsg').textContent='업로드 실패(연결)'; };
  x.send(fd);
}
const dz=$('drop');
['dragenter','dragover'].forEach(e=>dz.addEventListener(e,ev=>{ev.preventDefault();dz.classList.add('hot')}));
['dragleave','drop'].forEach(e=>dz.addEventListener(e,ev=>{ev.preventDefault();dz.classList.remove('hot')}));
dz.addEventListener('drop',ev=>up(ev.dataTransfer.files));
tick(); setInterval(tick,1000);
</script></div></body></html>)HTML";
