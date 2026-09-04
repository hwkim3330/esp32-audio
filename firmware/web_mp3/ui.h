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
<div class="sub">아날로그 출력이 죽은 보드라 소리는 <b>블루투스(A2DP)</b>로 나간다.</div>

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
    <h2 style="margin:0">블루투스 출력</h2>
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

<div class="card" id="sdcard" hidden>
  <div class="row spread" style="margin-bottom:8px">
    <h2 style="margin:0">SD 카드 (읽기 전용)</h2>
    <span id="sdpath" class="pill">/</span>
  </div>
  <ul id="sdlist"></ul>
  <div class="note">이 펌웨어는 SD 에 <b>쓰지 않는다</b> — 업로드·삭제는 내장 플래시에만 적용된다.
  카드를 통째로 내려받으려면 PC 에서 <code>tools/sd_backup.sh</code> 를 돌려라.</div>
</div>

<div class="card">
  <h2>버튼 6개</h2>
  <div class="keys">
    <div><b>KEY1</b> 재생/정지</div><div><b>KEY2</b> 다음곡</div><div><b>KEY3</b> 이전곡</div>
    <div><b>KEY4</b> 음량+</div><div><b>KEY5</b> 음량−</div><div><b>KEY6</b> BT 재연결<br>(길게=탐색)</div>
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
