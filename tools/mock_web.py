#!/usr/bin/env python3
"""보드 없이 웹 UI 를 띄운다. 페이지는 펌웨어에서 **그대로 뽑아** 쓴다.

왜 필요한가. `web_ui.cpp` 의 페이지는 빌드가 통과해도 렌더는 검증되지 않는다 —
uPlot 옵션 오타 하나면 빈 화면이 나오고, 그건 보드를 굽고 SoftAP 에 붙어야 알 수 있다.
그 사이에 "빌드 통과" 를 "동작 확인" 으로 착각하기 쉽다.

그래서 페이지를 복사하지 않고 `web_ui.cpp` 의 PAGE 원문을 정규식으로 떼어 쓴다.
복사하면 두 곳이 갈라지고, 갈라진 것은 나중에 못 찾는다(이 레포가 cn_infer.c 를
복제하지 않은 것과 같은 이유다).

`/s` 와 `/h` 는 **실제 CSI 녹음을 재생**한다. 합성 난수로 채우면 "그려진다" 만 알 수
있고, 색 범위나 축 상한이 실제 데이터에서 어떻게 보이는지는 모른다. 우리 녹음은 전부
빈 방이므로 여기 보이는 것도 빈 방의 모습이다 — 그게 정확한 기준선이다.

    python3 tools/mock_web.py                 # http://127.0.0.1:8899
    python3 tools/mock_web.py --shot out.png  # 헤드리스 크롬으로 찍고 끝낸다
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
FW = ROOT / "firmware/radar_display"
WARMUP_N = 40          # 펌웨어와 같은 값
SD_FLOOR = 0.5


def extract_page() -> bytes:
    """web_ui.cpp 의 PAGE 원문. 복사하지 않고 떼어 온다."""
    src = (FW / "web_ui.cpp").read_text(encoding="utf-8")
    m = re.search(r'PAGE\[\]\s*PROGMEM\s*=\s*R"HTML\((.*?)\)HTML";', src, re.S)
    if not m:
        raise SystemExit("web_ui.cpp 에서 PAGE 를 못 찾았다 — 형식이 바뀌었나")
    return m.group(1).encode("utf-8")


def load_csi(path: Path):
    """실제 녹음에서 펌웨어와 같은 식으로 프레임별 z 를 만든다."""
    d = np.load(path, allow_pickle=True)
    amp = np.asarray(d["amp"], dtype=np.float64)
    ms = np.asarray(d["ms"], dtype=np.float64)
    o = np.argsort(ms)
    amp, ms = amp[o], ms[o]
    mu = amp[:WARMUP_N].mean(axis=0)
    sd = amp[:WARMUP_N].std(axis=0)
    use = sd > SD_FLOOR
    z = np.zeros_like(amp)
    z[:, use] = np.abs(amp[:, use] - mu[use]) / sd[use]
    band = z[:, use].mean(axis=1) if use.any() else np.zeros(len(amp))
    return z[WARMUP_N:], band[WARMUP_N:], amp.shape[1]


class Mock:
    """보드 상태를 흉내낸다. 프레임은 녹음을 돌려 재생한다."""

    def __init__(self, csi: Path):
        self.z, self.band, self.n_sc = load_csi(csi)
        self.i = 0
        self.t0 = time.time()
        rng = np.random.default_rng(0)
        # 추세는 녹음 길이보다 긴 축이 있어서 재생만으로는 못 채운다. 빈 방 분포에서
        # 뽑아 채우고, 이게 합성이라는 것을 페이지가 아니라 여기 적어 둔다.
        self.trend = [np.clip(rng.gamma(2.0, 0.45, 72), 0, 6) for _ in range(4)]
        self.hour = np.zeros(24, int)

    def fast(self):
        k = self.i % len(self.z)
        self.i += 1
        zq = np.clip(np.round(self.z[k] * 8), 0, 127).astype(int)
        b = float(self.band[k])
        return {
            "n_sc": int(self.n_sc), "sc_z": zq.tolist(),
            "band": round(b, 2), "thresh": 3.0,
            "w_dev": [round(b, 2), round(b * 0.8, 2), round(b * 0.6, 2)],
            "base_ok": 7, "csi_hz": 21, "infer_ms": 322,
            "cls_hit": 118, "cls_tot": 231, "last_cls": 1, "last_score": 0.59,
        }

    def hist(self):
        up = int(time.time() - self.t0)
        return {
            "trend": [[round(float(v), 2) for v in t] for t in self.trend],
            "scale_sec": [1, 10, 50, 300],
            "hour_cnt": self.hour.tolist(),
            "events": [],           # 빈 방 녹음이므로 사건이 없다 — 실측과 같다
            "ev_total": 0, "n_anchor": 4,
            "anchor_z": [0.9, 1.4, 0.7, 2.1], "anchor_sd": [2.29, 4.21, 2.69, 5.51],
            "anchor_last": [0x78, 0x06, 0x81, 0x04], "anchor_rssi": [-50, -52, -55, -59],
            "have_clock": 1, "mark_n": 0, "unmark_n": 0, "cohen_d": 0.0,
            "verified": 0, "n_ble": 47, "ble_adv": 30412,
            "probe_tx": 5120, "probe_fail": 12,
            "infer_ms": 322, "uptime_s": up, "boot_n": 12,
        }


def serve(mock: Mock, port: int):
    page = extract_page()
    assets = {
        "/u.js": ((FW / "web/uPlot.iife.min.js").read_bytes(), "application/javascript"),
        "/u.css": ((FW / "web/uPlot.min.css").read_bytes(), "text/css"),
    }

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body: bytes, mime: str):
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            p = self.path.split("?")[0]
            if p == "/":
                self._send(page, "text/html; charset=utf-8")
            elif p in assets:
                b, m = assets[p]
                self._send(b, m)
            elif p == "/s":
                self._send(json.dumps(mock.fast()).encode(), "application/json")
            elif p == "/h":
                self._send(json.dumps(mock.hist()).encode(), "application/json")
            elif p == "/t":
                self._send(b"ok", "text/plain")
            else:
                self.send_error(404)

    srv = ThreadingHTTPServer(("127.0.0.1", port), H)
    return srv


def shoot(url: str, out: Path, wait_s: float, width: int, height: int):
    for exe in ("google-chrome", "chromium-browser", "chromium"):
        try:
            subprocess.run([exe, "--version"], capture_output=True, check=True, timeout=20)
        except Exception:
            continue
        cmd = [exe, "--headless=new", "--no-sandbox", "--disable-gpu",
               "--hide-scrollbars", f"--window-size={width},{height}",
               f"--virtual-time-budget={int(wait_s * 1000)}",
               f"--screenshot={out}", url]
        r = subprocess.run(cmd, capture_output=True, timeout=180)
        if out.exists() and out.stat().st_size > 0:
            return exe
        print(r.stderr.decode()[-2000:])
    raise SystemExit("헤드리스 크롬으로 못 찍었다")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8899)
    ap.add_argument("--csi", default="model/data/csi/csi_ap_auto.npz")
    ap.add_argument("--shot", default="", help="이 경로에 스크린샷을 찍고 종료한다")
    ap.add_argument("--wait", type=float, default=6.0, help="찍기 전 대기(초)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=2400)
    args = ap.parse_args()

    mock = Mock(Path(args.csi))
    srv = serve(mock, args.port)
    url = f"http://127.0.0.1:{args.port}/"
    print(f"페이지는 web_ui.cpp 에서 떼어 왔다 ({len(extract_page()):,} 바이트)")
    print(f"CSI 재생: {args.csi}  프레임 {len(mock.z)}개, 서브캐리어 {mock.n_sc}개")
    print(f"→ {url}")
    if not args.shot:
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            pass
        return 0
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    time.sleep(0.4)
    out = Path(args.shot)
    exe = shoot(url, out, args.wait, args.width, args.height)
    srv.shutdown()
    print(f"{exe} 로 찍음 → {out}  ({out.stat().st_size/1024:.0f}KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
