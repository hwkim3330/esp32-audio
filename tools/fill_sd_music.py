#!/usr/bin/env python3
"""archive.org 의 CC 라이선스 음악을 받아 보드 SD 의 /MP3 로 채운다.

왜 archive.org 인가: 유튜브/스트리밍에서 받는 건 저작권·이용약관 문제가 있다.
archive.org 의 `netlabels` 컬렉션은 **크리에이티브 커먼즈로 배포되는 음악 앨범**
모음이고(8만 건 이상), 직접 다운로드 링크와 JSON API 가 공개돼 있다.

경로: archive.org → (PC 임시폴더) → 보드 HTTP 업로드 → SD 의 /MP3

주의 — **보드의 곡 목록은 메모리 때문에 200곡이 상한이다.** 카드가 16GB 라도
플레이어가 색인할 수 있는 건 수백 곡이라, 무작정 꽉 채우는 것보다 상한에 맞추는
편이 실제로 쓸 수 있다. `--tracks` 로 조절한다.

사용:
    tools/fill_sd_music.py                    # 기본 180곡
    tools/fill_sd_music.py --tracks 60
    tools/fill_sd_music.py --host 192.168.4.1 --max-mb 8
    tools/fill_sd_music.py --list-only        # 받지 않고 후보만 출력
"""
import argparse
import json
import os
import random
import re
import subprocess
import sys
import time
import urllib.parse
import urllib.request

SEARCH = "https://archive.org/advancedsearch.php"
META = "https://archive.org/metadata/"
DL = "https://archive.org/download/"
UA = {"User-Agent": "esp32-audio-kit-filler/1.0"}


def get_json(url, timeout=30):
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)


def human(n):
    for u in ("B", "KB", "MB", "GB"):
        if n < 1024 or u == "GB":
            return f"{int(n)} B" if u == "B" else f"{n:.1f} {u}"
        n /= 1024


def find_items(pages, seed):
    """netlabels 컬렉션에서 아이템 식별자를 모은다. 페이지를 무작위로 골라 매번
    다른 앨범이 나오게 한다(같은 것만 반복해 받지 않도록)."""
    rnd = random.Random(seed)
    out = []
    ROWS = 50
    # archive.org 는 깊은 페이징을 거부한다(start = page×rows 가 약 10000 을 넘으면
    # 응답에 response 키가 없다 — 이걸로 한 번 죽었다). start 를 9000 아래로 묶는다.
    MAX_PAGE = 9000 // ROWS
    for _ in range(pages):
        page = rnd.randint(1, MAX_PAGE)
        q = urllib.parse.urlencode({
            "q": "collection:netlabels AND mediatype:audio",
            "fl[]": "identifier",
            "rows": ROWS,
            "page": page,
            "output": "json",
        })
        try:
            d = get_json(f"{SEARCH}?{q}")
            docs = d["response"]["docs"]
        except Exception as e:
            print(f"  ! 검색 실패(page {page}): {e}", file=sys.stderr)
            continue
        except KeyError:
            print(f"  ! 응답 형식 이상(page {page})", file=sys.stderr)
            continue
        out += [x["identifier"] for x in docs]
    seen, uniq = set(), []
    for i in out:
        if i not in seen:
            seen.add(i)
            uniq.append(i)
    return uniq


def item_tracks(ident, max_bytes):
    """아이템 안의 mp3 파일 목록. VBR/128k 중 작은 것을 고른다."""
    try:
        m = get_json(META + ident)
    except Exception:
        return []
    files = m.get("files", [])
    out = []
    for f in files:
        name = f.get("name", "")
        if not name.lower().endswith(".mp3"):
            continue
        try:
            size = int(f.get("size", 0))
        except ValueError:
            continue
        if size <= 0 or size > max_bytes:
            continue
        out.append((ident, name, size))
    return out


def safe_name(ident, name):
    """SD·LittleFS 에 안전한 짧은 파일명. 8.3 은 아니지만 특수문자·길이를 줄인다."""
    base = os.path.basename(name)
    base = re.sub(r"[^A-Za-z0-9._-]+", "_", base)
    base = re.sub(r"_+", "_", base).strip("_")
    if len(base) > 48:
        base = base[-48:]
    pref = re.sub(r"[^A-Za-z0-9]+", "", ident)[:10]
    return f"{pref}_{base}"


def board_files(host):
    try:
        d = get_json(f"http://{host}/api/status", timeout=15)
        return {f["n"] for f in d.get("files", [])}, d
    except Exception:
        return set(), None


def upload(host, path, name):
    """보드의 멀티파트 업로드. curl 을 쓴다 — 큰 파일에서 파이썬 멀티파트보다
    안정적이고 진행률·속도를 그대로 볼 수 있다."""
    r = subprocess.run(
        ["curl", "-s", "--max-time", "300", "-F", f"f=@{path};filename={name}",
         f"http://{host}/api/upload"],
        capture_output=True, text=True)
    return r.stdout.strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--tracks", type=int, default=180, help="목표 곡 수 (보드 색인 상한 200)")
    ap.add_argument("--max-mb", type=float, default=6.0, help="곡당 최대 크기")
    ap.add_argument("--stage", default="/tmp/esp32_music")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--list-only", action="store_true")
    a = ap.parse_args()

    have, st = board_files(a.host)
    if st is None and not a.list_only:
        print(f"보드({a.host})에 연결되지 않았다.", file=sys.stderr)
        return 1
    if st is not None:
        print(f"보드: 역할 {st['role']} · 저장소 {st['storage']} · 이미 {len(have)}곡")

    os.makedirs(a.stage, exist_ok=True)
    max_bytes = int(a.max_mb * 1024 * 1024)
    seed = a.seed or int(time.time()) % 100000

    print("archive.org netlabels(CC 라이선스)에서 후보를 모은다…")
    idents = find_items(4, seed)
    print(f"앨범 후보 {len(idents)}개")

    pool = []
    for i, ident in enumerate(idents):
        if len(pool) >= a.tracks * 2:
            break
        tr = item_tracks(ident, max_bytes)
        if tr:
            pool += tr[:6]                 # 한 앨범에서 최대 6곡만 (다양성)
        if (i + 1) % 10 == 0:
            print(f"  {i+1}/{len(idents)} 앨범 조회 · 후보 {len(pool)}곡")

    print(f"후보 {len(pool)}곡, 합계 {human(sum(s for _,_,s in pool))}")
    if a.list_only:
        for ident, name, size in pool[:60]:
            print(f"  {human(size):>9}  {ident}/{name}")
        return 0

    t0 = time.time()
    up_bytes = 0
    done = 0
    for ident, name, size in pool:
        if done >= a.tracks:
            break
        dst = safe_name(ident, name)
        if dst in have:
            continue
        local = os.path.join(a.stage, dst)
        try:
            if not os.path.exists(local) or os.path.getsize(local) != size:
                url = DL + urllib.parse.quote(ident) + "/" + urllib.parse.quote(name)
                req = urllib.request.Request(url, headers=UA)
                with urllib.request.urlopen(req, timeout=90) as r, open(local, "wb") as f:
                    while True:
                        b = r.read(1 << 16)
                        if not b:
                            break
                        f.write(b)
        except Exception as e:
            print(f"  ! 내려받기 실패 {ident}/{name}: {e}", file=sys.stderr)
            continue

        res = upload(a.host, local, dst)
        ok = (res == "ok")
        if ok:
            up_bytes += os.path.getsize(local)
            done += 1
            have.add(dst)
        el = max(time.time() - t0, 1e-6)
        print(f"[{done}/{a.tracks}] {dst}  {human(size)}  {'ok' if ok else '실패: ' + res}"
              f"  · 평균 {human(up_bytes/el)}/s")
        try:
            os.remove(local)               # 스테이징은 바로 지운다(PC 디스크 절약)
        except OSError:
            pass

    el = time.time() - t0
    print(f"\n끝. {done}곡 / {human(up_bytes)} 을 {el/60:.1f}분에 올렸다"
          f" (평균 {human(up_bytes/max(el,1e-6))}/s)")
    _, st2 = board_files(a.host)
    if st2:
        print(f"보드 곡 목록: {len(st2['files'])}곡 · 저장소 {st2['storage']}"
              f" · 사용 {human(st2['used'])} / {human(st2['total'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
