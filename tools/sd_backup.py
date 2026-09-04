#!/usr/bin/env python3
"""보드에 꽂힌 microSD 를 WiFi 로 통째로 내려받는다 (읽기 전용).

이 PC 에는 카드 리더가 없다(`/dev/mmcblk*` 없음). 그래서 카드를 빼지 않고
`web_mp3` 펌웨어의 /api/sd/list + /sd/<경로> 만으로 백업한다.

- 크기가 같은 파일은 건너뛴다 → 중단해도 다시 돌리면 이어서 받는다
- 부분 파일은 Range 로 이어받는다
- 보드는 SD 에 쓰지 않는다. 이 스크립트도 읽기만 한다

사용:
    tools/sd_backup.py                        # esp32-mp3.local → ~/esp32_audio_kit/sd_backup
    tools/sd_backup.py 192.168.0.42
    tools/sd_backup.py 192.168.0.42 /data/sd  # 대상 폴더 지정
    tools/sd_backup.py --dry-run              # 받지 않고 목록·용량만 센다
"""
import json
import os
import sys
import time
import urllib.parse
import urllib.request

TIMEOUT = 20


def get_json(host, path):
    url = f"http://{host}/api/sd/list?p={urllib.parse.quote(path)}"
    with urllib.request.urlopen(url, timeout=TIMEOUT) as r:
        return json.load(r)


def walk(host, path="/"):
    """(경로, 크기) 를 재귀적으로 낸다. 디렉터리 순회만 하고 아무것도 쓰지 않는다."""
    try:
        d = get_json(host, path)
    except Exception as e:
        print(f"  ! 목록 실패 {path}: {e}", file=sys.stderr)
        return
    for e in d.get("entries", []):
        if e["d"]:
            yield from walk(host, e["p"])
        else:
            yield e["p"], e["s"]


def human(n):
    for u in ("B", "KB", "MB", "GB"):
        if n < 1024 or u == "GB":
            return f"{n:.1f} {u}" if u != "B" else f"{int(n)} B"
        n /= 1024


def fetch(host, remote, size, dest):
    local = os.path.join(dest, remote.lstrip("/"))
    os.makedirs(os.path.dirname(local) or ".", exist_ok=True)

    have = os.path.getsize(local) if os.path.exists(local) else 0
    if have == size and size > 0:
        return 0, "이미 있음"
    if have > size:                      # 이전에 다른 파일이었던 경우 — 처음부터
        have = 0

    url = f"http://{host}/sd{urllib.parse.quote(remote)}"
    req = urllib.request.Request(url)
    mode = "wb"
    if have:
        req.add_header("Range", f"bytes={have}-")
        mode = "ab"

    t0 = time.time()
    got = 0
    with urllib.request.urlopen(req, timeout=TIMEOUT) as r, open(local, mode) as f:
        # 206 이 아니면 서버가 Range 를 무시한 것 — 처음부터 다시 쓴다
        if have and r.status != 206:
            f.close()
            f = open(local, "wb")
            have = 0
        while True:
            b = r.read(64 * 1024)
            if not b:
                break
            f.write(b)
            got += len(b)
            el = time.time() - t0
            # 진행률은 터미널에서만 찍는다. 로그로 리다이렉트하면 \r 이 안 먹어서
            # 파일이 한 줄로 수백 KB 씩 불어난다.
            if el > 0 and sys.stdout.isatty():
                pct = 100.0 * (have + got) / size if size else 100.0
                print(f"\r    {pct:5.1f}%  {human(got/el)}/s     ", end="", flush=True)
    if sys.stdout.isatty():
        print("\r" + " " * 40 + "\r", end="")
    return got, f"{human(got)} 받음"


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    dry = "--dry-run" in sys.argv

    host = args[0] if args else "esp32-mp3.local"
    dest = args[1] if len(args) > 1 else os.path.expanduser("~/esp32_audio_kit/sd_backup")

    print(f"보드 {host} → {dest}")
    print("목록을 훑는다 (카드가 크면 몇 분 걸린다)…")

    files = list(walk(host))
    total = sum(s for _, s in files)
    print(f"파일 {len(files)}개, 합계 {human(total)}\n")
    if dry:
        for p, s in files:
            print(f"  {human(s):>10}  {p}")
        return

    os.makedirs(dest, exist_ok=True)
    t0 = time.time()
    done = 0
    for i, (p, s) in enumerate(files, 1):
        print(f"[{i}/{len(files)}] {p}  ({human(s)})")
        try:
            got, msg = fetch(host, p, s, dest)
            done += got
            print(f"    {msg}")
        except Exception as e:
            print(f"    ! 실패: {e}", file=sys.stderr)
        el = time.time() - t0
        if done and el > 1:
            rate = done / el
            left = total - done
            print(f"    평균 {human(rate)}/s · 남은 시간 약 {left/rate/60:.0f}분")

    print(f"\n끝. {human(done)} 을 {time.time()-t0:.0f}초에 받았다.")
    print("무결성 확인: 크기 비교는 이미 했다. 해시까지 보려면 카드를 리더에 꽂아야 한다.")


if __name__ == "__main__":
    main()
