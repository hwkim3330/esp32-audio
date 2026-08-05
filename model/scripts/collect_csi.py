#!/usr/bin/env python3
"""CSI 로거의 바이너리 스트림을 받아 학습용으로 저장한다.

음성은 Supertonic 으로 공짜로 합성했지만 전파는 실측이 필요하다. 이 스크립트가
그 실측을 받는다. 라벨은 보드 버튼이 찍으므로 여기서는 사람이 아무것도 안 해도 된다.

프레임 동기가 이 스크립트의 유일한 까다로운 부분이다. 바이너리 프레임 사이에 텍스트
'#STAT' 줄이 섞여 나오므로, 매직(0xC511)을 찾아 프레임 경계를 잡고 길이가 맞는지
확인한 뒤에만 받아들인다. 매직만 믿으면 데이터 안의 우연한 0x11 0xC5 에 속는다 —
그래서 n_sc 범위와 seq 연속성까지 같이 본다.
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

import numpy as np
import serial

MAGIC = 0xC511
HDR = 12
LABEL_NAME = {0: "빈 방", 1: "왼쪽", 2: "가운데", 3: "오른쪽", 4: "이동 중",
              0xFF: "라벨없음"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", default="model/data/csi")
    ap.add_argument("--tag", default=None, help="파일명에 붙일 태그")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    s = serial.Serial(args.port, args.baud, timeout=0.2)
    # 보드를 리셋해 헤더부터 받는다
    s.setDTR(False); s.setRTS(True); time.sleep(0.15)
    s.setRTS(False); s.setDTR(False); time.sleep(0.05)

    buf = bytearray()
    frames: list[np.ndarray] = []
    labels: list[int] = []
    rssis: list[int] = []
    times: list[int] = []
    seqs: list[int] = []
    n_sc_seen: set[int] = set()
    resync = 0
    notes: list[str] = []

    t0 = time.time()
    last_report = t0
    print(f"수집 {args.seconds:.0f}초 시작 — 보드 버튼으로 라벨을 찍으세요")
    print("  K1=빈방  K2=왼쪽  K3=가운데  K4=오른쪽  K5=이동중  K6=라벨없음\n")

    while time.time() - t0 < args.seconds:
        chunk = s.read(8192)
        if chunk:
            buf += chunk

        # 프레임 추출
        i = 0
        while True:
            # 텍스트 줄(#...)은 넘긴다
            if len(buf) - i >= 1 and buf[i:i + 1] == b'#':
                nl = buf.find(b'\n', i)
                if nl < 0:
                    break
                line = bytes(buf[i:nl]).decode('utf-8', 'replace').strip()
                if line and (not notes or notes[-1] != line):
                    notes.append(line)
                i = nl + 1
                continue

            if len(buf) - i < HDR:
                break
            magic, seq, label, rssi_u, n_sc, flags, ms = struct.unpack_from(
                "<HHBBBBI", buf, i)
            # 매직만으로는 부족하다 — n_sc 가 말이 되는지도 본다
            if magic != MAGIC or not (8 <= n_sc <= 128):
                i += 1
                resync += 1
                continue
            need = HDR + 2 * n_sc
            if len(buf) - i < need:
                break
            # bytes() 로 먼저 복사한다. np.frombuffer 를 bytearray 에 직접 걸면
            # 뷰가 export 로 남아 아래 del buf[:i] 가 BufferError 를 낸다.
            raw = np.frombuffer(bytes(buf[i + HDR:i + need]), dtype=np.int8)
            frames.append(raw)
            labels.append(label)
            rssis.append(rssi_u - 256 if rssi_u > 127 else rssi_u)
            times.append(ms)
            seqs.append(seq)
            n_sc_seen.add(n_sc)
            i += need

        if i:
            del buf[:i]

        now = time.time()
        if now - last_report >= 5.0:
            last_report = now
            cnt = {}
            for l in labels:
                cnt[l] = cnt.get(l, 0) + 1
            dist = "  ".join(f"{LABEL_NAME.get(k, k)}:{v}"
                             for k, v in sorted(cnt.items(), key=lambda kv: -kv[1]))
            print(f"  {now - t0:5.1f}s  {len(frames):6d} 프레임  "
                  f"{len(frames) / max(now - t0, 1e-9):5.1f} Hz  |  {dist}")

    s.close()

    if not frames:
        print("\n프레임을 하나도 못 받았다.")
        print("확인할 것: 보드에 csi_logger 가 올라갔는지, baud 가 921600 인지,")
        print("            주변에 WiFi 트래픽이 있는지.")
        for n in notes[:6]:
            print(f"  보드: {n}")
        return 1

    # 서브캐리어 수가 섞이면 학습이 안 되므로 최빈값만 남긴다
    sizes = np.array([len(f) for f in frames])
    mode = int(np.bincount(sizes).argmax())
    keep = sizes == mode
    X = np.stack([frames[i] for i in np.nonzero(keep)[0]])
    y = np.array(labels, np.uint8)[keep]
    rs = np.array(rssis, np.int16)[keep]
    ts = np.array(times, np.uint32)[keep]
    sq = np.array(seqs, np.uint16)[keep]

    # (프레임, 2*n_sc) → 진폭 (프레임, n_sc)
    iq = X.astype(np.float32).reshape(len(X), -1, 2)
    amp = np.sqrt(iq[:, :, 0] ** 2 + iq[:, :, 1] ** 2)

    tag = args.tag or time.strftime("%Y%m%d_%H%M%S")
    path = out / f"csi_{tag}.npz"
    np.savez_compressed(path, amp=amp, iq=X, label=y, rssi=rs, ms=ts, seq=sq)

    dur = time.time() - t0
    print(f"\n저장 {path}")
    print(f"  프레임 {len(X)} (버림 {int((~keep).sum())}), 서브캐리어 {mode // 2}, "
          f"{len(X) / dur:.1f} Hz, 재동기 {resync}회")
    print(f"  진폭 평균 {amp.mean():.1f}  표준편차 {amp.std():.1f}  "
          f"RSSI {rs.mean():.0f} dBm")
    # seq 결손 — 보드 큐 오버플로나 시리얼 손실을 잡는다
    d = np.diff(sq.astype(np.int32))
    lost = int(((d - 1) % 65536)[((d - 1) % 65536) < 1000].sum())
    print(f"  seq 결손 약 {lost} 프레임")
    print("  라벨 분포:")
    for k in sorted(set(y.tolist())):
        n = int((y == k).sum())
        print(f"    {LABEL_NAME.get(k, k):8s} {n:6d} 프레임 ({n / len(y) * 100:4.1f}%)")
    for n in notes[:4]:
        print(f"  보드: {n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
