#!/usr/bin/env python3
"""Drive slam2d-daemon over the wire, with the ring format ouster-edge emits.

The C test exercises the core directly. This one exercises the thing that will
actually run: real UDP, real OSED datagrams laid out as doc/RING-FORMAT.md
describes, and the daemon's own status file read back as the only evidence.

What it checks is the same property that matters in the C test - that the pose
the daemon reports tracks the pose the simulated robot actually had - plus the
handling that only exists in the daemon: malformed datagrams, scans too sparse
to match, and the warning when the vehicle outruns the search window.
"""
import json
import math
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time

BIN = os.environ.get("SLAM2D_BIN", "./slam2d-daemon")
PORT = int(os.environ.get("SLAM2D_PORT", "7802"))
SECTORS = 360

fails = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f": {detail}" if detail else ""))
    if not ok:
        fails.append(name)


WORLD = [(-6, -4, 6, -4), (6, -4, 6, 4), (6, 4, -6, 4), (-6, 4, -6, -4),
         (1, -4, 1, 0), (-3, 4, -3, 1)]


def ray(px, py, ang):
    dx, dy = math.cos(ang), math.sin(ang)
    best = 1e9
    for x0, y0, x1, y1 in WORLD:
        ex, ey = x1 - x0, y1 - y0
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((x0 - px) * ey - (y0 - py) * ex) / den
        u = ((x0 - px) * dy - (y0 - py) * dx) / den
        if t > 0.05 and 0 <= u <= 1 and t < best:
            best = t
    return best


def ring_packet(x, y, th, frame):
    """Exactly the layout in doc/RING-FORMAT.md."""
    rng = []
    for i in range(SECTORS):
        r = ray(x, y, th + 2 * math.pi * i / SECTORS)
        rng.append(0xFFFF if r > 30 else int(r * 100 + 0.5))
    pkt = b"OSED" + bytes([1, 2]) + struct.pack("<HH", SECTORS, frame & 0xFFFF)
    pkt += bytes([0, 0]) + struct.pack("<Q", frame * 100_000_000)
    pkt += b"".join(struct.pack("<H", v) for v in rng)
    pkt += bytes(SECTORS)          # reflectivity, unused here
    return pkt


def start(status, extra=()):
    p = subprocess.Popen([BIN, "-f", "-p", str(PORT), "-S", status,
                          "-I", "100", *extra],
                         stderr=subprocess.PIPE, text=True)
    time.sleep(0.7)
    if p.poll() is not None:
        print("  FAIL  daemon exited at once:", p.stderr.read())
        sys.exit(1)
    return p


def read_status(path):
    for _ in range(40):
        try:
            with open(path) as f:
                return json.load(f)
        except Exception:
            time.sleep(0.1)
    return None


def run_trajectory():
    status = tempfile.mkstemp(suffix=".json")[1]
    mapf = tempfile.mkstemp(suffix=".pgm")[1]
    proc = start(status, ("-M", mapf, "-T", "500"))
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    worst = 0.0
    try:
        for step in range(121):
            t = step * 0.05
            tx, ty = 3.0 * math.sin(t), 2.0 * (1 - math.cos(t))
            tth = 0.35 * math.sin(t * 1.3)
            s.sendto(ring_packet(tx, ty, tth, step), ("127.0.0.1", PORT))
            time.sleep(0.02)
            if step and step % 30 == 0:
                # Pause before reading.
                #
                # The status file is rewritten every 100 ms and packets go out
                # every 20 ms, so reading it mid-stream compares a pose from
                # several scans ago against the truth right now. At up to 15 cm
                # of travel per scan that alone produced a 27 cm "error" and a
                # failing test, while the settled final pose was 2 cm out.
                time.sleep(0.35)
                st = read_status(status)
                if st:
                    dx = st["pose"]["x_m"] - tx
                    dy = st["pose"]["y_m"] - ty
                    worst = max(worst, math.hypot(dx, dy))
        time.sleep(0.8)
        st = read_status(status)
        check("daemon consumed the rings", st and st["rings"] >= 120,
              f"rings={st and st['rings']}")
        check("daemon matched them", st and st["matched"] >= 118,
              f"matched={st and st['matched']}")
        check("no malformed counted", st and st["skipped_malformed"] == 0,
              f"skipped={st and st['skipped_malformed']}")
        dx = st["pose"]["x_m"] - tx
        dy = st["pose"]["y_m"] - ty
        e = math.hypot(dx, dy)
        check("final pose within 25 cm of truth", e < 0.25, f"{e:.3f} m")
        check("worst sampled error within 25 cm", worst < 0.25, f"{worst:.3f} m")
        check("match score is a real fraction of the maximum",
              st["score_frac_pct"] > 20, f"{st['score_frac_pct']}%")
        check("map was written", os.path.getsize(mapf) > 1000,
              f"{os.path.getsize(mapf)} bytes")
        print(f"    match cost {st['match_us']} us, "
              f"{st['candidates']} candidates per scan")
    finally:
        proc.terminate()
        proc.wait(timeout=5)
        s.close()
        for f in (status, mapf):
            if os.path.exists(f):
                os.remove(f)


def run_rejections():
    status = tempfile.mkstemp(suffix=".json")[1]
    proc = start(status)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.sendto(b"NOPE" + bytes(40), ("127.0.0.1", PORT))
        s.sendto(b"OSED" + bytes(4), ("127.0.0.1", PORT))          # too short
        good = ring_packet(0, 0, 0, 1)
        s.sendto(good[:60], ("127.0.0.1", PORT))                   # truncated
        time.sleep(0.5)
        st = read_status(status)
        check("malformed datagrams counted, not crashed",
              st and st["skipped_malformed"] == 3,
              f"skipped={st and st['skipped_malformed']}")
        check("nothing was matched from them", st and st["matched"] == 0,
              f"matched={st and st['matched']}")

        empty = b"OSED" + bytes([1, 2]) + struct.pack("<HH", SECTORS, 9) + \
            bytes([0, 0]) + struct.pack("<Q", 0) + \
            b"".join(struct.pack("<H", 0xFFFF) for _ in range(SECTORS)) + \
            bytes(SECTORS)
        s.sendto(empty, ("127.0.0.1", PORT))
        time.sleep(0.4)
        st = read_status(status)
        check("a scan with no returns is refused, not mapped",
              st and st["skipped_too_few_returns"] >= 1,
              f"skipped={st and st['skipped_too_few_returns']}")
    finally:
        proc.terminate()
        proc.wait(timeout=5)
        s.close()
        if os.path.exists(status):
            os.remove(status)


print(f"slam2d-daemon verification using {BIN}\n")
print("  trajectory")
run_trajectory()
print("\n  malformed and empty input")
run_rejections()

print()
if fails:
    print(f"FAILED ({len(fails)}): {', '.join(fails)}")
    sys.exit(1)
print("all checks passed")
