#!/usr/bin/env python3
"""Measure the two latencies the design claims to have shortened.

1. Zone alarm: from the arrival of the first intruding column to the action
   script running. Evaluating per column instead of per revolution should make
   this independent of the rotation rate.

2. Ring delivery: from the last packet of a revolution to a reader having the
   ring. Server-Sent Events push should beat polling a status file, which costs
   up to one write interval plus one poll interval.

Numbers here are from a desktop; on an 880 MHz MIPS router they will be larger.
What the test pins down is the shape: per-column, not per-revolution, and
pushed, not polled.
"""
import http.client
import json
import os
import socket
import stat
import struct
import subprocess
import sys
import tempfile
import threading
import time

BIN = os.environ.get("OUSTER_EDGE_BIN", "./ouster-edge")
CH, COLS, WIDTH, SECTORS = 64, 16, 1024, 360
PORT, SSE_PORT = 26532, 26603
FAR, NEAR = 30000, 1500          # mm
ZONE = "0:20:5.0:1:1:0"          # 0-20 deg, 5 m, fire on one column
                                 # (confirm=1) so this measures the reflex path


def px(r):
    return struct.pack("<IBBHHH", r, 200, 0, 0, 0, 0)


def packet(frame, mids, near_mids=()):
    body = b""
    for mid in mids:
        body += struct.pack("<QHH", mid * 1000, mid, 1)
        body += px(NEAR if mid in near_mids else FAR) * CH
    return struct.pack("<HHI", 0x1, frame, 0) + b"\0" * 24 + body + b"\0" * 32


def make_action():
    fd, path = tempfile.mkstemp(suffix=".sh")
    stamp = path + ".t"
    os.write(fd, f'#!/bin/sh\ndate +%s.%N >> {stamp}\n'.encode())
    os.close(fd)
    os.chmod(path, os.stat(path).st_mode | stat.S_IEXEC)
    return path, stamp


def start(action, status):
    p = subprocess.Popen(
        [BIN, "-f", "-p", str(PORT), "-c", str(CH), "-C", str(COLS),
         "-w", str(WIDTH), "-s", str(SECTORS), "-m", "0.1", "-M", "300",
         "-z", ZONE, "-a", action, "-S", status, "-I", "200",
         "-E", str(SSE_PORT)],
        stderr=subprocess.DEVNULL)
    time.sleep(0.7)
    return p


def main():
    action, stamp = make_action()
    status = tempfile.mkstemp(suffix=".json")[1]
    for f in (stamp,):
        if os.path.exists(f):
            os.remove(f)

    proc = start(action, status)
    if proc.poll() is not None:
        print("  FAIL  daemon exited immediately - is another instance holding "
              f"port {PORT}? check 'pgrep -x ouster-edge'")
        return 1
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fails = []

    # ---------- 1. zone alarm latency ----------
    # A clean revolution first, so the alarm is genuinely off beforehand.
    for start_mid in range(0, WIDTH, COLS):
        tx.sendto(packet(1, range(start_mid, start_mid + COLS)), ("127.0.0.1", PORT))
        time.sleep(0.0005)
    time.sleep(0.5)
    if os.path.exists(stamp):
        os.remove(stamp)

    # Now one packet whose columns land inside the zone, mid-revolution, so a
    # per-revolution design could not have reacted yet.
    intruding = set(range(16, 32))
    t_send = time.time()
    tx.sendto(packet(2, range(16, 32), intruding), ("127.0.0.1", PORT))

    deadline = time.time() + 3
    t_fire = None
    while time.time() < deadline:
        if os.path.exists(stamp) and os.path.getsize(stamp) > 0:
            t_fire = float(open(stamp).read().split()[0])
            break
        time.sleep(0.001)

    if t_fire is None:
        print("  FAIL  zone alarm never fired")
        fails.append("alarm")
    else:
        ms = (t_fire - t_send) * 1000
        # One revolution at 10 Hz is 100 ms; the point is to be well under it
        # while only 1/64th of the revolution has been delivered.
        ok = ms < 50
        print(f"  {'PASS' if ok else 'FAIL'}  zone alarm latency: {ms:.1f} ms "
              f"(after 1 of {WIDTH // COLS} packets; a per-revolution design "
              f"could not fire before ~100 ms at 10 Hz)")
        if not ok:
            fails.append("alarm-latency")

    # ---------- 2. SSE push vs status-file polling ----------
    events = []

    def reader():
        c = http.client.HTTPConnection("127.0.0.1", SSE_PORT, timeout=5)
        c.request("GET", "/")
        r = c.getresponse()
        buf = b""
        while len(events) < 3:
            # read1(), not read(): read() blocks until the buffer is full, so an
            # event only surfaces when the *next* one arrives, which looks
            # exactly like a full revolution of latency.
            chunk = r.read1(65536)
            if not chunk:
                break
            buf += chunk
            while b"\n\n" in buf:
                raw, buf = buf.split(b"\n\n", 1)
                if raw.startswith(b"data: "):
                    events.append((time.time(), json.loads(raw[6:])))

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    time.sleep(0.4)

    # A revolution is published when the *next* frame's first packet lands, so
    # that instant is the baseline to measure delivery against.
    t_close = {}
    for frame in (3, 4, 5, 6, 7):
        for i, start_mid in enumerate(range(0, WIDTH, COLS)):
            if i == 0:
                t_close[frame - 1] = time.time()
            tx.sendto(packet(frame, range(start_mid, start_mid + COLS)),
                      ("127.0.0.1", PORT))
            time.sleep(0.0005)
        time.sleep(0.06)

    th.join(timeout=3)

    if len(events) < 2:
        print(f"  FAIL  SSE delivered {len(events)} events, expected >= 2")
        fails.append("sse")
    else:
        lats = [(t_ev - t_close[ev["frame_id"]]) * 1000
                for t_ev, ev in events if ev["frame_id"] in t_close]
        if not lats:
            print("  FAIL  no event matched a known revolution")
            fails.append("sse-match")
        else:
            worst = max(lats)
            ok = worst < 20
            print(f"  {'PASS' if ok else 'FAIL'}  SSE delivery latency: "
                  f"{min(lats):.2f}-{worst:.2f} ms after the revolution closed")
            if not ok:
                fails.append("sse-latency")
        print(f"  INFO  status file write interval is 200 ms, and a poller adds "
              f"its own interval on top - that is what SSE removes")
        sectors = events[0][1]["sectors"]
        ok = sectors == SECTORS and len(events[0][1]["ring_cm"]) == SECTORS
        print(f"  {'PASS' if ok else 'FAIL'}  SSE payload carries the full ring "
              f"({len(events[0][1]['ring_cm'])} sectors)")
        if not ok:
            fails.append("sse-payload")

    proc.terminate()
    try:
        proc.wait(timeout=3)
        print("  PASS  SIGTERM stopped the daemon")
    except subprocess.TimeoutExpired:
        proc.kill()
        print("  FAIL  SIGTERM did not stop the daemon")
        fails.append("sigterm")

    for f in (action, stamp, status):
        if os.path.exists(f):
            os.remove(f)

    print()
    if fails:
        print(f"FAILED: {', '.join(fails)}")
        return 1
    print("all checks passed")
    return 0


def cleanup():
    for pat in ("ouster-edge",):
        pass


if __name__ == "__main__":
    rc = 1
    try:
        rc = main()
    finally:
        # main() terminates the daemon on the happy path; this catches the
        # exception paths, which is how a stray got left behind before.
        for line in subprocess.run(["pgrep", "-x", "ouster-edge"],
                                   capture_output=True, text=True).stdout.split():
            try:
                os.kill(int(line), 9)
            except (ProcessLookupError, ValueError):
                pass
    sys.exit(rc)
