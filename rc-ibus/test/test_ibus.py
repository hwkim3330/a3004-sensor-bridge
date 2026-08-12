#!/usr/bin/env python3
"""Verify rc-ibus against synthesised i-BUS frames over a pty pair.

No FlySky receiver on the bench, so a pty stands in for the USB-serial adapter.
What is checked is everything that could silently go wrong on real hardware:

  - channel values decode exactly, including the extremes
  - a frame with a bad checksum is rejected and counted, not acted on
  - the parser resynchronises after garbage without losing the stream
  - link loss is declared when frames stop, and recovered when they resume
  - the UDP forward carries the same values and the link flag
  - SIGTERM stops it
"""
import json
import os
import pty
import socket
import struct
import subprocess
import sys
import tempfile
import time

BIN = os.environ.get("RC_IBUS_BIN", "./rc-ibus")
UDP_PORT = 26710
CHANNELS = 14

fails = []


def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: got {got!r} want {want!r}")
    if not ok:
        fails.append(name)


def frame(channels, corrupt=False):
    body = bytes([0x20, 0x40]) + b"".join(
        struct.pack("<H", c) for c in channels)
    chk = 0xFFFF - sum(body)
    if corrupt:
        chk ^= 0x1234
    return body + struct.pack("<H", chk & 0xFFFF)


def read_status(path, tries=60):
    for _ in range(tries):
        try:
            with open(path) as fh:
                return json.load(fh)
        except (OSError, ValueError):
            time.sleep(0.05)
    return None


def wait_for(path, pred, timeout=4.0):
    end = time.time() + timeout
    last = None
    while time.time() < end:
        st = read_status(path, tries=1)
        if st:
            last = st
            if pred(st):
                return st
        time.sleep(0.05)
    return last


def main():
    status = tempfile.mkstemp(suffix=".json")[1]
    os.remove(status)

    master, slave = pty.openpty()
    dev = os.ttyname(slave)

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", UDP_PORT))
    rx.settimeout(3)

    proc = subprocess.Popen(
        [BIN, "-f", "-D", dev, "-b", "115200",
         "-r", f"127.0.0.1:{UDP_PORT}", "-t", "300",
         "-S", status, "-I", "50"],
        stderr=subprocess.PIPE, text=True)
    time.sleep(0.7)
    if proc.poll() is not None:
        print("  FAIL  rc-ibus exited immediately:", proc.stderr.read())
        return 1

    try:
        # ---- 1. a normal frame ----
        print("\n--- decoding ---")
        chans = [1000, 1500, 2000, 1234, 988, 2012,
                 1100, 1200, 1300, 1400, 1600, 1700, 1800, 1900]
        os.write(master, frame(chans))
        st = wait_for(status, lambda s: s["frames"] >= 1)
        check("channels decoded", st["channels"], chans)
        check("link up", st["link"], True)
        check("bad_crc", st["bad_crc"], 0)

        # ---- 2. the UDP forward agrees ----
        data, _ = rx.recvfrom(4096)
        check("udp magic", data[:4], b"IBUS")
        check("udp version", data[4], 1)
        check("udp channel count", data[5], CHANNELS)
        check("udp link flag", data[6], 1)
        check("udp channels",
              list(struct.unpack_from(f"<{CHANNELS}H", data, 8)), chans)

        # ---- 3. a corrupt frame must not be acted on ----
        print("\n--- integrity ---")
        before = st["frames"]
        bad = [4242] * CHANNELS
        os.write(master, frame(bad, corrupt=True))
        time.sleep(0.5)
        st = read_status(status)
        check("corrupt frame rejected", st["channels"], chans)
        check("frames unchanged", st["frames"], before)
        check("bad_crc counted", st["bad_crc"] >= 1, True)

        # ---- 4. resync after garbage ----
        print("\n--- resync ---")
        os.write(master, bytes([0x11, 0x22, 0x33, 0x20, 0x40, 0x99]))
        chans2 = [1555] * CHANNELS
        os.write(master, frame(chans2))
        st = wait_for(status, lambda s: s["channels"] == chans2)
        check("recovered after garbage", st["channels"], chans2)
        check("resyncs counted", st["resyncs"] >= 1, True)

        # ---- 5. link loss and recovery ----
        print("\n--- link loss ---")
        st = wait_for(status, lambda s: not s["link"], timeout=3)
        check("link lost after silence", st["link"], False)
        check("link_losses counted", st["link_losses"] >= 1, True)

        # downstream is told immediately, with the flag cleared
        try:
            while True:
                rx.settimeout(0.3)
                data, _ = rx.recvfrom(4096)
                if data[6] == 0:
                    break
            check("udp link flag cleared", data[6], 0)
        except socket.timeout:
            check("udp link flag cleared", "no datagram", "one with link=0")

        rx.settimeout(3)
        chans3 = [1777] * CHANNELS
        os.write(master, frame(chans3))
        st = wait_for(status, lambda s: s["link"] and s["channels"] == chans3)
        check("link recovered", st["link"], True)
        check("channels after recovery", st["channels"], chans3)

        # ---- 6. a realistic burst at the real frame rate ----
        print("\n--- 7.5 ms cadence, 100 frames ---")
        before = st["frames"]
        for i in range(100):
            v = 1000 + i * 10
            os.write(master, frame([v] * CHANNELS))
            time.sleep(0.0075)
        st = wait_for(status, lambda s: s["frames"] >= before + 100)
        check("all 100 frames seen", st["frames"] - before >= 100, True)
        check("last value", st["channels"][0], 1990)
        # 2, not 1: the garbage written in the resync test contained a
        # coincidental 0x20 0x40 pair, which is indistinguishable from a
        # genuinely corrupted frame and is counted the same way.
        check("no new bad crc in the burst", st["bad_crc"], 2)

        # ---- 7. shutdown ----
        print("\n--- shutdown ---")
        proc.terminate()
        try:
            proc.wait(timeout=3)
            check("SIGTERM stopped it", True, True)
        except subprocess.TimeoutExpired:
            proc.kill()
            check("SIGTERM stopped it", False, True)
    finally:
        if proc.poll() is None:
            proc.kill()
        os.close(master)
        os.close(slave)
        rx.close()
        if os.path.exists(status):
            os.remove(status)

    print()
    if fails:
        print(f"FAILED ({len(fails)}): {', '.join(fails)}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
