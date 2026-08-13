#!/usr/bin/env python3
"""Verify the teleop safety properties, which are the only reason this daemon
exists. Everything here is a failure mode of driving something over WiFi:

  - nothing is emitted as armed until explicitly armed
  - a stale command trips the deadman: neutral output AND the armed flag drops,
    rather than simply going quiet
  - the disarmed frames keep flowing, so a receiver can tell "neutral" from
    "gone"
  - replayed or reordered commands are rejected, not acted on
  - axes are clamped
  - an explicit disarm takes effect immediately
  - shutdown emits a final neutral disarmed frame
"""
import http.client
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time

BIN = os.environ.get("TELEOP_BIN", "./teleop")
HTTP_PORT = 26830
UDP_PORT = 26720
CMD_PORT = 26721
RATE = 20
TIMEOUT_MS = 300

fails = []


def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: got {got!r} want {want!r}")
    if not ok:
        fails.append(name)


def decode(pkt):
    assert pkt[:4] == b"TELE", pkt[:4]
    ver, flags = pkt[4], pkt[5]
    (seq,) = struct.unpack_from("<I", pkt, 8)
    (ts,) = struct.unpack_from("<Q", pkt, 12)
    axes = [v / 10000.0 for v in struct.unpack_from("<4h", pkt, 20)]
    (buttons,) = struct.unpack_from("<H", pkt, 28)
    return dict(ver=ver, armed=bool(flags & 1), seq=seq, ts=ts,
                axes=axes, buttons=buttons)


def main():
    status = tempfile.mkstemp(suffix=".json")[1]
    os.remove(status)

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", UDP_PORT))
    rx.settimeout(2)

    proc = subprocess.Popen(
        [BIN, "-f", "-p", str(HTTP_PORT), "-r", f"127.0.0.1:{UDP_PORT}",
         "-H", str(RATE), "-t", str(TIMEOUT_MS), "-S", status,
         "-c", str(CMD_PORT)],
        stderr=subprocess.PIPE, text=True)
    time.sleep(0.7)
    if proc.poll() is not None:
        print("  FAIL  teleop exited immediately:", proc.stderr.read())
        return 1

    conn = http.client.HTTPConnection("127.0.0.1", HTTP_PORT, timeout=3)
    seq = [0]

    def cmd(arm=1, a0=0, a1=0, a2=0, a3=0, b=0, s=None):
        if s is None:
            seq[0] += 1
            s = seq[0]
        conn.request("GET", f"/cmd?s={s}&arm={arm}&a0={a0}&a1={a1}"
                            f"&a2={a2}&a3={a3}&b={b}")
        r = conn.getresponse()
        r.read()
        return r.status

    def flush():
        """Discard queued frames. They accumulate at the forward rate while the
        test is doing something else, and reading them later would measure the
        past rather than the present."""
        rx.settimeout(0.01)
        try:
            while True:
                rx.recvfrom(4096)
        except (socket.timeout, BlockingIOError):
            pass

    def drain(n=1, timeout=2):
        """Collect the next n forwarded datagrams."""
        out = []
        rx.settimeout(timeout)
        for _ in range(n):
            try:
                out.append(decode(rx.recvfrom(4096)[0]))
            except socket.timeout:
                break
        return out

    def st():
        for _ in range(40):
            try:
                return json.load(open(status))
            except (OSError, ValueError):
                time.sleep(0.05)
        return {}

    try:
        # ---- 1. forwarding happens before arming, marked disarmed ----
        print("\n--- disarmed baseline ---")
        flush()
        f = drain(3)
        check("frames arrive unarmed", len(f), 3)
        check("marked disarmed", [x["armed"] for x in f], [False] * 3)
        check("neutral axes", f[0]["axes"], [0.0, 0.0, 0.0, 0.0])
        check("wire version", f[0]["ver"], 1)
        check("seq increments",
              f[1]["seq"] - f[0]["seq"] == 1 and f[2]["seq"] - f[1]["seq"] == 1,
              True)

        # ---- 2. arming and axis values ----
        print("\n--- armed ---")
        flush()
        # a0 strafe, a1 forward, a2 yaw - three axes because the vehicle is
        # holonomic; see doc/TELEOP.md
        check("http status", cmd(arm=1, a0=5000, a1=-2500, a2=7500, b=5), 204)
        time.sleep(0.15)
        f = drain(3)
        armed = [x for x in f if x["armed"]]
        check("armed frames seen", len(armed) >= 1, True)
        check("a0 strafe", armed[0]["axes"][0], 0.5)
        check("a1 forward", armed[0]["axes"][1], -0.25)
        check("a2 yaw", armed[0]["axes"][2], 0.75)
        check("a3 unused stays zero", armed[0]["axes"][3], 0.0)
        check("buttons", armed[0]["buttons"], 5)

        # ---- 3. clamping ----
        print("\n--- clamping ---")
        flush()
        cmd(arm=1, a0=99999, a1=-99999)
        time.sleep(0.15)
        f = [x for x in drain(3) if x["armed"]]
        check("clamped high", f[0]["axes"][0], 1.0)
        check("clamped low", f[0]["axes"][1], -1.0)

        # ---- 4. replay / reorder rejection ----
        # Kept tight on purpose: the deadman is 300 ms, and it fires correctly,
        # so anything that sleeps its way through this section is measuring the
        # deadman rather than the sequence check.
        print("\n--- sequence ---")
        cmd(arm=1, a0=4200)               # a distinctive fresh value
        time.sleep(0.12)
        before = st()
        check("fresh command applied", before["axes"][0], 0.42)
        cmd(arm=1, a0=7777, s=1)          # ancient sequence number
        time.sleep(0.12)
        s2 = st()
        check("replay not counted as command", s2["commands"], before["commands"])
        check("replay rejected", s2["rejected_seq"] >= 1, True)
        check("replayed axis ignored", s2["axes"][0], 0.42)
        check("still armed", s2["armed"], True)

        # A negative sequence has to be refused rather than cast. "s=-5" is
        # 4294967291 as a uint32, and once that is stored every ordinary
        # sequence looks like a backwards jump of more than 1000 - a client
        # restart - so replay protection would be gone for the rest of the
        # process's life.
        #
        # Asserted on counters, not on axis values: the deadman is 300 ms, and
        # the section above deliberately leaves the last accepted command
        # already ~240 ms old, so an axis check here measures the deadman.
        before = st()
        cmd(arm=1, a0=9999, s=-5)
        time.sleep(0.15)          # > the 100 ms status-file interval
        s2b = st()
        check("negative sequence counted malformed",
              s2b["malformed"] >= before["malformed"] + 1, True)
        check("negative sequence not counted as command",
              s2b["commands"], before["commands"])

        # The real property: replay protection is still intact afterwards.
        rej = s2b["rejected_seq"]
        cmd(arm=1, a0=8888, s=2)          # ancient again
        time.sleep(0.15)
        check("replay still rejected after a negative sequence",
              st()["rejected_seq"] >= rej + 1, True)

        # ---- 5. the deadman ----
        print("\n--- deadman ---")
        cmd(arm=1, a0=6000)
        time.sleep(0.12)
        check("armed before silence", st()["armed"], True)
        time.sleep((TIMEOUT_MS + 250) / 1000.0)
        s3 = st()
        check("disarmed after silence", s3["armed"], False)
        check("deadman tripped", s3["deadman_trips"] >= 1, True)
        check("axes neutralised", s3["axes"], [0.0, 0.0, 0.0, 0.0])

        flush()
        f = drain(3)
        check("still forwarding after trip", len(f), 3)
        check("frames marked disarmed", [x["armed"] for x in f], [False] * 3)
        check("frames neutral", f[0]["axes"], [0.0, 0.0, 0.0, 0.0])

        # ---- 6. re-arm, then explicit disarm ----
        print("\n--- explicit disarm ---")
        cmd(arm=1, a0=3000)
        time.sleep(0.15)
        check("re-armed", st()["armed"], True)
        cmd(arm=0)
        time.sleep(0.15)
        s4 = st()
        check("disarmed on request", s4["armed"], False)
        check("axes neutral", s4["axes"], [0.0, 0.0, 0.0, 0.0])

        # ---- 7. malformed input ----
        print("\n--- malformed ---")
        before = st()["malformed"]
        conn.request("GET", "/cmd?arm=1&a0=5000")   # no sequence number
        conn.getresponse().read()
        conn.request("GET", "/nonsense")
        conn.getresponse().read()
        time.sleep(0.15)
        check("malformed counted", st()["malformed"] >= before + 2, True)
        check("still disarmed", st()["armed"], False)

        # ---- 7b. the UDP command path a native app uses ----
        # A browser must use HTTP; an app should not have to, and UDP avoids the
        # connection-limit and keep-alive problems that path already hit once.
        print("\n--- udp commands ---")
        ctx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        def ucmd(arm, a0, a1, s_):
            pkt = b"TCMD" + bytes([1, 1 if arm else 0, 0, 0])
            pkt += struct.pack("<I", s_)
            pkt += struct.pack("<4h", a0, a1, 0, 0)
            pkt += struct.pack("<H", 0) + b"\0\0"
            ctx.sendto(pkt, ("127.0.0.1", CMD_PORT))

        useq = 500000
        for _ in range(6):
            useq += 1
            ucmd(True, -6000, 4500, useq)
            time.sleep(0.03)
        time.sleep(0.15)
        s5 = st()
        check("udp armed", s5["armed"], True)
        check("udp a0 strafe", s5["axes"][0], -0.6)
        check("udp a1 forward", s5["axes"][1], 0.45)
        check("udp commands counted", s5["udp_commands"] >= 6, True)

        flush()
        time.sleep(0.12)
        f = [x for x in drain(3) if x["armed"]]
        check("udp reaches the wire", f[0]["axes"][0], -0.6)

        # A short packet must be rejected, not half-applied. Re-send a fresh
        # command first: the deadman is 300 ms and the drain above spends most
        # of it, so without this the check measures the deadman instead.
        useq += 1
        ucmd(True, -6000, 4500, useq)
        time.sleep(0.08)
        before = st()["udp_malformed"]
        ctx.sendto(b"TCMD\x01\x01\x00\x00", ("127.0.0.1", CMD_PORT))
        time.sleep(0.10)
        s6 = st()
        check("truncated udp rejected", s6["udp_malformed"] >= before + 1, True)
        check("axes unchanged", s6["axes"][0], -0.6)

        # a restarted app resets its sequence; it must not be locked out
        ucmd(True, 1000, 0, 3)
        time.sleep(0.15)
        check("udp client restart accepted", st()["axes"][0], 0.1)

        useq += 1
        ucmd(False, 0, 0, useq + 100000)
        time.sleep(0.12)
        check("udp disarm", st()["armed"], False)
        ctx.close()

        # ---- 8. a final neutral frame on shutdown ----
        print("\n--- shutdown ---")
        cmd(arm=1, a0=9000)
        time.sleep(0.15)
        check("armed again", st()["armed"], True)
        flush()
        proc.terminate()
        try:
            proc.wait(timeout=3)
            check("SIGTERM stopped it", True, True)
        except subprocess.TimeoutExpired:
            proc.kill()
            check("SIGTERM stopped it", False, True)
        last = drain(4, timeout=1)
        if last:
            check("final frame disarmed", last[-1]["armed"], False)
            check("final frame neutral", last[-1]["axes"], [0.0] * 4)
        else:
            check("final frame present", "none", "one")
    finally:
        if proc.poll() is None:
            proc.kill()
        try:
            conn.close()
        except OSError:
            pass
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
