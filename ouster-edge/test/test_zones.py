#!/usr/bin/env python3
"""Zone confirmation and hysteresis.

Firing on a single intruding column is the safe direction taken alone, but a
reflex that false-alarms on one dust return gets switched off by whoever works
next to it, and a disabled reflex is worse than a slightly slower one. So a zone
now needs N columns to agree, needs M quiet revolutions to release, and releases
only once the range clears the threshold plus a margin.

These are the properties that matter:
  - one stray close return does NOT fire a zone with confirm=2
  - enough columns in the same revolution DO fire it
  - a single quiet revolution does not release a zone with clear_after=2
  - an object parked exactly on the boundary does not chatter
  - confirm=1 still fires on the first column, for anyone who wants that
"""
import json, os, socket, struct, subprocess, sys, tempfile, time

BIN = os.environ.get("OUSTER_EDGE_BIN", "./ouster-edge")
CH, COLS, WIDTH, SECTORS = 64, 16, 1024, 360
PORT = 26552
FAR, NEAR = 30000, 1500

fails = []


def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: got {got!r} want {want!r}")
    if not ok:
        fails.append(name)


def px(r):
    return struct.pack("<IBBHHH", r, 200, 0, 0, 0, 0)


def packet(frame, mids, near_mids=(), near_mm=NEAR):
    body = b""
    for mid in mids:
        body += struct.pack("<QHH", mid * 1000, mid, 1)
        body += px(near_mm if mid in near_mids else FAR) * CH
    return struct.pack("<HHI", 0x1, frame, 0) + b"\0" * 24 + body + b"\0" * 32


def main():
    status = tempfile.mkstemp(suffix=".json")[1]
    os.remove(status)
    # zone 0: confirm 3, clear_after 2, 0.5 m release margin
    # zone 1: confirm 1 (fire immediately), clear_after 1
    proc = subprocess.Popen(
        [BIN, "-f", "-p", str(PORT), "-c", str(CH), "-C", str(COLS),
         "-w", str(WIDTH), "-s", str(SECTORS), "-m", "0.1", "-M", "300",
         "-z", "0:20:5.0:3:2:0.5", "-z", "40:60:5.0:1:1:0",
         "-S", status, "-I", "50"],
        stderr=subprocess.PIPE, text=True)
    time.sleep(0.7)
    if proc.poll() is not None:
        print("  FAIL  exited immediately:", proc.stderr.read())
        return 1

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    frame = [0]

    def send_rev(near_mids=(), near_mm=NEAR):
        """Send one full revolution.

        A revolution is published when the *next* one's first packet arrives, so
        this closes the previous revolution, not the one it sends. Modelling that
        precisely matters: a helper that also emitted a stub frame would close
        two revolutions per call and count the clear_after hysteresis twice.
        """
        frame[0] += 1
        for start in range(0, WIDTH, COLS):
            tx.sendto(packet(frame[0], range(start, start + COLS),
                             near_mids, near_mm), ("127.0.0.1", PORT))
            time.sleep(0.0004)
        time.sleep(0.2)

    def st():
        for _ in range(40):
            try:
                return json.load(open(status))
            except (OSError, ValueError):
                time.sleep(0.05)
        return {}

    # zone 0 covers 0-20 deg -> mids 0..56; zone 1 covers 40-60 -> mids ~114..170
    try:
        print("\n--- a single stray return must not fire confirm=3 ---")
        send_rev(near_mids={10})
        send_rev()                      # closes the one above
        check("zone 0 quiet on one column", st()["zones"][0], False)
        check("no aggregate alarm", st()["zone_alarm"], False)

        print("\n--- two is still not enough ---")
        send_rev(near_mids={10, 11})
        send_rev()
        check("zone 0 quiet on two columns", st()["zones"][0], False)

        print("\n--- three agreeing columns fire it ---")
        send_rev(near_mids={10, 11, 12})
        send_rev()                      # closing packet; zone fires on the way in
        check("zone 0 entered", st()["zones"][0], True)
        check("aggregate alarm", st()["zone_alarm"], True)
        check("enter counted", st()["zone_enters"][0], 1)

        print("\n--- one quiet revolution must NOT release clear_after=2 ---")
        send_rev()                      # closes quiet #1
        check("zone 0 still active", st()["zones"][0], True)
        check("alarm held", st()["zone_alarm"], True)

        print("\n--- the second releases it ---")
        send_rev()                      # closes quiet #2
        check("zone 0 released", st()["zones"][0], False)
        check("alarm released", st()["zone_alarm"], False)

        print("\n--- confirm=1 fires on the first column ---")
        send_rev(near_mids={130})
        send_rev()
        check("zone 1 entered on one column", st()["zones"][1], True)
        send_rev()
        check("zone 1 released after one quiet revolution", st()["zones"][1], False)

        print("\n--- an object on the boundary must not chatter ---")
        # 5.0 m threshold with a 0.5 m release margin: an already-active zone
        # holds anywhere below 5.5 m rather than flapping revolution to
        # revolution.
        send_rev(near_mids={10, 11, 12}, near_mm=4800)
        send_rev(near_mids={10, 11, 12}, near_mm=5200)
        check("entered just inside", st()["zones"][0], True)
        enters = st()["zone_enters"][0]
        for i in range(3):
            send_rev(near_mids={10, 11, 12}, near_mm=5200)
            check(f"held at 5.2 m, revolution {i + 1}", st()["zones"][0], True)
        check("no re-entry counted", st()["zone_enters"][0], enters)

        print("\n--- beyond the margin it releases ---")
        send_rev(near_mids={10, 11, 12}, near_mm=6000)
        send_rev(near_mids={10, 11, 12}, near_mm=6000)
        send_rev(near_mids={10, 11, 12}, near_mm=6000)
        check("released past the margin", st()["zones"][0], False)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
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
