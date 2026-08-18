#!/usr/bin/env python3
"""How far the router's clock is from this machine's, to a few milliseconds.

frames.py compares the arrival time of a frame here against the X-Timestamp
ustreamer wrote there, and those are two different clocks. On this bench the
router was 1017 s behind with ntpd enabled, so assuming they agree would have
made every frame look seventeen minutes old.

BusyBox date has no %N, so there is no sub-second reading to ask for. Instead the
router is asked for the time as fast as it can answer, and the moment the integer
changes is the moment its clock crossed a second boundary; the local time at that
instant gives the offset. The resolution is one turn of that loop, about 13 ms on
this board, and the answer is averaged over several boundaries.

    clock-offset.py [host] [--boundaries 5]

The loop is bounded on the router with `timeout`, not by killing ssh.

That is not a detail. Killing ssh does not stop it: each `date` dies of SIGPIPE
and the shell loop cheerfully spawns another, so an interrupted run leaves a
spinner burning a few percent of a core on the board indefinitely. This happened
twice here - once found in `top` an hour later, once four at a time - and no trap
on the remote side fixes it, because the shell is not what receives the signal.
A limit the router enforces on itself is the only version that cannot be orphaned.
"""
import argparse
import statistics
import subprocess
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("host", nargs="?", default="192.168.1.1")
    ap.add_argument("--boundaries", type=int, default=5)
    a = ap.parse_args()

    # Enough seconds to see the boundaries, plus a little; the router stops on
    # its own when this expires whatever happens at this end.
    secs = a.boundaries + 3
    p = subprocess.Popen(
        ["ssh", "-o", "ConnectTimeout=5", f"root@{a.host}",
         f"timeout {secs} sh -c 'while :; do date +%s; done'"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)

    offsets = []
    last = None
    n = 0
    try:
        while len(offsets) < a.boundaries:
            line = p.stdout.readline()
            if not line:
                break
            now = time.time()
            v = line.strip()
            if not v.isdigit():
                continue
            n += 1
            if last is not None and v != last:
                offsets.append(now - int(v))
            last = v
    finally:
        p.terminate()
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()

    if len(offsets) < 2:
        sys.exit(f"only {len(offsets)} boundaries from {a.host} - is ssh working?")
    off = statistics.median(offsets)
    spread = max(offsets) - min(offsets)
    print(f"  pc minus router: {off:+.3f} s   "
          f"(median of {len(offsets)}, spread {spread*1000:.0f} ms, "
          f"{n} samples)")
    print(f"  frames.py --clock-offset {off:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
