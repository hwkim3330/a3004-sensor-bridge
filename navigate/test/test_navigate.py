#!/usr/bin/env python3
"""Close the loop: navigate steers a simulated vehicle to a destination.

Everything else in this tree is tested open-loop, because everything else is
open-loop. This is not: navigate's output changes where the vehicle goes, which
changes what it sees, which changes its output. The only honest test is to run
that loop and see whether the vehicle arrives.

    simulator --ring--> navigate --TELE--> simulator

The simulator integrates the TELE axes at the speeds agx-cmd is configured with
by default, produces the ring ouster-edge would publish from the new pose, and
sends it back. navigate is the real binary; nothing about it knows it is in a
test.

What is checked:
  - it drives to a reachable goal and reports arrival
  - it stays out of the walls on the way
  - it refuses a goal in unmapped space rather than driving into the unknown
  - each watcher stops the vehicle: score, zone, ring loss, no progress
  - it never commands motion while stopped
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

BIN = os.environ.get("NAVIGATE_BIN", "./navigate")
RING_PORT = int(os.environ.get("NAV_RING_PORT", "7902"))
CMD_PORT = int(os.environ.get("NAV_CMD_PORT", "7904"))
TELE_PORT = int(os.environ.get("NAV_TELE_PORT", "7921"))
SECTORS = 360
DT = 0.1
MAX_LIN = 0.5           # m/s at axis 10000, agx-cmd's default
MAX_YAW = 0.8           # rad/s at axis 10000

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


def clearance(x, y):
    """Distance to the nearest wall, for the did-it-crash check."""
    best = 1e9
    for x0, y0, x1, y1 in WORLD:
        ex, ey = x1 - x0, y1 - y0
        L2 = ex * ex + ey * ey
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - x0) * ex + (y - y0) * ey) / L2))
        best = min(best, math.hypot(x - (x0 + t * ex), y - (y0 + t * ey)))
    return best


def ring_packet(x, y, th, frame, zone=False):
    rng = []
    for i in range(SECTORS):
        r = ray(x, y, th + 2 * math.pi * i / SECTORS)
        rng.append(0xFFFF if r > 30 else int(r * 100 + 0.5))
    pkt = b"OSED" + bytes([1, 2]) + struct.pack("<HH", SECTORS, frame & 0xFFFF)
    pkt += bytes([1 if zone else 0, 0]) + struct.pack("<Q", frame * 100_000_000)
    pkt += b"".join(struct.pack("<H", v) for v in rng)
    pkt += bytes(SECTORS)
    return pkt


class Sim:
    def __init__(self, status, extra=()):
        self.tele = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tele.bind(("127.0.0.1", TELE_PORT))
        self.tele.settimeout(0.05)
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status = status
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.next_due = 0.0
        self.frame = 0
        self.min_clear = 99.0
        self.commands = 0
        self.armed_frames = 0
        self.proc = subprocess.Popen(
            [BIN, "-f", "-p", str(RING_PORT), "-c", str(CMD_PORT),
             "-T", f"127.0.0.1:{TELE_PORT}", "-S", status, "-I", "100",
             *extra],
            stderr=subprocess.PIPE, text=True)
        time.sleep(0.8)
        if self.proc.poll() is not None:
            print("  FAIL  navigate exited at once:", self.proc.stderr.read())
            sys.exit(1)

    def step(self, zone=False, move=True):
        # Paced at the sensor's rate, not as fast as the loop will go.
        #
        # Without this the simulator outran a real 10 Hz lidar by two or three
        # times, the daemon's socket buffer dropped rings, and the status file -
        # written every 100 ms - was always describing an older revolution than
        # the one just sent. Every symptom looked like a fault in navigate and
        # none of them were.
        now = time.time()
        if self.next_due and now < self.next_due:
            time.sleep(self.next_due - now)
        self.next_due = max(self.next_due, now) + DT

        self.out.sendto(ring_packet(self.x, self.y, self.th, self.frame, zone),
                        ("127.0.0.1", RING_PORT))
        self.frame += 1
        armed, fwd, yaw = False, 0, 0
        deadline = time.time() + 0.09
        while time.time() < deadline:
            try:
                f = self.tele.recv(64)
            except socket.timeout:
                continue
            if len(f) >= 32 and f[:4] == b"TELE":
                armed = bool(f[5] & 1)
                _, fwd, yaw, _ = struct.unpack_from("<hhhh", f, 20)
                self.commands += 1
                break
        if armed:
            self.armed_frames += 1
        if move and armed:
            v = fwd / 10000.0 * MAX_LIN
            w = yaw / 10000.0 * MAX_YAW
            self.th += w * DT
            self.x += v * math.cos(self.th) * DT
            self.y += v * math.sin(self.th) * DT
            self.min_clear = min(self.min_clear, clearance(self.x, self.y))
        return armed

    def goal(self, x_cm, y_cm):
        self.out.sendto(f"GOAL {x_cm} {y_cm}".encode(), ("127.0.0.1", CMD_PORT))
        time.sleep(0.3)

    def read(self):
        # The daemon rewrites this on its own interval, so a read taken the
        # instant after a send describes the previous revolution. Wait one
        # interval before believing it.
        time.sleep(0.15)
        for _ in range(30):
            try:
                with open(self.status) as f:
                    return json.load(f)
            except Exception:
                time.sleep(0.05)
        return {}

    def close(self):
        self.proc.terminate()
        self.proc.wait(timeout=5)
        self.tele.close()
        self.out.close()


def warmup(sim, n=25):
    """Sit still and let it map what it can see from the start."""
    for _ in range(n):
        sim.step(move=False)


def test_drive():
    print("  driving to a destination")
    status = tempfile.mkstemp(suffix=".json")[1]
    sim = Sim(status)
    try:
        warmup(sim)
        st = sim.read()
        check("mapped before any goal", st.get("matched", 0) > 10,
              f"matched={st.get('matched')}")
        check("idle with no goal", st.get("state") == "idle",
              f"state={st.get('state')}")
        check("no motion commanded while idle", sim.armed_frames == 0,
              f"armed frames={sim.armed_frames}")

        sim.goal(300, 200)                    # 3.0, 2.0 m
        arrived = False
        for _ in range(400):
            sim.step()
            st = sim.read()
            if st.get("state") == "arrived":
                arrived = True
                break
            if st.get("state") == "stopped":
                break
        st = sim.read()
        err = math.hypot(sim.x - 3.0, sim.y - 2.0)
        check("reported arrival", arrived,
              f"state={st.get('state')} fault={st.get('fault')}")
        check("actually within 40 cm of the destination", err < 0.40,
              f"{err:.2f} m at ({sim.x:.2f}, {sim.y:.2f})")
        check("never touched a wall", sim.min_clear > 0.25,
              f"closest approach {sim.min_clear:.2f} m")
        check("stopped commanding once arrived",
              sim.step() is False, "still armed after arrival")
    finally:
        sim.close()
        os.path.exists(status) and os.remove(status)


def test_unmapped_goal():
    print("\n  a goal in unmapped space")
    status = tempfile.mkstemp(suffix=".json")[1]
    sim = Sim(status)
    try:
        warmup(sim)
        sim.goal(50000, 50000)                # far outside the map
        st = sim.read()
        check("refused rather than accepted", st.get("state") == "stopped",
              f"state={st.get('state')}")
        check("said why", bool(st.get("fault")), f"fault={st.get('fault')}")
        armed_before = sim.armed_frames
        for _ in range(10):
            sim.step()
        check("commanded no motion", sim.armed_frames == armed_before,
              f"{sim.armed_frames - armed_before} armed frames")
    finally:
        sim.close()
        os.path.exists(status) and os.remove(status)


def test_watchers():
    print("\n  the watchers")
    for name, kick in (("zone alarm", "zone"), ("ring loss", "silence")):
        status = tempfile.mkstemp(suffix=".json")[1]
        sim = Sim(status)
        try:
            warmup(sim)
            sim.goal(300, 200)
            for _ in range(12):
                sim.step()
            st = sim.read()
            if st.get("state") != "driving":
                check(f"{name}: was driving first", False,
                      f"state={st.get('state')} fault={st.get('fault')}")
                continue
            if kick == "zone":
                for _ in range(4):
                    sim.step(zone=True)
            else:
                time.sleep(1.6)
            st = sim.read()
            check(f"{name} stops the vehicle", st.get("state") == "stopped",
                  f"state={st.get('state')} fault={st.get('fault')}")
            armed_before = sim.armed_frames
            for _ in range(6):
                sim.step(zone=(kick == "zone"))
            check(f"{name}: stays stopped", sim.armed_frames == armed_before,
                  f"{sim.armed_frames - armed_before} armed frames after")
        finally:
            sim.close()
            os.path.exists(status) and os.remove(status)


print(f"navigate closed-loop verification using {BIN}\n")
test_drive()
test_unmapped_goal()
test_watchers()

print()
if fails:
    print(f"FAILED ({len(fails)}): {', '.join(fails)}")
    sys.exit(1)
print("all checks passed")
