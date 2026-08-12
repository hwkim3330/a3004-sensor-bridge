#!/usr/bin/env python3
"""Boot the sensor bridge under QEMU and check that first boot actually works.

Why this exists: flashing the router is slow and each iteration costs a
sysupgrade, but most of what can go wrong on first boot has nothing to do with
the board. A typo in a uci-defaults script, a procd service that exits without
saying why, a missing runtime dependency, a symlink into /var that is not there
yet - all of those are userspace, and all of them can be found here in a minute
instead of at the bench.

malta/le is mipsel_24kc, byte-for-byte the same architecture as ramips/mt7621,
so these are the *same package binaries*: same endianness, same soft-float, same
alignment rules, real procd, real uci, real uhttpd.

What this CANNOT tell you, and do not let it imply otherwise:
  - anything about the device tree, mt76 or DBDC. QEMU has no MT7615D.
  - whether the image flashes, or whether u-boot accepts its name.
  - real USB, a real camera, real lidar throughput, real timing under load.

Usage:
    python3 run-emu.py                 # one boot, run the checks, report
    python3 run-emu.py --loop 20       # repeat, to catch init-ordering flakiness
    python3 run-emu.py --shell         # boot and hand over an interactive console
"""

import argparse
import os
import re
import select
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TREE = os.path.abspath(os.path.join(HERE, "..", "..", ".."))

# Guest port -> host port. QEMU user-mode networking forwards both TCP and UDP,
# which is what lets the lidar feed be injected from outside.
FWD_TCP = {80: 8180, 8080: 8280, 8082: 8282, 8083: 8283, 7603: 8303}
FWD_UDP = {7502: 8502, 7721: 8721}

# OpenWrt's br-lan is statically 192.168.1.1 and never asks for DHCP, so QEMU's
# user network is pointed at the same subnet and forwards are addressed to that
# IP explicitly. With the default 10.0.2.0/24 the guest is simply not there.
GUEST_IP = "192.168.1.1"
NET_OPTS = f"net=192.168.1.0/24,host=192.168.1.2,dhcpstart=192.168.1.100"

PROMPT = re.compile(rb"root@[\w-]+:[^\n]*# ")


def find_kernel():
    for sub in ("le", "be"):
        d = os.path.join(TREE, "bin", "targets", "malta", sub)
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            if f.endswith("-initramfs-kernel.bin") or \
               f.endswith("vmlinux-initramfs.elf") or \
               f.endswith("vmlinux-initramfs"):
                return os.path.join(d, f), sub
    return None, None


class Emu:
    def __init__(self, kernel, endian, verbose=False):
        self.verbose = verbose
        binary = ("qemu-system-mipsel" if endian == "le"
                  else "qemu-system-mips")
        if not shutil.which(binary):
            sys.exit(f"{binary} not found: apt install qemu-system-mips")

        fwd = ",".join(
            [NET_OPTS] +
            [f"hostfwd=tcp::{h}-{GUEST_IP}:{g}" for g, h in FWD_TCP.items()] +
            [f"hostfwd=udp::{h}-{GUEST_IP}:{g}" for g, h in FWD_UDP.items()])

        self.proc = subprocess.Popen(
            [binary, "-M", "malta", "-m", "256", "-kernel", kernel,
             "-nographic", "-no-reboot",
             "-netdev", f"user,id=n0,{fwd}",
             "-device", "pcnet,netdev=n0",
             "-append", "console=ttyS0 rootfstype=squashfs,jffs2"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, bufsize=0)
        self.buf = b""

    def read_until(self, pattern, timeout=90):
        """Accumulate console output until pattern matches or time runs out."""
        end = time.time() + timeout
        rx = pattern if hasattr(pattern, "search") else re.compile(pattern)
        while time.time() < end:
            if self.proc.poll() is not None:
                return False
            r, _, _ = select.select([self.proc.stdout], [], [], 0.5)
            if r:
                chunk = self.proc.stdout.read(4096)
                if not chunk:
                    break
                self.buf += chunk
                if self.verbose:
                    sys.stdout.write(chunk.decode("utf-8", "replace"))
                    sys.stdout.flush()
            if rx.search(self.buf):
                return True
        return False

    def wake(self, attempts=20):
        """Nudge the console until a prompt appears.

        Sending one newline the instant the banner prints does not reliably wake
        it: the kernel is still emitting module output at that point and
        askconsole has only just started. A human presses Enter again; so does
        this.
        """
        for _ in range(attempts):
            try:
                self.proc.stdin.write(b"\n")
                self.proc.stdin.flush()
            except Exception:
                return False
            if self.read_until(PROMPT, 3):
                return True
        return False

    def wait_ready(self, timeout=120):
        """Wait until procd has actually finished bringing services up.

        The console prompt is not that signal: askconsole offers a shell early,
        while procd carries on starting services for tens of seconds afterwards.
        Checking at the prompt reported every service as down and cost a while to
        work out - the firmware was fine and the harness was early.
        """
        end = time.time() + timeout
        while time.time() < end:
            out = self.cmd("pgrep -x uhttpd >/dev/null && "
                           "pgrep -x ouster-edge >/dev/null && "
                           "echo $((11*11)) || echo 0", timeout=20)
            if out and "121" in out:
                return True
            time.sleep(3)
        return False

    def cmd(self, line, timeout=45):
        """Run a shell command and return only its output.

        Two defences against reading the shell's own echo as the answer, which
        is what this harness did at first and what made every service look
        down:

        1. echo is turned off on the guest once, at setup.
        2. the end of output is marked by a sentinel the *command text* cannot
           contain: the command says $((7*11)) and the output says 77. Even if
           echo came back, it could not be mistaken for the reply.

        Long command lines also wrap on an 80-column console, so anything that
        parses "the first line" is unreliable. This parses between markers
        instead.
        """
        self.buf = b""
        marker = 'EOC$((7*11))'
        want = b"EOC77"
        self.proc.stdin.write(f"{line}; echo '{marker[:3]}'$((7*11))\n".encode())
        self.proc.stdin.flush()
        if not self.read_until(re.compile(re.escape(want)), timeout):
            return None
        out = self.buf.decode("utf-8", "replace")
        # everything before the sentinel, minus any echoed command text
        out = out.split("EOC77")[0]
        keep = []
        for l in out.splitlines():
            t = l.strip()
            if not t:
                continue
            # drop echoes and prompts if the guest still has echo on
            if "EOC" in t or "$((" in t or t.endswith("# ") or \
               PROMPT.search(t.encode()):
                continue
            if line.split(";")[0].strip()[:24] and \
               line.split(";")[0].strip()[:24] in t:
                continue
            keep.append(t)
        return "\n".join(keep).strip()

    def stop(self):
        try:
            self.proc.stdin.write(b"\n")
            self.proc.stdin.flush()
        except Exception:
            pass
        self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def feed_lidar(host_port, seconds=2.0, hz=10):
    """Inject synthetic OS-64 packets so the ring path is exercised for real."""
    ch, cols, width = 64, 16, 1024
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    frame = 100
    t_end = time.time() + seconds
    sent = 0
    while time.time() < t_end:
        for start in range(0, width, cols):
            body = b""
            for mid in range(start, start + cols):
                body += struct.pack("<QHH", mid * 1000, mid, 1)
                # a wall at 4 m, and something close in one sector
                mm = 1800 if 100 <= mid < 130 else 4000
                body += struct.pack("<IBBHHH", mm, 200, 0, 0, 0, 0) * ch
            pkt = struct.pack("<HHI", 0x1, frame & 0xffff, 0) + b"\0" * 24 \
                  + body + b"\0" * 32
            s.sendto(pkt, ("127.0.0.1", host_port))
            sent += 1
            time.sleep(1.0 / hz / (width / cols))
        frame += 1
    s.close()
    return sent


def sse_probe(port, timeout=6):
    """Read the first bytes of a Server-Sent Events stream.

    urlopen().read() cannot be used here: the response never ends, so a blocking
    read waits for the timeout and reports failure on a working endpoint. Take
    the first chunk and look at it instead.
    """
    try:
        s = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        s.settimeout(timeout)
        s.sendall(b"GET / HTTP/1.0\r\nHost: x\r\n\r\n")
        buf = b""
        end = time.time() + timeout
        while time.time() < end and len(buf) < 4096:
            try:
                chunk = s.recv(2048)
            except socket.timeout:
                break
            if not chunk:
                break
            buf += chunk
            if b"data: " in buf:
                break
        s.close()
        return buf
    except Exception as e:
        return str(e).encode()


def http(port, path="/", timeout=4):
    import urllib.request
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}",
                                    timeout=timeout) as r:
            return r.status, r.read()
    except Exception as e:
        return None, str(e).encode()


def run_once(kernel, endian, verbose):
    fails = []

    def check(name, ok, detail=""):
        print(f"  {'PASS' if ok else 'FAIL'}  {name}" +
              (f"  {detail}" if detail and not ok else ""))
        if not ok:
            fails.append(name)

    emu = Emu(kernel, endian, verbose)
    try:
        print("  booting ...", flush=True)
        if not emu.read_until(rb"Please press Enter to activate", 180):
            print("  FAIL  never reached the console banner")
            tail = emu.buf.decode("utf-8", "replace")[-1500:]
            print(tail)
            return ["boot"]
        if not emu.wake():
            print("  FAIL  console never produced a prompt")
            print(emu.buf.decode("utf-8", "replace")[-1200:])
            return ["prompt"]

        # No echo, and no wrapping: both confused the output parser.
        emu.proc.stdin.write(b"stty -echo 2>/dev/null; stty columns 400 2>/dev/null\n")
        emu.proc.stdin.flush()
        time.sleep(1)
        emu.buf = b""

        print("  waiting for procd to settle ...", flush=True)
        settled = emu.wait_ready()
        check("services settled within 120 s", settled)

        # --- the thing most likely to be quietly broken ---
        print("\n  uci-defaults")
        left = emu.cmd("ls /etc/uci-defaults/ 2>/dev/null | wc -l")
        check("uci-defaults all consumed", left is not None and
              left.strip().endswith("0"), f"left: {left}")
        # a script that fails is *renamed*, not removed, so its absence is the
        # only evidence it succeeded
        for opt in ("ustreamer.video0.format", "ouster-edge.lidar.enabled",
                    "mic-stream.capture.device"):
            v = emu.cmd(f"uci -q get {opt}")
            check(f"uci {opt}", bool(v and v.strip()), f"got {v!r}")
        fw = emu.cmd("uci -q get firewall.sensorkit_stream.dest_port")
        check("firewall rule installed", bool(fw and "8080" in fw), f"got {fw!r}")

        # --- do the services actually run ---
        print("\n  services")
        for svc in ("ouster-edge", "mic-stream", "teleop", "uhttpd"):
            out = emu.cmd(f"pgrep -x {svc} >/dev/null && echo up || echo down")
            expect_up = svc in ("ouster-edge", "uhttpd")
            got_up = bool(out and "up" in out)
            if expect_up and not got_up:
                # A service that is down is only useful with the reason attached
                why = emu.cmd(f"logread | grep -i {svc} | tail -4")
                enabled = emu.cmd(f"ls -l /etc/rc.d/ 2>/dev/null | grep -c {svc}")
                print(f"  DIAG  {svc}: rc.d entries={enabled}")
                if why:
                    for l in why.splitlines()[-4:]:
                        print(f"  DIAG  {l}")
            if expect_up:
                check(f"{svc} running", got_up, f"got {out!r}")
            else:
                # mic-stream and teleop are off or hardware-dependent by
                # default; only report, do not fail
                print(f"  INFO  {svc}: {'up' if got_up else 'down'}")

        # --- the dashboard, from outside ---
        print("\n  dashboard over the forwarded port")
        st, body = http(FWD_TCP[80], "/sensors/")
        check("GET /sensors/ is 200", st == 200, f"got {st} {body[:80]}")
        check("page is the dashboard",
              st == 200 and b"A3004NS-M Sensor Bridge" in body)
        st, body = http(FWD_TCP[80], "/sensors/ouster.json")
        check("status symlink resolves", st == 200, f"got {st}")

        # --- real packets through the real daemon ---
        print("\n  lidar path with synthetic packets")

        # The SSE probe has to be listening *while* packets flow: an event is
        # only emitted when a revolution completes, so connecting afterwards
        # correctly gets headers and nothing else.
        sse_box = {}

        def probe():
            sse_box["data"] = sse_probe(FWD_TCP[7603], timeout=12)

        th = threading.Thread(target=probe, daemon=True)
        th.start()
        time.sleep(1.0)

        sent = feed_lidar(FWD_UDP[7502], seconds=4.0)
        th.join(timeout=14)
        time.sleep(0.7)

        st, body = http(FWD_TCP[80], "/sensors/ouster.json")
        ok = st == 200
        pkts = frames = -1
        if ok:
            import json
            try:
                j = json.loads(body)
                pkts, frames = j.get("packets", -1), j.get("frames", -1)
            except Exception:
                ok = False
        check(f"daemon saw packets (sent {sent})", pkts > 0, f"packets={pkts}")
        check("revolutions completed", frames > 0, f"frames={frames}")

        sse = sse_box.get("data", b"")
        check("SSE endpoint answers", b"text/event-stream" in sse,
              f"got {sse[:100]!r}")
        check("SSE pushed an event while packets flowed", b"data: " in sse,
              f"got {sse[-140:]!r}")
        check("SSE event carries a ring",
              b'"ring_cm"' in sse and b'"sectors"' in sse,
              f"got {sse[-160:]!r}")

        # --- the report script must run without erroring ---
        print("\n  first-boot-report")
        out = emu.cmd("first-boot-report 2>&1 | tail -3", timeout=60)
        check("report runs to the end", bool(out and "end" in out),
              f"tail: {out!r}")

        print("\n  logs")
        errs = emu.cmd("logread | grep -icE 'error|fatal' || true")
        print(f"  INFO  log lines matching error/fatal: {errs}")
        crash = emu.cmd("logread | grep -iE 'segfault|oom|kernel BUG' | head -3")
        check("no segfault/oom/BUG in the log", not (crash and crash.strip()),
              f"{crash!r}")
    finally:
        emu.stop()
    return fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--loop", type=int, default=1)
    ap.add_argument("--shell", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    kernel, endian = find_kernel()
    if not kernel:
        sys.exit("no malta initramfs image found. Build one first:\n"
                 "  CONFIG_TARGET_malta=y / CONFIG_TARGET_malta_le=y\n"
                 "  CONFIG_TARGET_ROOTFS_INITRAMFS=y")
    print(f"kernel: {os.path.relpath(kernel, TREE)}  ({endian})")

    if args.shell:
        binary = "qemu-system-mipsel" if endian == "le" else "qemu-system-mips"
        fwd = ",".join(
            [NET_OPTS] +
            [f"hostfwd=tcp::{h}-{GUEST_IP}:{g}" for g, h in FWD_TCP.items()] +
            [f"hostfwd=udp::{h}-{GUEST_IP}:{g}" for g, h in FWD_UDP.items()])
        print("dashboard: http://127.0.0.1:%d/sensors/" % FWD_TCP[80])
        print("lidar in:  udp 127.0.0.1:%d" % FWD_UDP[7502])
        print("exit with ctrl-a x")
        os.execvp(binary, [binary, "-M", "malta", "-m", "256",
                           "-kernel", kernel, "-nographic", "-no-reboot",
                           "-netdev", f"user,id=n0,{fwd}",
                           "-device", "pcnet,netdev=n0",
                           "-append", "console=ttyS0"])

    bad = 0
    for i in range(args.loop):
        if args.loop > 1:
            print(f"\n===== run {i + 1}/{args.loop} =====")
        fails = run_once(kernel, endian, args.verbose)
        if fails:
            bad += 1
            print(f"\n  run failed: {', '.join(fails)}")

    print()
    if bad:
        print(f"FAILED: {bad} of {args.loop} runs")
        return 1
    print(f"all {args.loop} run(s) passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
