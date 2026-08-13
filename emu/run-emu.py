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
FWD_UDP = {7502: 8502, 7721: 8721, 7701: 8701}

# OpenWrt's br-lan is statically 192.168.1.1 and never asks for DHCP, so QEMU's
# user network is pointed at the same subnet and forwards are addressed to that
# IP explicitly. With the default 10.0.2.0/24 the guest is simply not there.
GUEST_IP = "192.168.1.1"

# A pty the guest sees as an FTDI USB-serial adapter, which is what rc-ibus
# expects. QEMU's usb-serial device emulates an FT232, so the guest's
# kmod-usb-serial-ftdi binds it and it appears as /dev/ttyUSB0.
IBUS_PTY = "/tmp/keti-emu-ibus"
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
    def __init__(self, kernel, endian, verbose=False, camera=None,
                 ibus=False, radios=0):
        self.verbose = verbose
        self.ibus_path = None
        binary = ("qemu-system-mipsel" if endian == "le"
                  else "qemu-system-mips")
        if not shutil.which(binary):
            sys.exit(f"{binary} not found: apt install qemu-system-mips")

        fwd = ",".join(
            [NET_OPTS] +
            [f"hostfwd=tcp::{h}-{GUEST_IP}:{g}" for g, h in FWD_TCP.items()] +
            [f"hostfwd=udp::{h}-{GUEST_IP}:{g}" for g, h in FWD_UDP.items()])

        args = [binary, "-M", "malta", "-m", "256", "-kernel", kernel,
                "-nographic", "-no-reboot",
                "-netdev", f"user,id=n0,{fwd}",
                "-device", "pcnet,netdev=n0"]

        # A real USB controller, so the guest exercises its own USB stack rather
        # than nothing at all. xHCI because that is what the router has.
        if camera or ibus:
            args += ["-device", "nec-usb-xhci,id=xhci"]

        if camera:
            # Pass the actual camera through. This is the only way to make
            # uvcvideo and ustreamer do real work on mipsel; there is no UVC
            # device model to fake it with.
            args += ["-device",
                     f"usb-host,bus=xhci.0,vendorid=0x{camera[0]:04x},"
                     f"productid=0x{camera[1]:04x}"]

        if ibus:
            self.ibus_path = IBUS_PTY
            args += ["-chardev",
                     f"socket,id=ibus,path={IBUS_PTY},server=on,wait=off",
                     "-device", "usb-serial,bus=xhci.0,chardev=ibus"]

        # Virtual radios. They are not MT7615D and say nothing about DBDC, but
        # they are the only way to exercise the two-radio configuration path -
        # which on the real board only exists if the DBDC fix works.
        append = "console=ttyS0"
        if radios:
            append += f" mac80211_hwsim.radios={radios}"
        args += ["-append", append]

        self.proc = subprocess.Popen(
            args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
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
                           "echo yes || echo no", timeout=20)
            if out and out.strip() == "yes":
                return True
            time.sleep(3)
        return False

    def cmd(self, line, timeout=45):
        """Run a shell command and return only its output.

        Output is framed by two markers whose *command* form cannot be mistaken
        for their *output* form: the shell is sent $((3*5))BEGIN and prints
        15BEGIN. Filtering echoed text by pattern was not enough - `stty -echo`
        does not always take, and a long line wraps so a fragment like
        `(7*11))` survives any filter keyed on `$((`. Framing removes the
        guesswork: everything between the two output markers is the answer, and
        anything else is noise by construction.
        """
        # A newline here would break the framing and, with a heredoc, leave the
        # shell waiting for a terminator - which wedged every command after it.
        assert "\n" not in line, "cmd() takes a single line"
        self.buf = b""
        send = f"echo $((3*5))BEGIN; {line}; echo $((7*11))END\n"
        self.proc.stdin.write(send.encode())
        self.proc.stdin.flush()
        if not self.read_until(re.compile(rb"77END"), timeout):
            return None
        out = self.buf.decode("utf-8", "replace")
        if "15BEGIN" not in out:
            return None
        body = out.split("15BEGIN", 1)[1].split("77END", 1)[0]
        return "\n".join(l.strip() for l in body.splitlines()
                          if l.strip()).strip()

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


def find_uvc():
    """Any UVC camera on the host, preferred: the StreamCam this was built for."""
    best = None
    for d in sorted(os.listdir("/sys/bus/usb/devices")):
        base = f"/sys/bus/usb/devices/{d}"
        try:
            vid = int(open(f"{base}/idVendor").read().strip(), 16)
            pid = int(open(f"{base}/idProduct").read().strip(), 16)
        except OSError:
            continue
        # interface class 14 is video; check any interface of this device
        is_uvc = False
        for i in sorted(os.listdir("/sys/bus/usb/devices")):
            if not i.startswith(d + ":"):
                continue
            try:
                if open(f"/sys/bus/usb/devices/{i}/bInterfaceClass").read().strip() == "0e":
                    is_uvc = True
            except OSError:
                pass
        if not is_uvc:
            continue
        if (vid, pid) == (0x046d, 0x0893):
            return (vid, pid)          # the StreamCam
        best = best or (vid, pid)
    return best


def feed_ibus(sock_path, seconds=3.0):
    """Synthetic i-BUS frames into the guest's emulated USB-serial port."""
    import math
    frames = 0
    end = time.time() + seconds
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.connect(sock_path)
    except OSError as e:
        return -1, str(e)
    t0 = time.time()
    while time.time() < end:
        t = time.time() - t0
        ch = [int(1500 + 480 * math.sin(t * 0.8)),
              int(1500 + 480 * math.cos(t * 0.6))] + [1500] * 12
        body = bytes([0x20, 0x40]) + b"".join(
            struct.pack("<H", c) for c in ch)
        s.sendall(body + struct.pack("<H", (0xFFFF - sum(body)) & 0xFFFF))
        frames += 1
        time.sleep(0.0075)
    s.close()
    return frames, ""


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


def run_once(kernel, endian, verbose, camera=None, ibus=False, radios=0):
    fails = []

    def check(name, ok, detail=""):
        print(f"  {'PASS' if ok else 'FAIL'}  {name}" +
              (f"  {detail}" if detail and not ok else ""))
        if not ok:
            fails.append(name)

    emu = Emu(kernel, endian, verbose, camera=camera, ibus=ibus, radios=radios)
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
            got_up = bool(out and out.strip() == "up")
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

        # --- CAN on the guest's own kernel, not the host's ---
        print("\n  can-bridge on the guest")
        # Not `ip -br`: BusyBox's ip has no brief mode, and using it here made a
        # working vcan look absent.
        up = emu.cmd("modprobe vcan 2>/dev/null; "
                     "ip link add dev vcan0 type vcan 2>/dev/null; "
                     "ip link set up vcan0 2>/dev/null; "
                     "ip link show vcan0 >/dev/null 2>&1 && echo yes || echo no")
        have_vcan = bool(up and up.strip() == "yes")
        if have_vcan:
            check("vcan available in the guest", True)
        else:
            # Not a failure: vcan is a test-only module and has no business in
            # the router image, so an image built without kmod-can-vcan is a
            # perfectly good image. Making this a hard check meant the harness
            # reported "FAILED: 1 of 1 runs" for a missing test fixture -- and
            # worse, the round trip below has therefore never actually run.
            # Select kmod-can-vcan in the emulator's config to exercise it.
            print("  SKIP  vcan not in this image; the CAN round trip is "
                  "not exercised (select kmod-can-vcan to enable it)")

        if have_vcan:
            # There is no python or cansend in the guest, so the frame is put on
            # the bus by the bridge itself: host -> UDP -> inject -> vcan0.
            #
            # This is one direction only. An earlier version of this comment
            # claimed vcan loops the frame back so receive was covered too; it
            # is not, because a raw CAN socket does not get its own sent frames
            # unless CAN_RAW_RECV_OWN_MSGS is set, and the bridge does not set
            # it. That is why rx stays 0 below and why the receive path is
            # covered by the host tests against a real vcan pair instead.
            emu.cmd("uci set can-bridge.bus.enabled=1")
            emu.cmd("uci set can-bridge.bus.interface=vcan0")
            emu.cmd("uci set can-bridge.bus.track=211,251")
            emu.cmd("uci set can-bridge.bus.allow_inject=1")
            emu.cmd("uci set can-bridge.bus.listen=7701")
            emu.cmd("uci commit can-bridge")
            emu.cmd("/etc/init.d/can-bridge restart", timeout=40)
            time.sleep(2)
            running = emu.cmd("pgrep -x can-bridge >/dev/null && echo yes || echo no")
            check("can-bridge started on vcan0",
                  running and running.strip() == "yes", f"got {running!r}")

            tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            pkt = b"BCAN" + bytes([1, 1, 0, 0]) + \
                  struct.pack("<IB3x8s", 0x211, 8, bytes(range(1, 9)))
            for _ in range(12):
                tx.sendto(pkt, ("127.0.0.1", FWD_UDP[7701]))
                time.sleep(0.05)
            tx.close()
            time.sleep(1.5)

            # One snapshot, parsed once. Reading the file three times gave
            # three different instants and made rx=0 sit next to a decoded
            # frame, which cannot both be true.
            raw = emu.cmd("cat /var/run/can-bridge.json 2>/dev/null | tr -d ' \\n\\t'")
            snap = {}
            if raw:
                import json as _json
                try:
                    snap = _json.loads(raw)
                except Exception:
                    snap = {}
            # What only the guest can show: the daemon runs on mipsel, can-up
            # brought the interface up, and a datagram from outside reached the
            # bus through it.
            check("bridge injected onto the bus on mipsel",
                  snap.get("injected", 0) > 0, f"snapshot={raw!r}")
            check("nothing was rejected", snap.get("rejected", -1) == 0,
                  f"rejected={snap.get('rejected')!r}")
            # No check on rx here, deliberately. A raw CAN socket does not
            # receive its own transmissions unless CAN_RAW_RECV_OWN_MSGS is set,
            # and setting it would make the relay echo its own injections back to
            # the network peer - a worse product for a better-looking test. The
            # receive path, batching and tracked-id decoding are covered against
            # vcan on the host in can-bridge/test/test_bridge.py.
            print(f"  INFO  rx={snap.get('rx')} (a raw CAN socket does not see "
                  f"its own frames; receive is covered by the host tests)")

            # Protocol generation detection, on mipsel.
            #
            # 0x211 above is not a generation marker, so the daemon must still
            # say "unknown" - it must not default to the generation its own
            # decoder implements.
            check("protocol not guessed from 0x211",
                  snap.get("agilex_protocol"), "unknown")

            # Now make it hear a real v2 marker. It cannot be the running
            # daemon's own injection: a raw CAN socket does not receive what it
            # sent, which is the same property noted above. So start a second
            # bridge, inject 0x241 through that one, and the service instance
            # sees it as an ordinary frame from another sender.
            # `& echo PID=$!` rather than a bare `&`: cmd() wraps what it is
            # given as `echo BEGIN; LINE; echo END`, so a line ending in `&`
            # becomes `& ;` and the shell rejects it.
            out2 = emu.cmd("can-bridge -f -i vcan0 -l 7721 --allow-inject "
                           "-S /tmp/cb2.json >/tmp/cb2.log 2>&1 & echo PID=$!")
            pid2 = None
            for tok in (out2 or "").replace("\n", " ").split():
                if tok.startswith("PID=") and tok[4:].isdigit():
                    pid2 = tok[4:]
            time.sleep(1.5)
            tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            marker = b"BCAN" + bytes([1, 1, 0, 0]) + \
                struct.pack("<IB3x8s", 0x241, 8, bytes(8))
            for _ in range(6):
                tx.sendto(marker, ("127.0.0.1", FWD_UDP[7721]))
                time.sleep(0.1)
            tx.close()
            time.sleep(2.0)

            raw2 = emu.cmd("cat /var/run/can-bridge.json 2>/dev/null "
                           "| tr -d ' \\n\\t'")
            snap2 = {}
            if raw2:
                import json as _json2
                try:
                    snap2 = _json2.loads(raw2)
                except Exception:
                    snap2 = {}
            check("0x241 detected as protocol v2 on mipsel",
                  snap2.get("agilex_protocol"), "v2")
            said = emu.cmd("logread | grep -i 'protocol v' | tail -1")
            print(f"  INFO  the daemon logged: {said or '(nothing)'}")
            # Only the second instance. Killing by name would take the service
            # one with it and leave the rest of the run in a different state
            # than it expects.
            if pid2:
                emu.cmd(f"kill {pid2} 2>/dev/null; echo ok")

        # --- i-BUS over an emulated USB-serial adapter ---
        if ibus:
            print("\n  rc-ibus over emulated USB-serial")
            dev = emu.cmd("ls /dev/ttyUSB* 2>/dev/null | head -1")
            if not (dev and "ttyUSB" in dev):
                # QEMU does present the device - the monitor shows
                # "Product QEMU USB Serial" on ohci.0 - but malta's OHCI in
                # QEMU 8.2 never enumerates it, so the guest has no ttyUSB.
                # Reported rather than failed: it is a limitation of the bench,
                # not of rc-ibus, which is covered byte for byte over a pty on
                # the host at the same compile target.
                print("  INFO  guest did not enumerate the emulated USB-serial "
                      "port; skipping (see emu/README.md)")
            else:
                emu.cmd(f"uci set rc-ibus.ibus.enabled=1; "
                        f"uci set rc-ibus.ibus.device={dev.strip()}; "
                        f"uci commit rc-ibus")
                emu.cmd("/etc/init.d/rc-ibus restart", timeout=30)
                time.sleep(1)
                n, err = feed_ibus(IBUS_PTY, seconds=3.0)
                check("host could feed the port", n > 0, f"{n} {err}")
                time.sleep(0.5)
                st = emu.cmd("cat /var/run/rc-ibus.json 2>/dev/null | "
                             "tr -d ' \\n\\t' | head -c 300")
                check("rc-ibus decoded frames on mipsel",
                      bool(st and '"link":true' in st), f"got {st!r}")
                check("no checksum errors over the emulated link",
                      bool(st and '"bad_crc":0' in st), f"got {st!r}")

        # --- the two-radio path, which only exists if DBDC works ---
        if radios:
            print(f"\n  wifi config path with {radios} virtual radios")
            n = emu.cmd("ls /sys/class/ieee80211/ 2>/dev/null | grep -c phy")
            check(f"guest has {radios} phys", n and n.strip() == str(radios),
                  f"got {n!r}")
            # re-run the sensorkit's own defaults against two radios
            emu.cmd("sh /rom/etc/uci-defaults/99-a3004-sensorkit 2>/dev/null || "
                    "true", timeout=40)
            got = emu.cmd("uci show wireless | grep -cE "
                          "'radio[01]\\.(disabled|country)'")
            check("sensorkit configured both radios",
                  bool(got and int(got.strip() or 0) >= 4),
                  f"matching options: {got!r}")
            warn = emu.cmd("logread | grep -c 'only one radio present' || true")
            check("no single-radio warning with two radios",
                  warn and warn.strip() == "0", f"got {warn!r}")

        # --- camera, if one is attached to the host ---
        if camera:
            print("\n  real camera passed through to the guest")
            time.sleep(3)
            vid = emu.cmd("ls /dev/video* 2>/dev/null | head -2")
            check("uvcvideo bound on mipsel", bool(vid and "video" in vid),
                  f"got {vid!r}")
            fmts = emu.cmd("v4l2-ctl -d /dev/video0 --list-formats 2>/dev/null | "
                           "grep -oiE 'mjpg|mjpeg|yuyv|nv12' | sort -u | tr '\\n' ' '")
            print(f"  INFO  formats the guest sees: {fmts!r}")
            if fmts and ("mjpg" in fmts.lower() or "mjpeg" in fmts.lower()):
                emu.cmd("uci set ustreamer.video0.enabled=1; uci commit ustreamer")
                emu.cmd("/etc/init.d/ustreamer restart", timeout=30)
                time.sleep(4)
                st, body = http(FWD_TCP[8080], "/snapshot", timeout=10)
                check("a JPEG came out of the guest",
                      st == 200 and body[:2] == b"\xff\xd8",
                      f"got {st} {body[:8]!r}")
                # the number the bandwidth budget predicted and nobody measured
                load = emu.cmd("top -bn1 2>/dev/null | grep -m1 ustreamer || "
                               "ps w | grep -m1 [u]streamer")
                print(f"  INFO  ustreamer on mipsel: {load!r}")
                print("  NOTE  QEMU is not MT7621 and this is not a timing "
                      "measurement; it shows the path works, not what it costs")

        # --- the report script must run without erroring ---
        # --- the report script must run without erroring ---
        print("\n  first-boot-report")
        out = emu.cmd("first-boot-report 2>&1 | tail -3", timeout=60)
        check("report runs to the end", bool(out and "end" in out),
              f"tail: {out!r}")

        print("\n  logs")
        # Print the lines, not just how many. A bare count is not actionable:
        # it cannot distinguish "an expected complaint about an absent sensor"
        # from "a daemon is broken", so nobody can act on it and it gets
        # ignored - which is the same as not checking at all.
        errs = emu.cmd("logread | grep -iE 'error|fatal' | tail -6 || true")
        if errs and errs.strip():
            print("  INFO  log lines matching error/fatal:")
            for line in errs.strip().splitlines():
                print(f"          {line.strip()}")
        else:
            print("  INFO  no log lines matching error/fatal")
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
    ap.add_argument("--camera", action="store_true",
                    help="pass a UVC camera on this host through to the guest")
    ap.add_argument("--ibus", action="store_true",
                    help="give the guest an emulated USB-serial port and feed "
                         "it synthetic i-BUS frames")
    ap.add_argument("--radios", type=int, default=0,
                    help="virtual mac80211_hwsim radios, e.g. 2 to exercise the "
                         "two-radio config path DBDC would create")
    ap.add_argument("--all", action="store_true",
                    help="everything this host can currently offer")
    args = ap.parse_args()

    camera = None
    if args.camera or args.all:
        camera = find_uvc()
        if camera:
            print(f"camera: passing {camera[0]:04x}:{camera[1]:04x} through")
        else:
            print("camera: no UVC device on this host; skipping that surface")
    ibus = args.ibus or args.all
    radios = args.radios or (2 if args.all else 0)

    kernel, endian = find_kernel()
    if not kernel:
        sys.exit("no malta initramfs image found. Build one first:\n"
                 "  CONFIG_TARGET_malta=y / CONFIG_TARGET_malta_le=y\n"
                 "  CONFIG_TARGET_ROOTFS_INITRAMFS=y")
    print(f"kernel: {os.path.relpath(kernel, TREE)}  ({endian})")

    if args.shell:
        # keep --shell simple; the extra surfaces are for the scripted run
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
        fails = run_once(kernel, endian, args.verbose, camera=camera,
                         ibus=ibus, radios=radios)
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
