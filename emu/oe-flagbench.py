#!/usr/bin/env python3
"""Does -O2 actually beat -Os for ouster-edge on mipsel? Measure it on mipsel.

The -O2 change in the package Makefile rests on a measurement taken on this
desktop's x86 core, and x86 says very little about what a MIPS32r2 compiler
does. Two things in particular did not transfer:

  - on x86 the two builds were the same size to within 24 bytes, which makes a
    23% difference in cost surprising enough to need checking
  - on mipsel -O2 is 988 bytes *larger*, which is a different codegen decision
    entirely

So run both builds under the emulator, on the same architecture as the router,
and compare.

    python3 oe-flagbench.py                # both builds, both packet counts
    python3 oe-flagbench.py -v             # stream the guest console

Method, and why it is shaped this way:

  - The injector runs *inside* the guest. Feeding 12.5 kB datagrams through
    QEMU's user-mode NAT makes the host the bottleneck and measures slirp.
  - Cost is taken by subtraction between two packet counts. A single run's CPU
    time includes process startup, uci parsing and status writes; the difference
    between 4000 and 12000 packets does not.
  - Packets are counted by the daemon's own status file, not by the injector.
    At full tilt the socket buffer drops some, and what matters is cost per
    packet actually processed.

What this cannot tell you: wall-clock speed. QEMU's TCG translates rather than
executes, so a second in the guest is not a second on an MT7621. What it does
tell you is whether the same instruction stream a 1004Kc would run does less
work at -O2, which is the question the flag turns on.
"""
import argparse
import functools
import http.server
import importlib.util
import json
import os
import socketserver
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HOST_IP = "192.168.1.2"          # what QEMU's user-mode net calls this host
HTTP_PORT = 8899
COUNTS = (4000, 40000)
BUILDS = ("Os", "O2")


def load_emu():
    """run-emu.py has a dash in its name, so it needs loading by path."""
    spec = importlib.util.spec_from_file_location(
        "run_emu", os.path.join(HERE, "run-emu.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def serve(directory):
    handler = functools.partial(http.server.SimpleHTTPRequestHandler,
                                directory=directory)
    handler.log_message = lambda *a, **k: None
    srv = socketserver.TCPServer(("0.0.0.0", HTTP_PORT), handler)
    srv.allow_reuse_address = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def fetch(emu, name, dest):
    """Pull a file into the guest, whichever downloader the image has."""
    url = f"http://{HOST_IP}:{HTTP_PORT}/{name}"
    for tool in ("wget -q -O", "uclient-fetch -q -O"):
        emu.cmd(f"{tool} {dest} {url} 2>/dev/null; echo done", timeout=90)
        size = emu.cmd(f"[ -s {dest} ] && wc -c < {dest} || echo 0")
        if size and size.strip().isdigit() and int(size.strip()) > 0:
            emu.cmd(f"chmod +x {dest}")
            return int(size.strip())
    return 0


def one_run(emu, build, count):
    """Start a build, push `count` datagrams at it, return (packets, jiffies)."""
    st = "/tmp/oebench.json"
    log = "/tmp/oebench.err"
    emu.cmd(f"rm -f {st} {log}")
    #
    # Two things here were learned the hard way.
    #
    # The pid comes from $! rather than from pgrep. pgrep -f would also match
    # the shell that is asking, and matching a process by the text of its
    # command line is how a harness ends up killing itself - which happened
    # twice on the host side of this same session.
    #
    # The line ends in `& echo` rather than `&`, because cmd() wraps what it is
    # given as `echo BEGIN; LINE; echo END`, and a LINE ending in `&` produces
    # `& ;`, which is a syntax error. The first version of this failed every
    # measurement for exactly that reason and said only "FAILED to measure".
    #
    # stderr goes to a file rather than /dev/null so a daemon that refuses to
    # start can say why.
    #
    out = emu.cmd(f"/tmp/oe-{build} -p 7502 -w 1024 -c 64 -C 16 -S {st} "
                  f"-I 60000 -f >{log} 2>&1 & echo PID=$!")
    pid = None
    if out:
        for tok in out.replace("\n", " ").split():
            if tok.startswith("PID=") and tok[4:].isdigit():
                pid = tok[4:]
    if not pid:
        print(f"    -{build}: no pid; console said {out!r}")
        return None
    time.sleep(2)
    alive = emu.cmd(f"[ -d /proc/{pid} ] && echo yes || echo no")
    if not alive or alive.strip() != "yes":
        why = emu.cmd(f"cat {log} 2>/dev/null | head -3")
        print(f"    -{build}: died at once; it said {why!r}")
        return None

    # utime and stime kept apart, not summed.
    #
    # Summing them hid the answer the first time. A datagram here is 12.5 kB,
    # and the kernel copies every byte of it on the way through the socket -
    # work that is identical whatever flags the daemon was built with. That
    # lands in stime and dilutes any userspace difference towards nothing. The
    # flag can only change utime, so utime is what has to be compared.
    before = emu.cmd(f"awk '{{print $14, $15}}' /proc/{pid}/stat")
    emu.cmd(f"/tmp/oe-inject 7502 {count} 0", timeout=900)
    time.sleep(2)
    after = emu.cmd(f"awk '{{print $14, $15}}' /proc/{pid}/stat")
    # SIGUSR1 is not wired up, so ask for a status write by letting the interval
    # lapse would take a minute. Read the counter the daemon keeps instead by
    # stopping it: it writes a final status on the way out.
    emu.cmd(f"kill -TERM {pid}")
    time.sleep(2)
    raw = emu.cmd(f"cat {st} 2>/dev/null | tr -d '\\n'")
    emu.cmd(f"kill -KILL {pid} 2>/dev/null; echo ok")

    pkts = None
    if raw:
        try:
            pkts = json.loads(raw).get("packets")
        except Exception:
            pkts = None
    if before is None or after is None:
        return None
    try:
        u0, s0 = (int(x) for x in before.split()[:2])
        u1, s1 = (int(x) for x in after.split()[:2])
    except Exception:
        return None
    return pkts, u1 - u0, s1 - s0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--bindir", default=os.environ.get("OE_BENCH_BINDIR", ""))
    args = ap.parse_args()

    if not args.bindir or not os.path.isdir(args.bindir):
        sys.exit("pass --bindir DIR holding mips-Os, mips-O2 and oe-inject")
    for f in ("mips-Os", "mips-O2", "oe-inject"):
        if not os.path.exists(os.path.join(args.bindir, f)):
            sys.exit(f"missing {f} in {args.bindir}")

    run_emu = load_emu()
    kernel, endian = run_emu.find_kernel()
    if not kernel:
        sys.exit("no malta initramfs image found; see README.md")
    print(f"  kernel: {os.path.basename(kernel)} ({endian})")

    srv = serve(args.bindir)
    emu = run_emu.Emu(kernel, endian, verbose=args.verbose)
    try:
        if not emu.read_until(run_emu.PROMPT, 180) and not emu.wake():
            sys.exit("  FAIL  never reached a prompt")
        emu.proc.stdin.write(b"stty -echo 2>/dev/null; "
                             b"stty columns 400 2>/dev/null\n")
        emu.proc.stdin.flush()
        if not emu.wait_ready(180):
            print("  note: procd never settled; carrying on anyway")

        # The packaged daemon owns port 7502. Stop it so the binaries under test
        # can bind, and so its work is not mixed into the measurement.
        emu.cmd("/etc/init.d/ouster-edge stop 2>/dev/null; sleep 1; echo ok")

        for name, dest in (("mips-Os", "/tmp/oe-Os"),
                           ("mips-O2", "/tmp/oe-O2"),
                           ("oe-inject", "/tmp/oe-inject")):
            n = fetch(emu, name, dest)
            print(f"  fetched {name}: {n} bytes")
            if not n:
                sys.exit("  FAIL  could not get the binaries into the guest")

        results = {}
        for build in BUILDS:
            for count in COUNTS:
                r = one_run(emu, build, count)
                if r is None:
                    print(f"  -{build} n={count}: FAILED to measure")
                    continue
                pkts, ut, st_ = r
                results[(build, count)] = (pkts, ut, st_)
                print(f"  -{build} n={count}: {pkts} packets, "
                      f"utime {ut} + stime {st_} jiffies")

        print()
        print("  per-packet cost by subtraction (a jiffy is 10 ms):")
        summary = {}
        for build in BUILDS:
            lo = results.get((build, COUNTS[0]))
            hi = results.get((build, COUNTS[1]))
            if not lo or not hi or not lo[0] or not hi[0]:
                print(f"    -{build}: not enough data")
                continue
            dn = hi[0] - lo[0]
            if dn <= 0:
                print(f"    -{build}: packet counts did not increase")
                continue
            du, ds = hi[1] - lo[1], hi[2] - lo[2]
            summary[build] = (du, ds, dn)
            print(f"    -{build}: over {dn} packets, "
                  f"user {du} jiffies ({du * 10000.0 / dn:.1f} us/pkt), "
                  f"sys {ds} jiffies ({ds * 10000.0 / dn:.1f} us/pkt)")

        if len(summary) == 2:
            uo, ut2 = summary["Os"][0], summary["O2"][0]
            print()
            if uo <= 0:
                print("    userspace time too small to compare; raise COUNTS")
            else:
                d = (uo - ut2) * 100.0 / uo
                print(f"    -O2 changes userspace time by {d:+.0f}% "
                      f"({uo} -> {ut2} jiffies)")
                print("    Resolution is one jiffy, so treat anything under "
                      f"{100.0 / max(uo, 1):.0f}% as no difference.")
    finally:
        emu.stop()
        srv.shutdown()


if __name__ == "__main__":
    sys.exit(main())
