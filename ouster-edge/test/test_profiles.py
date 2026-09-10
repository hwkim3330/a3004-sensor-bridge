#!/usr/bin/env python3
"""End-to-end check of ouster-edge's packet parser against synthesised packets.

Builds byte-exact Ouster lidar packets for each documented profile, feeds them
to the daemon over a real UDP socket, and asserts the ring it publishes matches
what was encoded. This is the only way to verify the parser without a sensor.
"""
import json, os, socket, struct, subprocess, sys, tempfile, time

BIN = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("OUSTER_EDGE_BIN", "./ouster-edge")
PORT = 26502
RING_PORT = 26602
STATUS = tempfile.mkstemp(suffix=".json")[1]

CH, COLS, WIDTH = 64, 16, 1024
PROBE_CH = 32          # the one channel we encode a known range into
SECTORS = 1024         # 1:1 with columns, so no sector aliasing in the check

fails = []


def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: got {got!r} want {want!r}")
    if not ok:
        fails.append(name)


def px_single(range_mm, refl):
    """RNG19_RFL8_SIG16_NIR16: 19-bit range, 13 reserved, refl, 8 res, sig, nir, res"""
    assert range_mm < (1 << 19)
    return struct.pack("<IBBHHH", range_mm, refl, 0, 1234, 5678, 0)


def px_dual(range_mm, refl):
    """16 bytes: two returns; only the first is read"""
    return px_single(range_mm, refl) + struct.pack("<I", 0)


def px_lowrate(range_mm, refl):
    """RNG15_RFL8_NIR8: 15-bit range in 8 mm units, refl, nir"""
    return struct.pack("<HBB", (range_mm // 8) & 0x7FFF, refl, 0)


def px_legacy(range_mm, refl):
    """LEGACY: 20-bit range + 12 res, then 16-bit refl, signal, nir, res"""
    assert range_mm < (1 << 20)
    return struct.pack("<IHHHH", range_mm, refl, 1234, 5678, 0)


def build(profile, frame_id, mids, range_for):
    """One packet: COLS columns, each with a known range at PROBE_CH."""
    pxf, pxlen = {
        "single": (px_single, 12),
        "dual": (px_dual, 16),
        "lowrate": (px_lowrate, 4),
        "legacy": (px_legacy, 12),
    }[profile]

    body = b""
    for mid in mids:
        if profile == "legacy":
            col = struct.pack("<QHHI", mid * 1000, mid, frame_id, mid * 64)
        else:
            col = struct.pack("<QHH", mid * 1000, mid, 1)  # status bit0 = valid

        for ch in range(CH):
            if ch == PROBE_CH:
                col += pxf(range_for(mid), 200)
            else:
                col += pxf(400000 if profile != "lowrate" else 200000, 7)

        if profile == "legacy":
            col += struct.pack("<I", 0xFFFFFFFF)  # column valid
        body += col

    if profile == "legacy":
        return body
    return struct.pack("<HHI", 0x1, frame_id, 0) + b"\0" * 24 + body + b"\0" * 32


def run_profile(profile, expect_size):
    print(f"\n--- {profile} ---")
    for f in (STATUS,):
        if os.path.exists(f):
            os.remove(f)

    ring_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ring_sock.bind(("127.0.0.1", RING_PORT))
    ring_sock.settimeout(4)

    proc = subprocess.Popen(
        [BIN, "-f", "-p", str(PORT), "-c", str(CH), "-C", str(COLS),
         "-w", str(WIDTH), "-s", str(SECTORS),
         "-b", f"{PROBE_CH}:{PROBE_CH}", "-m", "0.1", "-M", "300",
         "-o", f"127.0.0.1:{RING_PORT}", "-S", STATUS, "-I", "50",
         "-z", "0:10:5.0"],
        stderr=subprocess.PIPE, text=True)
    time.sleep(0.6)

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # range in mm = 1000 + mid, so every sector gets a distinct known value
    def rng(mid):
        return 1000 + mid * 8 if profile == "lowrate" else 1000 + mid

    size_seen = None
    # two full revolutions, so at least one completes and gets published
    for frame in (7, 8):
        for start in range(0, WIDTH, COLS):
            pkt = build(profile, frame, range(start, start + COLS), rng)
            size_seen = len(pkt)
            tx.sendto(pkt, ("127.0.0.1", PORT))
            time.sleep(0.0004)

    # Wait for the daemon to have accounted for every packet, rather than
    # sleeping a fixed amount and hoping. A flat sleep(0.8) here failed roughly
    # one CI run in two with "json packets: got 127 want 128" - one short, with
    # missed_columns still 0 - because on a loaded runner the last packet had not
    # been folded into the status file by the time it was read. Polling removes
    # the timing dependence without weakening the assertion: if the count really
    # never arrives, this still ends up short and the check below still fails.
    expect_packets = 2 * WIDTH // COLS
    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline:
        try:
            with open(STATUS) as fh:
                if json.load(fh).get("packets") >= expect_packets:
                    break
        except (OSError, ValueError, TypeError):
            pass          # mid-write, or not written yet
        time.sleep(0.02)

    check("packet size", size_seen, expect_size)

    # --- the binary ring datagram ---
    try:
        data, _ = ring_sock.recvfrom(65535)
    except socket.timeout:
        check("ring datagram received", False, True)
        data = None

    if data:
        check("ring magic", data[:4], b"OSED")
        ver, prof_id = data[4], data[5]
        sect, fid = struct.unpack_from("<HH", data, 6)
        check("ring version", ver, 1)
        check("ring sectors", sect, SECTORS)
        check("ring length", len(data), 20 + 3 * SECTORS)
        check("profile id", prof_id,
              {"legacy": 1, "single": 2, "lowrate": 3, "dual": 4}[profile])
        rngs = struct.unpack_from(f"<{sect}H", data, 20)
        refl = struct.unpack_from(f"<{sect}B", data, 20 + 2 * sect)
        # mid N -> sector N (SECTORS == WIDTH), range mm -> cm
        for mid in (0, 1, 511, 1023):
            check(f"sector {mid} range cm", rngs[mid], rng(mid) // 10)
        check("reflectivity", refl[100], 200 if profile != "legacy" else 200)

    # --- the JSON status file ---
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
    ring_sock.close()

    with open(STATUS) as fh:
        st = json.load(fh)
    check("json packets", st["packets"], expect_packets)
    check("json bad_size", st["bad_size"], 0)
    check("json invalid_columns", st["invalid_columns"], 0)
    check("json missed_columns", st["missed_columns"], 0)
    check("json channels", st["channels"], CH)
    check("json packet_size", st["packet_size"], expect_size)
    check("json ring length", len(st["ring_cm"]), SECTORS)
    check("json ring[0]", st["ring_cm"][0], rng(0) // 10)
    # zone 0-10 deg at 5 m: nearest there is ~1.0 m, so it must fire
    check("zone_alarm", st["zone_alarm"], True)


print("ouster-edge parser verification")
run_profile("single", 12544)
run_profile("legacy", 12608)
run_profile("lowrate", 4352)
run_profile("dual", 16640)

print()
if fails:
    print(f"FAILED ({len(fails)}): {', '.join(fails)}")
    sys.exit(1)
print("all checks passed")
