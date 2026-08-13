#!/usr/bin/env python3
"""Look at an Ouster directly from a PC, without the router in the way.

The router's job is a cheap reflex ring, not a point cloud - see
doc/ARCHITECTURE.md for why. When you want the sensor's actual output, take it
straight off the wire here, where there is floating point hardware and memory.

    lidar_view.py 192.168.1.50 --seconds 3 --out /tmp/scan.png

It reads the UDP stream the sensor is already sending and projects it with the
sensor's own beam intrinsics. Deliberately not through ouster.sdk's
SensorScanSource: that reconfigures udp_dest to whichever local address it likes
the look of, and on this machine it aborts in a destructor on close. Reading
datagrams and doing the trigonometry is fifty lines and does neither.

Three things it prints rather than assumes, because each one cost time on the
bench:

  - The azimuth window. A sensor restricted to an arc emits nothing outside it,
    and the absent measurement ids are not packet loss. A wedge instead of a
    circle is this, not a fault.
  - The range unit. RNG15_RFL8_NIR8 stores range in 8 mm steps, not
    millimetres. Read as mm it gives distances eight times too small, which look
    plausible enough to believe.
  - How many beams return anything. With the sensor on a desk it was 0.7 of 64
    per column, which is why a plot can look sparse while nothing is wrong.

It does not write to the sensor.
"""
import argparse
import json
import socket
import struct
import sys
import time
import urllib.request

import numpy as np

# Column header: 8 byte timestamp, 2 byte measurement id, 2 byte status.
COL_HDR = 12
PKT_HDR = 32

# Pixel size and range decode per profile. The scale is the part worth stating.
PROFILES = {
    # name:                (bytes per pixel, mask, scale to mm)
    "RNG19_RFL8_SIG16_NIR16": (12, 0x0007FFFF, 1),
    "RNG15_RFL8_NIR8": (4, 0x00007FFF, 8),
    "LEGACY": (12, 0x000FFFFF, 1),
}


def api(host: str, path: str):
    with urllib.request.urlopen(f"http://{host}{path}", timeout=8) as r:
        return json.load(r)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("host")
    ap.add_argument("--port", type=int, default=7502)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--out", default="/tmp/lidar_view.png")
    ap.add_argument("--max-range", type=float, default=30.0)
    args = ap.parse_args()

    cfg = api(args.host, "/api/v1/sensor/config")
    info = api(args.host, "/api/v1/sensor/metadata/sensor_info")
    beams = api(args.host, "/api/v1/sensor/metadata/beam_intrinsics")
    fmt = api(args.host, "/api/v1/sensor/metadata/lidar_data_format")

    prof = cfg["udp_profile_lidar"]
    if prof not in PROFILES:
        print(f"  unhandled profile {prof}", file=sys.stderr)
        return 1
    px, mask, scale = PROFILES[prof]
    nch = fmt["pixels_per_column"]
    ncol = fmt["columns_per_packet"]
    width = fmt["columns_per_frame"]
    col_stride = COL_HDR + nch * px
    expect = PKT_HDR + ncol * col_stride + 32     # 32 byte footer

    print(f"  {info['prod_line']}  sn {info['prod_sn']}  fw {info['build_rev']}")
    print(f"  {cfg['lidar_mode']}  {prof}  {nch}ch x {ncol}col, frame {width}")
    print(f"  range unit {scale} mm, {px} bytes per pixel, {expect} B datagram")
    win = cfg.get("azimuth_window")
    if win and tuple(win) != (0, 360000):
        span = ((win[1] - win[0]) % 360000) / 1000.0
        print(f"  azimuth_window {win}: a {span:.0f} degree arc, so the plot is a "
              f"wedge and the ids outside it are absent by design")
    print(f"  udp_dest {cfg['udp_dest']}:{cfg['udp_port_lidar']} (not modified)")

    alt = np.radians(np.array(beams["beam_altitude_angles"], dtype=np.float64))
    azoff = np.radians(np.array(beams["beam_azimuth_angles"], dtype=np.float64))
    n0 = beams["lidar_origin_to_beam_origin_mm"] / 1000.0

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    s.bind(("0.0.0.0", args.port))
    s.settimeout(2.0)

    xs, ys, zs = [], [], []
    # One column per measurement id, not per packet slot. Sizing this by
    # columns_per_packet and folding mid into it aliased four azimuths onto each
    # column, which read as gaps in the image.
    last_img = np.full((nch, width), np.nan)
    got = bad = ncols = nret = 0
    end = time.time() + args.seconds
    while time.time() < end:
        try:
            p, _ = s.recvfrom(65535)
        except socket.timeout:
            break
        if len(p) != expect:
            bad += 1
            continue
        got += 1
        for c in range(ncol):
            b = PKT_HDR + c * col_stride
            mid, st = struct.unpack_from("<HH", p, b + 8)
            if not (st & 1):
                continue
            ncols += 1
            raw = np.frombuffer(p, dtype="<u4" if px >= 4 else "<u2",
                                count=nch, offset=b + COL_HDR
                                ) if px == 4 else np.array(
                [struct.unpack_from("<I", p, b + COL_HDR + i * px)[0]
                 for i in range(nch)], dtype=np.uint32)
            rng_m = (raw & mask).astype(np.float64) * scale / 1000.0
            hit = rng_m > 0
            if not hit.any():
                continue
            nret += int(hit.sum())
            # theta measured from the frame start, plus the per-beam offset
            theta = 2 * np.pi * (mid / width) + azoff
            r = rng_m - n0
            xs.append(r[hit] * np.cos(theta[hit]) * np.cos(alt[hit]))
            ys.append(-r[hit] * np.sin(theta[hit]) * np.cos(alt[hit]))
            zs.append(r[hit] * np.sin(alt[hit]))
            if 0 <= mid < width:
                last_img[:, mid] = np.where(hit, rng_m, np.nan)
    s.close()

    if not xs:
        print("  no datagrams arrived. Is udp_dest pointing at this machine, and "
              "is this the only address it has in that subnet?", file=sys.stderr)
        return 1

    x = np.concatenate(xs); y = np.concatenate(ys); z = np.concatenate(zs)
    d = np.sqrt(x * x + y * y + z * z)
    print(f"  {got} datagrams, {bad} of the wrong size, {ncols} valid columns")
    print(f"  {nret} returns, {nret / max(ncols, 1):.1f} of {nch} beams per column")
    print(f"  nearest {d.min():.2f} m, median {np.median(d):.2f} m, "
          f"farthest {d.max():.2f} m")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (a, b) = plt.subplots(2, 1, figsize=(9, 11),
                               gridspec_kw={"height_ratios": [3, 1]})
    fig.patch.set_facecolor("#0d0f13")

    a.set_facecolor("#0d0f13")
    for ring in range(5, int(args.max_range) + 1, 5):
        a.add_artist(plt.Circle((0, 0), ring, fill=False,
                                color="#ffffff1c", lw=0.8))
        a.text(ring * 0.71, ring * 0.71, f"{ring} m", color="#ffffff44",
               fontsize=7)
    a.scatter(x, y, s=1.2, c=z, cmap="viridis", vmin=-2, vmax=3, linewidths=0)
    a.plot(0, 0, marker="o", color="#4c9aff", ms=6)
    a.set_xlim(-args.max_range, args.max_range)
    a.set_ylim(-args.max_range, args.max_range)
    a.set_aspect("equal")
    a.set_title(f"{info['prod_line']}  top-down, {got} datagrams, "
                f"colour = height", color="#e6ecf5", fontsize=11)
    a.tick_params(colors="#8695ab", labelsize=8)
    for sp in a.spines.values():
        sp.set_color("#ffffff20")

    b.set_facecolor("#0d0f13")
    b.imshow(last_img, aspect="auto", cmap="magma", interpolation="nearest")
    b.set_title(f"range image, {width} azimuths x {nch} channels",
                color="#e6ecf5", fontsize=10)
    b.tick_params(colors="#8695ab", labelsize=8)
    for sp in b.spines.values():
        sp.set_color("#ffffff20")

    fig.tight_layout()
    fig.savefig(args.out, dpi=110, facecolor=fig.get_facecolor())
    print(f"  wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
