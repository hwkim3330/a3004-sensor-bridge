#!/usr/bin/env python3
"""Record the router's relayed lidar stream, and its metadata beside it.

The router relays the sensor's own UDP packets untouched, so what arrives here
is exactly what the sensor sent - which means the Ouster SDK can parse it with
no translation layer, and a recording made this way replays like a direct
connection.

Metadata is fetched from the sensor over HTTP rather than assumed. Without it a
packet stream is undecodable: the beam angles, the profile, the column count and
the range scale all live there, and they change with the sensor's mode.

    capture.py out.d [seconds] [--port 7502] [--sensor 192.168.1.50]

Writes out.d/metadata.json and out.d/lidar.pcap-like raw dump, plus a summary.
The dump is length-prefixed rather than a real pcap: this is the same bytes with
a four-byte length in front of each datagram, which is enough to replay and
avoids depending on a capture library to write it.
"""
import argparse
import json
import os
import socket
import struct
import sys
import time
import urllib.request


def fetch(host, path, timeout=6):
    with urllib.request.urlopen(f"http://{host}{path}", timeout=timeout) as r:
        return json.loads(r.read().decode())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("seconds", nargs="?", type=float, default=30.0)
    ap.add_argument("--port", type=int, default=7502)
    ap.add_argument("--imu-port", type=int, default=7503)
    ap.add_argument("--sensor", default="192.168.1.50")
    ap.add_argument("--bind", default="")
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)

    # Metadata first: if the sensor cannot be reached there is no point
    # recording packets that can never be interpreted.
    try:
        meta = fetch(a.sensor, "/api/v1/sensor/metadata")
    except Exception as e:
        sys.exit(f"cannot read sensor metadata from {a.sensor}: {e}")
    with open(os.path.join(a.out, "metadata.json"), "w") as f:
        json.dump(meta, f, indent=1)
    # The v2.4 firmware nests these. An earlier version of this line read
    # beam_altitude_angles from the top level and printed "0 beams" against a
    # sensor that was reporting 64 of them - the metadata was fine and the
    # reader was wrong, which is the sort of thing that gets blamed on the
    # sensor later.
    fmt = meta.get("lidar_data_format", {})
    beams = (meta.get("beam_intrinsics", {}).get("beam_altitude_angles")
             or meta.get("beam_altitude_angles") or [])
    print(f"  sensor {a.sensor}: {fmt.get('udp_profile_lidar')}, "
          f"{fmt.get('columns_per_frame')} cols/frame, {len(beams)} beams")

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # A big receive buffer, because the whole point is not to be the thing that
    # drops packets. 12544-byte datagrams at 640/s fill the default in
    # milliseconds if anything hiccups.
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
    s.bind((a.bind, a.port))
    s.settimeout(1.0)

    path = os.path.join(a.out, "lidar.raw")
    n = 0
    nbytes = 0
    t0 = time.time()
    last = t0
    with open(path, "wb") as f:
        while time.time() - t0 < a.seconds:
            try:
                d = s.recv(65535)
            except socket.timeout:
                continue
            f.write(struct.pack("<I", len(d)))
            f.write(d)
            n += 1
            nbytes += len(d)
            now = time.time()
            if now - last >= 2:
                last = now
                el = now - t0
                print(f"  {el:5.0f}s  {n:7d} packets  "
                      f"{nbytes/el/1024/1024:5.2f} MB/s", flush=True)
    s.close()

    el = time.time() - t0
    summary = {
        "packets": n, "bytes": nbytes, "seconds": el,
        "rate_pkt_s": n / el if el else 0,
        "profile": fmt.get("udp_profile_lidar"),
        "lidar_mode": meta.get("config_params", {}).get("lidar_mode"),
        "beams": len(beams),
    }
    with open(os.path.join(a.out, "capture.json"), "w") as f:
        json.dump(summary, f, indent=1)
    print(f"  wrote {path}: {n} packets, {nbytes/1024/1024:.1f} MB in {el:.0f}s "
          f"({n/el:.0f}/s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
