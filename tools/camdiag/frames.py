#!/usr/bin/env python3
"""Where the camera's latency goes, measured from the stream alone.

The flashing-screen test in latency.py gives one number for the whole chain.
This splits the part that is on the wire out of it, and needs nothing but the
stream - no display, no aiming the camera.

Per frame it records when the first byte arrived, when the EOI arrived, and how
many bytes there were. Two numbers come out of that:

  on the wire   EOI minus first byte. A frame is not displayable until its last
                byte lands, so this is latency the client cannot avoid, and it
                is set by frame size against link rate.
  idle          the gap between one frame's EOI and the next frame's first byte.
                Time the link was not carrying video, which is what a higher
                frame rate would fill.

Anything in the glass-to-glass figure that is not on the wire is the camera, the
monitor, and the router's own handling - which is the half that resolution and
desired_fps move.

    frames.py [--url http://192.168.1.1:8080/stream] [--seconds 12]
"""
import argparse
import socket
import statistics
import sys
import time
import urllib.parse
import urllib.request


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://192.168.1.1:8080/stream")
    ap.add_argument("--seconds", type=float, default=12.0)
    # ustreamer stamps every part with X-Timestamp, the time it grabbed the
    # frame. Against a known clock offset that turns into the age of a frame
    # when it reaches the client, which separates the router's handling from the
    # camera's own exposure and encode - without needing a display.
    #
    # The offset is not assumed. Measure it with clock-offset.py, which brackets
    # the router's second boundary; on this bench the router was 1017 s behind
    # despite ntpd being enabled, so a guess of zero would have made every frame
    # look 17 minutes old.
    ap.add_argument("--clock-offset", type=float, default=None,
                    help="pc clock minus router clock, in seconds")
    a = ap.parse_args()

    u = urllib.parse.urlparse(a.url)
    port = u.port or 80
    # A raw socket rather than urllib: the timing of the first byte of a frame is
    # the measurement, and a buffered reader hands over whole blocks whenever it
    # feels like it, which would attribute the wait to the wrong frame.
    s = socket.create_connection((u.hostname, port), timeout=6)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.sendall(f"GET {u.path or '/'} HTTP/1.0\r\nHost: {u.hostname}\r\n"
              f"Connection: close\r\n\r\n".encode())

    t0 = time.time()
    frames = []          # (first_byte, eoi, size, grab_time_or_None)
    cur_start = None
    size = 0
    in_frame = False
    prev = -1
    stamps = []          # X-Timestamp values, in header order
    tail = b""           # carry, so a header split across reads is still seen
    s.settimeout(2.0)
    try:
        while time.time() - t0 < a.seconds:
            try:
                chunk = s.recv(65536)
            except socket.timeout:
                break
            if not chunk:
                break
            now = time.time()
            # Headers are ASCII in the gaps between JPEGs. Scanning the raw
            # bytes for them costs nothing and avoids a second parser.
            hay = tail + chunk
            i = 0
            while True:
                j = hay.find(b"X-Timestamp:", i)
                if j < 0:
                    break
                k = hay.find(b"\r\n", j)
                if k < 0:
                    break
                try:
                    stamps.append(float(hay[j + 12:k]))
                except ValueError:
                    pass
                i = k
            tail = hay[-24:]
            for b in chunk:
                if not in_frame:
                    if prev == 0xFF and b == 0xD8:
                        in_frame = True
                        # The SOI is in this chunk, so the frame started no
                        # earlier than the chunk did; within one chunk the
                        # resolution is the read, which is what we have.
                        cur_start = now
                        size = 2
                else:
                    size += 1
                    if prev == 0xFF and b == 0xD9:
                        in_frame = False
                        frames.append((cur_start, now, size,
                                       stamps[len(frames)]
                                       if len(frames) < len(stamps) else None))
                prev = b
    finally:
        s.close()

    if len(frames) < 5:
        sys.exit(f"only {len(frames)} frames - is the stream up?")

    wire = [(e - st) * 1000.0 for st, e, _, _ in frames]
    idle = [(frames[i + 1][0] - frames[i][1]) * 1000.0
            for i in range(len(frames) - 1)]
    sizes = [n for _, _, n, _ in frames]
    span = frames[-1][1] - frames[0][0]
    mbit = sum(sizes) * 8 / span / 1e6

    print(f"  {len(frames)} frames in {span:.1f}s   {len(frames)/span:.1f} fps"
          f"   {mbit:.1f} Mbit/s")
    print(f"  frame size   median {statistics.median(sizes)/1024:6.1f} kB"
          f"   max {max(sizes)/1024:.1f} kB")
    print(f"  on the wire  median {statistics.median(wire):6.1f} ms"
          f"   worst {max(wire):.1f} ms")
    print(f"  link idle    median {statistics.median(idle):6.1f} ms"
          f"   worst {max(idle):.1f} ms")

    if a.clock_offset is not None:
        aged = [(e - (g + a.clock_offset)) * 1000.0
                for _, e, _, g in frames if g is not None]
        held = [((st - (g + a.clock_offset)) * 1000.0)
                for st, _, _, g in frames if g is not None]
        if aged:
            print(f"  grab to arrival    median {statistics.median(aged):6.1f} ms"
                  f"   worst {max(aged):.1f} ms   ({len(aged)} stamped frames)")
            print(f"    of which, held in the router before the first byte left:"
                  f" {statistics.median(held):.1f} ms")
    print()
    print(f"  so of the glass-to-glass figure, {statistics.median(wire):.0f} ms is "
          f"bytes arriving and the rest is upstream of the client")
    return 0


if __name__ == "__main__":
    sys.exit(main())
