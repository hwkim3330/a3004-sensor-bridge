#!/usr/bin/env python3
"""Measure the camera's glass-to-glass latency, rather than reasoning about it.

The method is a flashing screen. A fullscreen window flips between black and
white at known times; the MJPEG stream is read at the same moment, and each
frame is reduced to the mean brightness of the region that actually changes.
The latency of one flip is the arrival time of the first frame that has moved
halfway to the new level, minus the time the flip was drawn.

What this includes: the monitor's own response, the camera's exposure and
encode, ustreamer, and the network. What it excludes: the tablet's decode and
draw, which is measured separately in the app's own log. That split is the
point - it says which half to work on.

Two things bound the resolution. The stream runs at 20 fps, so an arrival time
is quantised to 50 ms, and the reported figure is a median over many flips
rather than one reading. And a camera with auto-exposure will keep moving after
the step, which is why the threshold is a half-step and not the settled level.

    latency.py [--url http://192.168.1.1:8080/stream] [--flips 16]

Point the camera at the monitor before running. The script says whether it
could see the screen at all instead of reporting a number from noise.
"""
import argparse
import io
import statistics
import struct
import sys
import threading
import time
import urllib.request

import numpy as np

# Shared between the reader thread and the flip loop. Only ever appended to.
frames = []          # (arrival_time, HxW uint8 thumbnail)
flips = []           # (draw_time, level) level 1 = white, 0 = black
stop = threading.Event()


def reader(url, thumb=48):
    """Pull the multipart stream, keeping a thumbnail and arrival time per frame.

    Frames are delimited by their own SOI/EOI markers rather than by the
    boundary string, the same way the app does it, so this measures the same
    stream the app sees.
    """
    from PIL import Image
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req, timeout=6) as r:
        acc = bytearray()
        in_frame = False
        prev = -1
        while not stop.is_set():
            chunk = r.read(32768)
            if not chunk:
                return
            for b in chunk:
                if not in_frame:
                    if prev == 0xFF and b == 0xD8:
                        acc = bytearray(b"\xff\xd8")
                        in_frame = True
                else:
                    acc.append(b)
                    if prev == 0xFF and b == 0xD9:
                        in_frame = False
                        t = time.time()
                        try:
                            im = Image.open(io.BytesIO(bytes(acc)))
                            im.draft("L", (thumb, thumb))
                            g = np.asarray(im.convert("L").resize(
                                (thumb, thumb)), dtype=np.uint8)
                            frames.append((t, g))
                        except Exception:
                            pass
                prev = b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://192.168.1.1:8080/stream")
    ap.add_argument("--flips", type=int, default=16)
    ap.add_argument("--hold", type=float, default=1.1,
                    help="seconds to hold each level; must exceed the latency")
    # Windowed by default, and deliberately so: fullscreen takes over whatever
    # the person at the machine is doing, with no warning, for as long as the run
    # lasts. A large window in a corner is enough for the camera to see, and asks
    # for the screen instead of seizing it.
    ap.add_argument("--size", default="1400x1000",
                    help="flashing window size")
    ap.add_argument("--fullscreen", action="store_true",
                    help="take the whole screen; louder, and only needed if the "
                         "camera is too far away for a window to register")
    a = ap.parse_args()

    import tkinter as tk

    th = threading.Thread(target=reader, args=(a.url,), daemon=True)
    th.start()
    # Let the stream establish before the first flip, so the first measurement
    # is not the connection setup.
    time.sleep(2.0)
    if not frames:
        sys.exit(f"no frames from {a.url} - is the stream up?")
    print(f"  stream up: {len(frames)} frames in 2 s")

    root = tk.Tk()
    root.title("camera latency - flashing")
    if a.fullscreen:
        root.attributes("-fullscreen", True)
    else:
        root.geometry(f"{a.size}+40+40")
    cv = tk.Canvas(root, highlightthickness=0, bd=0)
    cv.pack(fill="both", expand=True)
    level = [0]

    def flip():
        level[0] ^= 1
        cv.configure(bg="white" if level[0] else "black")
        root.update_idletasks()
        root.update()
        # After update, not before: the timestamp has to be when the pixels were
        # handed to the compositor, or the measurement starts early and every
        # latency comes out too large.
        flips.append((time.time(), level[0]))

    for i in range(a.flips):
        flip()
        time.sleep(a.hold)
    root.destroy()
    stop.set()
    time.sleep(0.3)

    print(f"  {len(frames)} frames, {len(flips)} flips")
    if len(frames) < 10:
        sys.exit("too few frames to measure")

    ts = np.array([f[0] for f in frames])
    stack = np.stack([f[1] for f in frames]).astype(np.float32)

    # Which pixels are the screen? The ones whose brightness moves with the
    # flips. Taking the whole image would bury a monitor that fills a fifth of
    # the frame under a room that does not change at all.
    var = stack.var(axis=0)
    keep = var > np.quantile(var, 0.90)
    if keep.sum() < 8:
        sys.exit("nothing in the picture changed - the camera cannot see the screen")
    sig = stack[:, keep].mean(axis=1)
    lo, hi = np.quantile(sig, 0.1), np.quantile(sig, 0.9)
    print(f"  responding pixels: {int(keep.sum())} of {keep.size}, "
          f"level {lo:.0f} to {hi:.0f} of 255")
    if hi - lo < 12:
        sys.exit("the picture barely moved - aim the camera at the monitor")

    def measure(frac):
        """Latencies at a crossing threshold `frac` of the way to the new level.

        Levels are taken per flip, not once for the whole run. A camera with
        auto-exposure stops down after the screen goes white, so the settled
        level drifts between flips, and a global threshold makes some flips
        cross before the light even reached the sensor. The first version of
        this used global quantiles and reported a best case of 24 ms, which is
        less than a monitor takes to change - that number was the detector, not
        the camera.
        """
        out = []
        for i, (t, lv) in enumerate(flips):
            end = flips[i + 1][0] if i + 1 < len(flips) else ts[-1] + 1
            before = np.flatnonzero(ts < t)
            m = (ts > t) & (ts < end)
            if not before.size or not m.any():
                continue
            idx = np.flatnonzero(m)
            start = sig[before[-1]]
            # The settled level: the tail of this hold, once the camera has
            # stopped moving.
            tail = idx[int(len(idx) * 0.7):]
            if not len(tail):
                continue
            target = float(np.median(sig[tail]))
            if abs(target - start) < 10:
                continue                      # this flip did not register
            th = start + (target - start) * frac
            up = target > start
            crossed = [j for j in idx if (sig[j] >= th if up else sig[j] <= th)]
            if not crossed:
                continue
            out.append((ts[crossed[0]] - t) * 1000.0)
        return sorted(out)

    lat = measure(0.5)
    if len(lat) < 4:
        sys.exit(f"only {len(lat)} usable flips - increase --hold")
    print()
    print(f"  glass-to-glass, {len(lat)} flips (monitor + camera + router + net):")
    print(f"    median {statistics.median(lat):6.0f} ms")
    print(f"    best   {lat[0]:6.0f} ms     worst {lat[-1]:6.0f} ms")
    # Whether the answer is the camera or the threshold. If these disagree by
    # much more than the 50 ms frame interval, the signal is too soft to trust
    # and the number should not be quoted.
    for f in (0.3, 0.7):
        o = measure(f)
        if o:
            print(f"    at {f:.0%} of the step: median {statistics.median(o):.0f} ms "
                  f"({len(o)} flips)")
    gaps = np.diff(ts) * 1000.0
    print(f"  frame arrivals: {1000.0/np.median(gaps):.1f} fps, "
          f"median gap {np.median(gaps):.0f} ms, worst {gaps.max():.0f} ms")
    return 0


if __name__ == "__main__":
    sys.exit(main())
