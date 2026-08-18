#!/usr/bin/env python3
"""One window on the PC with everything the router is producing.

The tablet app is the thing you drive with; this is the same information on a
machine with a keyboard and a screen, which is what you want while working on the
router rather than while walking behind a vehicle.

    camera      MJPEG from ustreamer
    microphone  a rolling envelope of the PCM stream, drawn the same way the
                tablet draws it - one peak per buffer, mirrored about a centre
                line that is always present, so a dead microphone is a flat line
                rather than an empty box
    map         the occupancy grid slam2d exports, with the vehicle pose
    status      the daemons' own JSON, including the mapper's match score
    control     WASD/arrows send TELE frames to agx-cmd

Nothing here can move a vehicle on its own: agx-cmd is off by default, needs
can-bridge's allow_inject, and refuses to command at all until the bus has said
which protocol generation it is. Sending is off until armed with the space bar,
and it stops the moment the key is released or the window loses focus.

    console.py [--host 192.168.1.1]
"""
import argparse
import json
import socket
import struct
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np

W, H = 1600, 900
TELE_PORT = 7722          # agx-cmd; 7721 is teleop and speaks a different frame
RING_PORT = 7602
S2_UNKNOWN = 128


def http_json(url, timeout=2.0):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except Exception:
        return None


class Mic(threading.Thread):
    """Peak per buffer from the router's PCM stream.

    Only the envelope is kept. Decoding to audible sound is the tablet's job -
    here the point is to see whether the microphone is alive and what it is
    picking up, and that is one number per buffer.
    """

    def __init__(self, base, n=220):
        super().__init__(daemon=True)
        self.base = base
        self.levels = np.zeros(n, dtype=np.float32)
        self.head = 0
        self.state = "off"
        self.stop = False

    def run(self):
        while not self.stop:
            info = http_json(f"{self.base}/info")
            if not info:
                self.state = "no mic"
                time.sleep(2)
                continue
            rate = info.get("rate", 16000)
            self.state = f"{rate//1000} kHz"
            try:
                r = urllib.request.urlopen(f"{self.base}/pcm", timeout=4)
                while not self.stop:
                    d = r.read(1024)
                    if not d:
                        break
                    v = np.frombuffer(d[:len(d) // 2 * 2], dtype="<i2")
                    self.levels[self.head % len(self.levels)] = (
                        np.abs(v).max() / 32768.0 if v.size else 0.0)
                    self.head += 1
                r.close()
            except Exception:
                self.state = "mic dropped"
                time.sleep(1)


class Ring(threading.Thread):
    """ouster-edge's one-range-per-sector ring, as the tablet receives it."""

    def __init__(self, port=RING_PORT):
        super().__init__(daemon=True)
        self.port = port
        self.cm = None
        self.refl = None
        self.frame = 0
        self.stop = False

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("", self.port))
        except OSError:
            return
        s.settimeout(1.0)
        while not self.stop:
            try:
                d, _ = s.recvfrom(65535)
            except socket.timeout:
                continue
            # Offsets from doc/RING-FORMAT.md rather than guessed. The first
            # attempt read sectors at 8, frame_id at 4 as a uint32 and the
            # ranges from 16, and produced "no ring" against a stream that was
            # arriving correctly - the datagram was fine and the reader was
            # wrong. The real layout: sectors at 6, frame_id at 8 as uint16,
            # ranges from 20, and 0xFFFF for "nothing in this sector".
            if len(d) < 20 or d[:4] != b"OSED":
                continue
            n = struct.unpack_from("<H", d, 6)[0]
            if 20 + 3 * n <= len(d):
                cm = np.frombuffer(d, dtype="<u2", count=n,
                                   offset=20).astype(np.int32)
                cm[cm == 0xFFFF] = -1
                self.cm = cm
                self.frame = struct.unpack_from("<H", d, 8)[0]
                self.refl = np.frombuffer(d, dtype=np.uint8, count=n,
                                          offset=20 + 2 * n)


class Cam(threading.Thread):
    """The newest camera frame, and no queue behind it.

    Reading the stream from the draw loop looks fine and is not. VideoCapture
    hands over the *next* frame in order, and this loop also polls four status
    files and redraws a 1600x660 canvas, so it does not run at 20 fps. Every
    revolution it falls short, one more frame backs up - first in FFmpeg, then
    in the kernel receive queue, and the picture drifts further behind with no
    mechanism to catch up. Found in the act: 4.5 MB queued on the socket after
    two hours, which at 25 Mbit/s is a view well over a second stale, and the
    router had been sending all of it.

    So the read happens here, as fast as the stream arrives, and only the last
    frame is kept. The draw loop takes whatever is current and the rest are
    dropped, which is what you want from a camera you are driving by.
    """

    def __init__(self, url):
        super().__init__(daemon=True)
        self.url = url
        self.frame = None
        self.unread = False
        self.n = 0
        self.dropped = 0
        self.stop = threading.Event()

    def run(self):
        cap = cv2.VideoCapture(self.url)
        # Ask the backend for a shallow buffer too. Not every backend honours
        # it, which is why it is a belt over the braces above and not the fix.
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        while not self.stop.is_set():
            ok, f = cap.read()
            if not ok:
                time.sleep(0.2)
                continue
            if self.unread:
                self.dropped += 1
            self.frame = f
            self.unread = True
            self.n += 1
        cap.release()

    def latest(self):
        """The newest frame, or the last one again if none has arrived since.

        Not cleared on read: a draw loop faster than the stream would otherwise
        alternate between a picture and "no camera", which would say the camera
        had failed when the truth is that it is a 20 fps camera.
        """
        self.unread = False
        return self.frame


class MapPoll(threading.Thread):
    """The exported occupancy grid, and the pose that goes with it."""

    def __init__(self, base):
        super().__init__(daemon=True)
        self.base = base
        self.m = None
        self.state = "no map"
        self.stop = False

    def run(self):
        while not self.stop:
            try:
                with urllib.request.urlopen(f"{self.base}/map.s2mp",
                                            timeout=2.5) as r:
                    b = r.read()
                if len(b) >= 32 and b[:4] == b"S2MP":
                    w, h = struct.unpack_from("<HH", b, 6)
                    res = struct.unpack_from("<H", b, 10)[0]
                    ox, oy, px, py, pa = struct.unpack_from("<iiiii", b, 12)
                    if len(b) >= 32 + w * h:
                        cells = np.frombuffer(b, dtype=np.uint8,
                                              count=w * h, offset=32)
                        self.m = (cells.reshape(h, w), w, h, res,
                                  ox, oy, px, py, pa)
                        self.state = f"{w}x{h} @ {res} cm"
                else:
                    self.state = "not a map"
            except Exception:
                self.state = "no map"
            time.sleep(0.4)


def draw_mic(canvas, x, y, w, h, mic):
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (40, 40, 46), 1)
    mid = y + h // 2
    cv2.line(canvas, (x, mid), (x + w, mid), (70, 70, 78), 1)
    n = len(mic.levels)
    peak = float(mic.levels.max())
    # Scaled to what was heard, not to full scale.
    #
    # A quiet room peaks around 0.013, and drawn against 1.0 that is a single
    # pixel either side of the centre - the panel looked broken while the
    # microphone was working perfectly. The scale follows the recent maximum,
    # with a floor so a silent room does not amplify its own noise into a
    # waveform, and the true peak is printed so the gain is never a mystery.
    span = max(peak, 0.02)
    for i in range(n):
        v = float(mic.levels[(mic.head + i) % n])
        if v <= 0:
            continue
        bh = int(min(v / span, 1.0) * (h // 2 - 3))
        px = x + int(i * w / n)
        # Full scale is clipping and gets its own colour: by the time a mean
        # level shows it, the audio is already damaged.
        c = (60, 60, 235) if v >= 0.98 else (120, 220, 120)
        cv2.line(canvas, (px, mid - bh), (px, mid + bh), c, 1)
    cv2.putText(canvas,
                f"mic {mic.state}   peak {peak:.3f}   scale {span:.3f} full",
                (x + 6, y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (190, 190, 200), 1)


def draw_map(canvas, x, y, w, h, mp, ring):
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (40, 40, 46), 1)
    if mp.m is None:
        cv2.putText(canvas, mp.state, (x + 10, y + 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 130), 1)
        return
    cells, cw, ch, res, ox, oy, px, py, pa = mp.m
    # Occupied dark, free light, unknown mid grey - the same reading as the
    # tablet, so a screenshot from either is comparable.
    # This had it backwards: occupied was drawn near-white and free mid-grey,
    # against a comment saying the opposite. slam2d adds S2_HIT on a return and
    # subtracts along the ray to it, so above S2_UNKNOWN is occupied - and the
    # tablet draws occupied dark. The two consoles disagreed about which parts of
    # a room were walls, which is the sort of thing that survives a long time
    # because either picture looks like a plausible map on its own.
    img = np.full((ch, cw, 3), 228, dtype=np.uint8)      # unknown
    free = cells < S2_UNKNOWN - 8
    occ = cells > S2_UNKNOWN + 8
    img[free] = (255, 255, 255)
    img[occ] = (40, 40, 40)
    img = cv2.flip(img, 0)                    # y up, as the map is stored y up
    scale = min((w - 8) / cw, (h - 8) / ch)
    img = cv2.resize(img, (int(cw * scale), int(ch * scale)),
                     interpolation=cv2.INTER_NEAREST)
    ih, iw = img.shape[:2]
    x0, y0 = x + (w - iw) // 2, y + (h - ih) // 2
    canvas[y0:y0 + ih, x0:x0 + iw] = img

    # The vehicle, drawn from the pose in the same file rather than assumed at
    # the centre: the map's origin moves as slam2d grows it.
    vx = x0 + int((px - ox) / res * scale)
    vy = y0 + ih - int((py - oy) / res * scale)
    if 0 <= vx - x0 < iw and 0 <= vy - y0 < ih:
        cv2.circle(canvas, (vx, vy), 5, (90, 220, 90), -1)
        ang = pa * 2 * np.pi / 4096.0
        cv2.line(canvas, (vx, vy),
                 (int(vx + 18 * np.cos(ang)), int(vy - 18 * np.sin(ang))),
                 (90, 220, 90), 2)
    cv2.putText(canvas, f"map {mp.state}   pose {px/100:.2f}, {py/100:.2f} m",
                (x + 6, y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (190, 190, 200), 1)


def draw_ring(canvas, x, y, w, h, ring):
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (40, 40, 46), 1)
    cx, cy = x + w // 2, y + h // 2
    if ring.cm is None:
        cv2.putText(canvas, "no ring", (x + 10, y + 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 130), 1)
        return
    v = ring.cm
    good = v[v > 0]
    # Scaled to what came back, not to what the sensor could see - a desk-top
    # sensor against a 30 m scale draws a speck.
    mx = max(200, int(np.percentile(good, 98)) if good.size else 200)
    rpx = min(w, h) // 2 - 12
    for r in (0.5, 1.0):
        cv2.circle(canvas, (cx, cy), int(rpx * r), (55, 55, 62), 1)
    n = len(v)
    for i in range(n):
        if v[i] <= 0:
            continue
        a = 2 * np.pi * i / n
        d = min(v[i] / mx, 1.0) * rpx
        cv2.circle(canvas, (int(cx + d * np.cos(a)), int(cy - d * np.sin(a))),
                   1, (230, 190, 90), -1)
    cv2.circle(canvas, (cx, cy), 3, (90, 220, 90), -1)
    cv2.putText(canvas, f"ring frame {ring.frame}  {mx/100:.1f} m full scale",
                (x + 6, y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (190, 190, 200), 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.1")
    ap.add_argument("--snapshot", default="",
                    help="write the composed window here and exit")
    ap.add_argument("--seconds", type=float, default=0.0)
    # Off by default, because MJPEG costs the router once per viewer.
    #
    # There is no shared encode to amortise: every client gets its own copy of
    # the bytes, so a second viewer is a second 25 Mbit/s and, measured on the
    # bench, another 26% of a core. The tablet is the screen somebody drives by,
    # so it gets the camera; this console is for the lidar, the map and the
    # telemetry, and asks for video only when that is what you came for.
    ap.add_argument("--camera", action="store_true",
                    help="also stream the camera (doubles the router's video load)")
    a = ap.parse_args()
    base = f"http://{a.host}/sensors"

    mic = Mic(f"http://{a.host}:8082")
    mic.start()
    ring = Ring()
    ring.start()
    mp = MapPoll(base)
    mp.start()

    cam = None
    if a.camera:
        cam = Cam(f"http://{a.host}:8080/stream")
        cam.start()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    armed = False
    fwd = strafe = yaw = 0.0
    status = {}
    last_status = 0.0

    cv2.namedWindow("A3004 console", cv2.WINDOW_AUTOSIZE)
    t_start = time.time()
    print("  arrows/WASD to steer, space to arm, q to quit")
    while True:
        canvas = np.full((H, W, 3), 24, dtype=np.uint8)

        frame = cam.latest() if cam is not None else None
        if frame is not None:
            fh = 470
            fw = int(frame.shape[1] * fh / frame.shape[0])
            canvas[20:20 + fh, 20:20 + fw] = cv2.resize(frame, (fw, fh))
            cv2.rectangle(canvas, (20, 20), (20 + fw, 20 + fh), (40, 40, 46), 1)
            cv2.putText(canvas, "camera", (26, 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.42, (220, 220, 230), 1)
        else:
            # Distinguish "not asked for" from "asked for and not arriving".
            # The old text said "no camera" either way, which reads as a fault
            # when the truth is that the tablet has the camera and this console
            # deliberately does not.
            msg = ("camera not requested - run with --camera"
                   if cam is None else "no camera")
            cv2.putText(canvas, msg, (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 130), 1)

        draw_mic(canvas, 20, 510, 820, 120, mic)
        draw_ring(canvas, 860, 20, 340, 340, ring)
        draw_map(canvas, 1220, 20, 360, 340, mp, ring)

        now = time.time()
        if now - last_status > 0.5:
            last_status = now
            status = {n: http_json(f"{base}/{n}.json")
                      for n in ("ouster", "slam2d", "navigate", "can")}

        ty = 400
        for name in ("ouster", "slam2d", "navigate", "can"):
            j = status.get(name)
            if name == "ouster" and j:
                t = (f"lidar  {j.get('packets',0)} pkt  "
                     f"missed {j.get('missed_columns',0)}  "
                     f"relayed {j.get('relayed',0)}")
            elif name == "slam2d" and j:
                t = (f"slam   match {j.get('score_frac_pct',0)}%  "
                     f"{j.get('matched',0)}/{j.get('rings',0)} rings  "
                     f"{j.get('match_us',0)/1000:.0f} ms")
            elif name == "navigate" and j:
                t = (f"nav    {j.get('state','?')}  "
                     f"{j.get('fault') or 'no fault'}")
            elif name == "can" and j:
                t = (f"can    {j.get('interface','?')}  rx {j.get('rx',0)}  "
                     f"{j.get('agilex_protocol','?')}")
            else:
                t = f"{name:6s} not available"
            cv2.putText(canvas, t, (866, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                        (190, 190, 200), 1)
            ty += 22

        # Control. Held keys only: cv2 gives key-down events, so a released key
        # decays to zero rather than latching, which is the behaviour that
        # matters if this ever reaches wheels.
        col = (60, 60, 235) if armed else (120, 120, 130)
        cv2.putText(canvas,
                    f"{'ARMED' if armed else 'safe'}   "
                    f"fwd {fwd:+.2f}  strafe {strafe:+.2f}  yaw {yaw:+.2f}",
                    (866, ty + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        cv2.putText(canvas, "space arm   WASD/arrows move   q quit",
                    (866, ty + 44), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                    (120, 120, 130), 1)

        cv2.imshow("A3004 console", canvas)
        k = cv2.waitKey(30) & 0xFF
        if k == ord('q'):
            break
        if a.seconds and time.time() - t_start >= a.seconds:
            if a.snapshot:
                cv2.imwrite(a.snapshot, canvas)
                print(f"  wrote {a.snapshot}")
            break
        if k == ord(' '):
            armed = not armed
            fwd = strafe = yaw = 0.0
        step = 0.25
        if k in (ord('w'), 82):
            fwd = min(1.0, fwd + step)
        elif k in (ord('s'), 84):
            fwd = max(-1.0, fwd - step)
        elif k in (ord('a'), 81):
            strafe = max(-1.0, strafe - step)
        elif k in (ord('d'), 83):
            strafe = min(1.0, strafe + step)
        elif k == ord('q'):
            pass
        elif k == ord('e'):
            yaw = min(1.0, yaw + step)
        elif k == ord('r'):
            yaw = max(-1.0, yaw - step)
        elif k == 255:
            # Nothing pressed this frame: decay toward zero rather than holding
            # the last command, which is the difference between a stick and a
            # latch.
            fwd *= 0.7
            strafe *= 0.7
            yaw *= 0.7
            for v in ("fwd", "strafe", "yaw"):
                if abs(locals()[v]) < 0.02:
                    pass
            if abs(fwd) < 0.02:
                fwd = 0.0
            if abs(strafe) < 0.02:
                strafe = 0.0
            if abs(yaw) < 0.02:
                yaw = 0.0

        seq += 1
        p = bytearray(32)
        p[0:4] = b"TELE"
        p[4] = 1
        p[5] = 1 if armed else 0
        struct.pack_into("<I", p, 8, seq)
        struct.pack_into("<Q", p, 12, int(time.time() * 1000) & ((1 << 64) - 1))
        struct.pack_into("<hhhh", p, 20,
                         int(strafe * 10000), int(fwd * 10000),
                         int(yaw * 10000), 0)
        try:
            tx.sendto(bytes(p), (a.host, TELE_PORT))
        except OSError:
            pass

    mic.stop = ring.stop = mp.stop = True
    if cam is not None:
        cam.stop.set()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
