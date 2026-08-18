#!/usr/bin/env python3
"""A console for the vehicle that runs on this machine and is opened in a browser.

Why a server here rather than a page served by the router:

  1. A browser cannot send a datagram. The control frame is UDP by design - the
     web dashboard used to send one HTTP request per command and hit a browser's
     parallel-connection cap at 20 Hz, which is why the tablet app exists. Here
     the browser sends intent over one WebSocket and *this process* keeps the
     50 Hz TCMD stream going, so the rate discipline lives where it can be met.

  2. MJPEG is charged per viewer. Measured on the board: ustreamer costs 33% of a
     core for one client and 50% for two, with no shared encode to amortise. This
     pulls the camera once and fans it out, so five browsers cost the router what
     one does.

  3. Anything heavy belongs here anyway. The router has four small cores and is
     already running the lidar, the mapper and the camera; a 3D map or a detector
     wants this machine.

What stays on the router is the part that must not depend on this machine being
up: the ring, the 2D map, `navigate`, and `teleop`'s deadman. This console sends
intent and destinations. If it dies, or the wifi goes, the vehicle stops on its
own - which is the whole reason the split is drawn here.

    server.py [--host 192.168.1.1] [--port 8090]

Then open http://localhost:8090/.
"""
import argparse
import asyncio
import json
import socket
import struct
import time

from aiohttp import web, ClientSession, ClientTimeout

RING_PORT = 7602        # ouster-edge's ring, broadcast
TELEOP_PORT = 7721      # teleop, TCMD 24 B - operator intent
NAV_PORT = 7604         # navigate, "GOAL x y" / "ROUTE ..." / "STOP"
MAP_PORT = 7605         # slam2d, "SAVE path" / "LOAD path" / "RESET"
CAM_PORT = 8080
TELE_HZ = 50
# Shorter than teleop's own 300 ms, so the browser going quiet is caught here
# first and the frames stop being armed rather than stopping altogether. Both
# deadmen do the same thing; this one just gets there sooner and can say why.
INTENT_TIMEOUT = 0.25


class Camera:
    """One upstream MJPEG reader, any number of browser viewers.

    The upstream read never waits for a viewer. A browser on a slow link gets
    whatever frame is current when its own write completes and misses the ones in
    between, which is correct for a camera someone is driving by and also means a
    stalled viewer cannot back up the router's send queue. That failure mode is
    not hypothetical: the OpenCV console had 4.5 MB queued on this socket after
    two hours of reading it from its draw loop.
    """

    def __init__(self, url):
        self.url = url
        self.frame = None
        self.seq = 0
        self.arrived = 0
        self.event = asyncio.Event()
        self.state = "starting"

    async def run(self):
        while True:
            try:
                timeout = ClientTimeout(total=None, sock_read=8)
                async with ClientSession(timeout=timeout) as s:
                    async with s.get(self.url) as r:
                        self.state = "live"
                        await self._read(r)
            except Exception as e:
                self.state = f"no camera ({type(e).__name__})"
            await asyncio.sleep(1.0)

    async def _read(self, r):
        acc = bytearray()
        in_frame = False
        prev = -1
        while True:
            chunk = await r.content.read(32768)
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
                        self.frame = bytes(acc)
                        self.seq += 1
                        self.arrived += 1
                        self.event.set()
                        self.event.clear()
                    elif len(acc) > (4 << 20):
                        in_frame = False
                        acc = bytearray()
                prev = b


class Ring:
    """The lidar ring, as the tablet receives it.

    Offsets are from doc/RING-FORMAT.md: sectors at 6, frame_id at 8 as uint16,
    ranges from 20, and 0xFFFF meaning nothing in that sector.
    """

    def __init__(self):
        self.cm = []
        self.frame = 0
        self.at = 0.0
        self.on_ring = None        # set by the app: called with each new ring
        self.dups = 0

    def start(self, loop):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        s.bind(("", RING_PORT))
        s.setblocking(False)
        self.sock = s
        loop.add_reader(s.fileno(), self._drain)

    def _drain(self):
        while True:
            try:
                d = self.sock.recv(65535)
            except (BlockingIOError, OSError):
                return
            if len(d) < 20 or d[:4] != b"OSED":
                continue
            n = struct.unpack_from("<H", d, 6)[0]
            if 20 + 2 * n > len(d):
                continue
            fid = struct.unpack_from("<H", d, 8)[0]
            #
            # The same ring arrives more than once here.
            #
            # ouster-edge broadcasts to 192.168.1.255, and this machine has two
            # interfaces on that subnet - wired to the router and wifi - so the
            # kernel delivers one broadcast to a socket bound to 0.0.0.0 twice.
            # Measured: 18.9 datagrams a second carrying 10.1 distinct frame ids.
            # The tablet has one interface and never saw it.
            #
            # It matters for teaching. A recorder that keeps both copies gives a
            # model two identical inputs for every one the vehicle will actually
            # see, which is a bias with no counterpart at inference.
            #
            # A frame id repeats only 65536 frames later, or 1.8 hours at 10 Hz,
            # and never as the immediately following ring, so comparing with the
            # last one is enough.
            if fid == self.frame and self.cm:
                self.dups += 1
                continue
            cm = list(struct.unpack_from(f"<{n}H", d, 20))
            self.cm = [-1 if v == 0xFFFF else v for v in cm]
            self.frame = fid
            self.at = time.time()
            if self.on_ring:
                self.on_ring(self.cm)


class Recorder:
    """Episodes of (what the lidar saw, what the operator asked for).

    One row per ring, not per wall-clock tick, because the ring is what a model
    will be handed at inference: sampling on a timer would train on interpolated
    inputs that never occur. The intent recorded beside it is the one in force
    when that ring arrived.

    It lives in the server because the server is the only place that holds both
    with one clock. A separate recorder would have to bind the ring itself and
    ask over the network what the operator was doing, and the two would drift by
    however long that took.

    Rows are kept in memory and written on stop: at 10 Hz a row is 360 int16 plus
    a handful of floats, so an hour is about 26 MB.
    """

    def __init__(self, outdir):
        self.dir = outdir
        self.rows = []
        self.on = False
        self.name = ""
        self.written = None

    def start(self, name):
        self.rows = []
        self.on = True
        self.name = name
        self.written = None

    def add(self, cm, drv, pose):
        if not self.on:
            return
        self.rows.append((time.time(), cm, drv.x, drv.y, drv.r,
                          1 if drv.armed else 0, pose))

    def stop(self):
        self.on = False
        if not self.rows:
            self.written = "nothing recorded"
            return
        import numpy as np
        import os
        os.makedirs(self.dir, exist_ok=True)
        n = len(self.rows)
        width = max(len(r[1]) for r in self.rows)
        ring = np.zeros((n, width), dtype=np.int16)
        for i, r in enumerate(self.rows):
            cm = r[1]
            # -1 already means "no return" in the ring; keep it rather than
            # substituting a range, so a model can learn that a direction is open
            # sky rather than a wall at the clip distance.
            ring[i, :len(cm)] = np.array(cm, dtype=np.int16)
        out = os.path.join(self.dir, f"{self.name}.npz")
        np.savez_compressed(
            out,
            t=np.array([r[0] for r in self.rows], dtype=np.float64),
            ring=ring,
            x=np.array([r[2] for r in self.rows], dtype=np.float32),
            y=np.array([r[3] for r in self.rows], dtype=np.float32),
            r=np.array([r[4] for r in self.rows], dtype=np.float32),
            armed=np.array([r[5] for r in self.rows], dtype=np.int8),
            # x, y in cm and heading in slam2d's turn units, so a trajectory can
            # be reconstructed from an episode without the status files.
            pose=np.array([r[6] for r in self.rows], dtype=np.float32),
        )
        self.rows = []
        self.written = f"{out} ({n} rows)"


class Driver:
    """The 50 Hz TCMD stream, and the one browser allowed to steer it.

    One driver at a time, held by whoever asked for it, because two operators
    pushing different intent at one vehicle is not a thing to resolve by
    averaging. Everyone else watches; the page says who has it.

    The frame is 24 bytes: 'TCMD', version 1, an armed flag, a little-endian
    sequence number at 8, and three axes at 12/14/16 as int16 in 1/10000 -
    integers so nothing depends on three languages agreeing about floats. Axis
    order is strafe, forward, yaw, as in doc/TELEOP.md.
    """

    def __init__(self, host):
        self.host = host
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.owner = None          # ws that holds control
        self.armed = False
        self.x = self.y = self.r = 0.0
        self.last_intent = 0.0
        self.sent = 0
        self.lapsed = 0            # times the browser went quiet while armed
        self.trailing = 0

    def take(self, ws):
        if self.owner is not None and self.owner is not ws:
            return False
        self.owner = ws
        return True

    def release(self, ws):
        if self.owner is ws:
            self.owner = None
            self.armed = False
            self.x = self.y = self.r = 0.0
            # A few neutral frames on the way out, so teleop goes to neutral now
            # rather than when its own deadman notices in 300 ms.
            self.trailing = 5

    def intent(self, ws, x, y, r, armed):
        if self.owner is not ws:
            return False
        self.x, self.y, self.r = x, y, r
        self.armed = bool(armed)
        self.last_intent = time.monotonic()
        return True

    async def run(self):
        period = 1.0 / TELE_HZ
        while True:
            #
            # Silence when nobody is driving, rather than a stream of disarmed
            # frames.
            #
            # Sending anyway looked harmless - the frames carry armed=0 and zero
            # axes - but it means teleop's deadman never trips, so teleop reports
            # a live operator link whenever this process is running. That is the
            # one thing its status is for, and it would have been reassuring at
            # exactly the wrong moment.
            #
            if self.owner is None and self.trailing <= 0:
                await asyncio.sleep(period)
                continue
            if self.trailing > 0:
                self.trailing -= 1
            stale = (time.monotonic() - self.last_intent) > INTENT_TIMEOUT
            if stale and self.armed:
                # Not an error to report once and forget: this is the ordinary
                # way a browser tab stops being in charge - closed, backgrounded
                # by the OS, or on the far side of a wifi hole. Count it and go
                # neutral.
                self.armed = False
                self.x = self.y = self.r = 0.0
                self.lapsed += 1
            self._send()
            await asyncio.sleep(period)

    def _send(self):
        p = bytearray(24)
        p[0:4] = b"TCMD"
        p[4] = 1
        p[5] = 1 if self.armed else 0
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        struct.pack_into("<I", p, 8, self.seq)
        for off, v in ((12, self.x), (14, self.y), (16, self.r)):
            q = int(max(-1.0, min(1.0, v if self.armed else 0.0)) * 10000)
            struct.pack_into("<h", p, off, q)
        try:
            self.sock.sendto(bytes(p), (self.host, TELEOP_PORT))
            self.sent += 1
        except OSError:
            pass


def udp_line(host, port, text):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.sendto(text.encode(), (host, port))
        return True
    except OSError:
        return False
    finally:
        s.close()


async def status_poller(app):
    """The daemons' own JSON, and the exported map, polled once for everyone.

    Results go into app["state"], a dict created before startup and mutated in
    place: aiohttp deprecates assigning new app keys once the application has
    started, and it is right to - a running app's key set should be fixed.
    """
    st = app["state"]
    host = app["host"]
    names = ("ouster", "slam2d", "navigate", "can", "teleop")
    async with ClientSession(timeout=ClientTimeout(total=3)) as s:
        while True:
            out = {}
            for n in names:
                try:
                    async with s.get(f"http://{host}/sensors/{n}.json") as r:
                        out[n] = json.loads(await r.text())
                except Exception:
                    out[n] = None
            st["status"] = out
            try:
                async with s.get(f"http://{host}/sensors/map.s2mp") as r:
                    st["map"] = await r.read()
            except Exception:
                pass
            await asyncio.sleep(0.5)


# ------------------------------------------------------------------ handlers

async def index(request):
    return web.FileResponse(request.app["static"] / "index.html")


async def camera(request):
    """multipart/x-mixed-replace, so an <img> tag is the whole client."""
    cam = request.app["cam"]
    r = web.StreamResponse(headers={
        "Content-Type": "multipart/x-mixed-replace; boundary=frame",
        "Cache-Control": "no-store",
    })
    await r.prepare(request)
    last = -1
    try:
        while True:
            if cam.frame is None or cam.seq == last:
                await asyncio.wait_for(cam.event.wait(), timeout=5)
                continue
            last = cam.seq
            f = cam.frame
            await r.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                          b"Content-Length: " + str(len(f)).encode() +
                          b"\r\n\r\n" + f + b"\r\n")
    except (asyncio.TimeoutError, ConnectionResetError, asyncio.CancelledError):
        pass
    return r


async def mapfile(request):
    b = request.app["state"].get("map")
    if not b:
        raise web.HTTPServiceUnavailable(text="no map yet")
    return web.Response(body=b, content_type="application/octet-stream")


async def ws_handler(request):
    ws = web.WebSocketResponse(heartbeat=5)
    await ws.prepare(request)
    app = request.app
    drv = app["drv"]
    rec = app["rec"]
    host = app["host"]

    async def telemetry():
        while not ws.closed:
            ring = app["ring"]
            await ws.send_json({
                "t": "tele",
                "ring": {"cm": ring.cm, "frame": ring.frame, "dups": ring.dups,
                         "age": round(time.time() - ring.at, 2) if ring.at else None},
                "status": app["state"].get("status", {}),
                "cam": {"state": app["cam"].state, "frames": app["cam"].arrived},
                "drive": {"mine": drv.owner is ws, "held": drv.owner is not None,
                          "armed": drv.armed, "sent": drv.sent,
                          "lapsed": drv.lapsed,
                          "x": round(drv.x, 3), "y": round(drv.y, 3),
                          "r": round(drv.r, 3)},
                "rec": {"on": rec.on, "rows": len(rec.rows),
                        "written": rec.written},
            })
            await asyncio.sleep(0.2)

    task = asyncio.create_task(telemetry())
    try:
        async for msg in ws:
            if msg.type is not web.WSMsgType.TEXT:
                continue
            try:
                m = json.loads(msg.data)
            except ValueError:
                continue
            t = m.get("t")
            if t == "intent":
                drv.intent(ws, float(m.get("x", 0)), float(m.get("y", 0)),
                           float(m.get("r", 0)), m.get("armed"))
            elif t == "take":
                await ws.send_json({"t": "control", "ok": drv.take(ws)})
            elif t == "release":
                drv.release(ws)
                await ws.send_json({"t": "control", "ok": False})
            elif t == "goal":
                udp_line(host, NAV_PORT,
                         f"GOAL {int(m['x'])} {int(m['y'])}")
            elif t == "route":
                pts = " ".join(f"{int(x)} {int(y)}" for x, y in m.get("pts", []))
                if pts:
                    udp_line(host, NAV_PORT, f"ROUTE {pts}")
            elif t == "navstop":
                udp_line(host, NAV_PORT, "STOP")
            elif t == "record":
                if m.get("on"):
                    rec.start(m.get("name") or time.strftime("ep-%Y%m%d-%H%M%S"))
                else:
                    rec.stop()
            elif t == "map":
                op = m.get("op")
                fl = int(m.get("floor", 1))
                if op == "save":
                    udp_line(host, MAP_PORT,
                             f"SAVE /etc/keti/maps/floor{fl}.s2mp")
                elif op == "load":
                    udp_line(host, MAP_PORT,
                             f"LOAD /etc/keti/maps/floor{fl}.s2mp")
                elif op == "reset":
                    udp_line(host, MAP_PORT, "RESET")
    finally:
        task.cancel()
        drv.release(ws)
    return ws


async def on_start(app):
    loop = asyncio.get_running_loop()
    app["ring"].start(loop)
    def _pose():
        p = ((app["state"].get("status", {}).get("slam2d") or {})
             .get("pose_cm") or {})
        return (p.get("x", 0), p.get("y", 0), p.get("a", 0))

    app["ring"].on_ring = lambda cm: app["rec"].add(cm, app["drv"], _pose())
    app["state"]["tasks"] = [
        asyncio.create_task(app["cam"].run()),
        asyncio.create_task(app["drv"].run()),
        asyncio.create_task(status_poller(app)),
    ]


async def on_stop(app):
    for t in app["state"].get("tasks", []):
        t.cancel()


def main():
    import pathlib
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.1", help="the router")
    ap.add_argument("--port", type=int, default=8090, help="serve here")
    # A list, and not 0.0.0.0 by default.
    #
    # This page can move a vehicle, and 0.0.0.0 is every interface - which on this
    # bench includes the office network as well as the router's. Naming the
    # router-side addresses gives the tablet and any other machine on the vehicle's
    # network access without publishing the steering wheel to the building.
    ap.add_argument("--bind", default="127.0.0.1",
                    help="comma-separated addresses to serve on; prefer the "
                         "router-side address over 0.0.0.0")
    a = ap.parse_args()

    app = web.Application()
    app["host"] = a.host
    app["static"] = pathlib.Path(__file__).resolve().parent / "static"
    app["cam"] = Camera(f"http://{a.host}:{CAM_PORT}/stream")
    app["ring"] = Ring()
    app["drv"] = Driver(a.host)
    app["state"] = {"status": {}}
    app["rec"] = Recorder(pathlib.Path(__file__).resolve().parent / "episodes")
    app.router.add_get("/", index)
    app.router.add_get("/camera.mjpg", camera)
    app.router.add_get("/map.s2mp", mapfile)
    app.router.add_get("/ws", ws_handler)
    app.on_startup.append(on_start)
    app.on_cleanup.append(on_stop)

    binds = [h.strip() for h in a.bind.split(",") if h.strip()]
    print(f"  router {a.host}, console on:")
    for h in binds:
        print(f"    http://{'localhost' if h == '127.0.0.1' else h}:{a.port}/")
    web.run_app(app, host=binds, port=a.port, print=None)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
