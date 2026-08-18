#!/usr/bin/env python3
"""Let the policy drive, through exactly the path a person drives through.

    pilot.py [--policy policy.pt] [--console ws://localhost:8090/ws]
             [--max 0.4] [--arm]

It is a client of the console server, not a replacement for it: it asks for
control, then sends intent, and every guard that applies to a human applies to it
unchanged. The server disarms it after 250 ms of silence, `teleop` disarms after
300 ms without frames, and the vehicle stops. Nothing new had to be trusted for
that to be true - which is the reason this is a WebSocket client and not a
process that writes TCMD to the wire itself.

**Disarmed unless --arm.** Without it the policy runs, the numbers print, and the
frames carry armed=0, so the vehicle cannot move. That is the default because a
behaviour-cloned net's first run is the one you least want to be a surprise, and
because the interesting failure - it steers into the wall it was trained to avoid -
is visible in the printout before it is visible in the room.

--max scales the output. A policy trained from a careful operator will ask for
whatever that operator asked for, and the first time it asks for it should not be
at full speed.
"""
import argparse
import asyncio
import json
import os
import socket
import struct
import sys
import time

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import load, rings_to_input           # noqa: E402

RING_PORT = 7602


class RingSource:
    """The ring, deduplicated, newest only.

    The same dedupe the console server needs: this machine has two interfaces on
    the router's subnet, so a broadcast is delivered twice and the policy would
    otherwise be run twice on every scan.
    """

    def __init__(self):
        self.cm = None
        self.frame = -1
        self.dups = 0
        self.count = 0
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("", RING_PORT))
        s.setblocking(False)
        self.sock = s

    def start(self, loop):
        loop.add_reader(self.sock.fileno(), self._drain)

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
            if fid == self.frame:
                self.dups += 1
                continue
            cm = np.frombuffer(d, dtype="<u2", count=n, offset=20).astype(np.int32)
            cm[cm == 0xFFFF] = -1
            self.cm = cm
            self.frame = fid
            self.count += 1


async def main_async(a):
    import websockets

    dev = "cuda" if torch.cuda.is_available() and not a.cpu else "cpu"
    net, ck = load(a.policy, dev)
    print(f"  {a.policy}: {ck['beams']} beams, trained on {ck.get('rows','?')} rows, "
          f"val {ck.get('val', float('nan')):.5f}")
    print(f"  running on {dev}, output scaled by {a.max}, "
          f"{'ARMED - the vehicle can move' if a.arm else 'disarmed (dry run)'}")

    ring = RingSource()
    ring.start(asyncio.get_running_loop())

    async with websockets.connect(a.console) as ws:
        await ws.send(json.dumps({"t": "take"}))
        held = None
        last_frame = -1
        n = 0
        t0 = time.time()
        infer_ns = 0
        last_print = t0

        async def reader():
            nonlocal held
            try:
                async for raw in ws:
                    m = json.loads(raw)
                    if m.get("t") == "control":
                        held = m.get("ok")
                    elif m.get("t") == "tele":
                        held = m["drive"]["mine"]
            except Exception:
                pass

        rd = asyncio.create_task(reader())
        try:
            while True:
                await asyncio.sleep(0.005)
                if ring.cm is None or ring.frame == last_frame:
                    continue
                last_frame = ring.frame
                if ring.cm.shape[0] != ck["beams"]:
                    print(f"  ring has {ring.cm.shape[0]} beams, the policy wants "
                          f"{ck['beams']} - refusing to guess")
                    break
                x = rings_to_input(ring.cm)[None, :]
                t1 = time.perf_counter_ns()
                with torch.no_grad():
                    out = net(torch.from_numpy(x).to(dev))[0].cpu().numpy()
                infer_ns += time.perf_counter_ns() - t1
                out = np.clip(out * a.max, -1.0, 1.0)
                n += 1
                await ws.send(json.dumps({
                    "t": "intent", "x": float(out[0]), "y": float(out[1]),
                    "r": float(out[2]), "armed": bool(a.arm and held),
                }))
                now = time.time()
                if now - last_print >= 1.0:
                    last_print = now
                    print(f"  {n:5d} rings  {n/(now-t0):4.1f} Hz  "
                          f"infer {infer_ns/max(n,1)/1e6:5.2f} ms  "
                          f"strafe {out[0]:+.2f} forward {out[1]:+.2f} "
                          f"yaw {out[2]:+.2f}  "
                          f"{'armed' if (a.arm and held) else 'dry'}"
                          f"{'' if held else '  [no control]'}", flush=True)
        finally:
            rd.cancel()
            # Neutral and released on the way out, rather than leaving the last
            # intent to be caught by a deadman.
            try:
                await ws.send(json.dumps({"t": "intent", "x": 0, "y": 0, "r": 0,
                                          "armed": False}))
                await ws.send(json.dumps({"t": "release"}))
            except Exception:
                pass
    print(f"  {n} rings, {ring.dups} duplicate datagrams dropped")
    return 0


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default=os.path.join(here, "policy.pt"))
    ap.add_argument("--console", default="ws://localhost:8090/ws")
    ap.add_argument("--max", type=float, default=0.4)
    ap.add_argument("--arm", action="store_true",
                    help="actually let it move the vehicle")
    ap.add_argument("--cpu", action="store_true")
    ap.add_argument("--seconds", type=float, default=0.0)
    a = ap.parse_args()
    if not os.path.exists(a.policy):
        sys.exit(f"no policy at {a.policy} - train.py writes one")
    try:
        if a.seconds:
            return asyncio.run(asyncio.wait_for(main_async(a), a.seconds))
        return asyncio.run(main_async(a))
    except (KeyboardInterrupt, asyncio.TimeoutError):
        return 0


if __name__ == "__main__":
    sys.exit(main())
