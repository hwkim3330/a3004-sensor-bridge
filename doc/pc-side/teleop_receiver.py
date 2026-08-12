#!/usr/bin/env python3
"""Reference receiver for teleop intent, with its own deadman.

The point of this file is the deadman, not the plumbing. The router already has
one, but a deadman on the sending side cannot detect the failure that matters:
the link between the two. So this one is independent and does not trust the
sender's armed flag on its own — it also requires the sender's timestamp to be
recent and the sequence to be advancing.

Three conditions must all hold for the command to be considered live:

  1. the sender says it is armed
  2. a datagram arrived within `timeout` (local clock)
  3. the sender's own monotonic clock is advancing between datagrams

(3) catches a case (1) and (2) miss: a stuck sender that keeps retransmitting
one frozen frame. Then, and only then, are the axes passed on.

Run it standalone to watch, or with --ros to publish geometry_msgs/Twist:

    python3 teleop_receiver.py --port 7720
    python3 teleop_receiver.py --port 7720 --ros

Nothing here actuates anything. Wire the `on_command` callback to whatever does,
and keep the rule that a not-live state means commanding zero, not commanding
nothing.
"""
import argparse
import socket
import struct
import sys
import time

MAGIC = b"TELE"
PKT_LEN = 32
AXIS_SCALE = 10000.0


class Receiver:
    def __init__(self, port, bind="0.0.0.0", timeout=0.3, stale_ticks=3):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind, port))
        self.sock.settimeout(0.05)

        self.timeout = timeout
        self.stale_ticks = stale_ticks

        self.live = False
        self.axes = [0.0] * 4
        self.buttons = 0
        self.last_rx = 0.0
        self.last_seq = None
        self.last_sender_ts = None
        self.frozen_count = 0
        self.stats = dict(packets=0, bad=0, gaps=0, deadman=0, frozen=0,
                          resyncs=0)

    # --- override or reassign this ---
    def on_command(self, live, axes, buttons):
        pass

    def _neutralise(self, why):
        if self.live:
            self.live = False
            self.stats["deadman"] += 1
            print(f"[deadman] {why} -> commanding zero", flush=True)
        self.axes = [0.0] * 4
        self.buttons = 0

    def poll(self):
        now = time.monotonic()

        try:
            while True:
                data, _ = self.sock.recvfrom(2048)
                self._ingest(data, now)
        except socket.timeout:
            pass
        except BlockingIOError:
            pass

        # Condition 2: nothing recent enough.
        if self.last_rx and now - self.last_rx > self.timeout:
            self._neutralise(f"no datagram for {now - self.last_rx:.2f}s")

        self.on_command(self.live, list(self.axes), self.buttons)

    def _ingest(self, data, now):
        if len(data) < PKT_LEN or data[:4] != MAGIC or data[4] != 1:
            self.stats["bad"] += 1
            return

        self.stats["packets"] += 1
        armed = bool(data[5] & 1)
        (seq,) = struct.unpack_from("<I", data, 8)
        (sender_ts,) = struct.unpack_from("<Q", data, 12)
        axes = [v / AXIS_SCALE for v in struct.unpack_from("<4h", data, 20)]
        (buttons,) = struct.unpack_from("<H", data, 28)

        if self.last_seq is not None:
            if seq <= self.last_seq:
                # A sender restart resets the sequence to 1. Without this the
                # receiver would reject every frame from then on and stay
                # neutral forever - a router reboot would wedge it permanently.
                if self.last_seq - seq > 1000:
                    print(f"[resync] sender restarted (seq {self.last_seq} "
                          f"-> {seq})", flush=True)
                    self.stats["resyncs"] = self.stats.get("resyncs", 0) + 1
                    self.last_seq = None
                    self.last_sender_ts = None
                    self.frozen_count = 0
                else:
                    # a replay or a reordered datagram: ignore it entirely
                    self.stats["bad"] += 1
                    return
            elif seq != self.last_seq + 1:
                self.stats["gaps"] += seq - self.last_seq - 1
        self.last_seq = seq
        self.last_rx = now

        # Condition 3: the sender's clock must be moving. A sender wedged on one
        # frame still increments seq if it is looping, so the clock is the
        # tell-tale.
        if self.last_sender_ts is not None and sender_ts <= self.last_sender_ts:
            self.frozen_count += 1
            if self.frozen_count >= self.stale_ticks:
                self.stats["frozen"] += 1
                self._neutralise("sender clock not advancing")
                return
        else:
            self.frozen_count = 0
        self.last_sender_ts = sender_ts

        # Condition 1.
        if not armed:
            self._neutralise("sender disarmed") if self.live else None
            self.axes = [0.0] * 4
            self.buttons = 0
            self.live = False
            return

        if not self.live:
            self.live = True
            print("[live] armed command accepted", flush=True)
        self.axes = axes
        self.buttons = buttons


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=7720)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--timeout", type=float, default=0.3)
    ap.add_argument("--ros", action="store_true",
                    help="publish geometry_msgs/Twist on ~/cmd_vel")
    ap.add_argument("--max-linear", type=float, default=1.0,
                    help="metres per second at full axis deflection")
    ap.add_argument("--max-angular", type=float, default=1.5,
                    help="radians per second at full axis deflection")
    args = ap.parse_args()

    rx = Receiver(args.port, args.bind, args.timeout)

    if args.ros:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist

        rclpy.init(args=sys.argv)
        node = Node("teleop_receiver")
        pub = node.create_publisher(Twist, "~/cmd_vel", 10)

        def publish(live, axes, buttons):
            t = Twist()
            # Not live means publishing zero, not publishing nothing: a consumer
            # holding the last message would otherwise keep driving.
            if live:
                t.linear.x = axes[1] * args.max_linear
                t.angular.z = -axes[0] * args.max_angular
            pub.publish(t)

        rx.on_command = publish
        node.get_logger().info(
            f"listening on {args.bind}:{args.port}, deadman {args.timeout}s")

        try:
            while rclpy.ok():
                rx.poll()
                rclpy.spin_once(node, timeout_sec=0.0)
        except KeyboardInterrupt:
            pass
        finally:
            rx.on_command(False, [0.0] * 4, 0)   # one last zero
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return

    print(f"listening on {args.bind}:{args.port}, deadman {args.timeout}s")
    last_print = 0.0
    try:
        while True:
            rx.poll()
            now = time.monotonic()
            if now - last_print > 0.25:
                last_print = now
                axes = " ".join(f"{a:+.2f}" for a in rx.axes)
                print(f"\r{'LIVE ' if rx.live else 'zero '} [{axes}] "
                      f"btn={rx.buttons:04x} pkt={rx.stats['packets']} "
                      f"deadman={rx.stats['deadman']} gaps={rx.stats['gaps']} "
                      f"resync={rx.stats['resyncs']}  ",
                      end="", flush=True)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
