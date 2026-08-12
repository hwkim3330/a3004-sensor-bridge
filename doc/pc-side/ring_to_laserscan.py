#!/usr/bin/env python3
"""Republish an ouster-edge range ring as sensor_msgs/LaserScan.

The ring is already a LaserScan in all but name: one minimum range per fixed
azimuth increment, one message per revolution. So this is a plain translation,
not a reconstruction - nothing is inferred or interpolated.

Run it on the machine the router relays to:

    ros2 run ... # or simply, with ROS 2 sourced:
    python3 ring_to_laserscan.py --ros-args \
        -p port:=7602 -p frame_id:=os_sensor -p range_max:=100.0

Topics published:
    ~/scan        sensor_msgs/LaserScan
    ~/zone_alarm  std_msgs/Bool   (latched state, published every revolution)

If you also want real point clouds, that is what the relayed raw stream is for:
point ouster-ros at the router's relay output. This node is the cheap channel,
not a replacement for the driver.
"""

import socket
import struct
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

MAGIC = b"OSED"
HEADER = 20
NO_RETURN = 0xFFFF

PROFILE_NAMES = {
    1: "LEGACY",
    2: "RNG19_RFL8_SIG16_NIR16",
    3: "RNG15_RFL8_NIR8",
    4: "RNG19_RFL8_SIG16_NIR16_DUAL",
}


class RingToLaserScan(Node):
    def __init__(self):
        super().__init__("ring_to_laserscan")

        self.declare_parameter("port", 7602)
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("frame_id", "os_sensor")
        self.declare_parameter("range_min", 0.3)
        self.declare_parameter("range_max", 100.0)
        # Rotation rate of the sensor, used only to fill scan_time. Reading it
        # from the sensor would be better, but the ring carries a timestamp we
        # can difference instead - see below.
        self.declare_parameter("rotation_hz", 10.0)
        # Clockwise-positive azimuth (the sensor's convention) is the opposite
        # of REP-103's counter-clockwise-positive yaw. Flip by default so the
        # scan lines up with a right-handed frame in RViz.
        self.declare_parameter("invert_azimuth", True)

        self.frame_id = self.get_parameter("frame_id").value
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.rotation_hz = float(self.get_parameter("rotation_hz").value)
        self.invert = bool(self.get_parameter("invert_azimuth").value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_scan = self.create_publisher(LaserScan, "~/scan", qos)
        self.pub_alarm = self.create_publisher(Bool, "~/zone_alarm", 10)

        port = int(self.get_parameter("port").value)
        addr = self.get_parameter("bind_address").value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((addr, port))
        self.sock.settimeout(0.5)

        self.last_ts = None
        self.last_scan_time = 1.0 / self.rotation_hz
        self.warned_version = False
        self.count = 0

        self.get_logger().info(f"listening for rings on {addr}:{port}")
        self.create_timer(0.001, self.pump)

    def pump(self):
        """Drain whatever has arrived. Called often; never blocks for long."""
        for _ in range(64):
            try:
                data, _src = self.sock.recvfrom(65535)
            except socket.timeout:
                return
            except BlockingIOError:
                return
            self.on_ring(data)

    def on_ring(self, data: bytes):
        if len(data) < HEADER or data[:4] != MAGIC:
            return

        version, profile = data[4], data[5]
        sectors, frame_id = struct.unpack_from("<HH", data, 6)
        alarm = data[10] != 0
        (ts_ns,) = struct.unpack_from("<Q", data, 12)

        if version != 1:
            if not self.warned_version:
                self.get_logger().warning(
                    f"ring format version {version}, this node speaks 1"
                )
                self.warned_version = True
            return

        expected = HEADER + 3 * sectors
        if len(data) != expected:
            self.get_logger().warning(
                f"ring says {sectors} sectors ({expected} B) but got {len(data)} B"
            )
            return

        rng = struct.unpack_from(f"<{sectors}H", data, HEADER)
        refl = struct.unpack_from(f"<{sectors}B", data, HEADER + 2 * sectors)

        # Derive the real revolution period from consecutive timestamps rather
        # than trusting the rotation_hz parameter, so a sensor configured for
        # 20 Hz reports honest timing without anyone editing a launch file.
        if self.last_ts is not None:
            dt = (ts_ns - self.last_ts) / 1e9
            if 0.01 < dt < 1.0:
                self.last_scan_time = dt
        self.last_ts = ts_ns

        ranges = [
            float("inf") if r == NO_RETURN else r / 100.0
            for r in rng
        ]
        intensities = [float(v) for v in refl]

        if self.invert:
            # Reverse so azimuth increases counter-clockwise, keeping sector 0
            # at angle 0 rather than shifting the whole scan by one increment.
            ranges = [ranges[0]] + ranges[:0:-1]
            intensities = [intensities[0]] + intensities[:0:-1]

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_increment = 2.0 * math.pi / sectors
        msg.angle_max = msg.angle_increment * (sectors - 1)
        msg.scan_time = self.last_scan_time
        msg.time_increment = self.last_scan_time / sectors
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        msg.intensities = intensities
        self.pub_scan.publish(msg)

        self.pub_alarm.publish(Bool(data=alarm))

        self.count += 1
        if self.count == 1:
            self.get_logger().info(
                f"first ring: {sectors} sectors, source profile "
                f"{PROFILE_NAMES.get(profile, profile)}, frame {frame_id}"
            )


def main():
    rclpy.init(args=sys.argv)
    node = RingToLaserScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
