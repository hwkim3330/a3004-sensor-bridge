#!/usr/bin/env python3
"""Publish the router's relayed lidar as ROS 2 PointCloud2.

This is the one piece needed to reach everything already working on this machine:
RViz, GPU CenterPoint detection, and Autoware's localisation. Nothing else has to
change, because the router relays the sensor's own datagrams untouched - what
arrives here is what the sensor sent.

    source /opt/ros/jazzy/setup.bash
    ./ros2_bridge.py                      # publishes /ouster/points at 10 Hz
    ros2 run rviz2 rviz2                  # fixed frame: os_sensor

Why not the official ouster-ros driver: it wants to configure and own the sensor,
and the sensor here belongs to the router - which is the whole point of the
relay, since a sensor has one destination and two consumers. This listens to the
relayed copy and leaves the sensor alone.

A note on scope. Full Autoware indoors is a poor fit: its planning stack is built
around lanelet2 maps and lanes, and a corridor has neither. What does fit is the
perception and localisation half - and the map this project's build_map.py
produces can be written as the .pcd that Autoware's NDT localisation consumes,
so an indoor map made here is usable there.
"""
import argparse
import socket
import struct
import sys
import time
import urllib.request

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def metadata(host):
    with urllib.request.urlopen(f"http://{host}/api/v1/sensor/metadata",
                               timeout=6) as r:
        return r.read().decode()


class Bridge(Node):
    def __init__(self, args):
        super().__init__("ouster_relay_bridge")
        from ouster.sdk.core import (ScanBatcher, LidarScan, XYZLut, SensorInfo,
                                     LidarPacket, ChanField)
        self._ChanField = ChanField
        self._LidarPacket = LidarPacket
        self._LidarScan = LidarScan

        info = SensorInfo(metadata(args.sensor))
        self.info = info
        self.w = info.format.columns_per_frame
        self.h = info.format.pixels_per_column
        self.batch = ScanBatcher(info)
        self.xyz = XYZLut(info)
        self.scan = LidarScan(self.h, self.w, info.format.udp_profile_lidar,
                              info.format.columns_per_packet)
        self.frame_id = args.frame

        # Best-effort with a depth of 1: a point cloud that is one revolution old
        # is worthless, so dropping it is better than queueing it. This is what
        # the Ouster driver and Autoware's own sensor topics use.
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(PointCloud2, args.topic, qos)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 << 20)
        self.sock.bind((args.bind, args.port))
        self.sock.setblocking(False)

        self.n = 0
        self.t0 = time.time()
        self.last = self.t0
        # Polled rather than threaded: rclpy's executor owns this thread, and a
        # timer at 1 ms drains whatever arrived without a second thread needing
        # a lock around the batcher.
        self.create_timer(0.001, self.drain)
        self.get_logger().info(
            f"{info.format.udp_profile_lidar}, {self.h} beams x {self.w} cols, "
            f"publishing {args.topic} as {args.frame}")

    def drain(self):
        for _ in range(400):
            try:
                d = self.sock.recv(65535)
            except BlockingIOError:
                return
            except OSError:
                return
            pkt = self._LidarPacket(len(d))
            pkt.buf[:] = np.frombuffer(d, dtype=np.uint8)
            if not self.batch(pkt, self.scan):
                continue
            self.publish(self.scan)
            self.scan = self._LidarScan(
                self.h, self.w, self.info.format.udp_profile_lidar,
                self.info.format.columns_per_packet)

    def publish(self, scan):
        pts = self.xyz(scan)
        rng = scan.field(self._ChanField.RANGE)
        mask = rng > 0
        xyz = pts[mask].astype(np.float32)
        try:
            refl = scan.field(self._ChanField.REFLECTIVITY)[mask]
        except Exception:
            refl = np.zeros(xyz.shape[0], dtype=np.uint16)
        inten = refl.astype(np.float32)

        # x, y, z, intensity as four float32 - the layout Autoware's perception
        # and RViz both accept without a converter in between.
        cloud = np.empty((xyz.shape[0], 4), dtype=np.float32)
        cloud[:, :3] = xyz
        cloud[:, 3] = inten

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = cloud.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * cloud.shape[0]
        msg.is_dense = True
        msg.data = cloud.tobytes()
        self.pub.publish(msg)

        self.n += 1
        now = time.time()
        if now - self.last >= 5:
            self.last = now
            self.get_logger().info(
                f"{self.n} clouds, {self.n/(now-self.t0):.1f} Hz, "
                f"{cloud.shape[0]} points last")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor", default="192.168.1.50")
    ap.add_argument("--port", type=int, default=7502)
    ap.add_argument("--bind", default="")
    ap.add_argument("--topic", default="/ouster/points")
    ap.add_argument("--frame", default="os_sensor")
    a, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = Bridge(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
