#!/usr/bin/env python3
"""Watch the lidar in 3D, live, from the stream the router relays.

Opens a window and keeps one revolution on screen at a time, coloured by
reflectivity so surfaces read as surfaces rather than as a fog of range values.
Nothing is accumulated - this is for pointing the sensor and seeing what it sees.
For a map that grows, record with capture.py and run build_map.py.

    live_view.py [--port 7502] [--sensor 192.168.1.50] [--colour range|refl]
                 [--accumulate] [--snapshot out.png]

--accumulate keeps every revolution, which without registration only looks right
while the sensor is still. It is here because a stationary sensor filling in its
own gaps over a few seconds is genuinely useful, and because it makes the
difference between "needs odometry" and "does not" obvious the moment you move.
"""
import argparse
import json
import socket
import struct
import sys
import time
import urllib.request

import numpy as np


def metadata(host):
    with urllib.request.urlopen(f"http://{host}/api/v1/sensor/metadata",
                               timeout=6) as r:
        return r.read().decode()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=7502)
    ap.add_argument("--sensor", default="192.168.1.50")
    ap.add_argument("--bind", default="")
    ap.add_argument("--colour", choices=("range", "refl"), default="refl")
    ap.add_argument("--accumulate", action="store_true")
    ap.add_argument("--accumulate-voxel", type=float, default=0.03,
                    help="merge accumulated points at this size, in metres")
    ap.add_argument("--snapshot", default="")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="exit after this long; 0 means run until closed")
    a = ap.parse_args()

    import open3d as o3d
    from ouster.sdk.core import (ScanBatcher, LidarScan, XYZLut, SensorInfo,
                                 LidarPacket, ChanField)

    meta_text = metadata(a.sensor)
    info = SensorInfo(meta_text)
    w = info.format.columns_per_frame
    h = info.format.pixels_per_column
    print(f"  {a.sensor}: {info.format.udp_profile_lidar}, {h} beams x {w} cols")

    batch = ScanBatcher(info)
    xyz = XYZLut(info)
    scan = LidarScan(h, w, info.format.udp_profile_lidar,
                     info.format.columns_per_packet)

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
    s.bind((a.bind, a.port))
    s.settimeout(0.5)

    vis = o3d.visualization.Visualizer()
    vis.create_window("OS-1-64 live", 1280, 800)
    pc = o3d.geometry.PointCloud()
    added = False
    # A frame at the origin, so the sensor's own position is visible and the
    # scale of the room is readable without measuring anything.
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))
    opt = vis.get_render_option()
    opt.background_color = np.array([0.05, 0.05, 0.07])
    # 1.5 px left a room looking like scattered dust at any sensible zoom. The
    # sensor's returns are sparse vertically - 64 beams over 35 degrees - so the
    # points have to be big enough to read as the surface they came from.
    opt.point_size = 3.0

    acc_pts = []
    acc_col = []
    frames = 0
    t0 = time.time()
    last = t0
    try:
        while True:
            try:
                d = s.recv(65535)
            except socket.timeout:
                if not vis.poll_events():
                    break
                vis.update_renderer()
                continue
            pkt = LidarPacket(len(d))
            pkt.buf[:] = np.frombuffer(d, dtype=np.uint8)
            if not batch(pkt, scan):
                continue

            pts = xyz(scan)
            rng = scan.field(ChanField.RANGE)
            mask = rng > 0
            p = pts[mask]
            if a.colour == "refl":
                try:
                    v = scan.field(ChanField.REFLECTIVITY)[mask].astype(np.float32)
                    # Reflectivity is long-tailed: a retroreflector saturates the
                    # scale and everything else goes black. Clip at a high
                    # percentile so the room keeps its contrast.
                    hi = np.percentile(v, 99) if v.size else 1.0
                    t = np.clip(v / max(hi, 1.0), 0, 1)
                except Exception:
                    t = np.clip(np.linalg.norm(p, axis=1) / 20.0, 0, 1)
            else:
                t = np.clip(np.linalg.norm(p, axis=1) / 20.0, 0, 1)
            # Blue-to-warm ramp, dark for low values, so structure reads at a
            # glance without a legend.
            col = np.stack([t, 0.35 + 0.5 * t, 1.0 - 0.7 * t], axis=1)

            if a.accumulate:
                # Bounded, not unbounded.
                #
                # Appending every revolution reached a million points in ten
                # seconds and the frame rate fell from 7.6 Hz to 3.5 Hz - the
                # viewer became the slowest thing in the chain while the sensor
                # was still at 10 Hz. Merging into a voxel grid keeps the picture
                # filling in while the point count stops growing, because a
                # stationary sensor is re-measuring surfaces it already has.
                acc_pts.append(p)
                acc_col.append(col)
                if len(acc_pts) > 1:
                    merged = o3d.geometry.PointCloud()
                    merged.points = o3d.utility.Vector3dVector(
                        np.concatenate(acc_pts))
                    merged.colors = o3d.utility.Vector3dVector(
                        np.concatenate(acc_col))
                    merged = merged.voxel_down_sample(a.accumulate_voxel)
                    acc_pts = [np.asarray(merged.points)]
                    acc_col = [np.asarray(merged.colors)]
                p = acc_pts[0]
                col = acc_col[0]

            pc.points = o3d.utility.Vector3dVector(p)
            pc.colors = o3d.utility.Vector3dVector(col)
            if not added:
                vis.add_geometry(pc)
                # Fit the camera once there is something to fit it to. Open3D's
                # default view is set when the window opens, which is before the
                # first revolution arrives - so without this the room sits in a
                # corner of the frame at whatever scale the empty scene implied.
                vis.reset_view_point(True)
                vc = vis.get_view_control()
                vc.set_front([0.0, -0.6, 0.8])
                vc.set_up([0.0, 0.0, 1.0])
                vc.set_zoom(0.45)
                added = True
            else:
                vis.update_geometry(pc)
            if not vis.poll_events():
                break
            vis.update_renderer()

            scan = LidarScan(h, w, info.format.udp_profile_lidar,
                             info.format.columns_per_packet)
            frames += 1
            now = time.time()
            if now - last >= 2:
                last = now
                print(f"  {frames:5d} revolutions  {len(p):8d} points on screen "
                      f"  {frames/(now-t0):.1f} Hz", flush=True)
            if a.seconds and now - t0 >= a.seconds:
                break
    finally:
        if a.snapshot:
            vis.capture_screen_image(a.snapshot, do_render=True)
            print(f"  wrote {a.snapshot}")
        vis.destroy_window()
    print(f"  {frames} revolutions shown")
    return 0


if __name__ == "__main__":
    sys.exit(main())
