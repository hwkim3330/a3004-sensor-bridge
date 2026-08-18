#!/usr/bin/env python3
"""Turn a recording into a registered indoor point cloud, and then a mesh.

Three stages, each of which can be inspected on its own:

  1. unpack   the raw datagrams into per-revolution point clouds, using the
              sensor's own metadata for beam angles and range scaling
  2. register KISS-ICP, which is lidar-only. That matters here: it needs no IMU,
              no camera and no calibration between them, which is the whole
              reason it is the first thing to try. Its weakness is a constant
              velocity assumption, so it prefers smooth motion over jerks
  3. mesh     Poisson surface reconstruction over the accumulated cloud

Why lidar-only rather than one of the Gaussian-splatting systems: those fuse
lidar with a camera and an IMU and need the extrinsics between them. There is a
camera on this router but nothing calibrating it to the lidar, so that is a
separate job rather than a flag.

    build_map.py <capture.d> [--voxel 0.05] [--max-frames N] [--no-mesh]

Writes map.ply (points), mesh.ply, and trajectory.txt into the capture dir.
"""
import argparse
import json
import os
import struct
import sys
import time

import numpy as np


def frames_from_raw(path, meta_path, max_frames=None):
    """Yield (frame_id, Nx3 points in the sensor frame, reflectivity).

    Assembly is left to the SDK's ScanBatcher rather than done here. The first
    version of this walked packets and wrote columns by measurement id, which
    needs PacketFormat accessors that moved between SDK releases - and getting
    revolution boundaries right by hand is exactly the kind of detail the
    batcher already handles, including a dropped packet mid-revolution.
    """
    from ouster.sdk.core import (ScanBatcher, LidarScan, XYZLut, SensorInfo,
                                 LidarPacket, ChanField)

    info = SensorInfo(open(meta_path).read())
    w = info.format.columns_per_frame
    h = info.format.pixels_per_column
    batch = ScanBatcher(info)
    xyz = XYZLut(info)
    scan = LidarScan(h, w, info.format.udp_profile_lidar,
                     info.format.columns_per_packet)
    out = 0

    with open(path, "rb") as f:
        while True:
            hdr = f.read(4)
            if len(hdr) < 4:
                break
            n = struct.unpack("<I", hdr)[0]
            buf = f.read(n)
            if len(buf) < n:
                break
            pkt = LidarPacket(n)
            pkt.buf[:] = np.frombuffer(buf, dtype=np.uint8)
            # The batcher returns True on the packet that completes a scan.
            if batch(pkt, scan):
                pts = xyz(scan)
                rng = scan.field(ChanField.RANGE)
                mask = rng > 0
                try:
                    refl = scan.field(ChanField.REFLECTIVITY)[mask]
                except Exception:
                    refl = np.zeros(int(mask.sum()), dtype=np.uint16)
                yield int(scan.frame_id), pts[mask], refl
                out += 1
                if max_frames and out >= max_frames:
                    return
                scan = LidarScan(h, w, info.format.udp_profile_lidar,
                                 info.format.columns_per_packet)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--voxel", type=float, default=0.05,
                    help="downsample size in metres for the saved map")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--no-mesh", action="store_true")
    a = ap.parse_args()

    raw = os.path.join(a.capture, "lidar.raw")
    meta = os.path.join(a.capture, "metadata.json")
    for p in (raw, meta):
        if not os.path.exists(p):
            sys.exit(f"missing {p}")

    import open3d as o3d
    from kiss_icp.kiss_icp import KissICP
    from kiss_icp.config import KISSConfig

    cfg = KISSConfig()
    cfg.data.max_range = 40.0
    cfg.data.min_range = 0.5
    # voxel_size defaults to None because KISS-ICP's own CLI derives it from
    # max_range; constructed directly it reaches the C++ side as None and the
    # error names the constructor rather than the missing setting. Indoors a
    # small voxel is the point - 0.5 m would smear a doorway.
    cfg.mapping.voxel_size = 0.10
    # Deskewing needs per-point timestamps, which this unpacker does not carry
    # through yet. Off rather than wrong.
    cfg.data.deskew = False
    odom = KissICP(cfg)

    print("  unpacking and registering ...", flush=True)
    acc = o3d.geometry.PointCloud()
    traj = []
    t0 = time.time()
    nf = 0
    for fid, pts, refl in frames_from_raw(raw, meta,
                                          a.max_frames or None):
        pts = pts.astype(np.float64)
        # 1.3.0 exposes the current pose as `last_pose` and keeps no history;
        # an earlier draft read `poses[-1]`, which is the older API.
        odom.register_frame(pts, np.array([]))
        pose = np.asarray(odom.last_pose)
        traj.append((fid, pose))
        # The registered cloud in world coordinates, downsampled as it goes so
        # memory stays flat over a long run rather than growing per revolution.
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        pc.transform(pose)
        acc += pc.voxel_down_sample(a.voxel)
        if nf % 20 == 0:
            acc = acc.voxel_down_sample(a.voxel)
        nf += 1
        if nf % 20 == 0:
            t = pose[:3, 3]
            print(f"    frame {nf:4d}  pose ({t[0]:6.2f}, {t[1]:6.2f}, "
                  f"{t[2]:5.2f}) m  {len(acc.points):8d} pts", flush=True)

    acc = acc.voxel_down_sample(a.voxel)
    el = time.time() - t0
    print(f"  {nf} revolutions in {el:.1f}s, {len(acc.points)} points")
    if nf == 0:
        sys.exit("no complete revolutions in the recording")

    tpath = os.path.join(a.capture, "trajectory.txt")
    with open(tpath, "w") as f:
        for fid, p in traj:
            f.write(f"{fid} " + " ".join(f"{v:.6f}" for v in p[:3, :].ravel()) + "\n")
    d = np.linalg.norm(traj[-1][1][:3, 3] - traj[0][1][:3, 3])
    print(f"  trajectory: {tpath}  net displacement {d:.2f} m")

    mp = os.path.join(a.capture, "map.ply")
    o3d.io.write_point_cloud(mp, acc)
    print(f"  wrote {mp}")

    # Also as .pcd, which is what Autoware's NDT localisation loads as its
    # point cloud map. Writing both costs nothing and means an indoor map made
    # here is usable by the stack already on this machine, rather than needing a
    # conversion step nobody remembers.
    pc = os.path.join(a.capture, "map.pcd")
    o3d.io.write_point_cloud(pc, acc)
    print(f"  wrote {pc}  ({len(acc.points)} points)")

    if not a.no_mesh:
        print("  meshing ...", flush=True)
        acc.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3,
                                                              max_nn=30))
        acc.orient_normals_consistent_tangent_plane(20)
        mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            acc, depth=10)
        # Poisson closes surfaces everywhere, including across the open space a
        # lidar simply never saw. Trimming the least-supported vertices removes
        # the invented parts rather than presenting them as geometry.
        keep = dens > np.quantile(dens, 0.02)
        mesh.remove_vertices_by_mask(~keep)
        mpath = os.path.join(a.capture, "mesh.ply")
        o3d.io.write_triangle_mesh(mpath, mesh)
        print(f"  wrote {mpath}: {len(mesh.vertices)} vertices, "
              f"{len(mesh.triangles)} triangles")
    return 0


if __name__ == "__main__":
    sys.exit(main())
