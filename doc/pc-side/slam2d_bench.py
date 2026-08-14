#!/usr/bin/env python3
"""What does 2D SLAM actually cost, against the 3D number already measured?

The 3D figure was 11-13 ms per scan on this desktop for ~50000 returns through
KISS-ICP. 2D is a different problem size entirely: ouster-edge's ring is 360
sectors at 10 Hz, so 3600 points per second against 1.3 million.

This times the two things a 2D SLAM front end actually does per scan - match the
new scan against the previous one, and fold it into an occupancy grid - using
plain numpy. It is not slam_toolbox, and the numbers should be read as an order
of magnitude for the work rather than as that package's performance. What makes
them useful is that they are measured on the same machine as the 3D figure, so
the ratio between them is meaningful even if the absolute values are not.
"""
import statistics as st
import time

import numpy as np
from scipy.spatial import cKDTree

N = 360                     # sectors in the ring ouster-edge emits
GRID = 0.05                 # 5 cm occupancy grid
EXTENT = 30.0               # 30 m square map


def room_scan(pose, n=N):
    """Range to the walls of a 12 x 8 m room with a couple of obstacles."""
    x0, y0, th0 = pose
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False) + th0
    dx, dy = np.cos(ang), np.sin(ang)
    t = np.full(n, np.inf)
    # walls as four axis-aligned segments
    for lo, hi, axis, val in ((-4, 4, 0, 6.0), (-4, 4, 0, -6.0),
                              (-6, 6, 1, 4.0), (-6, 6, 1, -4.0)):
        d = dx if axis == 0 else dy
        p = x0 if axis == 0 else y0
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = (val - p) / d
        other = (y0 + tt * dy) if axis == 0 else (x0 + tt * dx)
        ok = (tt > 0) & (other >= lo) & (other <= hi)
        t = np.where(ok & (tt < t), tt, t)
    # two pillars, so the scan is not four straight lines
    for cx, cy, r in ((2.0, 1.0, 0.3), (-3.0, -2.0, 0.4)):
        fx, fy = x0 - cx, y0 - cy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - r * r
        disc = b * b - 4 * c
        hit = disc >= 0
        tt = np.where(hit, (-b - np.sqrt(np.where(hit, disc, 0))) / 2, np.inf)
        t = np.where((tt > 0) & (tt < t), tt, t)
    t[~np.isfinite(t)] = 0.0
    return ang - th0, t


def to_xy(ang, rng, pose):
    x0, y0, th = pose
    good = rng > 0
    a = ang[good] + th
    return np.stack([x0 + rng[good] * np.cos(a),
                     y0 + rng[good] * np.sin(a)], axis=1)


def icp(src, dst, iters=12):
    """Point-to-point ICP with a KD-tree for the correspondence step.

    The first version of this built a full 360x360 distance matrix on every
    iteration and came out at 35 ms - slower than KISS-ICP on fifty thousand 3D
    points, which is not a fact about 2D SLAM but about that loop. Every real
    implementation uses a spatial index; so does this one now.
    """
    cur = src.copy()
    R = np.eye(2)
    t = np.zeros(2)
    tree = cKDTree(dst)
    for _ in range(iters):
        _, idx = tree.query(cur, k=1)
        p, q = cur, dst[idx]
        pc, qc = p.mean(0), q.mean(0)
        H = (p - pc).T @ (q - qc)
        U, _, Vt = np.linalg.svd(H)
        r = Vt.T @ U.T
        if np.linalg.det(r) < 0:
            Vt[-1] *= -1
            r = Vt.T @ U.T
        tt = qc - r @ pc
        cur = cur @ r.T + tt
        R, t = r @ R, r @ t + tt
    return R, t


def main():
    grid_n = int(EXTENT / GRID)
    grid = np.zeros((grid_n, grid_n), dtype=np.float32)

    poses = [(0.0 + 0.05 * i, 0.0 + 0.02 * i, 0.01 * i) for i in range(40)]
    scans = [room_scan(p) for p in poses]

    match_ms, grid_ms = [], []
    prev = to_xy(*scans[0], (0, 0, 0))
    for i in range(1, len(scans)):
        cur = to_xy(*scans[i], (0, 0, 0))

        t0 = time.perf_counter()
        icp(cur, prev)
        match_ms.append((time.perf_counter() - t0) * 1000)

        t0 = time.perf_counter()
        gx = np.clip(((cur[:, 0] + EXTENT / 2) / GRID).astype(int), 0, grid_n - 1)
        gy = np.clip(((cur[:, 1] + EXTENT / 2) / GRID).astype(int), 0, grid_n - 1)
        np.add.at(grid, (gy, gx), 1.0)
        grid_ms.append((time.perf_counter() - t0) * 1000)

        prev = cur

    pts = len(prev)
    print(f"  {N} sectors, {pts} returning points per scan, "
          f"{grid_n}x{grid_n} grid at {GRID*100:.0f} cm")
    print(f"  scan match (12 ICP iterations): median "
          f"{st.median(match_ms):.2f} ms")
    print(f"  occupancy grid update:          median "
          f"{st.median(grid_ms):.3f} ms")
    tot = st.median(match_ms) + st.median(grid_ms)
    print(f"  per scan total                  {tot:.2f} ms "
          f"= {tot / 100 * 100:.1f}% of the 100 ms budget at 10 Hz")
    print()
    print(f"  for comparison, measured earlier on this same machine:")
    print(f"    3D KISS-ICP, 50000 returns    11.4 ms  (median)")
    print(f"    ratio                         {11.4 / tot:.1f}x more expensive")


if __name__ == "__main__":
    main()
