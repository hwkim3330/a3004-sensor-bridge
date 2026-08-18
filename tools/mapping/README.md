# Indoor 3D mapping from the relayed lidar

The router relays the sensor's own UDP datagrams untouched, so a machine on the
LAN can rebuild the full 3D scan without the sensor knowing it exists. These two
programs turn that into a map.

```sh
python3 -m venv --system-site-packages ~/mapenv
~/mapenv/bin/pip install kiss-icp open3d      # ouster-sdk comes from the system

python3 capture.py run1 60                    # record a minute, moving the sensor
~/mapenv/bin/python build_map.py run1         # map.ply, mesh.ply, trajectory.txt
```

## Why lidar-only

The current literature on indoor reconstruction is mostly lidar fused with a
camera and an IMU, rendered as Gaussian splats - LiDAR-GS-SLAM (ECCV 2026),
GS-SDF, Gaussian-LIC. Those produce better-looking results and need the
extrinsics between all three sensors. There is a camera on this router and
nothing calibrating it to the lidar, so that is a project rather than a flag.

KISS-ICP needs the point cloud and nothing else, which is why it is first. Its
documented weakness is a constant-velocity assumption, so it prefers being
carried smoothly over being jerked; comparisons put it behind lidar-inertial
methods like FAST-LIO2 in exactly the situations where that assumption breaks.
The OS-1 has an IMU on port 7503, so moving to a lidar-inertial method later does
not need new hardware - only the plumbing.

## What the numbers should look like

Measured on this bench, wired, RNG19 at 1024x10:

- capture: 640 packets/s, 7.7 MB/s, no loss
- unpacking and registration: 30 revolutions in 0.4 s on an i7-10700K, so a
  60-second recording maps in about ten seconds
- a stationary sensor reports a net displacement of 0.05 m over 30 revolutions,
  which is the honest way to check the registration is not inventing motion

A stationary recording collapses to about 11.7k unique 5 cm voxels no matter how
long it is - every revolution sees the same surfaces. **The map only grows if the
sensor moves**, which is worth saying because the first instinct is to record for
longer rather than to walk further.

## Over WiFi

Use the low-rate profile. `RNG19_RFL8_SIG16_NIR16` is a 12544-byte datagram,
nine IP fragments, and only 58-60 % of it survives WiFi in either band;
`RNG15_RFL8_NIR8` is 4352 bytes, three fragments, and 99 % arrives. See
doc/COMPUTE.md - the limit is the frame rate, not the bit rate.
