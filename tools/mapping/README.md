# Indoor 3D mapping from the relayed lidar

The router relays the sensor's own UDP datagrams untouched, so a machine on the
LAN can rebuild the full 3D scan without the sensor knowing it exists. These two
programs turn that into a map.

```sh
pip3 install --user --break-system-packages open3d   # kiss-icp and ouster-sdk are
                                                     # already on this machine

./live_view.py --accumulate                   # watch it now, in 3D
./capture.py run1 60                          # record a minute, moving the sensor
./build_map.py run1                           # map.ply, mesh.ply, trajectory.txt
```

Not a venv any more. There was one under a temporary directory, which is how the
PC console came to be running out of `/tmp` — it worked until the directory went
away. `--user` keeps it in the home directory and out of the system packages this
machine shares with unrelated work.

One caution earned the hard way: installing more than `open3d` here drags `click`
and `rich` around, and this machine has packages that pin them in both directions
— `huggingface-hub` wants `click>=8.4.2`, `gtts` and `ouster-sdk` want `<8.2`.
Those two cannot both be satisfied and the conflict is not new; leave the versions
where they are and install nothing else.

## The relay switches itself on

`capture.py`, `live_view.py` and `ros2_bridge.py` all need the router's raw-lidar
relay pointed here, and it is off by default for a good reason: it was found
sending **64.2 Mbit/s to a port with nothing bound to it**, which cost about 1.4
of the board's 4 cores for packets nobody read.

So the three of them borrow it. Each turns the relay on at startup, pointed at
whichever of this machine's addresses actually routes to the router, and puts it
back exactly as it was on the way out — including after Ctrl-C or an exception.
Both moves are printed, and each costs about nine seconds because `--relay` is a
start argument for `ouster-edge`. `--relay keep` opts out.

`relay.py status|on|off` does the same by hand. That is the thing to reach for if
a reader is killed outright rather than interrupted, which is the one case the
automatic restore cannot cover; it will say if the relay was left pointing
somewhere.

`live_view.py` opens a window and draws the current revolution, coloured by
reflectivity. `--accumulate` merges revolutions into a voxel grid rather than
appending them: appending reached a million points in ten seconds and dropped the
viewer to 3.5 Hz while the sensor was still sending at 10, and a stationary
sensor is re-measuring surfaces it already has. Merged, it holds about 40k points
and 10.4 Hz - the sensor's own rate - and still fills in gaps.

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

## Into ROS 2, and what Autoware can and cannot use

```sh
source /opt/ros/jazzy/setup.bash
./ros2_bridge.py                  # /ouster/points, 10 Hz, ~24k points
ros2 run rviz2 rviz2              # fixed frame: os_sensor
```

Measured: 9.999 Hz with 2.3 ms of jitter, 23842 points per message, fields
`x y z intensity` as float32 - the layout RViz and Autoware's perception both
take with no converter.

The bridge listens to the relayed copy instead of using the official
`ouster-ros` driver, because that driver wants to configure and own the sensor
and the sensor belongs to the router. That is the point of the relay: a sensor
has one destination and there are two consumers.

**On Autoware.** The perception half fits and the planning half does not. Its
planners are built around lanelet2 maps and lanes, and a corridor has neither -
running the whole stack indoors means fighting it. What does fit:

- GPU detection (CenterPoint) on `/ouster/points`
- RViz, for looking at the real sensor rather than a simulated one
- NDT localisation, which loads a point cloud map - and `build_map.py` now writes
  `map.pcd` next to `map.ply` for exactly that, so an indoor map made here is
  usable there without a conversion step

What is missing for full autonomy indoors is a lanelet2 map of a building, which
is a different project from this one.
