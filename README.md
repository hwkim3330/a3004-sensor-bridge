# A3004NS-M sensor bridge

An ipTIME A3004NS-M running OpenWrt as a self-contained sensor node: a USB
camera and its microphone, an Ouster lidar on a gigabit port, optionally a
USB-CAN adapter and a FlySky receiver, all visible on a dashboard that an
Android tablet reaches over the router's own WiFi.

The vehicle it is aimed at is an **AgileX SCOUT MINI Omni**: mecanum wheels, so
holonomic, and the teleop path carries three axes rather than two.

## The four repositories

This started as one branch of an OpenWrt fork and did not stay legible that way.
It is now split along the lines things actually break along:

| | |
|---|---|
| **this repository** | the router software, as an OpenWrt feed |
| [`agilex-scout-mini`](https://github.com/hwkim3330/agilex-scout-mini) | everything that talks to the vehicle — CAN, RS232, `can-bridge`, `agx-cmd`, and the diagnostics. Separate because the vehicle outlives the router: if this board is replaced, that repository is unaffected |
| [`a3004-bridge-app`](https://github.com/hwkim3330/a3004-bridge-app) | the native Android client |
| [openwrt#24707](https://github.com/openwrt/openwrt/pull/24707) | the board port itself, in review upstream |

The port this sits on top of was originally
[openwrt#4915](https://github.com/openwrt/openwrt/pull/4915), written in 2022 and
closed unmerged in 2023 over one driver defect. That defect is fixed — see
[`doc/DBDC.md`](doc/DBDC.md).

## Using it

Add it as a feed to an OpenWrt tree:

```sh
echo "src-git keti https://github.com/hwkim3330/a3004-sensor-bridge.git" >> feeds.conf.default
./scripts/feeds update keti && ./scripts/feeds install -a -p keti
```

`navigate` depends on `agx-cmd`, which lives in the vehicle repository, so add
that feed too if you want the autonomous path:

```sh
echo "src-git agilex https://github.com/hwkim3330/agilex-scout-mini.git" >> feeds.conf.default
./scripts/feeds update agilex && ./scripts/feeds install -a -p agilex
```

Then `make menuconfig` and select `a3004-sensorkit`, which pulls in the rest.

## Read these in order

| | |
|---|---|
| [`doc/ARCHITECTURE.md`](doc/ARCHITECTURE.md) | what runs on the router and what does not, with the bandwidth, CPU and latency numbers behind each decision |
| [`doc/BRINGUP.md`](doc/BRINGUP.md) | flashing and first boot, with the reasoning attached |
| [`doc/DBDC.md`](doc/DBDC.md) | why the upstream port was never merged, and the fix |
| [`doc/RC-AND-WIFI.md`](doc/RC-AND-WIFI.md) | why no WiFi chip can receive FlySky AFHDS 2A, what to do instead, and how many radios and SSIDs the DBDC fix buys |
| [`doc/AFHDS2A.md`](doc/AFHDS2A.md) | the protocol analysed, and how the router *can* be the transmitter — with an A7105, not with its WiFi |
| [`doc/TELEOP.md`](doc/TELEOP.md) | why a tablet cannot emulate a 2.4 GHz transmitter, and how it drives things over IP instead — with two independent deadmen |
| [`doc/RING-FORMAT.md`](doc/RING-FORMAT.md) | the lidar range-ring wire format and JSON status |
| [`doc/COMPUTE.md`](doc/COMPUTE.md) | what this hardware can and cannot be asked to do |
| [`doc/A5004NS-M.md`](doc/A5004NS-M.md) | the sibling board, surveyed but never flashed |
| [`doc/UPSTREAM.md`](doc/UPSTREAM.md) | the pull requests this work becomes, and what to check before opening one |
| [`doc/VERIFY.md`](doc/VERIFY.md) | what the first flash session actually produced |

The vehicle's own protocol notes — the CAN IDs, the RS232 framing, and the bench
runbook — are in
[`agilex-scout-mini/doc`](https://github.com/hwkim3330/agilex-scout-mini/tree/main/doc).

## Packages

| package | what it does |
|---|---|
| `a3004-sensorkit` | pulls the rest in, configures them, and installs the dashboard |
| `ouster-edge` | receives the lidar UDP stream, relays it verbatim, reduces each revolution to a range ring, evaluates polar zones per column, and reports IMU attitude |
| `slam2d` | 2D scan matching and an occupancy map from the ring |
| `navigate` | goal seeking and frontier exploration over that map, with the vehicle's real footprint |
| `mic-stream` | serves a USB microphone as uncompressed PCM over HTTP |
| `rc-ibus` | decodes a FlySky receiver's i-BUS channel output |
| `teleop` | takes joystick intent from the dashboard and forwards it with a deadman |
| `rc-tx` | AFHDS 2A frame building — the half that needs no radio (not an installable package) |

`doc/pc-side/ring_to_laserscan.py` republishes the ring as
`sensor_msgs/LaserScan` on a machine with ROS 2.
`doc/pc-side/teleop_receiver.py` is the reference control receiver, and the
place to look for how the second deadman is meant to work.

`tools/` is the host side: `webconsole` drives the vehicle from a browser on a
PC, `mapping` borrows the router's lidar relay for a run and puts it back,
`camdiag` measures glass-to-glass latency, `pilot` is the joystick client, and
`flash-router` does the flash.

A native tablet client lives at
<https://github.com/hwkim3330/a3004-bridge-app>. The web dashboard does the same
job in a browser; the app exists because control over UDP, the ring as a binary
datagram, and AudioTrack instead of a browser jitter buffer are all measurably
better, and because a tab cannot promise to disarm when it loses focus.

## Tests

There is no lidar and no RC receiver on the bench, so everything that can be
verified against synthesised input is. None of it needs the target hardware:

```sh
cd ouster-edge/test
cc -O2 -Wall -Wextra -o ouster-edge ../src/ouster-edge.c -lm
python3 test_profiles.py       # all four Ouster UDP profiles, byte-exact
python3 test_accounting.py     # packet accounting and the missed_columns counter
python3 test_latency.py        # zone and ring latency
python3 test_zones.py          # zone confirmation and hysteresis
sh    test_metadata.sh         # the sensor HTTP metadata probe

cd ../../rc-ibus/test
cc -O2 -Wall -Wextra -o rc-ibus ../src/rc-ibus.c
python3 test_ibus.py           # i-BUS over a pty

cd ../../teleop/test
cc -O2 -Wall -Wextra -o teleop ../src/teleop.c
python3 test_teleop.py         # arming, deadman, replay rejection, shutdown

cd ../../rc-tx/test
cc -O2 -Wall -Wextra -o test_afhds2a test_afhds2a.c ../src/afhds2a.c
./test_afhds2a                 # AFHDS 2A hop sets and frame layouts
```

`slam2d/test` and `navigate/test` exercise the map and the planner against
synthetic scans; they are run by hand rather than in CI.

The suites above run on every push in `.github/workflows/tests.yml` with
`-Werror`. They are what stands in for a lidar and an RC receiver that are not
on the bench, so a regression in them is one nobody would otherwise notice until
the hardware arrived. The vehicle-side suites moved with their code and run in
the [vehicle repository](https://github.com/hwkim3330/agilex-scout-mini).

If you run them by hand repeatedly, note that a daemon left behind by an aborted
run holds the port and quietly absorbs the traffic, which looks exactly like a
parser regression. `pgrep -x ouster-edge` before blaming the code.

## Emulator

`emu/run-emu.py` boots the whole thing under QEMU on `malta/le`, which is the
same `mipsel_24kc` triple as `ramips/mt7621` — the same package binaries, real
procd, real uci, real uhttpd. It checks that uci-defaults applied, that the
services are up once procd settles, that the dashboard serves, that synthetic
lidar packets injected from the host complete revolutions, and that an SSE event
is pushed while they flow.

It cannot say anything about the device tree, mt76 or DBDC — QEMU has no
MT7615D, and whether two phys appear is still a question only the board answers.
What it does is take the userspace failures out of the bench session. See
[`emu/README.md`](emu/README.md), including the two real defects it found.

## What is verified, and what needs the board

Measured or exercised here:

- the port builds against current OpenWrt master; both images are produced and
  the sysupgrade image uses 68% of the flash partition
- `mediatek,dbdc` is present in the built DTB and in `mt7615-common.ko`
- the Ouster parser against all four documented UDP profiles
- `missed_columns` counts a deliberately dropped packet exactly
- zone latency 1.4 ms, ring delivery 0.2–0.7 ms
- zones need N columns to agree and M quiet revolutions to release, so a single
  stray return does not fire and an object on the boundary does not chatter
- the lidar's IMU, checked against physics rather than against itself: gravity
  came out at 1.039 g with the sensor level and the gyro at zero
- a Logitech StreamCam VU0054: MJPEG to 1920×1080, USB 3.0 SuperSpeed, and its
  bitrates at each mode — 30 fps is the stable ceiling through the router, and
  60 fps only looks fine until you measure for longer than ten seconds
- its microphone: exact byte rate, valid WAV, real signal
- `rc-ibus` against synthesised i-BUS frames over a pty
- `teleop`: 30 safety checks, plus end to end from the tablet's joystick through
  the daemon to the reference receiver, with both deadmen firing
- `ring_to_laserscan.py` under ROS 2 jazzy
- the dashboard on a Galaxy Tab S7 FE with camera, microphone, lidar, CAN and
  RC all live at once
- a full first boot under QEMU on the same architecture: uci-defaults applied,
  services up, dashboard serving, synthetic lidar packets completing revolutions
  and pushing SSE events. Five consecutive boots, deterministic.

Still needs the hardware:

- whether DBDC actually comes up as two phys
- a real OS-64 at range: throughput, and whether MT7621 accepts a 9000-byte MTU
- **which way the lidar faces.** The code assumes sector 0 is forward and that
  has never been checked against the physical connector
- a real FlySky receiver emitting the frame layout the decoder assumes
- anything involving an A7105: there is no such chip on the bench, so the whole
  radio half of `doc/AFHDS2A.md` is analysis plus untested code
- anything actually being driven by teleop; the deadmen bound how long a runaway
  lasts, not whether one can happen
- the antenna split the EEPROM reports (2×2+2×2 expected, unread)

One thing that is known and unresolved: the OS1-64 has no fan, draws 16 W, and
reaches `SHOT_LIMITING` on a desk. It needs metal-to-metal mounting, not a
bracket.

## Licence

GPL-2.0-or-later, matching the SPDX headers on the C sources.
