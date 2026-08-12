# A3004NS-M sensor bridge

An ipTIME A3004NS-M running OpenWrt as a self-contained sensor node: a USB
camera and its microphone, an Ouster lidar on a gigabit port, optionally a
USB-CAN adapter and a FlySky receiver, all visible on a dashboard that an
Android tablet reaches over the router's own WiFi.

The port this sits on top of was [openwrt#4915](https://github.com/openwrt/openwrt/pull/4915),
written in 2022 and closed unmerged in 2023 over one driver defect. That defect
is fixed here — see `doc/DBDC.md`.

## Read these in order

| | |
|---|---|
| [`doc/ARCHITECTURE.md`](doc/ARCHITECTURE.md) | what runs on the router and what does not, with the bandwidth, CPU and latency numbers behind each decision |
| [`doc/TOMORROW.md`](doc/TOMORROW.md) | **the bench runbook** — one page, in order, with the decision at each step |
| [`doc/BRINGUP.md`](doc/BRINGUP.md) | the same ground with the reasoning attached |
| [`doc/DBDC.md`](doc/DBDC.md) | why the upstream port was never merged, and the fix |
| [`doc/CAN.md`](doc/CAN.md) | USB-CAN wiring, bus termination, and why injection is off by default |
| [`doc/RC-AND-WIFI.md`](doc/RC-AND-WIFI.md) | why no WiFi chip can receive FlySky AFHDS 2A, what to do instead, and how many radios and SSIDs the DBDC fix buys |
| [`doc/AFHDS2A.md`](doc/AFHDS2A.md) | the protocol analysed, and how the router *can* be the transmitter — with an A7105, not with its WiFi |
| [`doc/TELEOP.md`](doc/TELEOP.md) | why a tablet cannot emulate a 2.4 GHz transmitter, and how it drives things over IP instead — with two independent deadmen |
| [`doc/RING-FORMAT.md`](doc/RING-FORMAT.md) | the lidar range-ring wire format and JSON status |
| [`doc/UPSTREAM.md`](doc/UPSTREAM.md) | the two pull requests this work becomes, in which order, and what to check before opening either |

## Packages

| package | what it does |
|---|---|
| `a3004-sensorkit` | pulls the rest in, configures them, and installs the dashboard |
| `ouster-edge` | receives the lidar UDP stream, relays it verbatim, reduces each revolution to a range ring, evaluates polar zones per column |
| `mic-stream` | serves a USB microphone as uncompressed PCM over HTTP |
| `can-bridge` | bridges a SocketCAN interface to UDP, read-only unless told otherwise |
| `rc-ibus` | decodes a FlySky receiver's i-BUS channel output |
| `teleop` | takes joystick intent from the dashboard and forwards it with a deadman |
| `rc-tx` | AFHDS 2A frame building — the half that needs no radio (not an installable package) |

`doc/pc-side/ring_to_laserscan.py` republishes the ring as
`sensor_msgs/LaserScan` on a machine with ROS 2.
`doc/pc-side/teleop_receiver.py` is the reference control receiver, and the
place to look for how the second deadman is meant to work.

A native tablet client lives at
<https://github.com/hwkim3330/a3004-bridge-app>. The web dashboard does the same
job in a browser; the app exists because control over UDP, the ring as a binary
datagram, and AudioTrack instead of a browser jitter buffer are all measurably
better, and because a tab cannot promise to disarm when it loses focus.

## Tests

There is no lidar, no vehicle and no RC receiver on the bench, so everything
that can be verified against synthesised input is. None of it needs the target
hardware:

```sh
cd ouster-edge/test
cc -O2 -Wall -Wextra -o ouster-edge ../src/ouster-edge.c
python3 test_profiles.py      # all four Ouster UDP profiles, byte-exact
python3 test_accounting.py     # packet accounting and the missed_columns counter
python3 test_latency.py        # zone and ring latency
python3 test_zones.py          # zone confirmation and hysteresis
sh    test_metadata.sh         # the sensor HTTP metadata probe

cd ../../can-bridge/test
cc -O2 -Wall -Wextra -o can-bridge ../src/can-bridge.c
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
python3 test_bridge.py

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

All of the above also runs in CI on every push that touches `package/keti`, in
`.github/workflows/keti-tests.yml`, with `-Werror`. These suites are what stands
in for a lidar, a vehicle and an RC receiver that are not on the bench, so a
regression in them is one nobody would otherwise notice until the hardware
arrived.

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
- a Logitech StreamCam VU0054: MJPEG to 1920×1080, USB 3.0 SuperSpeed, and its
  bitrates at each mode
- its microphone: exact byte rate, valid WAV, real signal
- `can-bridge` against `vcan`, including refusal to inject
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

- flashing, and whether DBDC actually comes up as two phys
- a real OS-64: throughput, and whether MT7621 accepts a 9000-byte MTU
- a real USB-CAN adapter enumerating and `gs_usb` binding on mipsel
- a real FlySky receiver emitting the frame layout the decoder assumes
- anything involving an A7105: there is no such chip on the bench, so the whole
  radio half of doc/AFHDS2A.md is analysis plus untested code
- anything actually being driven by teleop; the deadmen bound how long a runaway
  lasts, not whether one can happen
- the antenna split the EEPROM reports (2×2+2×2 expected, unread)
