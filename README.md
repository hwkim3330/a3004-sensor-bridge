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
| [`doc/BRINGUP.md`](doc/BRINGUP.md) | build, flash, and bring each sensor up in order |
| [`doc/DBDC.md`](doc/DBDC.md) | why the upstream port was never merged, and the fix |
| [`doc/CAN.md`](doc/CAN.md) | USB-CAN wiring, bus termination, and why injection is off by default |
| [`doc/RC-AND-WIFI.md`](doc/RC-AND-WIFI.md) | why no WiFi chip can receive FlySky AFHDS 2A, what to do instead, and how many radios and SSIDs the DBDC fix buys |
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
```

If you run these repeatedly, note that a daemon left behind by an aborted run
holds the port and quietly absorbs the traffic, which looks exactly like a
parser regression. `pgrep -x ouster-edge` before blaming the code.

## What is verified, and what needs the board

Measured or exercised here:

- the port builds against current OpenWrt master; both images are produced and
  the sysupgrade image uses 68% of the flash partition
- `mediatek,dbdc` is present in the built DTB and in `mt7615-common.ko`
- the Ouster parser against all four documented UDP profiles
- `missed_columns` counts a deliberately dropped packet exactly
- zone latency 1.4 ms, ring delivery 0.2–0.7 ms
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

Still needs the hardware:

- flashing, and whether DBDC actually comes up as two phys
- a real OS-64: throughput, and whether MT7621 accepts a 9000-byte MTU
- a real USB-CAN adapter enumerating and `gs_usb` binding on mipsel
- a real FlySky receiver emitting the frame layout the decoder assumes
- anything actually being driven by teleop; the deadmen bound how long a runaway
  lasts, not whether one can happen
- the antenna split the EEPROM reports (2×2+2×2 expected, unread)
