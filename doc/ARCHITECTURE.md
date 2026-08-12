# A3004NS-M as a sensor bridge — what runs where, and why

This is the part of the project that is easy to get wrong by wishful thinking,
so the numbers are written down.

## The hardware you actually have

| | |
|---|---|
| SoC | MediaTek MT7621A, 2× MIPS 1004Kc @ 880 MHz (4 hardware threads) |
| Endianness | little (`ARCH:=mipsel`) |
| Floating point | **none** — OpenWrt builds this target `CONFIG_SOFT_FLOAT=y` |
| RAM | 256 MiB DDR3 |
| Flash | 16 MiB SPI NOR → 15.75 MiB firmware partition, ~9–10 MiB usable rootfs |
| Ethernet | 5× 1GbE on the SoC's built-in MT7530 switch |
| USB | 1× USB 3.0 |
| WiFi | MT7615D, DBDC: 2×2 on 2.4 GHz + 2×2 on 5 GHz (see `DBDC.md`) |

The soft-float line is the one that decides most of what follows. Every
floating point operation is trapped and emulated by the kernel, costing
hundreds of cycles instead of one.

## Bandwidth budget

An OS-64 in its default 1024×10 mode, single-return profile:

```
1024 columns / 16 columns-per-packet × 10 Hz = 640 packets/s
640 × 12544 B                                = 8.03 MB/s = 64.2 Mbit/s
```

Doubling either the rotation rate or the horizontal resolution doubles that to
**128 Mbit/s**. Both figures fit a gigabit port with room to spare, and both
fit a clean 5 GHz 2×2 VHT80 link (realistically 300–400 Mbit/s of TCP).

The camera question is settled: a Logitech StreamCam (VU0054, `046d:0893`) was
plugged in and probed. It **does** offer MJPEG, up to 1920×1080, and enumerated
at USB 3.0 SuperSpeed. Measured bitrates, `-c copy` off the real device:

| mode | MB/s | Mbit/s |
|---|---|---|
| 1280×720 @ 30 | 2.26 | **19.0** |
| 1280×720 @ 60 | 4.73 | **39.6** |
| 1920×1080 @ 30 | 4.70 | **39.4** |
| 1920×1080 @ 60 | 9.89 | **82.9** |

MJPEG is intra-frame, so these scale with scene detail; a busy scene will run
higher. The microphone adds 0.26 Mbit/s (16 kHz mono S16, uncompressed).

Camera plus lidar together stay inside a single 5 GHz link — 1080p60 plus an
OS-64 is about 147 Mbit/s against 300–400 Mbit/s of realistic 2×2 VHT80 TCP.
That is what makes the tablet-on-the-router-AP setup work.

100 Mbit is not an option: the sensor will not negotiate it, and 64 Mbit/s of
UDP on a 100 Mbit link has no headroom for retransmit-free delivery.

## CPU budget

| Work | Cost |
|---|---|
| Switching lidar packets between LAN ports | 0% — the MT7530 does it in hardware |
| `ouster-edge` range-ring reduction | ~655k integer min-compares/s ≈ **under 1% of one core** |
| Raw relay (one extra `sendto` per packet) | small; 8 MB/s of copying |
| MJPEG pass-through (`ustreamer`) | small; USB DMA in, socket out, no encode |
| Microphone pass-through (`mic-stream`) | negligible; 32 kB/s copied, no codec |
| CAN bridge | negligible; a busy 500 kbps bus is under 4000 frames/s |
| i-BUS RC decode | negligible; 32 bytes every 7.5 ms |
| **IP fragment reassembly at 1500 MTU** | **the real cost** — see below |

12544-byte datagrams do not fit a 1500-byte MTU, so each one arrives as nine
IP fragments: 640 packets/s becomes **5760 fragments/s** for the CPU to
reassemble. That, not the parsing, is what will show up in `top`. A 9000-byte
MTU cuts it to about 1280/s — run `sensor-lan-tune` and watch
`missed_columns` in the status file.

## What is deliberately not done on the router

**ROS 2.** The rootfs budget is around 9 MiB. A minimal ROS 2 installation is
two orders of magnitude larger, there is no OpenWrt package for it, and there
is no musl/MIPS build of `rclcpp` plus a DDS implementation. Booting a rootfs
from USB storage would trade away the single USB port that the camera needs.
This is not a tuning problem; it is the wrong machine for the job.

**Autonomous driving.** Perception and planning need orders of magnitude more
compute and memory than this SoC has. Nothing about the port changes that.

**Cartesian point clouds.** This one is worth stating precisely, because it is
not impossible — it is pointless. Deprojecting 655k points/s needs a sin/cos
per column and three multiplies per point; on a soft-float target that is
genuinely expensive, but it could be done in fixed point. The reason not to is
downstream: nothing on the router consumes a point cloud, and sending one costs
at least as much bandwidth as forwarding the raw stream that a real driver can
parse better. So the router forwards raw and computes only things that are
*smaller* than their input.

## What the router does do

1. **Aggregates.** Lidar on a gigabit port, camera on USB, one uplink out —
   either a LAN port or its own 5 GHz AP.
2. **Serves the camera** as MJPEG over HTTP, pass-through, no transcode.
3. **Reduces.** Each lidar revolution becomes a 360-sector minimum-range ring:
   1100 bytes, against 64 packets × 12544 B = 803 kB of raw revolution. A 730×
   reduction, and enough for obstacle presence, nearest-range and zone logic.
4. **Reacts.** Polar zones are evaluated on **every column as it arrives**, not
   once per revolution, so an intrusion fires within a packet — measured at
   1.4 ms rather than the up-to-100 ms a per-revolution check would cost at
   10 Hz. Clearing is the asymmetric half and does wait for a full clean
   revolution, because that is what it takes to know nothing is there. A local
   reflex on this budget is exactly the work that belongs on the node next to
   the sensor rather than a machine across a WiFi link.
5. **Relays.** The raw stream is forwarded verbatim to a real machine when one
   is present, so nothing is lost by putting the router in the path.
6. **Reads RC input**, if a FlySky receiver's i-BUS output is wired to a
   USB-serial adapter. Not the RF — see `RC-AND-WIFI.md` for why no WiFi chip
   can demodulate AFHDS 2A.
7. **Hosts a dashboard** on port 80 that renders the camera, the ring, the
   microphone, CAN telemetry and the RC channels, so a tablet on the router's
   own WiFi is a complete client with no other machine involved.

## Latency

Measured on a desktop (`../ouster-edge/test/test_latency.py`); an 880 MHz MIPS
part will be slower, but the structure is what matters:

| path | latency | why |
|---|---|---|
| zone intrusion → action script | **1.4–1.7 ms** | zones are evaluated per *column*, not per revolution. A per-revolution design cannot beat ~100 ms at 10 Hz |
| completed revolution → dashboard | **0.2–0.7 ms** | pushed as Server-Sent Events. Polling the status file costs up to one write interval plus one poll interval, so 200–450 ms |
| camera frame interval | 16.7 ms | 60 fps rather than 30, with `drop_same_frames` off and `tcp_nodelay` on |
| microphone | one ALSA period, 10 ms | no codec, so no encoder delay; the browser adds ~80 ms of jitter buffer |
| RC channel → status file | ≤100 ms | one i-BUS frame is 7.5 ms; the status interval dominates |

The lidar itself sets the floor for anything ring-shaped: a revolution at 10 Hz
is 100 ms. Running the sensor at 20 Hz halves that and doubles the bandwidth to
128 Mbit/s. The zone path deliberately does not wait for a revolution, which is
why it is two orders of magnitude faster than the ring.

## Topologies

All three work with the same firmware; only `/etc/config/ouster-edge` changes.

```
(a) self-contained — nothing else required
    lidar ──1GbE──┐
                  ├── A3004NS-M ──5GHz AP──> Android tablet (dashboard)
    camera ─USB3──┘

(b) edge + full stack — the router reflexes locally, the PC does the real work
    lidar ──1GbE──┐                    relay (raw UDP)
                  ├── A3004NS-M ────────────────────────> PC: ouster-ros, ROS 2
    camera ─USB3──┘         │
                            └── ring UDP ──> PC: ring_to_laserscan node

(c) transparent bridge — router adds nothing but ports
    lidar ──1GbE── A3004NS-M ──1GbE── PC        (ouster-edge disabled)
```

For (b), point the sensor's `udp_dest_ip` at the *router* and set
`option relay '<pc-ip>'`. A sensor can only send to one destination, so the
router being in the path is what lets both consumers exist at once.
