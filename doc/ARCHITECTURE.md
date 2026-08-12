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

The camera, if it emits MJPEG itself, is roughly 30–60 Mbit/s at 720p30 and
quality 80 — very content-dependent. Camera plus lidar together stay inside a
single 5 GHz link, which is what makes the tablet-on-the-router-AP setup work.

100 Mbit is not an option: the sensor will not negotiate it, and 64 Mbit/s of
UDP on a 100 Mbit link has no headroom for retransmit-free delivery.

## CPU budget

| Work | Cost |
|---|---|
| Switching lidar packets between LAN ports | 0% — the MT7530 does it in hardware |
| `ouster-edge` range-ring reduction | ~655k integer min-compares/s ≈ **under 1% of one core** |
| Raw relay (one extra `sendto` per packet) | small; 8 MB/s of copying |
| MJPEG pass-through (`ustreamer`) | small; USB DMA in, socket out, no encode |
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
   1440 bytes per revolution instead of 6.4 MB. That is a 4400× reduction, and
   it is enough for obstacle presence, nearest-range, and zone logic.
4. **Reacts.** Polar zones are evaluated every revolution at 10 Hz, and a
   crossing runs a script. A local reflex with a 100 ms budget is exactly the
   kind of work that belongs on the node next to the sensor rather than on a
   machine at the other end of a WiFi link.
5. **Relays.** The raw stream is forwarded verbatim to a real machine when one
   is present, so nothing is lost by putting the router in the path.
6. **Hosts a dashboard** on port 80 that renders both the camera and the ring,
   so a tablet on the router's own WiFi is a complete client with no other
   machine involved.

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
