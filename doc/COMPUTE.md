# Choosing the onboard computer

The router is an aggregation and reflex node, not a perception one — see
ARCHITECTURE.md for why 880 MHz of soft-float MIPS with a 4.4 MiB overlay is the
wrong place for a point cloud. The question this file answers is what sits beside
it, and specifically whether the two Advantech boxes on hand are enough:

| | |
|---|---|
| MIC-710AIX-00A1 | Jetson **Xavier NX**, 6-core Carmel @ 1.4 GHz (1.9 GHz on two cores), 8 GB, label 2021.04 |
| MIC-730AI-10A1 | Jetson **AGX Xavier**, 8-core Carmel @ 2.26 GHz, 32 GB, label 2021.03 |

## What full rate actually costs

Measured, not estimated: live OS1-64 at `2048x10` with
`RNG19_RFL8_SIG16_NIR16`, KISS-ICP through `ouster.sdk.mapping.SlamEngine`, on
this desktop (i7-10700K, 16 threads). About 50,000 of the 131,072 pixels in a
scan came back — the sensor was indoors on a desk.

| input | voxel | median | p90 | max |
|---|---|---|---|---|
| 2048x10, 16 cores | 0.1 m | **11.4 ms** | 35.7 | 46.3 |
| 2048x10, 6 cores | 0.1 m | **12.7 ms** | 35.8 | 44.5 |
| 2048x10 | 0.5 m | 14.6 | 22.2 | 33.1 |
| 2048x10 | 0.05 m | 20.9 | 42.8 | 58.0 |
| 1024x10 | 0.1 m | 18.0 | — | — |
| 512x10 | 0.1 m | 14.6 | — | — |

Three things in that table are worth more than the headline number.

**Core count barely matters.** 16 cores to 6 cores moved the median 11.4 → 12.7
ms, and 4 cores gave 17.7. Whatever KISS-ICP does per scan here is close to
serial, so a smaller machine loses on clock speed and IPC rather than on width.

**CPU time overstates the work by an order of magnitude.** `getrusage` reported
**235 ms of CPU per scan** against 21 ms of wall — eleven cores' worth. Taking
that at face value says no Jetson can do this. But restricting the process to
four cores did not slow it down, which means most of that CPU time was a thread
pool busy-waiting, not work. Never size a smaller machine from CPU-seconds
measured on a wide one.

**Point count barely matters either.** 2048x10 to 512x10 is a quarter of the
data for two thirds of the time, and the voxel size from 0.5 m to 0.05 m costs
less than a factor of two. Downsampling dominates, and it runs before the
expensive part.

## So does it fit on the Xavier boxes

Carmel against this Skylake core: the clock ratio is 4.9/1.4 for the NX in
6-core mode and 4.9/2.26 for the AGX, and Carmel does roughly 0.6 of Skylake's
work per clock. That is **~5.8x** and **~3.6x** respectively.

This scaling is an estimate. Neither box was benchmarked — there is no Xavier
in front of this machine. Treat the table as a prediction to check, not a
measurement:

| | median | p90 | budget at 10 Hz |
|---|---|---|---|
| Xavier NX | ~74 ms | ~210 ms | 100 ms |
| AGX Xavier | ~46 ms | ~130 ms | 100 ms |

The medians fit. **The tail does not, on either box.** A p90 three times the
median is not noise — it is periodic map maintenance, and it will scale with the
core the same way the median does. At full rate both boxes would drop scans
occasionally, the NX often enough to matter. KISS-ICP is scan-to-map so a
dropped scan is survivable, but a run of them during a fast turn is where pose
is lost.

Also note the scene was static. A stationary sensor gives ICP an easy initial
guess; a moving robot costs more iterations, and the numbers above are therefore
a floor.

The mitigations are all cheap and none of them need better hardware: run SLAM at
5 Hz, feed it `1024x10`, or restrict beams. Full rate at 10 Hz into SLAM is a
choice, not a requirement — full rate is worth having on the wire for recording
and for perception, and SLAM can take a decimated copy.

## ROS is the harder half

Compute is not the constraint. Two other things are.

**The software generation.** Both boxes are Xavier, so they end at **JetPack 5 /
Ubuntu 20.04** — there is no JetPack 6 for Xavier. Ubuntu 20.04's ROS 2 options
are Foxy (EOL May 2023) and Galactic (EOL December 2022), and this PC runs
**Jazzy**. Mixing Jazzy and Foxy across a robot is a supported-on-paper,
painful-in-practice arrangement. The standard way out is a container —
`ros:humble` or one of the `dustynv/ros` L4T images — which works and is what
most Xavier deployments do, but it means the robot's ROS lives in a container
image you maintain rather than in the OS.

**Point cloud plumbing.** This is what actually falls over. Full rate is
1,310,720 px/s; as `PointCloud2` with xyz+intensity in float32 that is 16 B per
point, 2.1 MB per scan, **21 MB/s** per hop. One publisher into one composed
subscriber is a pointer. Every non-composed subscriber after that is another
21 MB/s of memcpy, and a normal graph — driver, filter, SLAM, costmap, RViz over
the network — multiplies it four or five times. The fix is intra-process
composition and loaned messages, decided when the graph is written; retrofitting
it later means rewriting the nodes.

## Recommendation

**AGX Xavier**, if the intent is full-rate lidar plus a camera with any
perception on it. Not for the compute — for the 32 GB, because model weights
plus frame buffers plus a voxel map plus a container's worth of ROS is where
8 GB goes. Its 2.26 GHz also buys the only real headroom in the table above.

**Xavier NX** is enough if the lidar stays at `1024x10` or below and the camera
stays a stream rather than an input to a network. It is the smaller, cooler, less
power-hungry box, and if the split of work stays as it is now — router does
reflexes, PC does perception — it is the better one to bolt to a SCOUT MINI.

Check the Ethernet port count on whichever is chosen before committing. Full
rate wants a port to itself; sharing one with the router uplink puts 126 Mbit/s
of unpaced UDP next to everything else.

## And the router's own cores

Worth stating because "MT7621 is dual core" invites the wrong expectation.

The kernel side is fully enabled and none of it needed changing — it is stock
ramips: `CONFIG_SMP=y`, `CONFIG_MIPS_MT_SMP=y`, `CONFIG_MIPS_CPS=y`,
`CONFIG_NR_CPUS=4`, `CONFIG_SCHED_SMT=y`. Linux sees four CPUs.

Four is not four cores. MT7621 is **two 1004Kc cores with two hardware threads
each**, and the second thread on a core shares that core's execution units. It
helps when one thread is stalled on memory, which on a router it often is, but it
is not a second core's worth of throughput.

More importantly, **ethernet receive does not scale across them at all.**
`mtk_eth_soc` registers a single RX NAPI instance for the whole block
(`struct napi_struct rx_napi`, one `netif_napi_add`), and the other three entries
in `rx_ring[MTK_MAX_RX_RING_NUM]` exist for hardware LRO, not for receive-side
scaling. So every packet the switch hands up is processed on whichever CPU takes
the `fe2` interrupt. TX is a separate interrupt (`fe1`) and can sit elsewhere,
which is the only split the hardware gives for free.

What does spread:

- the four sensor daemons are separate single-threaded processes, so the
  scheduler places them independently — which is the right shape for this SoC
  whether or not it was designed for it
- mt76 with DBDC has per-phy work, so two radios are not one radio's worth of
  one CPU
- RPS, via `network.globals.packet_steering`, moves protocol processing off the
  interrupt CPU after NAPI. It is **off by default and has never been measured on
  this board.** For a workload that terminates traffic locally — ustreamer
  serving frames, ouster-edge taking 126 Mbit/s of UDP — this is the one knob
  with a plausible effect, and the honest status is untested rather than
  recommended.

`first-boot-report` now prints the CPU count, the per-CPU interrupt counts for
eth/mt76/xhci, and the current RPS masks, so the next time the board is powered
this stops being an inference.

### What full-rate lidar would cost the router

Same method as the Xavier estimate: measure here, scale by clock and IPC, and
label it a prediction. `ouster-edge` built native and fed the live sensor at
`2048x10` — 1280 datagrams/s, zero missed columns — cost **31.6 and 33.9 µs of
CPU per datagram**, over two rounds.

A 1004Kc at 880 MHz against this Skylake core is a 5.6x clock deficit and
roughly 0.35–0.4 of its work per clock, so call it **15x**: about 480 µs per
datagram, and at 1280 datagrams/s **roughly 60% of one CPU** — the same CPU that
also has to run the ethernet RX softirq for 126 Mbit/s, because that softirq has
nowhere else to go.

So full rate through the router is plausible and not comfortable, and the ring
it produces is 88 kbit/s either way. Measuring it for real needs the board
powered, concurrently with the camera, and needs checking that `SO_RCVBUF` of
4 MB survives `rmem_max`.

### `-O2` was tried and refuted

Worth recording, because the x86 evidence for it was good and will tempt
somebody again.

On this desktop, `-O2` cost 24.5 and 25.6 µs per datagram against `-Os`'s 31.6
and 33.9 — reproducibly, alternating, about 23% cheaper. The mechanism was in
gcc's own `-fopt-info-inline` output: at `-O2` and not at `-Os`, `px_range_mm`
and `px_refl` are inlined into `packet_process`, which is **1024 calls per
datagram** it stops making, plus `zones_column`, `in_az_window` and `rd64` per
column.

On mipsel it does not happen. `emu/oe-flagbench.py` runs both builds under the
emulator with the injector inside the guest, and takes cost by subtraction
between two datagram counts:

| | user | sys | total |
|---|---|---|---|
| `-Os`, 36000 datagrams | 97 jiffies | 38 | **135** |
| `-O2`, 36000 datagrams | 116 jiffies | 26 | **142** |

An earlier round at 8000 datagrams had the two identical at 30 jiffies each. So
`-O2` buys nothing and may cost a little, while making the binary 988 bytes
larger — the wrong direction for a 32 kB instruction cache. The flag was
removed.

Two caveats on that measurement, so it is not over-read. QEMU's TCG has no cache
or pipeline model, so it shows how much *work* the instruction stream does and
not what an MT7621 would take in cycles. And the user/sys split is sampled per
tick, so it is noisy — the totals are the trustworthy figure, which is why the
totals are what the conclusion rests on.

The transferable lesson is the one in the table above: a per-datagram cost
measured on a wide out-of-order core says nothing reliable about a 1004Kc, in
either direction. The 60%-of-a-CPU estimate above inherits that same weakness
and stays labelled a prediction until the board is powered.

## 2D instead: tap a destination and drive there

A different question from the one above, and a much better-shaped one. Mapping a
building and navigating it in 2D does not need the 3D pipeline at all.

**The ring is already a 2D scan.** `ouster-edge` reduces each revolution to a
per-azimuth minimum range — 360 sectors at 10 Hz, 88 kbit/s. That is a
`LaserScan` in everything but name, and it is the thing the router is already
good at producing.

**What it costs.** Measured on this desktop, same machine as the 3D figure so the
ratio means something: scan matching by ICP over the 360 points takes **2.5 ms**
and folding them into a 5 cm occupancy grid takes **0.07 ms**, so 2.6% of the
100 ms budget at 10 Hz. Against 11.4 ms for KISS-ICP on 50000 3D returns, the 3D
problem is **4.4x** the work. Both figures are numpy with a KD-tree rather than a
production package, and the first attempt at the 2D number came out at 35 ms —
slower than the 3D one — purely because it built a 360x360 distance matrix per
iteration. That is worth remembering before concluding anything from a
measurement: the number described the loop, not the problem.

### The trap: the default ring is not a planar slice

`ouster-edge` takes the minimum over channels `ch_lo..ch_hi`, and the default is
`0..127` — **every beam**. On a desk that is harmless. Mounted on a vehicle it
destroys the scan, because the nearest return in any direction becomes the floor:

| sensor height | floor hit at −22.5° | at −15° | at −10° |
|---|---|---|---|
| 0.2 m | 0.5 m | 0.8 m | 1.2 m |
| 0.3 m | 0.8 m | 1.2 m | 1.7 m |
| 0.5 m | 1.3 m | 1.9 m | 2.9 m |

On a SCOUT MINI at roughly 0.3 m the ring becomes a uniform 0.8 m circle in every
direction and carries no map information whatever. The fix is configuration, not
code: set `channel_band` to the beams within a degree or two of horizontal. Read
`beam_altitude_angles` from the sensor to pick them — they are not evenly
indexed, and `ouster-metadata` already fetches that document.

Also worth knowing: 360 sectors is 1°, which is 20 cm of arc at 12 m. Fine for a
5 cm grid nearby and coarse far away. `--sectors 720` halves that and doubles the
88 kbit/s, which is still nothing.

### Where it runs — answered by removing the constraint

The obvious blocker was **`CONFIG_SOFT_FLOAT=y`**. MT7621 has no FPU, so every
floating-point operation is a libgcc call, and a conventional scan matcher is
almost nothing but floating point.

So the matcher was written without any. `package/keti/slam2d` is a correlative
scan matcher over an image pyramid, in integers throughout: ranges in
centimetres, angles in 1/4096 of a revolution, trigonometry from a Q15 table
built once by CORDIC. `objdump` on the core object finds **zero** floating-point
instructions and zero soft-float calls. The algorithm choice follows from the
constraint rather than fighting it — ICP needs an SVD per iteration, which is
unpleasant in fixed point, while scoring candidate poses against an occupancy
grid is a sum of table lookups.

Measured against a simulated room with known ground truth, 120 scans:

| | |
|---|---|
| position error | mean **5.3 cm**, worst 9.2 cm |
| heading error | worst 2.4° |
| match cost | **1.7 ms** per scan, 675 candidate poses |
| CORDIC sine vs libm | worst 0.00026 |

Cross-compiled and run under the emulator, mipsel produced **identical** figures
— 5.3 cm mean, 9.2 cm worst, 2.4° — which is the point of an integer core: the
router and the tablet cannot disagree about where the robot is. Scaled by the
same 15x, 1.7 ms becomes ~26 ms, comfortably inside the 100 ms budget, and
without a soft-float penalty because there is no float to penalise. That number
is a prediction until the board is powered; what the emulator establishes is
correctness, not speed.

So all three hosts are now open, and the choice is about where the map and the
planner should live rather than about who can do the arithmetic:

- **the router**, which already has the ring in memory and would not have to send
  it anywhere
- **the tablet**, which has an FPU and NEON going spare, is already receiving the
  ring, and is already where a destination would be tapped
- **a Xavier** with ROS 2, slam_toolbox and nav2, which is the conventional answer
  and brings the JetPack 5 problem with it

Whichever it is, the robot needs only to execute velocity commands, which
`agx-cmd` already encodes, and to stop by itself when the link goes quiet, which
`teleop`'s deadman and `ouster-edge`'s zones already do. That reflex layer is
what makes running a planner off-vehicle defensible rather than reckless.

### What is missing regardless of where it runs

- **Odometry.** Scan matching alone drifts; SLAM wants wheel odometry as a prior.
  AgileX publishes it and `agilex.c` decodes motion state, but nothing has ever
  read it, because there is no USB-CAN adapter.
- **The vehicle does not move.** Same blocker as everything else.
- **Which protocol generation** the vehicle speaks — see SCOUT-FIRMWARE.md.

So the honest order is unchanged: adapter first, listen, confirm the generation,
then the lateral sign with the wheels clear, and only then is a destination on a
map a meaningful thing to tap.

## Bluetooth: no, and what it would cost

There is none. No BT node in the device tree, and the mt7615 driver contains no
Bluetooth or coexistence code at all — MT7615D is a WiFi-only part, not one of
MediaTek's combo chips. So the only route is a USB dongle.

That was measured rather than guessed, by building the image both ways:

| | image | overlay left |
|---|---|---|
| baseline | 11328 KB | 4800 KB |
| with `kmod-bluetooth`, `kmod-btusb`, `bluez-utils` | 11968 KB | 4160 KB |

**640 KB**, or about 13% of the overlay, and the firmware partition is 16128 KB
so nothing overflows. Affordable, if a reason ever appears. It also pulls in
`kmod-hid`, `kmod-input-evdev` and three crypto modules.

The practical blocker is not size: there is no BT dongle on the router and none
on this desktop either, so the path cannot even be exercised through the
emulator's USB passthrough the way the camera can.

## The thing to do before either

None of this is the binding constraint right now. **The vehicle does not move
yet**: there is no USB-CAN adapter, so `can-bridge --discover` has never run, so
whether the SCOUT speaks protocol v1 or v2 is unknown — see SCOUT-FIRMWARE.md.
Picking a compute node before that is picking a spec for a workload nobody has
run. The adapter is €30 and answers a question the €2000 box cannot.
