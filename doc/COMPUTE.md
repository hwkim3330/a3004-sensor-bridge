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

### What full-rate lidar costs the router — measured

The prediction below was wrong by a factor of two, in the safe direction, and is
kept underneath because the method is what produced the error.

Measured on the board, sensor on a router LAN port at `2048x10` with
`RNG19_RFL8_SIG16_NIR16`:

| | |
|---|---|
| received | **1280 datagrams/s**, exactly the expected rate |
| losses | `bad_size` 0, `invalid_columns` 0, **`missed_columns` 0** |
| cost | **29.9% of one CPU**, **234 µs per datagram** |

Every mode the sensor offers, measured the same way on the board:

| mode | profile | datagrams/s | bytes | cpu | µs/datagram | missed |
|---|---|---|---|---|---|---|
| 512x10 | RNG15 | 321 | 4352 | 5.4% | 168 | 0 |
| 512x10 | RNG19 | 320 | 12544 | 8.2% | 256 | 0 |
| 1024x10 | RNG15 | 640 | 4352 | 9.6% | 150 | 0 |
| 1024x10 | RNG19 | 649 | 12544 | 14.3% | 221 | 0 |
| 2048x10 | RNG15 | 1297 | 4352 | 18.1% | 139 | 0 |
| **2048x10** | **RNG19** | **1297** | **12544** | **28.0%** | 216 | **0** |
| 512x20 | RNG19 | 649 | 12544 | 14.6% | 225 | 0 |
| 1024x20 | RNG19 | 1280 | 12544 | 27.7% | 216 | 0 |

**Not one dropped datagram anywhere in that table**, including 126 Mbit/s of
fragmented UDP. Three things it says beyond the headline:

- **The per-datagram cost falls as the rate rises** - 168 to 150 to 139 µs for
  RNG15 as the rate triples. That is the fixed cost of each wakeup being spread
  over a fuller `recvmmsg` batch, which is the batching doing its job.
- **RNG19 costs about half again as much as RNG15** per datagram, for three
  times the bytes. The work is per pixel and mostly independent of pixel width;
  only the stride changes.
- 1024x20 and 2048x10 are the same data rate by different means and cost the
  same, which is a small consistency check on the whole table.

So 126 Mbit/s of fragmented UDP arrives intact and is reduced to a ring for
under a third of one of the four hardware threads. The scaling estimate below
said 480 µs and 60%; the real ratio against this desktop is about **5x**, not the
15x assumed. Predictions from clock and IPC alone were too pessimistic, and the
right conclusion is not "the estimate was close enough" but that an estimate of
this kind is worth roughly a factor of two either way.

Also read off the board for the first time, since it had never been powered with
a report that asked: **4 logical CPUs, MIPS 1004Kc V2.15, 242 MB of RAM** with
151 MB available. The RAM matters - `slam2d-daemon` peaks at 2.4 MB, so the
40 m map at 5 cm is not close to a constraint.

### The earlier prediction, kept for the method

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

### It runs there. Measured on the board.

The image is flashed and the chain is live: sensor on a LAN port, ouster-edge,
slam2d, at 1024x10 with RNG19.

| | 40 m map | **20 m map, band 28:33** |
|---|---|---|
| slam2d | 67.8 ms/scan, 67.5% of one CPU | **29.1 ms/scan, 29.4%** |
| ouster-edge | 23.8% | **11.9%** |
| together | 91.3% of one CPU | **41.3% — 10.3% of the machine** |
| match score | 85% | **87%** |
| resident | 1792 kB | under 2 MB |

10.1 revolutions a second, which is the sensor's rate, with no drift from a
stationary sensor and `at_search_edge` false throughout.

**The prediction was 15 ms and it came out at 67.8, and the reason matters more
than the miss.** ouster-edge runs 5x slower on this part than on the desktop;
slam2d runs 71x slower. They are not the same kind of work. One streams through
packet bytes in order and caches perfectly; the other reads 675 candidate poses
times 250 points at random across the occupancy grid, and 818 kB does not fit a
1004Kc's cache. Halving the map edge to 20 m quarters the memory, does not change
the number of lookups at all, and more than halves the time. Scaling by clock and
IPC was never going to see that.

### The decision: on the router

All three hosts became possible once the floating point was gone, so the choice
stopped being about arithmetic. It is **the router**, for one reason that
outweighs the rest and several that support it.

**Autonomy must not depend on the operator still being there.** The goal is to
tap a destination and have the vehicle drive to it. If the map and the planner
live on the tablet, then putting the tablet in a pocket, walking behind a wall,
or any ordinary WiFi dropout ends the mission mid-route. The deadman makes that
*safe* — the vehicle stops — but a robot that stops whenever the operator looks
away is not doing the thing that was asked. On the router, the loop from sensor
to decision never leaves the vehicle, and WiFi carries only the destination going
out and the map coming back, neither of which is time-critical.

The supporting evidence, measured rather than argued:

| | |
|---|---|
| whole chain on mipsel | `oe-inject` → `ouster-edge` → `slam2d-daemon`, all target binaries under the emulator |
| result | 3840 packets, 0 missed columns, 59 revolutions, 59 matched, 19 cm error, score 69% of maximum |
| speed | 60 revolutions processed in 3 s of wall clock — **2x real time**, and that is inside QEMU's TCG, which is not faster than the real part |
| memory | `slam2d-daemon` peaks at **2.4 MB** RSS, of which 820 kB is the grid and pyramid for a 40 m map at 5 cm |
| the ring | already in the router's memory; nothing has to be sent anywhere to start |

The tablet keeps the job it already has and is good at: send a destination,
display the map and the pose. Because the core is integer, it can also run the
identical code on the identical data and get identical answers, which makes it a
genuine second opinion rather than an approximation.

A Xavier remains the answer if perception on camera imagery is ever wanted. It is
not needed for this.

What is still a prediction: 26 ms per scan on the real part. The emulator shows
correctness and suggests the real-time margin is real, but QEMU is not a cycle
model, and the router will also be relaying the camera at the same time. Both go
on the list for when the board is powered.

Whichever host, the robot needs only to execute velocity commands, which
`agx-cmd` already encodes, and to stop by itself when the link goes quiet, which
`teleop`'s deadman and `ouster-edge`'s zones already do.

### On real lidar, finally

Everything above was simulated until the sensor was plugged into this desktop.
What the first real run found is worth keeping, because none of it was visible
in simulation:

- **The sensor's stored configuration is not to be trusted.** Its
  `azimuth_window` was at 90 degrees, so two thirds of the packets were never
  sent, ouster-edge counted 124475 missed columns, the ring was empty and slam2d
  matched nothing - with every daemon reporting healthy statistics about
  nothing. `ouster-configure` now asserts the configuration at every start, and
  reads `beam_altitude_angles` to pick the horizontal rows, which on this sensor
  are 28:33.
- With the window restored: 277 packets/s, **zero missed columns**, 175 of 360
  sectors returning, and slam2d matching **all 266 revolutions at 91%** with no
  drift over thirty seconds from a stationary sensor. The map is a real room,
  7.0 x 6.6 m.
- Moving the sensor by hand: the ring changed by up to 101 cm, and the pose
  followed it - 58 degrees of heading and about 20 cm of translation. The match
  score fell to 35% during the movement and recovered to 85%, which is exactly
  the signal `navigate`'s `min_score_pct` watcher exists to catch. There is no
  ground truth here, so this says the pose responds coherently to real motion,
  not how accurately.
- `navigate` on the real map: 302 of the points within 3 m are valid
  destinations once the robot's radius is respected. Given one at 2.33 m it
  planned a **7.88 m** route around the real clutter, commanded, and then stopped
  itself with *no progress towards the goal* when nothing moved - because there
  is no vehicle. The stall watcher works on real data.

The measured cost on real data is **844 us per scan**, against 950 in the
simulated room.

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
