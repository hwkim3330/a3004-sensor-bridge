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
`2048x10` — 1280 datagrams/s, zero missed columns — cost **24.5 and 25.6 µs of
CPU per datagram at `-O2`**, and 31.6 and 33.9 at `-Os`, over two rounds each.

A 1004Kc at 880 MHz against this Skylake core is a 5.6x clock deficit and
roughly 0.35–0.4 of its work per clock, so call it **15x**:

| build | per datagram | at 1280/s |
|---|---|---|
| `-O2` | ~375 µs | **~48% of one CPU** |
| `-Os` | ~480 µs | ~61% of one CPU |

Which is why the flag is worth the twenty-four bytes: it buys about thirteen
points of a CPU that also has to run the ethernet RX softirq for 126 Mbit/s,
because that softirq has nowhere else to go.

So full rate through the router is plausible and not comfortable, and the ring
it produces is 88 kbit/s either way. Measuring it for real needs the board
powered, concurrently with the camera, and needs checking that `SO_RCVBUF` of
4 MB survives `rmem_max`.

## The thing to do before either

None of this is the binding constraint right now. **The vehicle does not move
yet**: there is no USB-CAN adapter, so `can-bridge --discover` has never run, so
whether the SCOUT speaks protocol v1 or v2 is unknown — see SCOUT-FIRMWARE.md.
Picking a compute node before that is picking a spec for a workload nobody has
run. The adapter is €30 and answers a question the €2000 box cannot.
