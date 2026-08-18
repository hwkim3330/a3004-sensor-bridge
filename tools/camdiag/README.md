# Where the camera's latency actually goes

Measured on 2026-08-18, tablet on wifi, PC on the wired port, 1280x720 MJPEG
passthrough at 20 fps output.

## The number

Glass to glass, screen change to picture on screen: **168 ms** median, 131 ms
best, 210 ms worst, over 12 flips. Stable when the detection threshold moves
from 30% to 70% of the step (161 / 168 / 177 ms), which is inside one 50 ms
frame interval - so it is the camera chain and not the detector.

## The split

| stage | measured |
|---|---|
| camera internal + monitor and display path | **~140 ms** |
| bytes on the wire, router to client | 12-16 ms |
| tablet JPEG decode | 10-11 ms |
| held in ustreamer before sending | ~0 ms |

The router is not the problem, and neither is the network. `frames.py` reads
`X-Timestamp` off each part and compares it with the arrival time; the frame
starts leaving within a millisecond of being stamped.

`stream.c` says why: it calls `_get_latest_hw()` and, when the fps limit rejects
a frame, drops it and waits for the next latest one. So the frames it serves are
always freshly captured - decimation, not queueing. That kills the obvious
theory that a 20 fps gate over a 60 fps camera serves stale frames.

## What does not help

**Setting the camera itself to 20 fps.** The idea is right - the camera captures
at 60 and ustreamer throws away two of every three, and 720p MJPG does offer a
discrete 20 fps interval. But it cannot be done from config, and it would not
buy latency:

- `capture.c:867` hardcodes `TPF(denominator) = -1 // Request maximum possible
  FPS`. `--desired-fps` never reaches the device; `stream.c:364` uses it as
  `take = ceil(captured_fps / desired_fps)`, a drop filter. Changing the device
  rate needs a patch to ustreamer, not a setting.
- `VIDIOC_S_PARM` from outside fails with `Resource busy` while ustreamer holds
  the stream, so it cannot be done alongside either.
- And it would likely make latency *worse*: at 60 fps the sensor's exposure is
  capped by a 16.7 ms frame period, at 20 fps by a 50 ms one, and
  `exposure_dynamic_framerate` is 0 so nothing else limits it.

It would save USB traffic and some capture overhead. That is a CPU argument, not
a latency one.

## What did help, and by a lot

Two clients where one would do, and one where none would do.

**The unconsumed lidar relay.** `ouster-edge --relay 192.168.1.20:7502` was
forwarding raw 12544-byte datagrams at **64.2 Mbit/s** to a PC with nothing bound
to the port. UDP has no way to tell the sender, so it had been doing it for as
long as the setting had been there. Clearing it:

| | before | after |
|---|---|---|
| kernel time (sys) | 56% | 21% |
| idle | 22% | 64% |
| load average | 5.43 | 3.24 |
| camera on the wire | 15.5 ms median, 41.8 ms worst | 11.6 ms, 21.5 ms |

About 1.4 of the board's 4 cores, spent on packets nobody read. Enable the relay
when running `capture.py`, `live_view.py` or `ros2_bridge.py`, and clear it after.
The tablet's lidar plot does not use it - that is the ring broadcast on 7602 -
so turning it off costs the console nothing.

**A second MJPEG viewer.** Measured: ustreamer at **33% of one core** with one
client, **50% with two**, plus another 25 Mbit/s. There is no shared encode to
amortise, so every viewer is charged in full. `console.py` now needs `--camera`
to ask for video, and its camera read moved to its own thread that keeps only the
newest frame - it had 4.5 MB backed up on the socket after two hours, which is a
picture more than a second stale and a router that was sending all of it.

The tablet had the same shape of bug waiting: reading and decoding on one thread
means a slow decode stops the socket being drained, and latency then grows
without bound. It now drains on the reader thread and decodes the newest frame
only. Measured after: 20.0 fps in, 20.0 fps drawn, 0 dropped, 11 ms per decode -
40 ms of headroom in a 50 ms budget, so the hazard was real but not yet firing.

## Also found

The router's clock was **1017 s behind** the PC with `ntpd` enabled. Unrelated to
video, but it makes every timestamp on the board wrong by 17 minutes.

## Tools

    clock-offset.py  pc-minus-router clock offset, needed by frames.py
    latency.py   glass to glass, by flashing a window and watching the stream.
                 Windowed by default - it used to go fullscreen, which takes over
                 the machine of whoever is sitting at it with no warning.
    frames.py    per-frame arrival, size, wire time and X-Timestamp age. Needs
                 no display. --clock-offset comes from clock-offset.py.
