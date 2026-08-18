# The console that runs on the PC

    pip3 install --break-system-packages aiohttp      # if it is not there already
    ./server.py --host 192.168.1.1
    # then open http://localhost:8090/

`--bind` takes a comma-separated list. To let the tablet or another machine on the
vehicle's network open it, name the router-side addresses:

    ./server.py --host 192.168.1.1 --bind 127.0.0.1,192.168.1.20,192.168.1.171

Prefer that over `0.0.0.0`. This page can move a vehicle, and `0.0.0.0` is every
interface — which on this bench means the office network as well as the router's.
Verified: with the three addresses above, the tablet at 192.168.1.175 gets a 200
and the office-side address refuses the connection.

## Where the work is split, and why there

The vehicle's own loop stays on the router: the ring, the 2D map, `navigate`, and
`teleop`'s 300 ms deadman. This console sends *intent* and *destinations*. If it
dies, or the wifi drops, or the browser tab is closed, the vehicle goes neutral on
its own — nothing here has to notice or behave well for that to happen.

Everything heavy belongs here. The router has four small cores already running the
lidar, the mapper and the camera; a 3D map, a detector or a planner wants this
machine.

## Why a server here rather than a page from the router

**A browser cannot send a datagram.** The control frame is UDP by design. The old
web dashboard sent one HTTP request per command and hit a browser's
parallel-connection cap at 20 Hz, which is why the tablet app exists at all. Here
the browser sends intent over one WebSocket at 20 Hz and *this process* keeps the
50 Hz `TCMD` stream going, so the rate discipline lives where it can be met.

**MJPEG is charged per viewer.** Measured on the board: ustreamer costs 33% of a
core for one client and 50% for two, with no shared encode to amortise. This pulls
the camera once and fans it out, so five browsers cost the router what one does.

## Deadmen, in order

Three, and they are not redundant so much as staged:

1. **The tab.** Releasing a key, losing focus, or hiding the tab clears the keys
   and disarms locally. The OS stops delivering `keyup` once focus moves, so
   without the focus handler a key released elsewhere would stay held here.
2. **This server, 250 ms.** No intent for that long and the frames stop being
   armed, and the count of lapses is on screen. It is deliberately shorter than
   the router's, so the ordinary case — a browser going quiet — is caught here and
   can be named.
3. **`teleop` on the router, 300 ms.** No frames at all and the output goes
   neutral and the armed flag drops. This is the one that matters, because it is
   the one that does not depend on this machine.

When nobody holds control the server sends nothing at all, rather than a stream of
disarmed frames. Sending anyway looked harmless — armed is 0, axes are 0 — but it
means `teleop`'s deadman never trips, so `teleop` would report a live operator link
for as long as this process was running. That is the one thing its status is for.

## One driver

Control is a single slot, taken by whoever asks and released on disconnect. Two
operators pushing different intent at one vehicle is not something to resolve by
averaging. Everyone else watches, and the page says who has it.

## Reading the map

Occupied dark, free white, unknown grey — the same as the tablet, so a screenshot
from either is comparable. `slam2d` adds `S2_HIT` on a return and subtracts along
the ray to it, so above `S2_UNKNOWN` is occupied. (`mapping/console.py` had this
inverted against its own comment for a while; both pictures look like a plausible
map on their own, which is why it lasted.)

The view crops to the surveyed region rather than showing the whole grid, because
the grid is 20 m square whatever has been seen. The crop is bounded by *free*
cells, not by everything known: the map carries isolated stray returns scattered
right across it, and one of those in a far corner puts the full 20 m back on
screen. Free cells only exist along rays that were actually traced.

## Saying when a control does nothing

`teleop`, `navigate`, `agx-cmd` and `can-bridge` are all off by default on the
router, and for a while this page showed a complete steering UI with `teleop`
stopped: the frames went to a port with nothing bound to them, and the console
looked exactly as it does when everything works.

The first fix was wrong in an instructive way. It tested whether the status file
existed — and `/var/run/*.json` is written by each daemon and **not** removed when
it stops, so a stopped `teleop` still looked present. What actually distinguishes
them is that a running daemon rewrites its file constantly: measured on all four,
six polls over 3.6 s give six different contents even with nothing happening,
because each carries a counter or an age. So the server times how long a file's
bytes have been unchanged and nulls anything frozen for more than three seconds.

A daemon that is gone then disables the controls that depended on it and says so:
the joystick dims, Arm and Go here grey out, and the banner names the init script.
There is a second, softer case — `teleop` running but `forwarding` false, because
`agx-cmd` is off — which is announced without disabling anything, since the
steering does reach `teleop`, just no further.

## Microphone

One upstream reader, fanned out, the same argument as the camera: `mic-stream`
charges per client too. Two products come from that read — the raw S16 on `/audio`
for a browser that wants to hear it, and one peak per buffer in the telemetry for
the waveform, computed once here rather than in every browser.

`Listen` is a button because an `AudioContext` can only be created from a gesture.
Playback schedules one `AudioBuffer` per arrival, each starting where the last
ended, and restarts from the clock if the stream stalls rather than dumping a
backlog. A listener that cannot keep up is dropped rather than queued for: audio
whose only value is being current should not be buffered at somebody.

## Not here yet

- 3D. The lidar relay is off by default (it was found sending 64 Mbit/s to a port
  with nothing bound to it, for 1.4 of the board's 4 cores). Turn it on for
  `mapping/capture.py` and friends, and off afterwards.
