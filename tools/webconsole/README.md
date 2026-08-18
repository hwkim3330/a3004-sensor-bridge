# The console that runs on the PC

    pip3 install --break-system-packages aiohttp      # if it is not there already
    ./server.py --host 192.168.1.1
    # then open http://localhost:8090/

Add `--bind 0.0.0.0` to let another machine on the LAN open it. That is off by
default: this page can move a vehicle.

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

## Not here yet

- Microphone. `mic-stream` is on 8082 and the tablet plays it; this page does not.
- 3D. The lidar relay is off by default (it was found sending 64 Mbit/s to a port
  with nothing bound to it, for 1.4 of the board's 4 cores). Turn it on for
  `mapping/capture.py` and friends, and off afterwards.
