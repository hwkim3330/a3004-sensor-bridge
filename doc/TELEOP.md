# Driving something from the tablet

Two questions get asked together and have opposite answers.

## No, the tablet cannot emulate a 2.4 GHz RC transmitter

The tablet on this bench is a Galaxy Tab S7 FE: platform `lito` (Snapdragon
750G/765G class), Qualcomm WCN39xx-family WiFi, Bluetooth SoC reported as
`cherokee`. That matters only to make the answer concrete — it would be the same
on any phone or tablet.

- **WiFi.** Same argument as `RC-AND-WIFI.md`: AFHDS 2A is GFSK frequency
  hopping driven by an A7105, and an 802.11 radio's PHY cannot produce or
  demodulate it. On Android there is additionally no API for it at any level —
  no monitor mode on this device, no raw IQ, no arbitrary waveform transmit.
  Root does not help, because the constraint is in the chip's firmware.
- **Bluetooth.** More tempting, since BLE really is GFSK in the same band. Still
  no: a BT controller only accepts BT-shaped packets from its own firmware. You
  cannot choose the sync word, the packet format, or the hop sequence, which is
  exactly what a different protocol is made of. The published work on coaxing
  raw transmit out of BT chips is Broadcom/Cypress-specific research; this is a
  Qualcomm part.

So the tablet cannot be the remote in the AFHDS 2A sense. What it can be is a
control surface that talks IP — which is what it is already good at.

## Yes, the tablet can drive it over IP — with two deadmen

The dashboard has a touch joystick and an ARM button. It sends intent to
`teleop` on the router, which forwards it to whatever holds the control loop.

The whole design is about one failure, because every failure here is the same
one: **commands stop arriving while the last one is still being acted on.** WiFi
has no latency bound, a browser tab can be backgrounded, a battery can die, a
person can walk out of range. CAN drive commands are level-triggered — the last
velocity keeps being obeyed — so "stop sending" is not a stop.

Three rules follow.

**1. Armed is deliberate, and one stale interval ends it.** Nothing is marked
armed until the ARM button is pressed. After `timeout` (default 300 ms) with no
command, the output goes neutral and the armed flag drops.

**2. Neutral is transmitted, not implied.** Frames go out at a fixed cadence
whether armed or not. A receiver cannot tell "no packet" from "packet lost", so
the disarmed state has to arrive as data. The daemon also emits one last
neutral, disarmed frame on shutdown.

**3. The receiver runs its own deadman.** A deadman on the sending side cannot
detect the failure that matters — the link between the two. So
`pc-side/teleop_receiver.py` requires all three of:

  - the sender says armed, **and**
  - a datagram arrived within its own timeout, measured on its own clock, **and**
  - the sender's monotonic clock is advancing between datagrams

The third catches what the other two miss: a sender wedged on one frame, still
looping and still incrementing its sequence number. Only when all three hold are
the axes passed on; otherwise the receiver commands **zero**, not nothing.

## Wire format

32 bytes, little-endian, sent at `rate` Hz to `remote`:

```
offset size  field
     0    4  magic "TELE"
     4    1  version, currently 1
     5    1  flags: bit0 = armed
     6    2  reserved
     8    4  uint32 sequence, increments every frame
    12    8  uint64 sender monotonic milliseconds
    20    8  4 x int16 axes, units of 1/10000, so -10000..10000 is -1.0..1.0
    28    2  uint16 buttons bitmask
    30    2  reserved
```

Integer axes on purpose: no float parsing, no locale, no rounding disagreement
between the browser, the daemon and the receiver.

The sequence number is what lets a receiver drop replays and count gaps. Note
that a sender restart resets it to 1, so a receiver must treat a large backwards
jump as a restart rather than a replay — otherwise a router reboot wedges it
permanently. The reference receiver does this; if you write your own, do too.

## The native app path

A browser must use HTTP. An app does not, so `teleop` also accepts commands as
UDP on `cmd_port` (default 7721), which removes the whole class of problems that
came with a request per command:

```
offset size  field
     0    4  magic "TCMD"
     4    1  version 1
     5    1  flags: bit0 = arm
     6    2  reserved
     8    4  uint32 sequence
    12    8  4 x int16 axes, units of 1/10000
    20    2  uint16 buttons
    22    2  reserved
```

Both paths go through the same arming, clamping and sequence rules, so they
cannot drift apart. A large backwards jump in the sequence is treated as a client
restart rather than a replay, for the same reason the receiver has to do it.

The client is at <https://github.com/hwkim3330/a3004-bridge-app> — it also takes
the lidar ring as the binary UDP datagram rather than polled JSON, and plays the
microphone through AudioTrack rather than a browser's jitter buffer.

## Setting it up

```sh
uci set teleop.control.enabled='1'
uci set teleop.control.remote='192.168.1.100:7720'   # the machine with the loop
uci set teleop.control.rate='20'
uci set teleop.control.timeout='300'
uci set teleop.control.cmd_port='7721'      # UDP commands, for the native app
uci commit teleop && /etc/init.d/teleop start
```

With `remote` empty the daemon still runs and the joystick still works, but
nothing is forwarded anywhere — a useful way to try the panel out with nothing
connected. The dashboard says `not forwarding` when that is the case, so it is
not mistaken for a working control path.

On the receiving machine:

```sh
python3 doc/pc-side/teleop_receiver.py --port 7720
# or, publishing geometry_msgs/Twist on ~/cmd_vel:
python3 doc/pc-side/teleop_receiver.py --port 7720 --ros
```

`--max-linear` and `--max-angular` set what full stick deflection means. Start
low.

## Where this must not go

Read `CAN.md` before wiring the receiver to a vehicle bus. The short version:
`can-bridge` refuses to inject onto CAN unless explicitly told to, and that
default should stay. A tablet joystick is fine for positioning something on a
bench or driving a robot in an open space with the operator watching. It is not
a substitute for a control loop with a real-time budget, and the deadmen here
bound the *duration* of a runaway, not its existence.

If you do connect it to something that moves: test the deadman first, with the
wheels off the ground, by walking away from the AP.

## What has been verified

`../teleop/test/test_teleop.py` — 30 checks, all passing, covering: frames flow
before arming and are marked disarmed; arming applies axes and buttons; axes are
clamped; replayed and reordered commands are rejected and counted; the deadman
neutralises and disarms; frames keep flowing after it trips; explicit disarm is
immediate; malformed input is counted and does not arm; a final neutral disarmed
frame is emitted on SIGTERM.

End to end on real hardware: the dashboard's joystick on a Galaxy Tab S7 FE
reaching the daemon (stick at 0.29/0.29 arriving as `axes: [0.29, 0.29]`,
commands accumulating at 20 Hz), and the daemon reaching the reference receiver
(`armed=True axes=[0.35, 0.80]` → receiver `LIVE [+0.35 +0.80]`), then both
deadmen firing when the commands stopped.

Two bugs were found by running it rather than reading it:

- the daemon's HTTP layer consumed a CRLF header terminator as two bytes instead
  of four, so every request after the first on a keep-alive connection was
  misparsed
- the connection table held four entries, which a browser's parallel connections
  exhausted; commands were then refused **silently**. The limit is 16 now, idle
  connections are reaped, and refusals are counted in the status file — the
  counter is the actual fix, since the limit can always be reached again
