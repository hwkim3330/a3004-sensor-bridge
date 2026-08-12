# FlySky AFHDS 2A, and how many things the WiFi can do at once

Two separate questions that get conflated because both say "2.4 GHz".

## Can the router receive a FlySky transmitter directly?

**No, and no amount of software changes that.**

AFHDS 2A is GFSK with frequency hopping over 2.408–2.475 GHz, driven by an
**A7105** transceiver, 14 channels, with an i-BUS telemetry side channel. The
MT7615D is an 802.11 radio: DSSS/OFDM, 20–80 MHz channels, a MAC that only
hands up frames its PHY has demodulated. There is no path from "GFSK burst at
a few hundred kbps" to "802.11 receive chain". Sharing a band is not sharing a
modulation.

The same applies to any WiFi chip. Monitor mode does not help: it exposes
802.11 frames, not raw IQ. Receiving AFHDS 2A means having an A7105 (which is
what the transmitters, and the Deviation project's reverse engineering, use).

So there are three real options.

### Option A — take i-BUS off a receiver (recommended)

Pair a normal FlySky receiver (FS-iA6B and friends) to the transmitter and read
its **i-BUS servo output**, which is plain asynchronous serial. This is the easy
and reliable route: the receiver does all the RF work, and the router only has
to read a UART.

From FlySky's documentation, the i-BUS servo frame is:

```
byte  0     0x20        length (32)
byte  1     0x40        command: servo data
bytes 2..29 14 x uint16 little-endian channel values, 1000..2000 (microseconds)
bytes 30,31 uint16      checksum = 0xFFFF - (sum of bytes 0..29)
```

32 bytes at 115200 8N1, repeated about every 7.5 ms. Decoding it is a few dozen
lines. (Format taken from documentation, not measured here — validate the
checksum on real data before trusting it.)

The router has no exposed UART header other than the console, so this means a
USB-serial adapter — and see `CAN.md` on the single USB port. A CP2102 or
CH340 is Full Speed and shares a USB 3.0 hub happily.

SBUS is the other common output, but it is 100000 baud 8E2 **inverted**, which
needs either an inverting buffer or a UART that can invert. i-BUS avoids that.

### Option B — an A7105 module on SPI

This is the only way to talk AFHDS 2A over the air, including *transmitting*
(acting as the remote rather than listening to one). It needs an A7105 breakout
on a SPI bus. On this board `spi0` carries the NOR flash and there is no header,
so it means soldering to a second chip-select. Possible, not pleasant, and there
is no OpenWrt driver — you would be writing the protocol in userspace on
`spidev`.

If the goal is "the router acts as the remote control", note that this is the
only option that does it over AFHDS 2A. Option A can only *listen* to a link
that a real transmitter already owns.

### Option C — skip RC entirely

The router already has a WiFi AP and a dashboard. If what you want is a human
steering something from a tablet, sending commands over WiFi to whatever holds
the control loop is simpler and has better telemetry than emulating a hobby RC
protocol. The caveat from `CAN.md` applies: whoever holds the control loop needs
a heartbeat and a stop-on-timeout, and that should not be behind a wireless hop.

**Nothing in Option A or B is implemented in this tree.** It is written down
because it is the sensible plan, not because it exists.

## How many things can the WiFi do at the same time?

With the DBDC fix in place (see `DBDC.md`), **two independent radios**:

| | |
|---|---|
| phy0 | 2.4 GHz, 2×2 |
| phy1 | 5 GHz, 2×2 |

They run on different channels simultaneously — that is what DBDC means, and
it is exactly what the unmerged port could not do. Without the fix you get one
radio that can be on one band at a time, and then none of what follows works.

Each radio can host **several virtual APs** (multi-SSID) at once; mt76 on
MT7615 allows well beyond the 4–8 that is sane in practice. Every extra BSS
costs beacon airtime, so more is not better.

**AP and STA together** works, with one rule: two interfaces on the *same* radio
must share a channel. So an AP and a STA on phy1 will both end up on whatever
channel the upstream AP uses, and throughput is shared. Putting them on
*different* radios avoids all of that:

```
phy1  5 GHz   AP    "A3004-SENSOR"   tablet + sensor data     ~300-400 Mbit/s
phy0  2.4 GHz STA   joins site WiFi  backhaul / internet
```

or, if no upstream network is involved:

```
phy1  5 GHz   AP    data (camera 20-83 Mbit/s + lidar 64 Mbit/s)
phy0  2.4 GHz AP    control/management, longer range, low rate
```

The measured numbers make the first arrangement comfortable: 1280×720 MJPEG is
19 Mbit/s and 1080p30 is 39 Mbit/s off a real StreamCam, the OS-64 is 64 Mbit/s
at 1024×10, and the microphone is 0.26 Mbit/s. Even 1080p60 plus lidar is about
147 Mbit/s, which a clean 5 GHz 2×2 VHT80 link carries.

What it will *not* do is give the lidar a latency guarantee. UDP over WiFi with
a shared camera stream means retries and jitter; `missed_columns` in the lidar
status file is the number to watch, and `sensor-lan-tune` matters more on a
wireless path than a wired one. If the lidar has to be lossless, wire it.

Set it up with:

```sh
uci set wireless.radio1.disabled='0'
uci set wireless.default_radio1.ssid='A3004-SENSOR'
uci set wireless.default_radio1.encryption='psk2'
uci set wireless.default_radio1.key='<password>'
uci commit wireless && wifi reload
iwinfo                      # both phys should appear
```

None of the multi-radio behaviour is verified on hardware — it all rests on the
DBDC override actually taking effect, which needs the board.
