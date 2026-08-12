# AFHDS 2A: making the router the transmitter

Correcting my own earlier framing first, because it was misleading by omission.

I said no WiFi chip can receive or transmit AFHDS 2A, and that is true. But the
question was whether **the router** can produce the remote's signal, and the
answer to that is **yes** — just not out of its WiFi radio. It needs an A7105 in
front of it, which is the option `RC-AND-WIFI.md` listed and I did not build. So
the protocol analysis was worth asking for.

Two separate claims, and only one of them is a "no":

| | |
|---|---|
| router's MT7615D generates AFHDS 2A | **no** — 802.11 PHY cannot produce GFSK FHSS at arbitrary hop timing |
| router drives an A7105 that generates it | **yes** — this document is how |

## What the protocol actually is

2.4 GHz GFSK, frequency hopping over 16 channels, driven by an **A7105**
transceiver. 14 channels of servo data plus a telemetry return path.

Facts below are cross-checked between two independent open implementations,
[DeviationTX](https://github.com/DeviationTX/deviation/blob/master/src/protocol/flysky_afhds2a_a7105.c)
and
[DIY-Multiprotocol-TX-Module](https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/blob/master/Multiprotocol/AFHDS2A_a7105.ino).
They were cross-checked rather than read singly because **they disagree**, and
both disagreements matter to anyone implementing this.

### Disagreement 1: frame length

Community write-ups describe "a 37 byte 0x58 packet". Both implementations
transmit **38** bytes; 37 is the *telemetry* frame coming back. Building a
transmitter on the 37 figure produces a frame no receiver acknowledges, with
nothing to indicate why.

```
#define AFHDS2A_TXPACKET_SIZE   38
#define AFHDS2A_RXPACKET_SIZE   37
```

### Disagreement 2: hop derivation — and this one is liberating

The two implementations derive the 16 hop channels **differently**:

```c
/* DeviationTX */
rnd = rnd * 0x0019660D + 0x3C6EF35F;
uint8_t next_ch = ((rnd >> (idx%32)) % 0xa8) + 1;      /* 1..168 */

/* DIY-Multiprotocol */
uint8_t band_no = ((((idx<<1) | ((idx>>1) & 0b01)) + rx_tx_addr[3]) & 0b11);
rnd = rnd * 0x0019660D + 0x3C6EF35F;
uint8_t next_ch = band_no*41 + 1 + ((rnd >> idx) % 41); /* 1..164 */
```

Both work with real FlySky receivers. That is only possible for one reason: **the
transmitter chooses the hop set and hands it to the receiver during bind**, in
frame bytes 11..26. It never has to match what a FlySky radio would have picked.

So FlySky's own PRNG does not need reverse engineering at all. Any legal,
well-spread set of 16 channels is a valid answer. Both implementations even
advance their generator with the same constant — `0x19660D`/`0x3C6EF35F` is the
Numerical Recipes LCG, borrowed, not discovered.

Worth noting for this particular box: the router is *also* transmitting on
2.4 GHz. A hop set that bunches into one 20 MHz slice will sit under the router's
own WiFi carrier, so spreading the set across the band is not cosmetic here.
`afhds2a_hop_calc()` caps five channels per quarter for that reason.

## Frame layouts

All frames are 38 bytes. Byte 0 is the type.

| type | meaning |
|---|---|
| `0x58` | sticks — the normal case |
| `0xaa` | settings (servo rate, PPM output) |
| `0x56` | failsafe positions |
| `0xbb` | bind phase 1 |
| `0xbc` | bind phases 2–4 |

**Sticks / failsafe**

```
0       type
1..4    txid
5..8    rxid
9..36   14 channels, uint16 little-endian, microseconds, 875..2125
37      0x00
```

9..36 is 28 bytes, which is exactly 14 × 2 — the arithmetic settles the channel
count on its own. Same 14 channels `rc-ibus` decodes on the way in, so a
receiver's output can be re-transmitted without remapping.

In a failsafe frame, `0xffff` for a channel means "no commanded position" and the
receiver falls back to its own configuration. That is the better default: a wrong
commanded failsafe position is worse than letting the receiver decide.

**Bind**

```
0       0xbb (phase 1) or 0xbc (phases 2..4)
1..4    txid
5..8    0xffffffff for phases 1..3; the receiver's id for phase 4
9       0x01 for phase 1, else phase-1
11..26  the 16 hop channels
27..37  0xff padding; phases 2..4 set 27=0x01, 28=0x80
```

Bind alternates between two fixed channels, **`0x0d` and `0x8c`** — not channel
0, which is where one would guess.

## Timing

```
transmit frame            ~1700 us  (1550 on faster MCUs)
switch to receive, listen ~2150 us
                          --------
per hop                    3850 us
```

Roughly 260 Hz, hopping every frame. Settings frames are interleaved about every
1313 frames and failsafe about every 1569.

3.85 ms is the constraint that decides where this code can run. It is comfortable
on a microcontroller and awkward under Linux: a missed slot is a dropped frame,
and enough dropped frames is a failsafe. On the router this wants a dedicated
process with real-time priority and a SPI controller that is not shared, and it
is still the weakest part of the plan.

## A7105 bring-up

```c
A7105_WriteID(0x5475c52A);        /* the id AFHDS2A uses */
/* load the register table, then: */
A7105_WriteReg(0x02, 1); while (A7105_ReadReg(0x02));   /* IF filter bank */
A7105_WriteReg(0x0F, 0x00); A7105_WriteReg(0x02, 2);
while (A7105_ReadReg(0x02));                            /* VCO cal, ch 0x00 */
A7105_WriteReg(0x0F, 0xa0); A7105_WriteReg(0x02, 2);
while (A7105_ReadReg(0x02));                            /* VCO cal, ch 0xa0 */
```

The register table is 50 bytes at 0x00..0x31, with `-1` meaning "leave alone",
plus 0x24 = 0x13 and 0x26 = 0x3b written separately. It is reproduced in the
DeviationTX source linked above; it is not repeated here because a table copied
by hand is a table with a typo in it.

## What is built, and what is not

`../rc-tx/src/afhds2a.[ch]` — hop derivation, and the sticks, failsafe and bind
frame builders. Plain C, no platform dependencies, so it drops into either a
router talking to an A7105 over `spidev` or an MCU using its own SPI.

`../rc-tx/test/test_afhds2a.c` — 40 checks, all passing, including a sweep of
4000 seeds where every derived hop set must be legal and none may exhaust the
retry guard. It also pins the frame geometry, so the 37-vs-38 confusion cannot
creep back in, and checks that clamping happens instead of rejection, that
unsupplied channels centre rather than sending zero, and that the hop table lands
at bytes 11..26 of the bind frame.

```sh
cd package/keti/rc-tx/test
cc -O2 -Wall -Wextra -o test_afhds2a test_afhds2a.c ../src/afhds2a.c && ./test_afhds2a
```

**Not built, and not verifiable here: everything involving the radio.** There is
no A7105 on this bench. The register table, the calibration sequence, the bind
handshake against a real receiver, and the 3.85 ms timing are all untested, and
the register writes are exactly where a transmitter fails silently. Treat the
code above as the half that is known good and the radio layer as unwritten.

## Getting the radio

Cheapest first:

1. **An A7105 breakout** (XL7105-D03 and similar) on SPI. Four wires plus power.
2. **The RF module out of a cheap FlySky transmitter.** An FS-i6 has exactly the
   part needed and its module is socketed.
3. **A ready-made Multiprotocol module.** These ship with an A7105 and firmware
   that already implements AFHDS 2A; if the goal is "the router commands a FlySky
   receiver" rather than "we implement AFHDS 2A", this is the short path — drive
   it over its serial input and skip this document entirely.

Where to attach it, in order of how much soldering:

- **A microcontroller over USB.** There are ESP32s in the drawer already. The
  3.85 ms budget is trivial there, and the MCU can take channel values as the
  same `TCMD` UDP datagram `teleop` already sends and `teleop/test` already
  verifies — no new protocol, and the router side is done.
- **The router's own SPI.** `spi0` carries the NOR flash and there is no header,
  so this means soldering to a second chip select, and then fighting Linux for
  3.85 ms determinism. Possible, and the worse of the two.

The first option is what I would build. It puts the timing-critical work on a
part that has nothing else to do, and it needs no soldering to the router.

## One practical note

A home-built transmitter is not type-approved equipment, even though the band and
the protocol are ordinary hobby RC. That matters for selling one, not for driving
your own receiver on your own bench.

And if this ends up controlling something that moves, the rule from `TELEOP.md`
still holds and matters more here, not less: a real transmitter has a failsafe
because the link *will* drop. The failsafe frame above is how you configure it —
send it, do not skip it because the sticks frame works.
