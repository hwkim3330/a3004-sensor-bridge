# rc-tx — AFHDS 2A frame building

The half of "make the router act as a FlySky transmitter" that does not need the
radio in front of you. See [`../doc/AFHDS2A.md`](../doc/AFHDS2A.md) for the
protocol analysis and for what is still missing.

`src/afhds2a.[ch]` is plain C with no platform dependencies: hop-set derivation
and the sticks, failsafe and bind frame builders. It drops into either a router
driving an A7105 over `spidev` or a microcontroller using its own SPI.

```sh
cd test
cc -O2 -Wall -Wextra -o test_afhds2a test_afhds2a.c ../src/afhds2a.c
./test_afhds2a
```

40 checks. Among them: the frame geometry, because the two reference
implementations disagree about it and the wrong number produces a transmitter no
receiver answers; a sweep of 4000 seeds where every derived hop set must be legal
and none may exhaust the retry guard; and that the hop table lands at bytes
11..26 of the bind frame, which is the mechanism that makes the derivation a
local choice rather than something to reverse engineer.

There is no OpenWrt package here on purpose. Which host drives the radio is not
decided — a microcontroller over USB is the better answer than the router's own
SPI, and in that case this code is not built for the router at all.

**The radio layer is unwritten and unverifiable here.** No A7105 on the bench
means the register table, the calibration sequence, the bind handshake and the
3.85 ms timing are all untested. The register writes are precisely where a
transmitter fails silently, so do not mistake "the tests pass" for "it
transmits".
