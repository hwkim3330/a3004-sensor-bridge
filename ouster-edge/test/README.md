# `ouster-edge` verification

There is no Ouster sensor on the bench, so the parser is verified against
byte-exact synthesised packets instead. Both scripts are plain Python 3 and
need nothing installed.

```sh
cc -O2 -Wall -Wextra -o ouster-edge ../src/ouster-edge.c
python3 test_profiles.py      # all four UDP profiles, ranges, ring wire format
python3 test_accounting.py    # packet accounting and the missed_columns counter
```

`test_profiles.py` builds packets for LEGACY (12608 B), RNG19_RFL8_SIG16_NIR16
(12544 B), RNG15_RFL8_NIR8 (4352 B) and the dual-return profile (16640 B) at
64 channels x 16 columns, encodes a known range into one channel of every
column, and asserts the published ring reproduces it - including the low-rate
profile's 8 mm range scaling and the 19- versus 20-bit range masks.

`test_accounting.py` answers the two questions that decide whether
`missed_columns` can be trusted as the tuning metric: does the daemon see every
packet sent to it, and does a deliberately dropped packet show up as exactly 16
missed columns. It does.

Note if you run these repeatedly: a daemon left over from an aborted run will
still hold the port and quietly absorb the traffic, which looks like a parser
bug. Check with `pgrep -x ouster-edge` first.
