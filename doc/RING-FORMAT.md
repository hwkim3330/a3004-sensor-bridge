# `ouster-edge` range-ring wire format

One datagram per lidar revolution, sent to the address given by
`option ring` (default port 7602). Everything is little-endian, matching the
sensor's own byte order. Total length is `20 + 3 × sectors` bytes — 1100 bytes
at the default 360 sectors, which stays inside a 1500-byte MTU.

```
offset  size          field
------  ------------  -----------------------------------------------------
     0             4  magic, ASCII "OSED"
     4             1  format version, currently 1
     5             1  source profile: 1=LEGACY 2=SINGLE 3=LOWRATE 4=DUAL
     6             2  sectors, uint16
     8             2  frame_id, uint16 (as reported by the sensor)
    10             1  zone_alarm: 1 if any configured zone is occupied
    11             1  reserved, zero
    12             8  timestamp of the revolution's first column, ns, uint64
    20   2 × sectors  minimum range per sector, uint16, centimetres
                      0xFFFF means no return in that sector
20+2×S   1 × sectors  reflectivity of the nearest return per sector, uint8
```

Sector `i` covers azimuth `[i/S × 360°, (i+1)/S × 360°)`, with sector 0 at the
sensor's own azimuth zero and increasing in the direction its `measurement_id`
increases.

Centimetres rather than millimetres so that 16 bits still spans the full 655 m
the range field can express, with no scaling ambiguity.

The value is the **minimum** range over the channel rows selected by
`option channel_band`, not a single row. That makes it an obstacle envelope
rather than a horizontal slice: anything intruding into the selected elevation
band shows up regardless of its height within that band.

## The same data over HTTP

`ouster-edge` also writes `/var/run/ouster-edge.json` every
`status_interval` ms (default 200), which is what the on-router dashboard
polls. It carries the ring as `ring_cm`, using `-1` instead of `0xFFFF` for
"no return":

```json
{
	"profile": "RNG19_RFL8_SIG16_NIR16",
	"channels": 64,
	"columns_per_packet": 16,
	"packet_size": 12544,
	"scan_width": 1024,
	"sectors": 360,
	"frame_id": 41233,
	"packets": 1846720,
	"bytes": 23165347840,
	"frames": 28855,
	"relayed": 1846720,
	"bad_size": 0,
	"invalid_columns": 0,
	"missed_columns": 0,
	"zone_alarm": false,
	"zones": [false],
	"ring_cm": [1247, 1251, -1, 903, ...]
}
```

`missed_columns` is the number that matters when tuning: it counts gaps in the
sensor's `measurement_id` sequence, so it is a direct measure of packets the
router failed to keep up with. It should stay at zero. If it climbs, run
`sensor-lan-tune` and check `ARCHITECTURE.md` on fragment reassembly.

## Consuming it

`pc-side/ring_to_laserscan.py` turns the datagram into a
`sensor_msgs/LaserScan`, which is the message this data already is — a single
ring of ranges at fixed angular increments. Nothing is lost in that conversion.
