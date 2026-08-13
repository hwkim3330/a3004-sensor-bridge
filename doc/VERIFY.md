# Verification gate

`TOMORROW.md` is the procedure: what to plug in, in what order. This is the
gate: every claim this tree currently makes, and the one measurement that
settles it. A claim with nothing in its evidence column has not been verified,
however confident the prose around it sounds.

The point is not tidiness. Two upstream PRs are waiting on this, and a PR whose
commit message asserts something nobody measured wastes a maintainer's time and
is hard to walk back.

## What is already verified, and by whom

Worth writing down because it is easy to re-derive badly. This did not come from
reasoning about the driver — it is in the PR thread, measured on real hardware by
the person who wrote the original port.

| Fact | Source |
|---|---|
| `0x3e` bits 5:4 encode band config: `00` DUAL_BAND, `01` 5G, `10` 2G, `11` DBDC | ptpt52, openwrt/openwrt#4915 |
| A3004NS-M reads **`0x0a`** at `0x3e` → band_sel `0` → DUAL_BAND | mans0n, same thread, `hexdump -s $((0x3e)) -n 1 /dev/mtd2` on the device |
| The port was closed because *"there is no way to override it in dts"* | mans0n, same thread, closing comment |
| No prior `mediatek,dbdc` exists; no mt76 PR proposes a DT DBDC override | searched openwrt/mt76, 0 hits |

So the mt76 patch is the fix the original author said he was waiting for. What is
*not* yet verified is that this particular unit matches, and that the override
actually produces two working phys rather than just setting a flag.

## What the bench no longer has to establish

Settled off-hardware, so do not spend bench time on it:

| Thing | How |
|---|---|
| Both patches are submission-clean | `checkpatch.pl --strict`: 0 errors, 0 warnings on the mt76 commit. The remaining reports on the OpenWrt side are false positives, each confirmed against a merged commit — trailing whitespace fires on any commit that adds a `.patch` file (merged f95def2098 produces 7), MAINTAINERS does not exist in this tree, and >75-char commit lines are not OpenWrt's convention (18 of 40 merged `ramips: add support` commits exceed it) |
| Author and signer agree | was `hwkim3330@gmail.com` vs a `hwkim3@keti.re.kr` sign-off; 58 of 60 merged commits match, so it mattered. Fixed |
| The flashed DTB really carries both properties | decompiled from `image-mt7621_iptime_a3004ns-m.dtb`: `mediatek,dbdc` and `linux,default-trigger = "phy0radio"` |
| Partition arithmetic | `IMAGE_SIZE` 16128k == firmware `0xfc0000`; partitions tile 0→16 MiB exactly; both MAC cells inside u-boot; EEPROM cell inside factory |
| The pins the LEDs and buttons need are actually freed | mt7621 pinctrl `FUNC(name, mode, base, count)`: `i2c`→GPIO 3,4 (wps, reset), `jtag`→13–17 (LED 17), `wdt`→18 (LED 18) |
| USB needs no device-tree work | `xhci` in mt7621.dtsi has no `status`, so it is enabled by default |
| Services start and run on this architecture | `emu/run-emu.py --radios 2` passes end to end on malta/le, same `mipsel_24kc` as the target: uhttpd, ouster-edge, the dashboard, the SSE ring, the two-radio config path, `first-boot-report` |
| CAN inject works on mipsel | same run, with `kmod-can-vcan`: bridge starts on vcan0 and injects, nothing rejected |
| The app speaks the daemon's protocol | `TeleopSender.send()` byte-for-byte against `handle_udp_cmd()`: magic, version, flags, LE sequence, axes at 12/14/16, length 24 |

## Gate A — before opening the mt76 PR

| # | Claim the patch makes | Measurement | Result |
|---|---|---|---|
| A1 | This unit's `0x3e` reads `0x0a`, band_sel `0` | `first-boot-report`, QUESTION 2 | |
| A2 | Without the property, one phy | boot the **stock-mt76** image, `ls /sys/class/ieee80211/` → expect `phy0` only | |
| A3 | With the property, two phys | boot our image, same command → expect `phy0` **and** `phy1` | |
| A4 | Both bands work *at the same time* | bring both up, associate a client to each, ping both concurrently | |
| A5 | `dbdc_support` took the ordinary path, not a half-state | `logread \| grep -i mt7615` shows no MCU/calibration errors | |

**A4 is the one that actually matters.** A phy count proves a flag was set; two
associated clients on different bands at once proves DBDC. Do not report success
on A3 alone.

**If A1 comes back band_sel `3`:** stop. The patch is unnecessary for this board
and the 2023 stall had another cause. Do not open the mt76 PR — find what really
blocks the second phy first.

**If A3 gives two phys but A4 fails:** the patch is incomplete, not wrong. Say
so in the PR rather than omitting it; `chainmask` (A6) is the first suspect.

| # | Claim | Measurement | Result |
|---|---|---|---|
| A6 | The DBDC split is 2x2+2x2, not 1x1+1x1 | `first-boot-report` prints `0x34`; then `iw phy phy0 info \| grep -i "tx.*stream"` on both phys | |

A6 is currently **an assumption stated as expectation** in `first-boot-report`
("Expected 2x2+2x2, never read"). If it turns out 1x1+1x1, no throughput claim
in this tree survives, and `README.md` needs correcting.

## Gate B — before opening the OpenWrt device PR

The mt76 patch does **not** have to be merged first. OpenWrt has carried ~57
mt76 patches in `package/kernel/mt76/patches/` over the years and drops them at
the next `mt76: update to Git HEAD` bump ("Already included in the last
update" — e40458a2ff, nbd). Bumps land every few weeks. So the device PR can
ship with the patch alongside it; the two PRs are parallel, not serialised.

| # | Claim | Measurement | Result |
|---|---|---|---|
| B1 | The stock web UI accepts our initramfs | flash it; it either boots or is refused (refusal is not a brick) | |
| B2 | The image fits and `sysupgrade` is safe | already verified statically: `IMAGE_SIZE` 16128k == firmware partition `0xfc0000`, partitions tile 0→16 MiB exactly | ✅ static |
| B3 | Ethernet: 4 LAN + 1 WAN, correct ports | plug each port in turn, watch `bridge link` / `swconfig`-free DSA names | |
| B4 | MAC addresses match the case sticker | `first-boot-report` QUESTION 3 vs the printed label | |
| B5 | LEDs and buttons are as described | CPU LED on boot, WLAN LED, reset and WPS via `logread` | |
| B6 | USB 3.0 port works | plug the camera, `lsusb` shows it at 5000M | |
| B7 | `sysupgrade` preserves config across a second flash | flash twice, check `/etc/config` survives | |

**B4 retires a deliberate omission.** `label-mac-device` was dropped from the
DTS because nobody had read the sticker. Once B4 is done, either add it back
with the right interface or note in the PR why it stays out.

## Gate C — repo honesty (not PR-blocking)

These are claims in this tree's own docs. They do not gate upstream, but they
should not sit in a public repo as if measured.

| # | Claim | Measurement | Result |
|---|---|---|---|
| C1 | The camera streams at a useful rate on an 880 MHz MIPS | measured MJPEG fps/bitrate at the dashboard, with CPU load | |
| C2 | Lidar keeps up: `missed_columns` stays 0 | `TOMORROW.md` §5, the number to watch | |
| C3 | Zone reflexes fire within the stated latency | already measured on host (1.4 ms zone, 0.2 ms SSE); repeat on target | |
| C4 | Mic capture works through the camera's USB audio | `arecord -l`, then listen | |
| C5 | CAN needs `ip-full`; BusyBox `ip` cannot create the interface | verified on target: BusyBox `ip` rejects `type can` | ✅ |
| C5b | The bridge comes up when the adapter is plugged in *after* boot | plug the USB-CAN adapter into a running router; `logread \| grep can-bridge` should show the hotplug rule starting it | |
| C6 | Teleop deadman actually stops the vehicle | **do this with the wheels off the ground** | |

C6 is the only item here with a physical consequence. Everything else is a
number being wrong; this one is a vehicle moving when it should not.

## Evidence to paste into each PR

- **mt76 PR:** A1 (the byte), A3 and A4 (before/after phy count and a concurrent
  two-band test), plus the mans0n and ptpt52 quotes already in the commit
  message.
- **OpenWrt device PR:** B1, B3, B4, B6, and a line stating that DBDC needs the
  mt76 change, with a link to that PR.

Not "it works for me". The numbers.
