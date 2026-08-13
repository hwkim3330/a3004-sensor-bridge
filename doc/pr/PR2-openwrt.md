# PR 2 — openwrt/openwrt

**Target:** `openwrt/openwrt`, branch `main`
**Branch:** `hwkim3330/openwrt` → `upstream/a3004ns-m`
**Shape:** two commits, four files, +248

1. `mt76: allow forcing DBDC from the device tree` — the driver change carried as
   a numbered patch, authored and signed by me
2. `ramips: add support for ipTIME A3004NS-M` — **authored by Sungbo Eo**, his
   `Signed-off-by` intact, with a bracketed note recording the rebase

---

## Title

```
ramips: add support for ipTIME A3004NS-M
```

## Body

This revives [#4915](https://github.com/openwrt/openwrt/pull/4915), which added
this device in 2021 and was closed unmerged by its author. The reason it was
closed is fixed here, so the port is proposed again rather than left in a fork.

@mans0n wrote the device support and the commit is still his, with his
`Signed-off-by` intact; the bracketed note in it records what the rebase changed.
He closed the original because the MT7615D on this board is wired for DBDC while
its factory EEPROM reports `MT_EE_DUAL_BAND` at 0x3e, so mt76 registered a single
dual-band phy and only 2.4 GHz worked. In his words there was *"no way to
override it in dts"*, and he would not merge it with the debugfs workaround — "a
major flaw".

The first commit is that override, carried as
`package/kernel/mt76/patches/100-mt7615-allow-forcing-DBDC-from-device-tree.patch`.
The same change is proposed against
[openwrt/mt76](https://github.com/openwrt/mt76) in a parallel pull request; this
patch should be dropped at whichever `mt76: update to Git HEAD` bump includes it,
and the patch header says so. Carrying it rather than waiting keeps the two
reviews independent — without it the port still builds and still works on
2.4 GHz, which is the state that got the original closed.

### Rebased onto current conventions

- `nvmem-layout` with `fixed-layout` rather than a bare `nvmem-cells` partition
- a `mac-base` cell instead of `mac-address-increment`
- LED nodes without the deprecated `label` property
- no `dsa-migration`: this device never shipped a swconfig release, so there is no
  configuration to migrate from and adding it would only force `-F` on users
- the WLAN LED gained `linux,default-trigger = "phy0radio"`; without a trigger it
  never lit. a3002mesh and a3004t are the same chip driving the same GPIO 17 for
  the same purpose and both use it

`label-mac-device` is deliberately absent. The stock firmware used the un-offset
u-boot value for its LAN address, which under OpenWrt belongs to the 5 GHz phy
while LAN is that value + 3 — so it is likely no netdev carries the label MAC.
Nobody has read the sticker on the underside to confirm it, so rather than guess
the property is left out; happy to add it if a maintainer would prefer a
specific interface named.

### Tested on hardware

ipTIME A3004NS-M, kernel 6.18.41. Flashed through the stock web interface, then
sysupgraded to the persistent image; everything below survived a power-cycle
reboot with the overlay on flash (`/dev/mtdblock6`, jffs2).

| | result |
|---|---|
| Wi-Fi | `phy0` 2412 MHz and `phy1` 5180 MHz, both Master **at the same time** — ch1 HT20 and ch36 VHT80 |
| a 5 GHz client | 866.7 Mbit/s, VHT-MCS 9, 80 MHz, VHT-NSS 2, took a DHCP lease |
| antenna split | `0x34` = `0x44` → phy0 `0x3`, phy1 `0xc`, i.e. 2x2 + 2x2 |
| EEPROM 0x3e | `0x0a`, matching what was dumped on this model in 2021 |
| Ethernet | lan1 links at 1000 Mbps; lan2-4 and wan correctly down when unplugged |
| USB 3.0 | a UVC camera enumerates at 5000 Mbps |
| LEDs, buttons | CPU LED on boot, WLAN LED follows phy0, reset and WPS both report |
| flash layout | partitions tile 0→16 MiB exactly; `IMAGE_SIZE` 16128k == the firmware partition |
| driver log | no mt76 or MCU errors |

MAC assignment, matching the table in the commit message:

| | measured | rule |
|---|---|---|
| LAN | `70:5d:cc:77:4c:03` | u-boot 0x1fc20 **+3** |
| WAN | `70:5d:cc:77:4c:01` | u-boot 0x1fc40 |
| WLAN 5 GHz | `70:5d:cc:77:4c:00` | factory 0x4 |
| WLAN 2.4 GHz | `72:5d:cc:77:4c:03` | LAN with the local bit set |

### Installation

Flash the **initramfs** image through the stock web interface, boot it, then
`sysupgrade` with the squashfs image. Reverting is a `sysupgrade` with an official
ipTIME image (`a3004nm_kr_*.bin`), which carries the same `a3004nm` uImage name
the stock updater checks.

### Release notes

> ramips: add support for the ipTIME A3004NS-M (MT7621A, 256 MiB RAM, 16 MiB
> SPI NOR, 5x 1GbE, MT7615D DBDC Wi-Fi 5, 1x USB 3.0). Requires the accompanying
> mt7615 change to bring up both bands; without it only 2.4 GHz is available.

---

## Checks

- both commits: subject under 60 chars, no body line over 100, no CRLF, no merge
  commits, `Signed-off-by` present
- author and signer match on my commit, and the address is the one linked to the
  GitHub account; @mans0n's commit keeps his authorship and sign-off
- `checkpatch.pl --strict` reports only known false positives for this kind of
  change: trailing whitespace inside an added `.patch` file (a merged commit,
  f95def2098, produces seven of the same), a MAINTAINERS file this tree does not
  have, and a >75-char line in the original commit body, which 18 of the last 40
  merged `ramips: add support` commits also exceed
- built for `ramips/mt7621` with the device profile selected; image is 70% of the
  firmware partition
