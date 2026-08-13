# PR 1 — openwrt/mt76

**Target:** `openwrt/mt76`, branch `master`
**Branch:** `hwkim3330/mt76` → `mt7615-dbdc-dt`
**Shape:** one commit, `mt7615/eeprom.c`, +14 lines

---

## Title

```
wifi: mt76: mt7615: allow forcing DBDC from the device tree
```

## Body

`mt7615_eeprom_parse_hw_band_cap()` decides the band configuration entirely from
`MT_EE_WIFI_CONF` (EEPROM offset 0x3e, bits 5:4). Some boards are wired for DBDC
but ship an EEPROM that still reports `MT_EE_DUAL_BAND` there. Their vendor
driver turns DBDC on unconditionally and never reads the field, so the mismatch
is invisible in stock firmware; under mt76 the board registers a single dual-band
phy and only one band is usable at a time.

This adds a `mediatek,dbdc` boolean so the device tree can correct the EEPROM.
The override is applied before the switch, so everything downstream takes the
ordinary DBDC path — `dbdc_support` at probe, the DBDC calibration cache in
`mt7615_mcu_init()`, and `mt7615_register_ext_phy()` for band 1. Devices that do
not set the property are bit-for-bit unaffected.

### Why not one of the alternatives

- **Rewrite the EEPROM bytes on the way to the driver.** There is no mechanism to
  modify an nvmem cell in flight, and editing the factory partition on the device
  would be destructive.
- **Enable DBDC from userspace via the existing debugfs knob.** This works and is
  what affected boards do today, but the device still comes up wrong on every
  boot and hardware description ends up in a startup script.
- **A new compatible string.** The chip is not different; only the board's EEPROM
  contents are, which is what device tree properties describe.

### Where this came from

The affected device is the ipTIME A3004NS-M (MT7615D). In
[openwrt/openwrt#4915](https://github.com/openwrt/openwrt/pull/4915) @ptpt52
identified the encoding of this field and @mans0n dumped it on the hardware:

```
root@OpenWrt:~# hexdump -e '1/1 "%02x" "\n"' -s $((0x3e)) -n 1 /dev/mtd2
0a
```

`0x0a` is band_sel 0, `MT_EE_DUAL_BAND`. That pull request added the device and
was then closed by its author because, in his words, *"there is no way to
override it in dts"* — he considered shipping the debugfs workaround "a major
flaw". This is that override.

### Tested on hardware

ipTIME A3004NS-M, ramips/mt7621, kernel 6.18.41:

| | result |
|---|---|
| `0x3e` on this unit | `0x0a` → band_sel 0, as reported in 2021 |
| without the property | one phy |
| with the property | `phy0` at 2412 MHz, `phy1` at 5180 MHz |
| both bands at once | `A3004-24G` ch1 HT20 and `A3004-5G` ch36 VHT80, both Master, simultaneously |
| a client on 5 GHz | associated at 866.7 Mbit/s, VHT-MCS 9, 80 MHz, **VHT-NSS 2** |
| chainmask | `0x34` = `0x44` → phy0 `0x3`, phy1 `0xc`, i.e. 2x2 + 2x2 |
| driver log | no MCU or calibration errors; `phy1: copying sband (band 1)` |

The `VHT-NSS 2` on a live association is worth noting: it confirms the second
phy is running two chains, not that a flag was merely set.

### Follow-up

If this is accepted, `mediatek,dbdc` should also be documented in
`Documentation/devicetree/bindings/net/wireless/mediatek,mt76.yaml` in the kernel
tree, which this repository does not carry. Happy to send that separately.

---

## Checks

- `checkpatch.pl --strict`: 0 errors, 0 warnings
- one commit, no merge commits, `Signed-off-by` present
- author and signer are the same address, and it is the one linked to the GitHub
  account
