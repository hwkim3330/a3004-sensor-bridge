# Why the A3004NS-M port was never merged, and what fixes it

## The history

Support for this board was written in January 2022 by mans0n (Sungbo Eo) as
[openwrt/openwrt#4915](https://github.com/openwrt/openwrt/pull/4915). It was
never merged. The author marked it draft in September 2022 and closed it in
June 2023 with:

> The problem is, A3004NS-M advertises itself as a non-DBDC device (I mean,
> single 4x4 MIMO dual-band phy device) in EEPROM and there is no way to
> override it in dts…at least at the moment I wrote this patch.
> And I consider it as a major *flaw*, so I'm not going to merge this before I
> can provide (or find) a *proper* fix.

So the port itself was fine. One driver-level defect held it back, and nobody
came back to it.

## The defect

`mt7615_eeprom_parse_hw_band_cap()` in mt76 decides the band configuration
from a single EEPROM field — offset `0x3e` (`MT_EE_WIFI_CONF`), bits 5:4
(`MT_EE_NIC_WIFI_CONF_BAND_SEL`):

| value | meaning |
|---|---|
| 0 | `MT_EE_DUAL_BAND` — one phy covering both bands |
| 1 | `MT_EE_5GHZ` |
| 2 | `MT_EE_2GHZ` |
| 3 | `MT_EE_DBDC` — two independent phys |

Only value 3 sets `dev->dbdc_support`, which is what gates
`mt7615_register_ext_phy()` and the DBDC calibration cache in
`mt7615_mcu_init()`.

On this board the field reads `0x0a`, so bits 5:4 are `00`:

```
root@OpenWrt:~# hexdump -e '1/1 "%02x" "\n"' -s $((0x3e)) -n 1 /dev/mtd2
0a
```

The hardware is a MT7615D wired for DBDC — 2×2 on 2.4 GHz plus 2×2 on 5 GHz —
but the EEPROM describes it as a single dual-band 4×4 phy. ipTIME's own driver
never looks: it turns DBDC on unconditionally, so the mismatch is invisible in
stock firmware. Under mt76 you get one phy that can be tuned to either band,
and half the radio sits idle.

The known workaround was runtime-only:

```sh
echo 1 > /sys/kernel/debug/ieee80211/phy0/mt76/dbdc
```

which is a poor fit for a device that should just come up correctly, and is
presumably why the author would not merge it.

## The fix

`package/kernel/mt76/patches/100-mt7615-allow-forcing-DBDC-from-device-tree.patch`
adds a `mediatek,dbdc` boolean to the wifi node and consults it right after the
EEPROM read:

```c
	val = FIELD_GET(MT_EE_NIC_WIFI_CONF_BAND_SEL,
			eeprom[MT_EE_WIFI_CONF]);

	if (of_property_read_bool(dev_of_node(dev->mt76.dev), "mediatek,dbdc"))
		val = MT_EE_DBDC;

	switch (val) {
	...
```

and the device tree declares it:

```dts
&pcie0 {
	wifi@0,0 {
		compatible = "mediatek,mt76";
		reg = <0x0000 0 0 0 0>;
		nvmem-cells = <&eeprom_factory_0>;
		nvmem-cell-names = "eeprom";
		mediatek,dbdc;
	};
};
```

Because the override happens before the switch, everything downstream follows
the normal DBDC path rather than a special case: `dbdc_support` is set at probe
time, `mt7615_mcu_init()` applies the DBDC calibration cache, and
`mt7615_register_ext_phy()` registers band 1 with
`mphy->chainmask = dev->chainmask & ~dev->mphy.chainmask`. With the EEPROM
reporting a 4-stream part that splits as 2×2 + 2×2, which is what the chip is.

Devices that do not set the property are bit-for-bit unaffected.

## Verifying it on hardware

```sh
# two phys, not one
ls /sys/class/ieee80211/
# expect phy0 and phy1

# phy0 = 2.4 GHz, phy1 = 5 GHz
iw phy phy0 info | grep -A2 Frequencies | head
iw phy phy1 info | grep -A2 Frequencies | head

# both radios in UCI
uci show wireless | grep -E 'radio[01]\.(band|path)'

# and the antenna split
iw phy phy0 info | grep -i 'antenna'
iw phy phy1 info | grep -i 'antenna'
```

`sensor-probe` also reports this at the bottom of its output.

If only `phy0` appears, the property is not reaching the driver. Check that the
patch actually applied:

```sh
dmesg | grep -i mt7615
strings /lib/modules/*/mt7615-common.ko | grep mediatek,dbdc
```

## Upstreaming

The two commits are kept separate on purpose:

1. `mt76: allow forcing DBDC from the device tree` — belongs upstream in
   [openwrt/mt76](https://github.com/openwrt/mt76), and the DT property should
   be documented in the kernel's `mediatek,mt76.yaml` binding at the same time.
2. `ramips: add support for ipTIME A3004NS-M` — the port itself, which becomes
   mergeable once (1) lands. It is mans0n's original work rebased onto current
   tree conventions (`nvmem-layout` with `fixed-layout`, `mac-base` cells,
   modern LED node naming, no `dsa-migration` since this device never shipped a
   swconfig release).

Credit for the port and for diagnosing the EEPROM cause goes to mans0n and to
ptpt52, who identified the `0x3e` bits 5:4 encoding in the PR discussion.
