# Getting this upstream

Two changes, two projects, in this order. The order is not optional: OpenWrt
does not carry patches for mt76 in its tree — it pins a mt76 commit and bumps it
— so the driver change has to land in mt76 first, and only then can the device
support be submitted without a patch attached.

## Branch map

| branch | what it is | for |
|---|---|---|
| `hwkim3330/mt76` `mt7615-dbdc-dt` | 13 lines in `mt7615/eeprom.c` | **PR 1**, to `openwrt/mt76` |
| `hwkim3330/openwrt` `upstream/ramips-a3004ns-m` | 3 files, 200 insertions | **PR 2**, to `openwrt/openwrt`, after PR 1 lands |
| `hwkim3330/openwrt` `upstream/a3004ns-m` | both, with the driver change as a local patch | works today, not for submission |
| `hwkim3330/openwrt` `iptime-a3004ns-m` | everything, including the KETI sensor bridge | day-to-day work |

Both PR branches were checked against their upstream: each is **1 commit ahead,
0 behind**, and touches only the files it should.

## PR 1 — openwrt/mt76

```
https://github.com/hwkim3330/mt76/tree/mt7615-dbdc-dt
```

One commit, `mt7615/eeprom.c`, +13 lines. It adds a `mediatek,dbdc` boolean that
lets the device tree override the EEPROM's band configuration.

The commit message states the three alternatives that were considered and why
each was rejected, because that is the first thing a reviewer will ask. In
particular: enabling DBDC through the existing debugfs knob does work, and is
what affected boards do today, so "why not just do that" needs an answer up
front — the answer being that the device still comes up wrong on every boot and
hardware description ends up in a startup script.

Expect one follow-up request: `mediatek,dbdc` should be documented in
`Documentation/devicetree/bindings/net/wireless/mediatek,mt76.yaml`, which lives
in the Linux kernel rather than in mt76. That is a separate, small kernel patch,
and the mt76 commit message already flags it.

## PR 2 — openwrt/openwrt

```
https://github.com/hwkim3330/openwrt/tree/upstream/ramips-a3004ns-m
```

One commit, three files: the DTS, the `mt7621.mk` recipe, and the MAC-fixup
hotplug case. This is Sungbo Eo's original submission
([openwrt#4915](https://github.com/openwrt/openwrt/pull/4915)) rebased, and the
commit is **authored by him** with his `Signed-off-by` intact — he wrote it, and
the rebase does not change that. A second `Signed-off-by` and a bracketed note
record what the rebase changed and why the port is now mergeable.

Do not open this until PR 1 is merged and OpenWrt has bumped its mt76 pin.
Without the driver change the port still builds and still works on 2.4 GHz, but
the 5 GHz phy will not appear, which is exactly the reason the original was
closed — submitting it again in that state would be repeating the mistake.

## About the sign-offs

`Signed-off-by:` is the Developer Certificate of Origin: it is a statement by the
person named that they have the right to submit the work under the project's
licence. The lines on these branches say `hwkim3330 <hwkim3@keti.re.kr>`. Check
you are content to make that statement, and change the name if the submission
should go out under a different one — `git rebase -i` and `git commit --amend -s`
is all it takes.

## Before opening either PR

- [ ] Flash the board and confirm two phys appear. Everything above is verified
      by building and reading; **none of it is verified on the hardware**, and
      submitting an unflashed device port is how the first attempt got stuck.
      `package/keti/doc/BRINGUP.md` step 3 is the check.
- [ ] Read this board's EEPROM and confirm the antenna split. `MT_EE_NIC_CONF_0`
      decides the chainmask, so 2×2 + 2×2 is expected but unread:
      `hexdump -C /dev/mtd2 | head -8`
- [ ] Rebase both branches on their upstream's current head, so CI builds
      against the tree as it is on the day you submit.
- [ ] Note in the PR description that the device has been tested on hardware,
      and by whom. Reviewers ask.
- [ ] Consider adding the device to the
      [OpenWrt hardware table](https://openwrt.org/toh/start) once it works.
      That is a wiki edit, not part of the PR.

## What CI will do

`openwrt/openwrt`'s `build-pr-profile` workflow builds the device profile the PR
touches. The same build was run here on the exact PR branch, with a config
containing nothing but the target and this device, which is what CI does.
