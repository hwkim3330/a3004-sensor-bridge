# Getting this upstream

Two changes, two projects. **They are parallel, not serialised** — an earlier
version of this document claimed the driver change had to land in mt76 first,
and that was wrong.

OpenWrt does pin an mt76 commit rather than developing against it, but it also
carries patches against that pin in `package/kernel/mt76/patches/` and drops
them once the pin catches up. There are ~57 such patches in its history, and the
removal commits say so outright — e40458a2ff, "mt76: remove obsolete patches /
Already included in the last update", by nbd himself. Bumps are frequent:
2026-03-01, 03-05, 03-19, 03-23, 06-24, 07-01.

So the device support can be submitted with the driver change attached as a
numbered patch, which is the ordinary holding pattern, and the patch disappears
at whichever bump includes it. Waiting for mt76 to merge first is allowed but
buys nothing.

## Branch map

| branch | what it is | for |
|---|---|---|
| `hwkim3330/mt76` `mt7615-dbdc-dt` | 14 lines in `mt7615/eeprom.c` | **PR 1**, to `openwrt/mt76` |
| `hwkim3330/openwrt` `upstream/a3004ns-m` | device support + the driver change as a local patch | **PR 2**, to `openwrt/openwrt` — submit this one |
| `hwkim3330/openwrt` `upstream/ramips-a3004ns-m` | device support alone, 3 files | PR 2 **only if** mt76 has already landed and been pinned |
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

Submit the `upstream/a3004ns-m` variant, which is these three files **plus**
`package/kernel/mt76/patches/100-mt7615-allow-forcing-DBDC-from-device-tree.patch`.
Without the driver change the port still builds and still works on 2.4 GHz, but
the 5 GHz phy will not appear — which is exactly the reason the original was
closed, so submitting it in that state would repeat the mistake. Carrying the
patch avoids that without waiting on anyone.

Say in the PR description that the patch is a temporary carry, that PR 1 is open
against `openwrt/mt76`, and that the patch should be dropped at the bump which
includes it. The patch header says the same thing, so a maintainer who reads only
the diff still sees it.

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
