# Emulator

Boot the sensor bridge under QEMU and check that first boot works, without
touching the router.

The point is iteration cost. Flashing is not slow in itself, but each round trip
through a sysupgrade to find out that a uci-defaults script had a typo is a bad
trade. Everything that is userspace can be found here in a couple of minutes.

```sh
python3 run-emu.py                 # one boot, run the checks
python3 run-emu.py --radios 2      # plus the two-radio config path
python3 run-emu.py --camera        # plus a real UVC camera off this host
python3 run-emu.py --all           # everything this host can currently offer
python3 run-emu.py --loop 5        # repeat, to catch init-ordering flakiness
python3 run-emu.py --shell         # interactive console, ctrl-a x to leave
python3 run-emu.py -v              # stream the guest console
```

Needs `qemu-system-mips` and a malta/le image:

```sh
cat > .config <<'EOF'
CONFIG_TARGET_malta=y
CONFIG_TARGET_malta_le=y
CONFIG_TARGET_malta_le_DEVICE_default=y
CONFIG_PACKAGE_ip-full=y
CONFIG_PACKAGE_a3004-sensorkit=y
CONFIG_PACKAGE_luci=y
CONFIG_PACKAGE_kmod-can-vcan=y
CONFIG_PACKAGE_kmod-mac80211-hwsim=y
CONFIG_PACKAGE_wpad-basic-mbedtls=y
CONFIG_PACKAGE_kmod-usb-xhci-pci=y
CONFIG_PACKAGE_kmod-usb-ohci-pci=y
CONFIG_PACKAGE_kmod-video-uvc=y
CONFIG_PACKAGE_kmod-usb-audio=y
CONFIG_TARGET_ROOTFS_INITRAMFS=y
EOF
make defconfig && make -j$(nproc)
```

### If `a3004-sensorkit` will not stay selected

`make defconfig` drops it silently and the build then produces an image with no
dashboard and no uci-defaults, which looks like a code failure and is not one.
Two causes, both worth knowing:

- **Switching targets invalidates the package metadata.** The first `defconfig`
  after changing `CONFIG_TARGET_*` regenerates `tmp/.packageinfo` and drops every
  package selection it could not yet resolve. Run `defconfig`, then append the
  package lines, then run it again.
- **`kmod-video-core` is a hard dependency and is not implied.** The generated
  entry in `tmp/.config-package.in` carries `depends on PACKAGE_kmod-video-core`
  and `depends on USB_SUPPORT` alongside a pile of `select`s, and a `depends on`
  is not satisfied by selecting the package that needs it. Set
  `CONFIG_PACKAGE_kmod-video-core=y` explicitly.

When a selection vanishes, read the entry rather than guessing:

```sh
awk '/config PACKAGE_a3004-sensorkit$/,/^$/' tmp/.config-package.in
```

## Why malta/le

`malta/le` is `mipsel` + `24kc` — the same architecture triple as
`ramips/mt7621`. The packages it boots are the *same binaries*: same endianness,
same soft-float, same alignment rules, real procd, real uci, real uhttpd,
real ustreamer. The toolchain is shared with the ramips build, so switching
targets rebuilds the kernel and not much else.

## What it checks

- every `uci-defaults` script ran and left the values it should. A script that
  fails is renamed rather than deleted, so its absence is the only evidence it
  worked.
- the firewall rule was installed
- `uhttpd` and `ouster-edge` are actually running after procd settles
- the dashboard answers on port 80 and its status symlink resolves
- synthetic OS-64 packets, injected from the host over a UDP forward, are counted
  by the daemon and complete revolutions
- an SSE event is pushed *while* those packets flow, and carries a ring
- `first-boot-report` runs to the end
- nothing segfaulted, OOMed or hit a kernel BUG

With `--radios 2`, using `mac80211_hwsim`:

- two phys exist, and the sensorkit's uci-defaults configure **both** of them.
  This is the only way to exercise that branch: on the real board it is only
  reached if the DBDC fix works, so without this it was untested code.
- no "only one radio present" warning is emitted when two are there

With `vcan` in the guest (always, if the module is available):

- `can-up` brings a virtual interface up, `can-bridge` starts on mipsel, and a
  `BCAN` datagram sent from the host reaches the bus through it with nothing
  rejected. The receive direction is deliberately not checked here - see below.

With `--camera`, if a UVC device is attached to this host, it is passed through
with `usb-host` on an emulated xHCI, and the guest's own `uvcvideo` + `ustreamer`
are made to produce a JPEG. There is no UVC device model in QEMU, so passthrough
is the only way to make that path do real work.

## What it cannot tell you

Do not let this stand in for the board:

- **nothing about the device tree, mt76 or DBDC.** QEMU has no MT7615D. Whether
  two phys appear is the one question that matters and this cannot answer it.
- whether the image flashes, or whether u-boot accepts the uImage name.
- real USB, a real camera, real lidar throughput, or timing under load. There is
  no sound card either, which is why `mic-stream` logs a missing device on every
  run — expected, and the reason it now retries instead of exiting.

## Comparing two builds: `oe-flagbench.py`

There is one thing the emulator answers that the host cannot: whether a compiler
flag helps *on this architecture*.

```sh
CC=staging_dir/toolchain-mipsel_24kc_gcc-*/bin/mipsel-openwrt-linux-musl-gcc
F="-pipe -mno-branch-likely -mips32r2 -mtune=24kc"
mkdir -p /tmp/bench
$CC -Os $F -o /tmp/bench/mips-Os package/keti/ouster-edge/src/ouster-edge.c
$CC -O2 $F -o /tmp/bench/mips-O2 package/keti/ouster-edge/src/ouster-edge.c
$CC -Os $F -o /tmp/bench/oe-inject package/keti/emu/oe-inject.c
python3 oe-flagbench.py --bindir /tmp/bench
```

It boots the guest, pulls the binaries in over HTTP from this host
(`192.168.1.2`, which is what QEMU's user-mode network calls it), runs each
build against a fixed number of datagrams from `oe-inject` **inside the guest**,
and reports CPU by subtraction between two counts so that startup and config
parsing fall out.

Read the caveats in `doc/COMPUTE.md` before trusting a number from it. TCG has no
cache or pipeline model, and the user/sys split is tick-sampled and noisy — the
totals are the part worth comparing. What it is good for is refuting a claim, and
it has already done that once: an `-O2` override for `ouster-edge`, which was 23%
cheaper per datagram on x86, turned out to be worth nothing on mipsel.

## Two things that look like bugs and are not

**`rx` stays 0 in the CAN check.** A raw CAN socket does not receive its own
transmissions unless `CAN_RAW_RECV_OWN_MSGS` is set, so a single bridge cannot
see the frame it injected. Setting that flag would make the relay echo its own
injections to the network peer - a worse product for a better-looking test. The
receive path, batching and tracked-id decoding are covered against `vcan` on the
host in `can-bridge/test/test_bridge.py`.

**`--ibus` does nothing.** QEMU really does present the device - the monitor
shows `Product QEMU USB Serial` on `ohci.0`, and `info qtree` lists it - but
malta's OHCI in QEMU 8.2 never enumerates it, so the guest has no `/dev/ttyUSB0`.
The flag is kept because it may work on another machine type or QEMU version. It
is reported, not failed: `rc-ibus` is covered byte for byte over a pty on the
host at the same compile target, so the gap is in the bench, not the code.

## Notes for whoever edits this

Three harness bugs were worked through here, all the same species — the test
modelling the system wrongly and blaming the system:

1. **One newline does not wake the console.** The banner prints while the kernel
   is still emitting module output; a single Enter at that moment is lost. It
   nudges repeatedly now.
2. **The shell's echo is not the answer.** `echo SETTLED` contains the string
   `SETTLED`, so matching on it passed instantly against the echoed command and
   reported every service as down. Output is now framed by a sentinel the command
   text cannot contain (`$((7*11))` in, `77` out), with echo turned off and the
   console widened so long lines do not wrap mid-parse.
3. **A heredoc through `cmd()` wedges the shell.** Sending multi-line input broke
   the output framing and left the shell waiting for a terminator, after which
   every later command returned nothing. `cmd()` now asserts a single line.
4. **Reading the same status file three times gives three instants.** `rx=0` sat
   next to a decoded frame, which cannot both be true. One snapshot, parsed once.
5. **QEMU's default guest IP is not this guest's IP.** `br-lan` is statically
   192.168.1.1 and never asks for DHCP, so forwards aimed at 10.0.2.15 went
   nowhere. The user network is configured for 192.168.1.0/24 and forwards are
   addressed explicitly.

And one about the thing being tested rather than the harness: an SSE event only
exists when a revolution completes, so the probe has to be listening while
packets flow. Connecting afterwards correctly gets headers and silence.

## What this exercise actually found

Three real defects, which is the return on building it:

- `mic-stream` exited when the capture device was absent, and procd respawned it
  immediately — a crash loop at *0 seconds since last crash*. On the router that
  is simply "the camera is not plugged in yet". It retries with a backoff now,
  says so once rather than flooding the log, and only claims the device is back
  when samples actually arrive.
- `ouster-edge`'s init script logged `sensor <ip> reports 64ch x 16col ...` when
  no sensor was reachable at all. `eval "$(cmd)"` returns the status of the
  string it evaluated, so a failing command evaluates `""` and succeeds. That is
  a lie at precisely the moment someone is trying to find out whether the sensor
  is talking.

- `can-up` could not work on the router at all. It configures the interface with
  `ip link set ... type can bitrate ...`, and BusyBox's `ip` rejects that
  outright with *"either dev is duplicate, or type is garbage"*. `can-bridge` now
  depends on `ip-full`, `can-up` checks for a capable `ip` and says which package
  to install, and a virtual interface skips the bitrate step entirely - which
  also means `can-bridge` can be exercised against `vcan` on the router itself.
  Note that `ip-full` is a non-default variant, so it has to be selected
  explicitly or the build stops at `package/install`; `BRINGUP.md` says so.

None of the three would have been visible from reading the code, and all three
would have cost bench time.
