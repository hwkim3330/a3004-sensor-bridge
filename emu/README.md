# Emulator

Boot the sensor bridge under QEMU and check that first boot works, without
touching the router.

The point is iteration cost. Flashing is not slow in itself, but each round trip
through a sysupgrade to find out that a uci-defaults script had a typo is a bad
trade. Everything that is userspace can be found here in a couple of minutes.

```sh
python3 run-emu.py                 # one boot, run the checks
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
CONFIG_PACKAGE_a3004-sensorkit=y
CONFIG_PACKAGE_luci=y
CONFIG_TARGET_ROOTFS_INITRAMFS=y
EOF
make defconfig && make -j$(nproc)
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

## What it cannot tell you

Do not let this stand in for the board:

- **nothing about the device tree, mt76 or DBDC.** QEMU has no MT7615D. Whether
  two phys appear is the one question that matters and this cannot answer it.
- whether the image flashes, or whether u-boot accepts the uImage name.
- real USB, a real camera, real lidar throughput, or timing under load. There is
  no sound card either, which is why `mic-stream` logs a missing device on every
  run — expected, and the reason it now retries instead of exiting.

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
3. **QEMU's default guest IP is not this guest's IP.** `br-lan` is statically
   192.168.1.1 and never asks for DHCP, so forwards aimed at 10.0.2.15 went
   nowhere. The user network is configured for 192.168.1.0/24 and forwards are
   addressed explicitly.

And one about the thing being tested rather than the harness: an SSE event only
exists when a revolution completes, so the probe has to be listening while
packets flow. Connecting afterwards correctly gets headers and silence.

## What this exercise actually found

Two real defects, which is the return on building it:

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

Neither would have been visible from reading the code, and both would have cost
bench time.
