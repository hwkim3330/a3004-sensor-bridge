# Bringing up the A3004NS-M sensor bridge

## 1. Build

```sh
git clone https://github.com/hwkim3330/openwrt.git -b iptime-a3004ns-m
cd openwrt
cp feeds.conf.default feeds.conf
./scripts/feeds update -a && ./scripts/feeds install -a

cat > .config <<'EOF'
CONFIG_TARGET_ramips=y
CONFIG_TARGET_ramips_mt7621=y
CONFIG_TARGET_ramips_mt7621_DEVICE_iptime_a3004ns-m=y
CONFIG_PACKAGE_a3004-sensorkit=y
CONFIG_PACKAGE_luci=y
CONFIG_PACKAGE_luci-app-ustreamer=y
CONFIG_PACKAGE_tcpdump-mini=y
EOF
make defconfig
make -j$(nproc)
```

Build host needs GNU awk, not mawk (`apt install gawk`) — mawk silently breaks
the feed metadata scan with `function asort never defined`, and the symptom is
feed packages appearing not to exist.

Output lands in `bin/targets/ramips/mt7621/`.

## 2. Flash

The board's u-boot checks the uImage name string, which is why the image is
built with `UIMAGE_NAME := a3004nm`.

1. On stock firmware, use the web interface's firmware upgrade page to flash
   **`...-initramfs-kernel.bin`**.
2. The router boots OpenWrt from RAM. Reach it at `192.168.1.1`.
3. From there, sysupgrade to the real image:

```sh
sysupgrade -n /tmp/openwrt-ramips-mt7621-iptime_a3004ns-m-squashfs-sysupgrade.bin
```

Going back to stock is a normal sysupgrade with the vendor image.

Serial console, if you need it: header **J4**, **57600** baud, pinout
`[3V3] (TXD) (RXD) (GND)`. Do not connect the 3V3 pin.

> If a Gear IconX cradle is plugged into this PC it will take `ttyACM0`; the
> A3004NS-M console is a plain 3.3 V UART and needs its own adapter anyway.

## 3. Verify the DBDC fix first

This is the thing that was broken for four years, so check it before anything
else. Two phys must appear:

```sh
ls /sys/class/ieee80211/          # expect phy0 and phy1
iw phy phy0 info | grep -m1 -A2 Frequencies   # 2.4 GHz
iw phy phy1 info | grep -m1 -A2 Frequencies   # 5 GHz
```

If only `phy0` exists, see `DBDC.md`.

Then set a WiFi password — the firmware ships with the radios enabled but
unconfigured on purpose, because a default password would be worse:

```sh
uci set wireless.default_radio1.ssid='A3004-SENSOR'
uci set wireless.default_radio1.encryption='psk2'
uci set wireless.default_radio1.key='<your password>'
uci commit wireless && wifi reload
```

## 4. Camera

Plug the camera into the USB 3.0 port (a Logitech StreamCam VU0054 is USB-C, so
a USB-C-male to USB-A-male cable, not a hub).

```sh
sensor-probe
```

Read the `VERDICT` line. Everything about the camera path depends on it:

- **"camera offers MJPEG"** — good. `ustreamer` copies frames from USB to the
  network with no encoding. 1280x720@30 is a fine starting point; raise it in
  `/etc/config/ustreamer` and watch CPU.
- **"NO MJPEG"** — the camera only offers uncompressed YUYV/NV12. MT7621 is a
  soft-float 880 MHz MIPS part with no JPEG encoder; it cannot compress 1080p,
  or 720p. Set `format 'YUYV'` and drop `resolution` to `320x240`, expect
  single-digit fps, and treat that as the ceiling. A camera with a hardware
  JPEG encoder is the fix, not tuning.

I could not settle this from documentation. Logitech's spec sheet lists MJPEG,
NV12 and YUY2 for the StreamCam, but there are credible reports of the
StreamCam exposing no MJPEG at all over UVC on Linux, and I have no VU0054 to
test. `sensor-probe` gives the real answer in one line — run it before
committing to a resolution.

```sh
/etc/init.d/ustreamer start
/etc/init.d/ustreamer enable
```

## 5. Lidar

Plug the OS-64 into a **LAN** port. It will not work on a 100 Mbit link and it
does not negotiate one — `sensor-probe` prints the link speeds.

Give it a reservation so it stays findable across power cycles. The MAC has to
come from the sensor itself:

```sh
cat /tmp/dhcp.leases            # find the Ouster's MAC
uci set dhcp.ouster.mac='<mac>' # the host entry already exists, IP 192.168.1.50
uci commit dhcp && /etc/init.d/dnsmasq restart
```

Point the sensor at the router — via its own HTTP API, from any machine on the
LAN:

```sh
curl -X POST http://192.168.1.50/api/v1/sensor/config \
     -H 'Content-Type: application/json' \
     -d '{"udp_dest": "192.168.1.1", "udp_port_lidar": 7502}'
```

Sending to the router rather than straight to the PC is what makes both
consumers possible: a sensor has one destination, so the router receives and
relays. See topology (b) in `ARCHITECTURE.md`.

```sh
uci set ouster-edge.lidar.enabled='1'
uci set ouster-edge.lidar.sensor_ip='192.168.1.50'
uci commit ouster-edge
/etc/init.d/ouster-edge start
/etc/init.d/ouster-edge enable

logread -e ouster-edge      # should report the detected profile and packet size
```

Then tune the network path and confirm nothing is being dropped:

```sh
sensor-lan-tune
# watch for a minute; missed_columns must stay at 0
while :; do sed -n 's/.*"missed_columns": \([0-9]*\).*/\1/p' \
    /var/run/ouster-edge.json; sleep 2; done
```

## 6. Dashboard

`http://192.168.1.1/sensors/` — camera and lidar ring side by side. Join the
router's 5 GHz AP from an Android tablet and open that URL; nothing else is
needed. The page is plain HTML/CSS/JS with no external resources, so it works
with no internet on either side.

## 7. Optional: hand the raw stream to ROS 2

```sh
uci set ouster-edge.lidar.relay='192.168.1.100'      # your ROS 2 machine
uci set ouster-edge.lidar.ring='192.168.1.100:7602'
uci commit ouster-edge && /etc/init.d/ouster-edge restart
```

On that machine, `ouster-ros` consumes the relayed raw stream exactly as if it
were connected directly. For the cheap channel:

```sh
python3 package/keti/doc/pc-side/ring_to_laserscan.py --ros-args \
    -p port:=7602 -p frame_id:=os_sensor
```

which publishes `sensor_msgs/LaserScan` on `~/scan`. See `RING-FORMAT.md`.

## 8. Optional: zones

```sh
uci add ouster-edge zone
uci set ouster-edge.@zone[-1].az_start='330'
uci set ouster-edge.@zone[-1].az_end='30'      # wraps through 0
uci set ouster-edge.@zone[-1].range='2.5'      # metres
uci set ouster-edge.lidar.action='/usr/bin/my-alarm-script'
uci commit ouster-edge && /etc/init.d/ouster-edge restart
```

The script is called as `my-alarm-script alarm` and `my-alarm-script clear`.
Evaluation happens once per revolution, so at 10 Hz the reflex latency is
around 100 ms plus whatever the script costs.

## What has and has not been tested

Built and verified here:

- the port builds clean against current OpenWrt master for
  `ramips/mt7621`, producing initramfs and sysupgrade images
- the mt76 DBDC patch applies and compiles
- `ouster-edge` compiles warning-free with `-Wall -Wextra`, and its packet
  layout arithmetic matches Ouster's documented sizes (12608 B legacy,
  12544 B single-return, 4352 B low-rate, 16640 B dual, all at 64×16)
- the dashboard renders

Not tested, because it needs the hardware:

- flashing, and whether DBDC actually comes up as two phys
- the camera's real format list, and therefore the whole camera performance story
- lidar throughput on real silicon, and whether MT7621 accepts a 9000-byte MTU
- the antenna split the EEPROM reports (2×2+2×2 is expected, but the chainmask
  comes from `MT_EE_NIC_CONF_0` and I have not read this board's EEPROM)
