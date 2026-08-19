#
# Copyright (C) 2026 KETI
#
# This is free software, licensed under the GNU General Public License v2.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=a3004-sensorkit
PKG_VERSION:=1.0
PKG_RELEASE:=8

PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=KETI

include $(INCLUDE_DIR)/package.mk

define Package/a3004-sensorkit
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Sensor bridge for ipTIME A3004NS-M (USB camera + Ouster lidar)
  DEPENDS:= \
	+kmod-usb3 \
	+kmod-video-uvc \
	+ustreamer \
	+v4l-utils \
	+ouster-edge \
	+mic-stream \
	+can-bridge \
	+rc-ibus \
	+teleop \
	+uhttpd
endef

define Package/a3004-sensorkit/description
 Turns the router into a self-contained sensor bridge: a UVC camera on the
 USB 3.0 port is served as MJPEG over HTTP, an Ouster lidar on a gigabit LAN
 port is reduced to a range ring by ouster-edge, the camera's microphone is
 served as uncompressed PCM, and an optional USB-CAN adapter is bridged
 read-only. A dashboard on port 80 shows all of it, so an Android tablet
 joining the router's own WiFi is a complete client with no other machine
 involved.
endef

define Build/Compile
endef

define Package/a3004-sensorkit/install
	$(INSTALL_DIR) $(1)/www/sensors
	$(INSTALL_DATA) ./files/www/index.html $(1)/www/sensors/index.html
	# the daemons write their status to tmpfs; expose them under the docroot
	$(LN) /var/run/ouster-edge.json $(1)/www/sensors/ouster.json
	$(LN) /var/run/can-bridge.json $(1)/www/sensors/can.json
	$(LN) /var/run/rc-ibus.json $(1)/www/sensors/rc.json
	$(LN) /var/run/teleop.json $(1)/www/sensors/teleop.json
	# The map is a binary blob rather than JSON on purpose: 40 kB of cells
	# at level 2, against about 400 kB if the same grid went out as numbers.
	# See slam2d.h for the header.
	$(LN) /var/run/slam2d.json $(1)/www/sensors/slam2d.json
	$(LN) /var/run/navigate.json $(1)/www/sensors/navigate.json
	$(LN) /var/run/map.s2mp $(1)/www/sensors/map.s2mp
	$(LN) /var/run/range.bin $(1)/www/sensors/range.bin
	# The beam angles, so a client can turn the range image into points. The
	# tablet cannot ask the sensor for them: it is on the access point and the
	# sensor is on the LAN, with no route between.
	$(LN) /var/run/beam.json $(1)/www/sensors/beam.json

	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) ./files/sensor-probe $(1)/usr/sbin/sensor-probe
	$(INSTALL_BIN) ./files/sensor-lan-tune $(1)/usr/sbin/sensor-lan-tune
	$(INSTALL_BIN) ./files/first-boot-report $(1)/usr/sbin/first-boot-report

	$(INSTALL_DIR) $(1)/etc/uci-defaults
	$(INSTALL_BIN) ./files/99-a3004-sensorkit $(1)/etc/uci-defaults/99-a3004-sensorkit
endef

define Package/a3004-sensorkit/postinst
#!/bin/sh
[ -n "$${IPKG_INSTROOT}" ] || {
	(. /etc/uci-defaults/99-a3004-sensorkit) && rm -f /etc/uci-defaults/99-a3004-sensorkit
	exit 0
}
endef

$(eval $(call BuildPackage,a3004-sensorkit))
