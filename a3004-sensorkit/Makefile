#
# Copyright (C) 2026 KETI
#
# This is free software, licensed under the GNU General Public License v2.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=a3004-sensorkit
PKG_VERSION:=1.0
PKG_RELEASE:=1

PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=KETI

include $(INCLUDE_DIR)/package.mk

define Package/a3004-sensorkit
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Sensor bridge for ipTIME A3004NS-M (USB camera + Ouster lidar)
  DEPENDS:= \
	+kmod-usb3 \
	+kmod-video-core \
	+kmod-video-uvc \
	+ustreamer \
	+v4l-utils \
	+ouster-edge \
	+uhttpd
endef

define Package/a3004-sensorkit/description
 Turns the router into a self-contained sensor bridge: a UVC camera on the
 USB 3.0 port is served as MJPEG over HTTP, an Ouster lidar on a gigabit LAN
 port is reduced to a range ring by ouster-edge, and a dashboard on port 80
 shows both. An Android tablet joining the router's own WiFi can open it
 directly, with no other machine involved.
endef

define Build/Compile
endef

define Package/a3004-sensorkit/install
	$(INSTALL_DIR) $(1)/www/sensors
	$(INSTALL_DATA) ./files/www/index.html $(1)/www/sensors/index.html
	# ouster-edge writes its status to tmpfs; expose it under the docroot
	$(LN) /var/run/ouster-edge.json $(1)/www/sensors/ouster.json

	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) ./files/sensor-probe $(1)/usr/sbin/sensor-probe
	$(INSTALL_BIN) ./files/sensor-lan-tune $(1)/usr/sbin/sensor-lan-tune

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
