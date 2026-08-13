#
# Copyright (C) 2026 KETI
#
# This is free software, licensed under the GNU General Public License v2.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=ouster-edge
PKG_VERSION:=1.0
PKG_RELEASE:=2

PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=KETI

include $(INCLUDE_DIR)/package.mk

define Package/ouster-edge
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=On-device edge processing for Ouster OS-series lidars
  DEPENDS:=+jsonfilter
endef

define Package/ouster-edge/description
 Receives the raw UDP stream from an Ouster OS-series lidar, relays it
 verbatim to a downstream host, and reduces each revolution to a compact
 per-azimuth minimum-range ring with polar zone intrusion detection. Sized
 for MT7621-class routers, where full point cloud processing is not viable.
endef

define Package/ouster-edge/conffiles
/etc/config/ouster-edge
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

#
# -O2 rather than the target's default -Os, for this package only.
#
# Measured, not assumed. Built native at both settings and fed the same live
# OS1-64 at 2048x10 - 1280 datagrams/s, the sensor's full rate - twice each:
#
#   -Os   31.6 and 33.9 us of cpu per datagram
#   -O2   24.5 and 25.6 us of cpu per datagram
#
# so about 23% less cpu per datagram, reproducibly, for 24 bytes of binary. The
# hot loop walks 64 channels per column and 16 columns per datagram with no
# floating point in it, which is the shape -Os hurts: it declines to unroll and
# to inline the per-pixel range decode.
#
# This is worth spending image size on here specifically because the work is
# per-packet and lands on the same cpu as the ethernet RX softirq - see
# doc/COMPUTE.md. Elsewhere in this kit -Os is the right default and is left
# alone.
#
TARGET_CFLAGS := $(filter-out -Os,$(TARGET_CFLAGS)) -O2

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_CPPFLAGS) $(TARGET_LDFLAGS) \
		-o $(PKG_BUILD_DIR)/ouster-edge $(PKG_BUILD_DIR)/ouster-edge.c
endef

define Package/ouster-edge/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/ouster-edge $(1)/usr/sbin/
	$(INSTALL_BIN) ./files/ouster-metadata $(1)/usr/sbin/
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/ouster-edge.init $(1)/etc/init.d/ouster-edge
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) ./files/ouster-edge.config $(1)/etc/config/ouster-edge
endef

$(eval $(call BuildPackage,ouster-edge))
