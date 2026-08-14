#
# Copyright (C) 2026 KETI
#
# This is free software, licensed under the GNU General Public License v2.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=ouster-edge
PKG_VERSION:=1.0
PKG_RELEASE:=10

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
# Built at the target's default -Os. There was an -O2 override here briefly;
# it is gone because it did not survive being measured on mipsel.
#
# The case for it looked strong on this desktop's x86 core: fed the same live
# OS1-64 at its full 2048x10 rate, -Os cost 31.6 and 33.9 us of cpu per
# datagram and -O2 cost 24.5 and 25.6, twice each, alternating. And the
# mechanism was visible in the compiler's own output - at -O2 and not at -Os,
# gcc inlines px_range_mm and px_refl into packet_process, which is 1024 calls
# per datagram it no longer makes.
#
# None of that transferred. Under emu/oe-flagbench.py, on mipsel, at 36000
# datagrams per build: -Os spent 135 jiffies of cpu and -O2 spent 142, and an
# earlier run at 8000 datagrams had them identical at 30 each. So -O2 buys
# nothing here and may cost a little, while making the binary 988 bytes bigger -
# which on a 32 kB instruction cache is the wrong direction.
#
# Left as a note rather than deleted because the x86 number is genuine and will
# tempt somebody again. Cross-architecture, per-datagram costs measured on a
# wide out-of-order core do not predict a 1004Kc.
#
define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_CPPFLAGS) $(TARGET_LDFLAGS) \
		-o $(PKG_BUILD_DIR)/ouster-edge $(PKG_BUILD_DIR)/ouster-edge.c
endef

define Package/ouster-edge/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/ouster-edge $(1)/usr/sbin/
	$(INSTALL_BIN) ./files/ouster-metadata $(1)/usr/sbin/
	$(INSTALL_BIN) ./files/ouster-configure $(1)/usr/sbin/
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/ouster-edge.init $(1)/etc/init.d/ouster-edge
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) ./files/ouster-edge.config $(1)/etc/config/ouster-edge
endef

$(eval $(call BuildPackage,ouster-edge))
