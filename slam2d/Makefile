#
# Copyright (C) 2026 KETI
#
# This is free software, licensed under the GNU General Public License v2.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=slam2d
PKG_VERSION:=1.0
PKG_RELEASE:=3

PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=KETI

include $(INCLUDE_DIR)/package.mk

define Package/slam2d
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Integer-only 2D occupancy mapping from the ouster-edge range ring
  DEPENDS:=+ouster-edge
endef

define Package/slam2d/description
 Builds an occupancy grid and tracks a pose from the per-azimuth minimum range
 ring that ouster-edge publishes, using correlative scan matching over an image
 pyramid. Contains no floating point at all, which is what lets it run on
 MT7621: the part has no FPU and OpenWrt builds mipsel with CONFIG_SOFT_FLOAT,
 so a conventional scan matcher spends its life in libgcc. The same property
 makes it bit-identical across hosts, so the tablet and the router agree.
endef

define Package/slam2d/conffiles
/etc/config/slam2d
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_CPPFLAGS) $(TARGET_LDFLAGS) \
		-I$(PKG_BUILD_DIR) \
		-o $(PKG_BUILD_DIR)/slam2d-daemon \
		$(PKG_BUILD_DIR)/slam2d-daemon.c $(PKG_BUILD_DIR)/slam2d.c
endef

define Package/slam2d/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/slam2d-daemon $(1)/usr/sbin/
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/slam2d.init $(1)/etc/init.d/slam2d
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) ./files/slam2d.config $(1)/etc/config/slam2d
endef

$(eval $(call BuildPackage,slam2d))
