#
# Copyright (C) 2026 KETI
#
# This is free software, licensed under the GNU General Public License v2.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=navigate
PKG_VERSION:=1.0
PKG_RELEASE:=3

PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=KETI

include $(INCLUDE_DIR)/package.mk

define Package/navigate
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Drive to a point on the map, with no floating point
  DEPENDS:=+slam2d +agx-cmd
endef

define Package/navigate/description
 Maps with slam2d's core, plans a wavefront to a destination over the coarse
 pyramid level, and steers by emitting the same TELE frames a tablet would - so
 agx-cmd's arming and deadman and can-bridge's inject gate all still apply.
 Integer arithmetic throughout, for the same reason as slam2d.

 It can move a vehicle. Nothing is sent without --tele, agx-cmd is disabled by
 default, and can-bridge refuses to write to the bus without allow_inject.
endef

define Package/navigate/conffiles
/etc/config/navigate
endef

# The slam2d core is shared rather than duplicated. Same arrangement as
# agx-cmd, which borrows agilex.c from can-bridge: copied into the build tree
# so the OpenWrt build stays self-contained, with the original the only copy
# anybody edits.
define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
	$(CP) ../slam2d/src/slam2d.c ../slam2d/src/slam2d.h $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_CPPFLAGS) $(TARGET_LDFLAGS) \
		-I$(PKG_BUILD_DIR) \
		-o $(PKG_BUILD_DIR)/navigate \
		$(PKG_BUILD_DIR)/navigate.c $(PKG_BUILD_DIR)/slam2d.c
endef

define Package/navigate/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/navigate $(1)/usr/sbin/
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/navigate.init $(1)/etc/init.d/navigate
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) ./files/navigate.config $(1)/etc/config/navigate
endef

$(eval $(call BuildPackage,navigate))
