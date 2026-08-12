include $(TOPDIR)/rules.mk

PKG_NAME:=teleop
PKG_VERSION:=1.0
PKG_RELEASE:=1

PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=KETI

include $(INCLUDE_DIR)/package.mk

define Package/teleop
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Forward control intent from a browser, with a deadman
endef

define Package/teleop/description
 Lets a tablet on the router's own AP drive something, without pretending a
 wireless link is a control bus. Nothing is emitted as armed until explicitly
 armed, a stale command neutralises the output and drops the armed flag, and
 every datagram carries a sequence number and the sender's monotonic clock so
 the receiver can run its own independent deadman. Holds no control loop
 itself - see doc/TELEOP.md.
endef

define Package/teleop/conffiles
/etc/config/teleop
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_CPPFLAGS) $(TARGET_LDFLAGS) \
		-o $(PKG_BUILD_DIR)/teleop $(PKG_BUILD_DIR)/teleop.c
endef

define Package/teleop/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/teleop $(1)/usr/sbin/
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/teleop.init $(1)/etc/init.d/teleop
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) ./files/teleop.config $(1)/etc/config/teleop
endef

$(eval $(call BuildPackage,teleop))
