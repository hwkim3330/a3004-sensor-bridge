include $(TOPDIR)/rules.mk

PKG_NAME:=rc-ibus
PKG_VERSION:=1.0
PKG_RELEASE:=1

PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=KETI

include $(INCLUDE_DIR)/package.mk

define Package/rc-ibus
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Read a FlySky receiver's i-BUS channel output
  DEPENDS:=+kmod-usb-serial +kmod-usb-serial-ch341 +kmod-usb-serial-cp210x \
	+kmod-usb-serial-ftdi
endef

define Package/rc-ibus/description
 No WiFi chip can receive FlySky AFHDS 2A: it is GFSK frequency hopping driven
 by an A7105, and an 802.11 radio shares the band but not the modulation. So the
 RF is left to a real receiver and this reads its i-BUS servo output over a
 USB-serial adapter - 32-byte frames at 115200 8N1, checksum verified on every
 one, with link loss reported when frames stop.
endef

define Package/rc-ibus/conffiles
/etc/config/rc-ibus
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_CPPFLAGS) $(TARGET_LDFLAGS) \
		-o $(PKG_BUILD_DIR)/rc-ibus $(PKG_BUILD_DIR)/rc-ibus.c
endef

define Package/rc-ibus/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/rc-ibus $(1)/usr/sbin/
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/rc-ibus.init $(1)/etc/init.d/rc-ibus
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) ./files/rc-ibus.config $(1)/etc/config/rc-ibus
endef

$(eval $(call BuildPackage,rc-ibus))
