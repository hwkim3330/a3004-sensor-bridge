include $(TOPDIR)/rules.mk

PKG_NAME:=mic-stream
PKG_VERSION:=1.0
PKG_RELEASE:=1

PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=KETI

include $(INCLUDE_DIR)/package.mk

define Package/mic-stream
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Low-latency uncompressed microphone streaming over HTTP
  DEPENDS:=+alsa-utils
endef

define Package/mic-stream/description
 Needs kmod-usb-audio, which the image selects rather than this package:
 depending on it here closes a kconfig recursion through alsa-lib.

 Serves a USB microphone as raw S16_LE PCM over HTTP, with no codec in the
 path. 16 kHz mono is 256 kbit/s, so on a link that already carries a camera
 and a lidar the bandwidth is free, and skipping compression removes both the
 encoder delay and any CPU cost on a soft-float target.
endef

define Package/mic-stream/conffiles
/etc/config/mic-stream
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_CPPFLAGS) $(TARGET_LDFLAGS) \
		-o $(PKG_BUILD_DIR)/mic-stream $(PKG_BUILD_DIR)/mic-stream.c
endef

define Package/mic-stream/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/mic-stream $(1)/usr/sbin/
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/mic-stream.init $(1)/etc/init.d/mic-stream
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) ./files/mic-stream.config $(1)/etc/config/mic-stream
endef

$(eval $(call BuildPackage,mic-stream))
