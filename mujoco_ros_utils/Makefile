all: mujoco

ARCH=$(shell uname -m)
VERSION = 2.3.7
MUJOCO = mujoco-$(VERSION)-linux-$(ARCH).tar.gz
TARBALL = build/$(MUJOCO)
TARBALL_URL = https://github.com/deepmind/mujoco/releases/download/$(VERSION)/$(MUJOCO)
SOURCE_DIR = build/mujoco-$(VERSION)
ifeq ($(ARCH),x86_64)
  SHA256SUM = 3f75e53e3356ce4ee6759cde5358b7a53f51632f495b910aedc9caa5bf98ac39
else ifeq ($(ARCH),aarch64)
  SHA256SUM = 422d52e98fba8b6328d67e00c87d59ba8108f1c0df8a4c048811fa73be9e8c71
else
  SHA256SUM = 
endif

SHA256SUM_ACTUAL = sha256sum $(TARBALL) | cut -d ' ' -f 1
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk


mujoco: $(SOURCE_DIR)/unpacked
	echo "$(ARCH)"
	echo "$(SHA256SUM) build/$(MUJOCO)" | sha256sum --check

clean:
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build
