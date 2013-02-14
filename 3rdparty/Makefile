all: ffmpeg-libvpx-make

FILENAME = ffmpeg_libvpx_3rdparty_sources.tar.gz
TARBALL = $(FILENAME)
TARBALL_URL = http://pr.willowgarage.com/downloads/$(FILENAME)
INITIAL_DIR = build/sources
SOURCE_DIR = sources
UNPACK_CMD = tar xzf
MD5SUM_FILE = $(FILENAME).md5sum
TARBALL_PATCH=

PACKAGE_DIR = $(shell rospack find graspit)

include $(shell rospack find mk)/download_unpack_build.mk

ffmpeg-libvpx-make: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./compile.sh

wipe: clean
	rm -rf $(SOURCE_DIR)

clean:
	rm -rf $(SOURCE_DIR) &&\
 	rm -rf $(TARBALL)
	

