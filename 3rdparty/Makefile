all: ffmpeg-libvpx-make

FILENAME = ffmpeg_libvpx_3rdparty_sources.tar.gz
TARBALL = $(FILENAME)
TARBALL_URL = http://pr.willowgarage.com/downloads/$(FILENAME)
INITIAL_DIR = build/sources
SOURCE_DIR = sources
MD5SUM_FILE = $(FILENAME).md5sum
TARBALL_PATCH=

ffmpeg-libvpx-make:
	./download_checkmd5.py $(TARBALL_URL) $(TARBALL) &&\
	tar xzf $(TARBALL) &&\
	cd ./$(SOURCE_DIR) && ./compile.sh

wipe: clean
	rm -rf $(SOURCE_DIR)

clean:
	rm -rf $(SOURCE_DIR) &&\
 	rm -rf $(TARBALL)
	

