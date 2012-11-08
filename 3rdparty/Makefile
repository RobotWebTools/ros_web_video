# Author: Julius Kammerl
# Date:   07. November 2012

all : ./libvpx ./ffmpeg

./libvpx:
	git clone http://git.chromium.org/webm/libvpx.git && cd libvpx && ./configure --enable-vp8 && make

./ffmpeg:
	git clone git://source.ffmpeg.org/ffmpeg.git ffmpeg && cd ffmpeg && ./configure --enable-libvpx --extra-cflags=-I../libvpx --extra-ldflags=-L../libvpx --enable-gpl  && make
		
clean:
	rm -rf ./libvpx
	rm -rf ./ffmpeg
	

