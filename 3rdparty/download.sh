#!/bin/sh

wget http://pr.willowgarage.com/downloads/ffmpeg_libvpx_3rdparty_sources.tar.gz && \
tar xzf ./ffmpeg_libvpx_3rdparty_sources.tar.gz &&\
cd libvpx && \
./configure --enable-vp8 && \
make && \
cd ../ffmpeg && \
./configure --enable-libvpx --extra-cflags="-I../libvpx" --extra-ldflags="-L../libvpx" --enable-gpl &&\ 
make
                                                                        