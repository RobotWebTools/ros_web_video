/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * ffmpeg_wrapper.h
 *
 *  Created on: Oct 30, 2012
 *      Author: jkammerl
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include <boost/cstdint.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "server_configuration.h"

#ifndef FFMPEG_WRAPPER_HEADER_INCLUDED
#define FFMPEG_WRAPPER_HEADER_INCLUDED

#ifdef HAVE_AV_CONFIG_H
#undef HAVE_AV_CONFIG_H
#endif

#define CODER_BUF_SIZE 50000


namespace ros_http_video_streamer
{

// ffmpeg forward declarations
class AVCodec;
class AVOutputFormat;
class AVCodecContext;
class AVFormatContext;
class AVPicture;
class AVStream;
class AVFrame;


class FFMPEG_Wrapper
{

public:

  FFMPEG_Wrapper();
  ~FFMPEG_Wrapper();

  // initialize ffmpeg coding
  void init(int input_width,
            int input_height,
            const ServerConfiguration& config);

  // shutdown ffmpeg coding
  void shutdown();

  // encode bgr8 encoded frame
  void encode_bgr_frame(uint8_t *bgr_data, std::vector<uint8_t>& encoded_frame);
  // encode rgb8 encoded frame
  void encode_rgb_frame(uint8_t *rgb_data, std::vector<uint8_t>& encoded_frame);

  // encode bgr8 encoded frame
  void encode_mono8_frame(uint8_t *gray8_data, std::vector<uint8_t>& encoded_frame);
  // encode bgr16 encoded frame
  void encode_mono16_frame(uint8_t *gray16_data, std::vector<uint8_t>& encoded_frame);

  // retrieve header
  void get_header(std::vector<uint8_t>& header);
  // retrieve trailer
  void get_trailer(std::vector<uint8_t>& trailer);

private:
  // templated encoding method
  template<int CODING_FORMAT>
    void encode_frame(uint8_t *image_data, std::vector<uint8_t>& encoded_frame);

  //int ff_lockmgr(void **mutex, enum AVLockOp op);

  // mutex to protect encoding queue
  boost::mutex frame_mutex_;
  boost::mutex codec_mutex_;

  // ffmpeg contexts 
  AVCodec *ffmpeg_codec_;
  AVOutputFormat *ffmpeg_output_format_;
  AVCodecContext *ffmpeg_codec_context_;
  AVFormatContext *ffmpeg_format_context_;
  AVPicture* ffmpeg_src_picture_;
  AVPicture* ffmpeg_dst_picture_;
  AVStream *ffmpeg_video_st_;
  AVFrame *ffmpeg_frame_;
  double ffmpeg_video_pts_;
  struct SwsContext *ffmpeg_sws_ctx_;

  // Server and codec configuration
  ServerConfiguration config_;

  // image dimensions
  int input_width_;
  int input_height_;
  int output_width_;
  int output_height_;

  boost::posix_time::ptime time_started_;

  bool init_;

};

} // ros_http_video_streamer

#endif

