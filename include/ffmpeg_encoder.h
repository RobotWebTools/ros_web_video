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
 * ffmpeg_encoder.h
 *
 *  Created on: Oct 30, 2012
 *      Author: jkammerl
 */

#ifndef FFMPEG_ENCODER_H_
#define FFMPEG_ENCODER_H_

#define DEPTH_TO_RGB_AVERAGE

#include <boost/thread.hpp>

#include <iostream>
#include <string>
#include <vector>

#include "ffmpeg_wrapper.h"

#include "image_subscriber.h"

#include <sensor_msgs/image_encodings.h>

namespace ros_http_video_streamer
{

class FFMPEGEncoder
{
public:

  typedef boost::shared_ptr<FFMPEGEncoder> ptr;

  FFMPEGEncoder(const std::string& refID,
                const std::string& topic,
                const std::string& codec,
                unsigned int bitrate,
                unsigned int framerate,
                int framewidth,
                int frameheight,
                bool depth_rgb_encoding);

  virtual ~FFMPEGEncoder();

  // receive new video packet
  // this method blocks until a new video packet is available
  void getVideoPacket(std::vector<uint8_t>& buf);

  // receive header data
  // this method blocks until the header data is availble from the codec
  void getHeader(std::vector<uint8_t>& buf);

  // retrieve reverence string
  const std::string& getRefID();

  // stop encoding
  void start();

  // stop encoding
  void stop();

private:

  // worker thread to perform video encoding
  void videoEncodingWorkerThread();

  // method to convert floating point image to 8-bit monochrome image
  void convertFloatingPointImageToMono8(float* input,
                                        const std::size_t width,
                                        const std::size_t height,
                                        std::vector<uint8_t>& output);

  // method to encode raw data into rgb8 format (used for depth map encoding)
  template<typename T>
  void encodeDepthImageToRGB8 (T* input,
                                const std::size_t width,
                                const std::size_t height,
                                std::vector<uint8_t>& output);

  // method to crop/resize input frame
  void cropFrame(sensor_msgs::ImageConstPtr input,
                  sensor_msgs::ImagePtr output,
                  std::size_t width,
                  std::size_t height);

  // method to crop/resize input frame
  void createPointCloudDataFrame(sensor_msgs::ImageConstPtr input,
                                 sensor_msgs::ImagePtr output);

  // encoding thread running?
  bool doEncoding_;

  const std::string refID_;
  const std::string topic_;
  const std::string codec_;
  unsigned bitrate_;
  unsigned framerate_;
  int framewidth_;
  int frameheight_;
  bool depth_rgb_encoding_;

  // ROS image subscriber class
  ImageSubscriber subscriber_;

  // encoding thread
  boost::thread* encoding_queue_thread_;

  // mutexes to protect I/O between ffmpeg_wrapper and the http_server
  boost::mutex encoding_header_mutex_;
  boost::mutex encoding_data_mutex_;
  boost::condition_variable condHeader_;
  boost::condition_variable condData_;
  bool header_ready_;
  bool data_ready_;

  // vector containing binary header data
  std::vector<uint8_t> header_buf_;
  // vector containing binary video packts
  std::vector<uint8_t> video_buf_;
  // current frame ID
  unsigned frameID_;

  // ffmpeg wrapper instance
  FFMPEG_Wrapper* ffmpeg_;

};

} // ros_http_video_streamer

#endif /* FFMPEG_ENCODER_H_ */
