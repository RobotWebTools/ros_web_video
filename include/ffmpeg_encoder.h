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

#include <iostream>
#include <string>
#include <vector>

#include <boost/thread.hpp>

#include <sensor_msgs/image_encodings.h>

#include "ffmpeg_wrapper.h"
#include "image_subscriber.h"
#include "server_configuration.h"

namespace ros_http_video_streamer
{

class FFMPEGEncoder
{
public:

  typedef boost::shared_ptr<FFMPEGEncoder> ptr;

  FFMPEGEncoder(const std::string& refID,
                const std::string& topic,
                const ServerConfiguration& config);

  virtual ~FFMPEGEncoder();

  // receive header data
  // this method blocks until the header data is availble from the codec
  void initEncoding(std::vector<uint8_t>& buf);

  // retrieve reverence string
  const std::string& getRefID();

  // stop encoding
  void start();

  // stop encoding
  void stop();

  // encode frame
  void getVideoPacket(std::vector<uint8_t>& buf);

private:

  // method to convert floating point image to 8-bit monochrome image
  void convertFloatingPointImageToMono8(float* input,
                                        const std::size_t width,
                                        const std::size_t height,
                                        std::vector<uint8_t>& output);

  // encoding thread running?
  bool doEncoding_;

  // reference ID used for registration
  const std::string refID_;

  // ROS topic
  const std::string topic_;

  // server&encoder configuration
  const ServerConfiguration& config_;

  // ROS image subscriber class
  ImageSubscriber subscriber_;

  // current frame ID
  unsigned frameID_;
  // init flag
  bool init_;

  boost::posix_time::ptime last_tick;

  // ffmpeg wrapper instance
  FFMPEG_Wrapper* ffmpeg_;

};

} // ros_http_video_streamer

#endif /* FFMPEG_ENCODER_H_ */
