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
 * server_configuration.h
 *
 *  Created on: Nov 1, 2012
 *      Author: jkammerl
 */

#ifndef SERVER_CONFIGURATION_H_
#define SERVER_CONFIGURATION_H_

#include <string>

namespace ros_http_video_streamer
{

struct ServerConfiguration
{
  ServerConfiguration() :
    // DEFAULT CONFIGURATION
      port_(8888),
      codec_("webm"),
      bitrate_(100000),
      framerate_(31),
      quality_(-1),
      gop_(60),
      frame_width_(-1),
      frame_height_(-1),
      profile_("realtime"),
      wwwroot_("./www"),
      ros_transport_("raw")
  {
  }

  ////////////////////////////
  // Server configuration
  ////////////////////////////

  // network port
  int port_;

  ////////////////////////////
  // Codec configuration
  ////////////////////////////

  // codec name
  std::string codec_;
  // maximum bit rate
  int bitrate_;
  // maximum frame rate
  int framerate_;
  // quality // quantization
  int quality_;
  // group of pictures -> I-frame rate
  int gop_;
  // width of frame
  int frame_width_;
  // height of frame
  int frame_height_;
  // codec profile
  std::string profile_;
  // codec profile
  std::string wwwroot_;
  // ros transport
  std::string ros_transport_;

};

} // ros_http_video_streamer

#endif /* SERVER_CONFIGURATION_H_ */
