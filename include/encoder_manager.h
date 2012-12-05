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
 * encoder_manager.h
 *
 *  Created on: Oct 30, 2012
 *      Author: jkammerl
 */


#ifndef ENCODING_MANAGER_H_
#define ENCODING_MANAGER_H_

#include "ffmpeg_encoder.h"

#include "server_configuration.h"

#include <map>
#include <string>

//#define SHARED_ENCODERS

namespace ros_http_video_streamer
{

class EncoderManager
{
public:
  EncoderManager() :
      mutex_(),
      image_encoder_map_(),
      request_counter_(0)
  {
  }

  virtual ~EncoderManager()
  {
  }

  //
  FFMPEGEncoder::ptr subscribe(const std::string& topic,
                               const ServerConfiguration& config)
  {
    boost::mutex::scoped_lock lock(mutex_);

    ++request_counter_;

    std::string refID = topic + ":" +
                        config.codec_ + ":" +
                        "BR:"+boost::lexical_cast<std::string>(config.bitrate_) +
                        "FR:"+boost::lexical_cast<std::string>(config.framerate_) +
                        "FW:"+boost::lexical_cast<std::string>(config.frame_width_) +
                        "FH:"+boost::lexical_cast<std::string>(config.frame_height_) +
                        "Q:"+boost::lexical_cast<std::string>(config.quality_) +
                        "GOP:"+boost::lexical_cast<std::string>(config.gop_);

    FFMPEGEncoder::ptr image_encoder;

#ifdef SHARED_ENCODERS
    // search for encoder instance in image_encoder_map_
    std::map<std::string, EncoderInfo>::iterator it;
    it = image_encoder_map_.find(refID);

    if (it!=image_encoder_map_.end())
    {
      image_encoder = (it->second).enc_;
      (it->second).listener_count_++;
    }
    else
    {
      // if not found, create new instance of encoder
      image_encoder = FFMPEGEncoder::ptr(new FFMPEGEncoder(refID, topic, config));

      EncoderInfo encInfo;
      encInfo.enc_ = image_encoder;
      encInfo.listener_count_ = 1;

      // add it to the image_encoder_map_
      image_encoder_map_[refID] = encInfo;
    }
#else
    // make request ID unique
    refID += "ReqID:" + boost::lexical_cast<std::string>(request_counter_);
    image_encoder = FFMPEGEncoder::ptr(new FFMPEGEncoder(refID, topic, config));

    EncoderInfo encInfo;
    encInfo.enc_ = image_encoder;
    encInfo.listener_count_ = 1;

    image_encoder_map_[refID] = encInfo;
#endif

    return image_encoder;
  }

  void unsubscribe(const std::string& refID)
  {
    boost::mutex::scoped_lock lock(mutex_);

    // search for encoder instance
    std::map<std::string, EncoderInfo>::iterator it;
    it = image_encoder_map_.find(refID);

    if (it != image_encoder_map_.end())
    {
      EncoderInfo& encInfo = it->second;
      encInfo.listener_count_--;

      if (!encInfo.listener_count_)
        image_encoder_map_.erase(refID);

      ROS_INFO("Deleting encoder: %s", refID.c_str());
    }
  }

private:

  // struct that contains a shared pointer to an encoder instance and
  // manages the listener count
  struct EncoderInfo
  {
    EncoderInfo() :
        listener_count_(0)
    {
    }

    // shared pointer to encoder
    FFMPEGEncoder::ptr enc_;
    // amount of subscribed http listeners
    std::size_t listener_count_;
  };

  // mutex to protect the image_encoder_map
  boost::mutex mutex_;
  // hash map that maps reference strings to image encoder instances
  std::map<std::string, EncoderInfo> image_encoder_map_;

  // total amounts of encoder requests to manager class (used to construct unique reference IDs)
  unsigned int request_counter_;

};

} // ros_http_video_streamer


#endif /* ENCODING_MANAGER_H_ */
