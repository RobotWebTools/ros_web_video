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
 * ffmpeg_encoder.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: jkammerl
 */

#include "boost/date_time/posix_time/posix_time.hpp"

#include <boost/bind.hpp>

#include "ffmpeg_encoder.h"
#include "image_subscriber.h"

#define MAX_DEPTH 4.0
#define RAW_DEPTH_IMAGE_RESOLUTION 512

namespace enc = sensor_msgs::image_encodings;

namespace ros_http_video_streamer
{

FFMPEGEncoder::FFMPEGEncoder(const std::string& refID, const std::string& topic, const ServerConfiguration& config) :
    doEncoding_(true),
    refID_(refID),
    topic_(topic),
    config_(config),
    subscriber_(topic, config.ros_transport_),
    frameID_(0),
    init_(false),
    ffmpeg_(0)
{
  start();
}

FFMPEGEncoder::~FFMPEGEncoder()
{
  stop();
}

void FFMPEGEncoder::start()
{
}

void FFMPEGEncoder::stop()
{

  if (doEncoding_)
  {
    doEncoding_ = false;

    if (ffmpeg_)
    {
      ffmpeg_->shutdown();
      delete (ffmpeg_);
      ffmpeg_ = 0;
    }
  }

}

const std::string& FFMPEGEncoder::getRefID()
{
  return refID_;
}

void FFMPEGEncoder::convertFloatingPointImageToMono8(float* input, const std::size_t width, const std::size_t height,
                                                     std::vector<uint8_t>& output)
{
  std::size_t image_size, i;
  float* input_ptr;
  uint8_t* output_ptr;

  // prepare output vector
  image_size = width * height;
  output.resize(image_size);

  // Find min. and max. pixel value
  float minValue = std::numeric_limits<float>::max();
  float maxValue = std::numeric_limits<float>::min();
  input_ptr = input;
  output_ptr = &output[0];
  bool valid_image = false;
  for (i = 0; i < image_size; ++i)
  {
    if (*input_ptr == *input_ptr) // check for NaN
    {
      minValue = std::min(minValue, *input_ptr);
      maxValue = std::max(maxValue, *input_ptr);
      valid_image = true;
    }
    input_ptr++;
  }

  // reset data pointer
  input_ptr = input;
  output_ptr = &output[0];

  // Rescale floating point image and convert it to 8-bit
  float dynamic_range = maxValue - minValue;
  if (valid_image && (dynamic_range > 0.0f))
  {
    // Rescale and quantize
    for (i = 0; i < image_size; ++i, ++input_ptr, ++output_ptr)
    {
      if (*input_ptr == *input_ptr) // check for NaN
      {
        *output_ptr = ((*input_ptr - minValue) / dynamic_range) * 255u;
      }
      else
      {
        *output_ptr = 0u;
      }
    }

  }
  else
  {
    // clear output buffer
    memset(output_ptr, 0, image_size * sizeof(uint8_t));
  }
}

void FFMPEGEncoder::initEncoding(std::vector<uint8_t>& header)
{
  sensor_msgs::ImageConstPtr frame;

  while (!init_)
  {
    // get frame from ROS iamge subscriber
    subscriber_.getImageFromQueue(frame);

    if ((frame) && (!ffmpeg_))
    {
      // ffmpeg wrapper has not been initialized yet
      ffmpeg_ = new FFMPEG_Wrapper();

      // first input frame defines resolution
      ffmpeg_->init(frame->width, frame->height, config_);

      // retrieve header data from ffmpeg wrapper
      ffmpeg_->get_header(header);

      ROS_DEBUG("Codec header received: %d bytes", (int)header.size());

      subscriber_.emptyQueue();

      // FFMPEG initialized
      init_ = true;

    }
    else
    {
      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
  }
}

void FFMPEGEncoder::getVideoPacket(std::vector<uint8_t>& buf)
{
  buf.clear();

  if (!init_)
  {
    ROS_ERROR("Video encoding not initialized.");
    return;
  }

  while (buf.size() == 0)
  {
    // time stamps used to control the encoder rate
    unsigned int milisec_used;
    const unsigned int milisec_per_frame = 1000 / config_.framerate_;

    sensor_msgs::ImageConstPtr frame;

    // get frame from ROS iamge subscriber
    subscriber_.getImageFromQueue(frame);

    ROS_DEBUG("Encoding triggered..");

    if (frame)
    {

      std::vector<uint8_t> frame_buf_temp;

      bool valid_format = true;

      // Bit depth of image frame
      unsigned int num_channels = enc::numChannels(frame->encoding);
      switch (num_channels)
      {
        case 1:
          // monochrome 8-bit encoded frame
          if ((!frame->encoding.compare("mono8")) || (!frame->encoding.compare("8UC1")))
          {
            ffmpeg_->encode_mono8_frame((uint8_t*)&frame->data[0], buf);
          }
          else if ((!frame->encoding.compare("mono16")) || (!frame->encoding.compare("16UC1")))
          {
            // monochrome 16-bit encoded frame

            ffmpeg_->encode_mono16_frame((uint8_t*)&frame->data[0], buf);

          }
          else if (!frame->encoding.compare("32FC1"))
          {
            // floating-point encoded frame

            std::vector<uint8_t> normalized_image;

            convertFloatingPointImageToMono8((float*)&frame->data[0], frame->width, frame->height, normalized_image);

            ffmpeg_->encode_mono8_frame(&normalized_image[0], buf);

          }
          else
          {
            valid_format = false;
          }
          break;
        case 3:
          if (frame->encoding.find("rgb") != std::string::npos)
          {
            // rgb encoded frame

            ffmpeg_->encode_rgb_frame((uint8_t*)&frame->data[0], buf);
          }
          else if (frame->encoding.find("bgr") != std::string::npos)
          {
            // bgr encoded frame

            ffmpeg_->encode_bgr_frame((uint8_t*)&frame->data[0], buf);
          }
          else
          {
            valid_format = false;
          }
          break;
        default:
          valid_format = false;
          break;
      }

      if (!valid_format)
      {
        ROS_ERROR("Invalid image format");
      }

    }

    // calculate encoding time
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration diff = now - last_tick;
    last_tick = now;
    milisec_used = diff.total_milliseconds();

    if (milisec_used < milisec_per_frame)
    {
      // if encoder worked faster than the desired frame rate -> go sleeping
      boost::this_thread::sleep(boost::posix_time::milliseconds(milisec_per_frame - milisec_used));
    }
  }

  ROS_DEBUG("Frame received: %d bytes", (int)buf.size());

}

} // ros_http_video_streamer
