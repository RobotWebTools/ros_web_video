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

FFMPEGEncoder::FFMPEGEncoder(const std::string& refID,
                             const std::string& topic,
                             const std::string& codec,
                             unsigned bitrate,
                             unsigned framerate,
                             bool depth_rgb_encoding) :
    doEncoding_(true),
    refID_(refID),
    topic_(topic),
    codec_(codec),
    bitrate_(bitrate),
    framerate_(framerate),
    depth_rgb_encoding_(depth_rgb_encoding),
    subscriber_(topic),
    encoding_queue_thread_(0),
    encoding_header_mutex_(),
    encoding_data_mutex_(),
    condHeader_(),
    condData_(),
    header_ready_(false),
    data_ready_(false),
    header_buf_(),
    video_buf_(),
    frameID_(0),
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
  doEncoding_ = true;

  // create thread for processing video encoding
  encoding_queue_thread_ = new boost::thread(boost::bind(&FFMPEGEncoder::videoEncodingWorkerThread, this));
}

void FFMPEGEncoder::stop()
{
  doEncoding_ = false;

  if (encoding_queue_thread_)
    encoding_queue_thread_->join();
}

const std::string& FFMPEGEncoder::getRefID()
{
  return refID_;
}

void FFMPEGEncoder::getVideoPacket(std::vector<uint8_t>& buf)
{
  boost::unique_lock<boost::mutex> lock(encoding_data_mutex_);

  condData_.wait(lock);

  buf = video_buf_;
}

void FFMPEGEncoder::getHeader(std::vector<uint8_t>& buf)
{
  boost::unique_lock<boost::mutex> lock(encoding_header_mutex_);
  while (!header_ready_)
  {
    condHeader_.wait(lock);
  }

  buf = header_buf_;
}

void FFMPEGEncoder::convertFloatingPointImageToMono8(float* input,
                                                     const std::size_t width,
                                                     const std::size_t height,
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

template<typename T>
  void FFMPEGEncoder::encodeDepthImageToRGB8(T* input,
                                           const std::size_t width,
                                           const std::size_t height,
                                           std::vector<uint8_t>& output)
  {
    std::size_t image_size, x, y;
    T* input_ptr;
    uint8_t* output_ptr;

    // prepare output vector
    image_size = width * height;
    output.resize(image_size * 3 * sizeof(uint8_t), 0);

    input_ptr = input;
    output_ptr = &output[0];

    for (y = 0; y < height; ++y)
      for (x = 0; x < height; ++x, ++input_ptr)
      {
        // encode depth image in rgb space
        /*
         uint32_t pix_val = (*input_ptr)*1000;

         // R
         *rgb_ptr = (pix_val & 0xF)<<4;
         pix_val >>= 4; ++output_ptr;

         // G
         *output_ptr = (pix_val & 0xF)<<4;
         pix_val >>= 4; ++output_ptr;

         // B
         *output_ptr = (pix_val & 0xF)<<4;
         pix_val >>= 4; ++output_ptr;
         */

        uint8_t pix_val = (1.0f - std::min(1.0f, ((float)(*input_ptr) / (float)MAX_DEPTH))) * 255;

        // R
        *output_ptr = pix_val;
        ++output_ptr;

        // G
        *output_ptr = pix_val;
        ++output_ptr;

        // B
        *output_ptr = pix_val;
        ++output_ptr;
      }

  }

void FFMPEGEncoder::crop_frame(sensor_msgs::ImageConstPtr input,
                               sensor_msgs::ImagePtr output,
                               std::size_t width,
                               std::size_t height)
{
  // input and output size
  const std::size_t output_width = width;
  const std::size_t output_height = height;

  const std::size_t input_width = input->width;
  const std::size_t input_height = input->height;

  const std::size_t pixel_depth = input->step / input_width;

  std::size_t y, top_x, top_y, width_x, width_y;

  // assign field of output sensor_msgs::ImagePtr
  output->width = output_width;
  output->height = output_height;

  output->encoding = input->encoding;
  output->is_bigendian = input->is_bigendian;

  output->step = output->width * pixel_depth;
  // allocate data
  output->data.resize(output_width * output_height * pixel_depth, 0);

  // calculate borders
  int top_bottom_corner = (input_height - output_height) / 2;
  int left_right_corner = (input_width - output_width) / 2;

  if (top_bottom_corner < 0)
  {
    top_y = 0;
    width_y = input_height;
  }
  else
  {
    top_y = top_bottom_corner;
    width_y = input_height - top_bottom_corner;
  }

  if (left_right_corner < 0)
  {
    top_x = 0;
    width_x = input_width;
  }
  else
  {
    top_x = left_right_corner;
    width_x = input_width - left_right_corner;
  }

  // calculate memory steps
  const std::size_t source_line_size = pixel_depth * (width_x - top_x);
  const std::size_t source_y_step = input_width * pixel_depth;
  const std::size_t destination_y_step = output_width * pixel_depth;

  const uint8_t* source_ptr = &input->data[(top_y * input_width + top_x) * pixel_depth];
  uint8_t* dest_ptr = &output->data[((top_y - top_bottom_corner) * output_width + top_x - left_right_corner) * pixel_depth];

  // copy image data
  for (y = top_y; y < width_y; ++y, source_ptr += source_y_step, dest_ptr += destination_y_step)
  {
    memcpy(dest_ptr, source_ptr, source_line_size);
  }
}

void FFMPEGEncoder::videoEncodingWorkerThread()
{
  // time stamps used to control the encoder rate
  unsigned int milisec_used;
  const unsigned int milisec_per_frame = 1000 / std::max(10u, framerate_);

  while (doEncoding_)
  {

    boost::posix_time::ptime tick = boost::posix_time::microsec_clock::local_time();

    sensor_msgs::ImageConstPtr frame;

    // get frame from ROS iamge subscriber
    subscriber_.getImageFromQueue(frame);

    if (frame)
    {

      if (depth_rgb_encoding_)
      {
        // depth image needs to be transformed into a 2^x X 2^x resolution in order to enable WebGL-based pointclouds
        sensor_msgs::ImagePtr frame_resized = sensor_msgs::ImagePtr(new sensor_msgs::Image());

        // change resolution of input frame
        crop_frame(frame, frame_resized, RAW_DEPTH_IMAGE_RESOLUTION, RAW_DEPTH_IMAGE_RESOLUTION);

        frame = frame_resized;
      }

      if (!ffmpeg_)
      {
        // ffmpeg wrapper has not been initialized yet
        ffmpeg_ = new FFMPEG_Wrapper();

        // first input frame defines resolution
        ffmpeg_->init(codec_, frame->width, frame->height, bitrate_, framerate_);
        {
          {
            boost::lock_guard<boost::mutex> lock(encoding_header_mutex_);

            // retrieve header data from ffmpeg wrapper
            ffmpeg_->get_header(header_buf_);

            if (header_buf_.size() > 0)
              header_ready_ = true;
          }

          condHeader_.notify_all();
          ROS_DEBUG("Codec header received: %d bytes", (int)header_buf_.size());

          subscriber_.emptyQueue();
        }
      }

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
            ffmpeg_->encode_mono8_frame((uint8_t*)&frame->data[0], frame_buf_temp);
          }
          else if ((!frame->encoding.compare("mono16")) || (!frame->encoding.compare("16UC1")))
          {
            // monochrome 16-bit encoded frame

            ffmpeg_->encode_mono16_frame((uint8_t*)&frame->data[0], frame_buf_temp);

          }
          else if (!frame->encoding.compare("32FC1"))
          {
            // floating-point encoded frame

            if (depth_rgb_encoding_)
            {
              std::vector<uint8_t> rgb_image;

              encodeDepthImageToRGB8<float>((float*)&frame->data[0], frame->width, frame->height, rgb_image);

              ffmpeg_->encode_rgb_frame(&rgb_image[0], frame_buf_temp);
            }
            else
            {
              std::vector<uint8_t> normalized_image;

              convertFloatingPointImageToMono8((float*)&frame->data[0], frame->width, frame->height, normalized_image);

              ffmpeg_->encode_mono8_frame(&normalized_image[0], frame_buf_temp);
            }

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

            ffmpeg_->encode_rgb_frame((uint8_t*)&frame->data[0], frame_buf_temp);
          }
          else if (frame->encoding.find("bgr") != std::string::npos)
          {
            // bgr encoded frame

            ffmpeg_->encode_bgr_frame((uint8_t*)&frame->data[0], frame_buf_temp);
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

      // check for new video packet
      if (frame_buf_temp.size() > 0)
      {
        {
          {
            boost::lock_guard<boost::mutex> lock(encoding_data_mutex_);
            data_ready_ = true;

            // copy video data to output vector
            video_buf_ = frame_buf_temp;

            ++frameID_;
          }

          condData_.notify_all();
          ROS_DEBUG("Frame received: %d bytes", (int)video_buf_.size());
        }
      }
    }

    // calculate encoding time
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration diff = now - tick;
    milisec_used = diff.total_milliseconds();

    if (milisec_used < milisec_per_frame)
    {
      // if encoder worked faster than the desired frame rate -> go sleeping
      boost::this_thread::sleep(boost::posix_time::milliseconds(milisec_per_frame - milisec_used));
    }
  }

}
