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
 * main.cpp
 *
 *  Created on: Nov 1, 2012
 *      Author: jkammerl
 */

#include <iostream>
#include <string>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/cstdint.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ros/ros.h"

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

class DepthRGBEncoder
{
public:
  DepthRGBEncoder() :
    nh_(""),
    depth_it_(nh_),
    depth_sub_(),
    color_it_(nh_),
    color_sub_(),
    pub_it_(nh_)
  {
    // read depth map topic from param server
    std::string depthmap_topic;
    nh_.param<std::string>("depth", depthmap_topic, "/camera/depth/image");

    // read depth map topic from param server
    std::string rgb_image_topic;
    nh_.param<std::string>("rgb", rgb_image_topic, "/camera/rgb/image_rect_color");

    subscribe(depthmap_topic, rgb_image_topic);

    pub_ = pub_it_.advertise("depth_color_combined", 1);
  }
  virtual ~DepthRGBEncoder()
  {
  }

  void subscribe(std::string& depth_topic, std::string& color_topic)
  {

    // reset all message filters
    sync_depth_color_.reset(new synchronizer_depth_color_(sync_policy_depth_color_(1)));
    depth_sub_.reset(new image_transport::SubscriberFilter());
    color_sub_.reset(new image_transport::SubscriberFilter());

    if (!depth_topic.empty()) {
       // subscribe to depth map topic
       depth_sub_->subscribe(depth_it_, depth_topic, 1,  image_transport::TransportHints("raw"));

       if (!color_topic.empty()) {
         // subscribe to color image topic
         color_sub_->subscribe(color_it_, color_topic, 1,  image_transport::TransportHints("raw"));

         // connect message filters to synchronizer
         sync_depth_color_->connectInput(*depth_sub_, *color_sub_);
         sync_depth_color_->setInterMessageLowerBound(0, ros::Duration(0.5));
         sync_depth_color_->setInterMessageLowerBound(1, ros::Duration(0.5));
         sync_depth_color_->registerCallback(boost::bind(&DepthRGBEncoder::depth_with_color_cb, this, _1, _2));
       } else
       {
         depth_sub_->registerCallback(boost::bind(&DepthRGBEncoder::depth_only_cb, this, _1));
       }
     }
  }

  void depth_only_cb(const sensor_msgs::ImageConstPtr& depth_msg)
  {
    process(depth_msg, sensor_msgs::ImageConstPtr());
  }

  void depth_with_color_cb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& color_msg)
  {
    process(depth_msg, color_msg);
  }

  void process(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& color_msg)
  {
    // Bit depth of image encoding
    int depth_bits = enc::bitDepth(depth_msg->encoding);
    int depth_channels = enc::numChannels(depth_msg->encoding);

    int color_bits = 0;
    int color_channels = 0;

    if ( (depth_bits!=32) || (depth_channels!=1) )
    {
      ROS_DEBUG_STREAM ( "Invalid color depth image format ("<<depth_msg->encoding <<")");
      return;
    }

    if (color_msg)
    {
      color_bits = enc::bitDepth(color_msg->encoding);
      color_channels = enc::numChannels(color_msg->encoding);

     /* if (depth_msg->header.frame_id != color_msg->header.frame_id)
      {
        ROS_DEBUG_STREAM ("Depth image frame id [" << depth_msg->header.frame_id.c_str()
            << "] doesn't match color image frame id [" << color_msg->header.frame_id.c_str() << "]");
        return;
      }
     */
      if (depth_msg->width != color_msg->width || depth_msg->height != color_msg->height)
      {
        ROS_DEBUG_STREAM ( "Depth image resolution (" << (int)depth_msg->width << "x" << (int)depth_msg->height << ") "
            "does not match color image resolution (" << (int)color_msg->width << "x" << (int)color_msg->height << ")");
        return;
      }

      if ( (color_bits!=8) || (color_channels!=3) )
      {
        ROS_DEBUG_STREAM ( "Invalid color image format ("<<color_msg->encoding <<")");
        return;
      }
    }


    sensor_msgs::ImagePtr depth_int_msg (new sensor_msgs::Image());
    sensor_msgs::ImagePtr mask_msg (new sensor_msgs::Image());

    depthInterpolation(depth_msg,depth_int_msg,mask_msg);

    std::size_t crop_size = 512;
    unsigned int pix_size = enc::bitDepth(enc::BGR8)/8*3;

    // output image message
    sensor_msgs::ImagePtr output_msg ( new sensor_msgs::Image );
    output_msg->header = depth_int_msg->header;
    output_msg->encoding = enc::BGR8;
    output_msg->width = crop_size * 2;
    output_msg->height = crop_size * 2;
    output_msg->step = output_msg->width * pix_size;
    output_msg->is_bigendian = depth_int_msg->is_bigendian;

    // allocate data
    output_msg->data.resize(output_msg->width * output_msg->height * pix_size, 0);

    std::size_t input_width = depth_int_msg->width;
    std::size_t input_height = depth_int_msg->height;

    // copy depth & color data to output image

    {
      std::size_t y, x, top_x, top_y, width_x, width_y;

      // calculate borders
      int top_bottom_corner = (input_height - crop_size) / 2;
      int left_right_corner = (input_width - crop_size) / 2;

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

      // calculate memory pointers
      const std::size_t source_depth_line_size = sizeof(float) * (width_x - top_x);
      const std::size_t source_depth_y_step = input_width * sizeof(float);
      const uint8_t* source_depth_ptr = &depth_int_msg->data[(top_y * input_width + top_x) * sizeof(float)];

      const std::size_t destination_y_step = output_msg->step;

      const uint8_t* source_color_ptr = 0;
      std::size_t source_color_line_size = 0;
      std::size_t source_color_y_step = 0;

      const std::size_t color_x_shift = crop_size * pix_size;;
      const std::size_t mask_x_shift = destination_y_step*crop_size;

      const std::size_t source_mask_line_size = sizeof(uint8_t) * (width_x - top_x);
      const std::size_t source_mask_y_step = input_width * sizeof(uint8_t);
      const uint8_t* source_mask_ptr = &mask_msg->data[(top_y * input_width + top_x) * sizeof(uint8_t)];


      if (color_msg)
      {
        source_color_line_size = (width_x - top_x) * pix_size;
        source_color_y_step = input_width * pix_size;
        source_color_ptr = &color_msg->data[(top_y * input_width + top_x) * pix_size];
      }


      uint8_t* dest_ptr = &output_msg->data[((top_y - top_bottom_corner) * output_msg->width + top_x - left_right_corner) * pix_size];

      // copy image data
      for (y = top_y; y < width_y; ++y, source_color_ptr+=source_color_y_step,
                                        source_depth_ptr += source_depth_y_step,
                                        source_mask_ptr += source_mask_y_step,
                                        dest_ptr += destination_y_step)
      {
        float *depth_ptr = (float*)source_depth_ptr;

        uint8_t *color_ptr = (uint8_t*)source_color_ptr;
        uint8_t *out_mask_ptr = (uint8_t*)source_mask_ptr;

        uint8_t *out_depth_low_ptr = (uint8_t*)dest_ptr;
        uint8_t *out_depth_high_ptr = (uint8_t*)dest_ptr+color_x_shift;
        uint8_t *out_color_ptr = (uint8_t*)dest_ptr+color_x_shift+mask_x_shift;

        for (x=top_x; x<width_x; ++x)
        {
          uint8_t depth_pix_low;
          uint8_t depth_pix_high;

          if (*depth_ptr==*depth_ptr)
          {
            depth_pix_low = std::min(std::max(0.0f,(*depth_ptr/2.0f)*(float)0xFF), (float)0xFF);
            depth_pix_high = std::min(std::max(0.0f,((*depth_ptr-2.0f)/2.0f)*(float)0xFF), (float)0xFF);
          } else
          {
            depth_pix_low = 0;
            depth_pix_high = 0;

          }

          if (*out_mask_ptr>0)
          {
            uint8_t *mast_ptr = out_depth_low_ptr+mask_x_shift;
            memset(mast_ptr, 0xFF, pix_size);
          }

          ++out_mask_ptr;
          ++depth_ptr;

          *out_depth_low_ptr = depth_pix_low; ++out_depth_low_ptr;
          *out_depth_low_ptr = depth_pix_low; ++out_depth_low_ptr;
          *out_depth_low_ptr = depth_pix_low; ++out_depth_low_ptr;

          *out_depth_high_ptr = depth_pix_high; ++out_depth_high_ptr;
          *out_depth_high_ptr = depth_pix_high; ++out_depth_high_ptr;
          *out_depth_high_ptr = depth_pix_high; ++out_depth_high_ptr;

          if (color_ptr)
          {
            *out_color_ptr = *color_ptr;
            ++color_ptr; ++out_color_ptr;

            *out_color_ptr = *color_ptr;
            ++color_ptr; ++out_color_ptr;

            *out_color_ptr = *color_ptr;
            ++color_ptr; ++out_color_ptr;
          }
        }
      }
    }

    {

    }

    pub_.publish(output_msg);

  }

  // encapsulate depth, point mask & color information in a single frame
  void depthInterpolation(sensor_msgs::ImageConstPtr depth_msg,
                          sensor_msgs::ImagePtr depth_int_msg,
                          sensor_msgs::ImagePtr mask_msg)
  {
    const std::size_t input_width = depth_msg->width;
    const std::size_t input_height = depth_msg->height;

    // output image message
    depth_int_msg->header = depth_msg->header;
    depth_int_msg->encoding = depth_msg->encoding;
    depth_int_msg->width = input_width;
    depth_int_msg->height = input_height;
    depth_int_msg->step = depth_msg->step;
    depth_int_msg->is_bigendian = depth_msg->is_bigendian;
    depth_int_msg->data.resize(depth_int_msg->step * depth_int_msg->height, 0);

    mask_msg->header = depth_msg->header;
    mask_msg->encoding = enc::TYPE_8UC1;
    mask_msg->width = input_width;
    mask_msg->height = input_height;
    mask_msg->step = depth_msg->step;
    mask_msg->is_bigendian = depth_msg->is_bigendian;
    mask_msg->data.resize(mask_msg->step * mask_msg->height, 0);

    const float* depth_ptr = (const float*)&depth_msg->data[0];
    float* depth_int_ptr = (float*)&depth_int_msg->data[0];
    uint8_t* mask_ptr = (uint8_t*)&mask_msg->data[0];

    float leftVal = 0.0f;

    unsigned int y, len;
    for (y = 0; y < input_height; ++y, depth_ptr+=input_width,
                                       depth_int_ptr+=input_width,
                                       mask_ptr+=input_width)
    {
      const float* in_depth_ptr = depth_ptr;
      float* out_depth_int_ptr = depth_int_ptr;
      uint8_t* out_mask_ptr = mask_ptr;

//      float leftVal = 0.0f;

      const float* in_depth_end_ptr = depth_ptr+input_width;

      while (in_depth_ptr < in_depth_end_ptr)
      {
        len = 0;
        const float* last_valid_pix_ptr = in_depth_ptr;
        while ((isnan(*in_depth_ptr) || (*in_depth_ptr==0) ) && (in_depth_ptr < in_depth_end_ptr))
        {
          ++in_depth_ptr;
          ++len;
        }
        if (len>0)
        {
          // we found a NaN hole

          // find valid pixel on right side of hole
          float rightVal;
          if (in_depth_ptr < in_depth_end_ptr)
          {
            rightVal = *in_depth_ptr;
          }
          else
          {
            rightVal = leftVal;
          }

          // find valid pixel on left side of hole
          if (isnan(leftVal) || (leftVal==0.0f))
          {
            leftVal = rightVal;
          }


          float incVal = (rightVal-leftVal)/(float)len;
          float fillVal = leftVal;
          const float* fill_ptr;

          for (fill_ptr = last_valid_pix_ptr; fill_ptr < in_depth_ptr; ++fill_ptr)
          {
            *out_depth_int_ptr = fillVal;
            ++out_depth_int_ptr;

            *out_mask_ptr = 255;
            ++out_mask_ptr;

            fillVal += incVal;
          }

          leftVal =  rightVal;
        }
        else
        {
          leftVal =  *in_depth_ptr;

          *out_depth_int_ptr = *in_depth_ptr;

          ++out_mask_ptr;
          ++in_depth_ptr;
          ++out_depth_int_ptr;
        }

      }

    }

  }


protected:
  ros::NodeHandle nh_;

  // ROS stuff
  image_transport::ImageTransport depth_it_;
  boost::shared_ptr<image_transport::SubscriberFilter > depth_sub_;

  image_transport::ImageTransport color_it_;
  boost::shared_ptr<image_transport::SubscriberFilter > color_sub_;

  typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy_depth_color_;
  typedef message_filters::Synchronizer<sync_policy_depth_color_> synchronizer_depth_color_;

  boost::shared_ptr<synchronizer_depth_color_> sync_depth_color_;

  image_transport::ImageTransport pub_it_;
  image_transport::Publisher pub_;

};



int main(int argc, char* argv[])
{
  ros::init(argc, argv, "webGLpointcloudTopicConverter");

  DepthRGBEncoder depth_enc;

  ros::spin();

  return 0;
}

