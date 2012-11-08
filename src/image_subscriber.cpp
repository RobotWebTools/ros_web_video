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
 * image_subscriber.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: jkammerl
 */

#include "image_subscriber.h"

// Size of image queue -
// a large queue might increase the latency on the video stream
#define IMAGE_BUFFER_SIZE 1

namespace ros_http_video_streamer
{

ImageSubscriber::ImageSubscriber(const std::string& topic) :
    topic_(topic),
    nh(""),
    it(nh),
    sub(),
    frame_mutex_(),
    frame_queue_()
{
  if (!topic.empty())
  {
    reset();
  }
}

ImageSubscriber::~ImageSubscriber()
{
  sub.reset();
}

void ImageSubscriber::emptyQueue()
{
  boost::mutex::scoped_lock lock(frame_mutex_);

  frame_queue_.clear();
}

void ImageSubscriber::image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  boost::mutex::scoped_lock lock(frame_mutex_);

  frame_queue_.push_front(msg);
}

void ImageSubscriber::reset()
{
  sub.reset(new image_transport::SubscriberFilter());
  sub->subscribe(it, topic_, 1, image_transport::TransportHints("raw"));
  sub->registerCallback(boost::bind(&ImageSubscriber::image_cb, this, _1));
}

void ImageSubscriber::getImageFromQueue(sensor_msgs::ImageConstPtr& frame)
{
  frame.reset();
  {
    boost::mutex::scoped_lock lock(frame_mutex_);

    // if queue not empty..
    if (frame_queue_.size())
    {
      if (frame_queue_.size() > IMAGE_BUFFER_SIZE)
        frame_queue_.resize(IMAGE_BUFFER_SIZE);

      // pop frame from back of queue
      frame = frame_queue_.back();
      frame_queue_.pop_back();
    }
  }
}

} // ros_http_video_streamer
