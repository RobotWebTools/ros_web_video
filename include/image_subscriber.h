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
 * image_subscriber.h
 *
 *  Created on: Oct 30, 2012
 *      Author: jkammerl
 */

#ifndef ROS_IMAGE_SUBSCRIBE_H_
#define ROS_IMAGE_SUBSCRIBE_H_

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <deque>


#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>


namespace ros_http_video_streamer
{

  //////////////////////////////////////////////////////////
  // This class subscribes to a ROS image publisher
  // and manages a queue of received images
  //////////////////////////////////////////////////////////
  class ImageSubscriber
  {
  public:

    typedef boost::shared_ptr<ImageSubscriber> ptr;

    // Specify ROS topic in constructor
    ImageSubscriber(const std::string& topic, const std::string& transport);
    virtual ~ImageSubscriber();

    // get image from receiver queue
    void getImageFromQueue( sensor_msgs::ImageConstPtr& frame);

    // empty queue
    void emptyQueue( );

  private:
    // ROS subscriber callback
    void image_cb(const sensor_msgs::ImageConstPtr& msg);
    // reset connection
    void reset();
    // subscribe
    void subscribe();

    // ROS topic
    const std::string topic_;
    // ROS transport scheme
    const std::string transport_;

    // ROS node handle
    ros::NodeHandle nh;
    // ROS image transport & subscriber
    image_transport::ImageTransport it;
    boost::shared_ptr<image_transport::SubscriberFilter> sub;

    // mutex to protect image queue
    boost::mutex frame_mutex_;
    // image queue
    std::deque<sensor_msgs::ImageConstPtr> frame_queue_;

  };

} // ros_http_video_streamer

#endif /* ROS_IMAGE_SUBSCRIBE_H_ */
