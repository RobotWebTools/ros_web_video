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
#include "server.h"

#include "ros/ros.h"

#include "server_configuration.h"

void showConfig(const ros_http_video_streamer::ServerConfiguration& config)
{
  std::cout << "ROS Http Video Streamer" << std::endl << std::endl;
  std::cout << "Settings:" << std::endl;
  std::cout << "        Port: " << config.port_ << std::endl;
  std::cout << "        Codec: " << config.codec_ << std::endl;
  std::cout << "        Bitrate: " << config.bitrate_ << std::endl;
  std::cout << "        Framerate: " << config.framerate_ << std::endl;
  std::cout << "        Codec profile: " << config.profile_ << std::endl;
  std::cout << "        Fileserver Root: " << config.wwwroot_ << std::endl;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ROS_Http_Video_Streamer");

  // Default server configuration
  ros_http_video_streamer::ServerConfiguration server_conf;

  // ROS parameters
  ros::NodeHandle priv_nh_("~");

  // read network port from param server
  priv_nh_.param<int>("port", server_conf.port_, server_conf.port_);

  // read default codec from param server
  priv_nh_.param<std::string>("codec", server_conf.codec_, server_conf.codec_);

  // read default bitrate from param server
  priv_nh_.param<int>("bitrate", server_conf.bitrate_, server_conf.bitrate_);

  // read default frame rate from param server
  priv_nh_.param<int>("framerate", server_conf.framerate_, server_conf.framerate_);

  // read quality parameter from param server
  priv_nh_.param<int>("quality", server_conf.quality_, server_conf.quality_);

  // read group of pictures from param server
  priv_nh_.param<int>("gop", server_conf.gop_, server_conf.gop_);

  // read default encoding profile from param server
  priv_nh_.param<std::string>("profile", server_conf.profile_, server_conf.profile_);

  // read default encoding profile from param server
  priv_nh_.param<std::string>("wwwroot", server_conf.wwwroot_, server_conf.wwwroot_);

  // read default ROS transport scheme
  priv_nh_.param<std::string>("transport", server_conf.ros_transport_, server_conf.ros_transport_);

  showConfig(server_conf);

  try
  {

    // Run server in background thread.
    ros_http_video_streamer::server s(server_conf, 10);
    boost::thread t(boost::bind(&ros_http_video_streamer::server::run, &s));

    ros::spin();

    // Stop the server.
    s.stop();
    t.join();

  }
  catch (std::exception& e)
  {
    std::cerr << "exception: " << e.what() << "\n";
  }

  return 0;
}

