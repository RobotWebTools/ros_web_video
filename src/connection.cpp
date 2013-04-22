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
 * connection.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: jkammerl
 *
 *  & partly:
 *  Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
 */

#include "connection.h"

#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/any.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "ros/master.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

namespace ros_http_video_streamer
{

namespace misc_strings {

const char name_value_separator[] = { ':', ' ' };
const char crlf[] = { '\r', '\n' };

#ifdef HTTP_TRANSFER_ENCODING
const std::string ok =
  "HTTP/1.1 200 OK\r\n";

} // namespace misc_strings
#else
const std::string ok =
  "HTTP/1.0 200 OK\r\n";

} // namespace misc_strings
#endif

struct mime_map
{
  const char* extension;
  const char* mime_type;
} mime_mapping[] =
{
  { "gif", "image/gif" },
  { "htm", "text/html" },
  { "html", "text/html" },
  { "jpg", "image/jpeg" },
  { "png", "image/png" },
  { "css", "text/css" },
  { "js", "text/javascript" },
  { 0, 0 } // Marks end of list.
};

connection::connection(boost::asio::io_service& io_service,
                       EncoderManager& encoder_manager,
                       const ServerConfiguration& default_server_conf)
  : server_conf_(default_server_conf),
    image_topics_(),
    strand_(io_service),
    socket_(io_service),
    buffer_(),
    request_(),
    request_parser_(),
    reply_(),
    encoder_manager_(encoder_manager),
    streaming_thread_(),
    do_streaming_(false)
{
}

connection::~connection()
{
  do_streaming_ = false;

  if (streaming_thread_)
    streaming_thread_->join();

}

// send http headers for data streaming
void connection::sendHTTPStreamingHeaders()
{
  std::vector<boost::asio::const_buffer> buffers;

  buffers.push_back(boost::asio::buffer(misc_strings::ok));

#ifdef HTTP_HEADERS

  buffers.push_back(boost::asio::buffer("Date"));
  buffers.push_back(boost::asio::buffer(misc_strings::name_value_separator));

  std::ostringstream datetime_ss;
  boost::posix_time::time_facet * p_time_output = new boost::posix_time::time_facet;
  std::locale special_locale (std::locale(""), p_time_output);
  datetime_ss.imbue (special_locale);
  (*p_time_output).format("%a, %d %b %Y %H:%M:%S%F %z"); // date time
  datetime_ss << boost::posix_time::second_clock::local_time();

  buffers.push_back(boost::asio::buffer(datetime_ss.str().c_str()));
  buffers.push_back(boost::asio::buffer(misc_strings::crlf));

  buffers.push_back(boost::asio::buffer("Content-Type"));
  buffers.push_back(boost::asio::buffer(misc_strings::name_value_separator));
  buffers.push_back(boost::asio::buffer("video/webm"));
  buffers.push_back(boost::asio::buffer(misc_strings::crlf));


  buffers.push_back(boost::asio::buffer("Cache-Control"));
  buffers.push_back(boost::asio::buffer(misc_strings::name_value_separator));
  buffers.push_back(boost::asio::buffer("no-cache"));
  buffers.push_back(boost::asio::buffer(misc_strings::crlf));

  buffers.push_back(boost::asio::buffer("Connection"));
  buffers.push_back(boost::asio::buffer(misc_strings::name_value_separator));
  buffers.push_back(boost::asio::buffer("Close"));
  buffers.push_back(boost::asio::buffer(misc_strings::crlf));

  buffers.push_back(boost::asio::buffer("Access-Control-Allow-Origin"));
  buffers.push_back(boost::asio::buffer(misc_strings::name_value_separator));
  buffers.push_back(boost::asio::buffer("*"));
  buffers.push_back(boost::asio::buffer(misc_strings::crlf));

  buffers.push_back(boost::asio::buffer("Pragma"));
  buffers.push_back(boost::asio::buffer(misc_strings::name_value_separator));
  buffers.push_back(boost::asio::buffer("no-cache"));
  buffers.push_back(boost::asio::buffer(misc_strings::crlf));

  buffers.push_back(boost::asio::buffer("Expires"));
  buffers.push_back(boost::asio::buffer(misc_strings::name_value_separator));
  buffers.push_back(boost::asio::buffer("0"));
  buffers.push_back(boost::asio::buffer(misc_strings::crlf));

  buffers.push_back(boost::asio::buffer("Max-Age"));
  buffers.push_back(boost::asio::buffer(misc_strings::name_value_separator));
  buffers.push_back(boost::asio::buffer("0"));
  buffers.push_back(boost::asio::buffer(misc_strings::crlf));

  buffers.push_back(boost::asio::buffer("Trailer"));
  buffers.push_back(boost::asio::buffer(misc_strings::name_value_separator));
  buffers.push_back(boost::asio::buffer("Expires"));
  buffers.push_back(boost::asio::buffer(misc_strings::crlf));


  #ifdef HTTP_TRANSFER_ENCODING
    buffers.push_back(boost::asio::buffer("Transfer-Encoding"));
    buffers.push_back(boost::asio::buffer(misc_strings::name_value_separator));
    buffers.push_back(boost::asio::buffer("chunked"));
    buffers.push_back(boost::asio::buffer(misc_strings::crlf));
  #endif

#endif

  buffers.push_back(boost::asio::buffer(misc_strings::crlf));

  boost::asio::write(socket_, buffers);
}

void connection::streamingWorkerThread( const std::string& topic,
                                        const ServerConfiguration& config)
{
  FFMPEGEncoder image_encoder("WebEncoder", topic, config);

#ifdef HTTP_TRANSFER_ENCODING
  char hexSize[256];
#endif

   std::vector<uint8_t> header;

  if (image_encoder.initEncoding(header) >= 0)
  {

    std::vector<boost::asio::const_buffer> buffers;
    sendHTTPStreamingHeaders();

#ifdef HTTP_TRANSFER_ENCODING
    sprintf(hexSize, "%X", (unsigned int)header.size());
    buffers.push_back(boost::asio::buffer(hexSize, strlen(hexSize)));
    buffers.push_back(boost::asio::buffer(misc_strings::crlf));
#endif

    buffers.push_back(boost::asio::buffer(header));
#ifdef HTTP_TRANSFER_ENCODING
    buffers.push_back(boost::asio::buffer(misc_strings::crlf));
#endif

    try
    {
      boost::asio::write(socket_, buffers);
      ROS_DEBUG("Video header sent (%d bytes)", (int) header.size());

      std::vector<uint8_t> packet;

      while (do_streaming_)
      {
        buffers.clear();
        packet.clear();

        image_encoder.getVideoPacket(packet);
        ROS_DEBUG("Video packet sent (%d bytes)", (int) packet.size());

#ifdef HTTP_TRANSFER_ENCODING
        sprintf(hexSize, "%X", (unsigned int)packet.size());
        buffers.push_back(boost::asio::buffer(hexSize, strlen(hexSize)));
        buffers.push_back(boost::asio::buffer(misc_strings::crlf));
#endif

        buffers.push_back(boost::asio::buffer(packet));
#ifdef HTTP_TRANSFER_ENCODING
        buffers.push_back(boost::asio::buffer(misc_strings::crlf));
#endif

        boost::asio::write(socket_, buffers);
      }
    }
    catch (const boost::system::system_error& err)
    {
      do_streaming_ = false;
    }

  }

   streaming_thread_.reset();

   ROS_DEBUG("Video encoding thread endded");
}

void connection::getImageTopics()
{
  image_topics_.clear();

  std::string image_message_type = ros::message_traits::datatype<sensor_msgs::Image>();

  ros::master::V_TopicInfo topics;
  ros::master::getTopics( topics );

  // Loop through all published topics
  ros::master::V_TopicInfo::iterator it;
  for( it = topics.begin(); it != topics.end(); ++it )
  {
    const ros::master::TopicInfo& topic = *it;

    // Only add topics whose type matches.
    if ( ( topic.datatype == image_message_type ) &&
          (topic.name.find("_raw")==std::string::npos ) )
    {
      image_topics_.push_back(topic.name);
    }
  }
}


void connection::generateImageTopicHTML()
{
  reply_.content =
      "<html>"
      "<head><title>ROS Image Topic List</title></head>"
      "<body><h1>Available ROS Image Topics:</h1>";

  BOOST_FOREACH( const std::string& topic_name, image_topics_ )
  {
    reply_.content += "<p><a href=";
    reply_.content += WEB_PATH;
    reply_.content += topic_name+">"+topic_name+"</a></p>";
  }
  reply_.content +=
 //     "<a href=\"/webgl_pointcloud_streaming.html\">WebGL-based pointcloud decoder</a>"
      "</body>"
      "</html>";

  reply_.status = reply::ok;
  reply_.headers.resize(2);
  reply_.headers[0].name = "Content-Length";
  reply_.headers[0].value = boost::lexical_cast<std::string>(reply_.content.size());
  reply_.headers[1].name = "Content-Type";
  reply_.headers[1].value = "text/html";
}

void connection::generateVideoStreamHTML(const std::string& image_topic,
                                         const ServerConfiguration& config)
{
  reply_.content =

      "<!DOCTYPE html>"
      "<html>"
      "<head>"
          "<meta charset=\"utf-8\">"
              "<title>ROS Topic: ";

  reply_.content += image_topic;

  reply_.content +=
               "</title>"
      "</head>"
      "<body>"
              "<div id=\"wrapper\">"
                              "<h1>Video stream of ROS topic: ";

  reply_.content += image_topic;

  reply_.content +=
                               "</h1>"
              "<div id=\"content\">"

                  "<div id=\"movie\">"
                          "<video src=\"";

  //reply_.content += server_conf_.address_+":"+boost::lexical_cast<std::string>(config.port_);
  reply_.content += STREAM_PATH;
  reply_.content += image_topic;
  reply_.content +=".webm?enc=";
  reply_.content +=config.codec_;
  reply_.content +="&bitrate=";
  reply_.content +=boost::lexical_cast<std::string>( config.bitrate_ );
  reply_.content +="&framerate=";
  reply_.content +=boost::lexical_cast<std::string>( config.framerate_ );

  reply_.content +=

                                      "\" autoplay=\"true\" preload=\"none\" >  "
                          "</video>"
                  "</div>"
              "</div>"
          "</div>"
      "</body>"
      "</html>";

  reply_.status = reply::ok;
  reply_.headers.resize(2);
  reply_.headers[0].name = "Content-Length";
  reply_.headers[0].value = boost::lexical_cast<std::string>(reply_.content.size());
  reply_.headers[1].name = "Content-Type";
  reply_.headers[1].value = "text/html";
}

void connection::getStreamingParametersFromURL(const std::string url, ServerConfiguration& config)
{
  std::vector<std::string> parameters;
  boost::split(parameters,url,boost::is_any_of("&"));
  BOOST_FOREACH( const std::string& p, parameters )
  {
    std::vector<std::string> setting;
    boost::split(setting,p,boost::is_any_of("="));

    if (setting.size()==2)
    {
      try {
        if (!setting[0].compare("bitrate"))
        {
          config.bitrate_ = boost::lexical_cast<int>( setting[1] );
        } else
        if (!setting[0].compare("framerate"))
        {
          config.framerate_ = boost::lexical_cast<int>( setting[1] );
        } else
        if (!setting[0].compare("enc"))
        {
          config.codec_ = setting[1] ;
        } else
        if (!setting[0].compare("profile"))
        {
          config.profile_ = setting[1] ;
        } else
        if (!setting[0].compare("width"))
        {
          config.frame_width_ = boost::lexical_cast<int>( setting[1] );
        }
        else
        if (!setting[0].compare("height"))
        {
          config.frame_height_ = boost::lexical_cast<int>( setting[1] );
        }
        else
        if (!setting[0].compare("quality"))
        {
          config.quality_ = boost::lexical_cast<int>( setting[1] );
        }
        else
        if (!setting[0].compare("gop"))
        {
          config.gop_ = boost::lexical_cast<int>( setting[1] );
        } else
        {
          ROS_WARN_STREAM("Invalid configuration parameter in http request: "<<setting[1]);
        }
      } catch (boost::bad_lexical_cast& e)  {}
    }
  }

}


boost::asio::ip::tcp::socket& connection::socket()
{
  return socket_;
}

void connection::start()
{

  socket_.async_read_some(boost::asio::buffer(buffer_),
      strand_.wrap(
        boost::bind(&connection::handleRead, shared_from_this(),
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred)));

}

std::string connection::mimeExtensionToType(const std::string& extension)
{
  for (mime_map* m = mime_mapping; m->extension; ++m)
  {
    if (m->extension == extension)
    {
      return m->mime_type;
    }
  }

  return "text/plain";
}

void connection::handleRead(const boost::system::error_code& e,
    std::size_t bytes_transferred)
{

  if (!e)
  {
    boost::tribool result;
    boost::tie(result, boost::tuples::ignore) = request_parser_.parse(
        request_, buffer_.data(), buffer_.data() + bytes_transferred);
        
    const std::string remote_ip = boost::lexical_cast<std::string>(socket_.remote_endpoint());

    if (result)
    {
      ROS_DEBUG("Http request received from %s: %s", remote_ip.c_str(), request_.uri.c_str());

      std::string request_path;
      // update list of available image topics
      getImageTopics();

      if (!urlDecode(request_.uri, request_path))
      {
        reply_ = reply::stock_reply(reply::bad_request);
      } else
      if (request_path[request_path.size() - 1] == '/')
      {
        // generate a HTML page showing image topics
        generateImageTopicHTML();
      }
      else if (!request_path.empty() && request_path.find(STREAM_PATH) != std::string::npos)
      {
        std::string request_topic = request_path.substr(strlen(STREAM_PATH));

        std::string image_topic = request_topic.substr(0, request_topic.find(".webm"));
        std::string parameter_req = request_topic.substr(request_topic.find("?")+1);

        // check for requested stream
        bool stream_found = false;
        BOOST_FOREACH( const std::string& topic_name, image_topics_ )
        {
          if (!image_topic.compare(topic_name))
            stream_found = true;
        }

        if (stream_found)
        {
          ServerConfiguration config = server_conf_;

          // get stream parameters from URL
          getStreamingParametersFromURL(parameter_req, config);

          // start streaming thread
          do_streaming_ = true;
          streaming_thread_ = boost::shared_ptr<boost::thread>(
              new boost::thread( boost::bind(&connection::streamingWorkerThread, shared_from_this(), image_topic, config )  ) );

          ROS_INFO("Starting encoder for topic %s (codec: %s, bitrate: %d, framerate: %d)",
                   image_topic.c_str(),
                   config.codec_.c_str(),
                   config.bitrate_,
                   config.framerate_);

          return;
        }
      }
      else if (!request_path.empty() && request_path.find(WEB_PATH) != std::string::npos)
      {
        std::string request_topic = request_path.substr(strlen(WEB_PATH));

        std::string image_topic = request_topic.substr(0, request_topic.find("?"));

        bool stream_found = false;
        BOOST_FOREACH( const std::string& topic_name, image_topics_ )
        {
          if (!image_topic.compare(topic_name))
            stream_found = true;
        }

        if (stream_found)
        {
          ROS_INFO("Requesting www for %s", image_topic.c_str());

          // display HTML page that displays a video stream
          generateVideoStreamHTML(image_topic, server_conf_);

        } else
        {
          reply_ = reply::stock_reply(reply::not_found);
        }
      } else
      {

        // Determine the file extension.
        std::size_t last_slash_pos = request_path.find_last_of("/");
        std::size_t last_dot_pos = request_path.find_last_of(".");
        std::string extension;
        if (last_dot_pos != std::string::npos && last_dot_pos > last_slash_pos)
        {
          extension = request_path.substr(last_dot_pos + 1);
        }

        // Open the file to send back.
        std::string full_path = server_conf_.wwwroot_ + request_path;
        std::ifstream is(full_path.c_str(), std::ios::in | std::ios::binary);
        if (!is)
        {
          reply_ = reply::stock_reply(reply::not_found);
          ROS_INFO("Http request from client %s: %s - file not found ", remote_ip.c_str(), request_.uri.c_str(), full_path.c_str());
          return;
        }

        // Fill out the reply to be sent to the client.
        reply_.status = reply::ok;
        char buf[2048];
        while (is.read(buf, sizeof(buf)).gcount() > 0)
          reply_.content.append(buf, is.gcount());
        reply_.headers.resize(2);
        reply_.headers[0].name = "Content-Length";
        reply_.headers[0].value = boost::lexical_cast < std::string > (reply_.content.size());
        reply_.headers[1].name = "Content-Type";
        reply_.headers[1].value = mimeExtensionToType(extension);
      }

  //    request_handler_.handle_request(request_path, reply_);
      boost::asio::async_write(socket_, reply_.to_buffers(),
          strand_.wrap(
            boost::bind(&connection::handleWrite, shared_from_this(),
              boost::asio::placeholders::error)));
    }
    else if (!result)
    {
      reply_ = reply::stock_reply(reply::bad_request);
      boost::asio::async_write(socket_, reply_.to_buffers(),
          strand_.wrap(
            boost::bind(&connection::handleWrite, shared_from_this(),
              boost::asio::placeholders::error)));
    }
    else
    {
      socket_.async_read_some(boost::asio::buffer(buffer_),
          strand_.wrap(
            boost::bind(&connection::handleRead, shared_from_this(),
              boost::asio::placeholders::error,
              boost::asio::placeholders::bytes_transferred)));
    }
  } else
  {
    ROS_INFO("Http read error: %s", e.message().c_str());
  }

  // If an error occurs then no new asynchronous operations are started. This
  // means that all shared_ptr references to the connection object will
  // disappear and the object will be destroyed automatically after this
  // handler returns. The connection class's destructor closes the socket.
}


void connection::handleWrite(const boost::system::error_code& e)
{
  if (!e)
  {
    // Initiate graceful connection closure.
    boost::system::error_code ignored_ec;
    socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ignored_ec);
  }

  // No new asynchronous operations are started. This means that all shared_ptr
  // references to the connection object will disappear and the object will be
  // destroyed automatically after this handler returns. The connection class's
  // destructor closes the socket.
}

bool connection::urlDecode(const std::string& in, std::string& out)
{
  out.clear();
  out.reserve(in.size());
  for (std::size_t i = 0; i < in.size(); ++i)
  {
    if (in[i] == '%')
    {
      if (i + 3 <= in.size())
      {
        int value = 0;
        std::istringstream is(in.substr(i + 1, 2));
        if (is >> std::hex >> value)
        {
          out += static_cast<char>(value);
          i += 2;
        }
        else
        {
          return false;
        }
      }
      else
      {
        return false;
      }
    }
    else if (in[i] == '+')
    {
      out += ' ';
    }
    else
    {
      out += in[i];
    }
  }
  return true;
}

} // ros_http_video_streamer
