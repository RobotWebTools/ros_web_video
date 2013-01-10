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
 * connection.h
 *
 *  Created on: Oct 30, 2012
 *      Author: jkammerl
 *
 *  & partly:
 *  Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
 */

#ifndef HTTP_CONNECTION_H
#define HTTP_CONNECTION_H

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include "reply.h"
#include "request.h"
#include "request_parser.h"

#include "ffmpeg_encoder.h"
#include "encoder_manager.h"

#include "server_configuration.h"

#define STREAM_PATH "/streams"
#define WEB_PATH "/www"

//#define HTTP_TRANSFER_ENCODING

namespace ros_http_video_streamer
{

/// Represents a single connection from a client.
class connection
  : public boost::enable_shared_from_this<connection>,
    private boost::noncopyable
{
public:
  /// Construct a connection with the given io_service.
  explicit connection(boost::asio::io_service& io_service,
                      EncoderManager& encoder_manager,
                      const ServerConfiguration& default_server_conf);
  virtual ~connection();

  /// Get the socket associated with the connection.
  boost::asio::ip::tcp::socket& socket();

  /// Start the first asynchronous operation for the connection.
  void start();

private:
  // Get list of available image topics
  void getImageTopics();

  // send http headers for data streaming
  void sendHTTPStreamingHeaders();

  // Worker thread for streaming http data to the client
  void streamingWorkerThread(const std::string& topic,
                             const ServerConfiguration& config);

  // Generate a HTML page showing a list of available image topics
  void generateImageTopicHTML();

  // Generate a HTML page that displays a video stream
  void generateVideoStreamHTML(const std::string& image_topic,
                               const ServerConfiguration& config);

  // Parse streaming parameters from URL
  void getStreamingParametersFromURL(const std::string url, ServerConfiguration& config);

  /// Handle completion of a read operation.
  void handleRead(const boost::system::error_code& e,
      std::size_t bytes_transferred);

  /// Handle completion of a write operation.
  void handleWrite(const boost::system::error_code& e);

  /// Perform URL-decoding on a string. Returns false if the encoding was
  /// invalid.
  static bool urlDecode(const std::string& in, std::string& out);

  // find mime extension for file type
  std::string mimeExtensionToType(const std::string& extension);

  // Default server configutaion
  const ServerConfiguration& server_conf_;

  // Vector of available image topics
  std::vector<std::string> image_topics_;

  /// Strand to ensure the connection's handlers are not called concurrently.
  boost::asio::io_service::strand strand_;

  /// Socket for the connection.
  boost::asio::ip::tcp::socket socket_;

  /// Buffer for incoming data.
  boost::array<char, 8192> buffer_;

  /// The incoming request.
  request request_;

  /// The parser for the incoming request.
  request_parser request_parser_;

  /// The reply to be sent back to the client.
  reply reply_;

  // Encoder manager used to lookup existing encoder instances
  EncoderManager& encoder_manager_;

  // streaming thread
  boost::shared_ptr<boost::thread> streaming_thread_;

  // streaming control
  bool do_streaming_;

};

typedef boost::shared_ptr<connection> connection_ptr;


} // ros_http_video_streamer

#endif // HTTP_SERVER3_CONNECTION_HPP
