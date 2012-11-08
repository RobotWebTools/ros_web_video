//
// request.hpp
// ~~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef HTTP_SERVER3_REQUEST_H
#define HTTP_SERVER3_REQUEST_H

#include <string>
#include <vector>
#include "header.h"

namespace ros_http_video_streamer
{

/// A request received from a client.
struct request
{
  std::string method;
  std::string uri;
  int http_version_major;
  int http_version_minor;
  std::vector<header> headers;
};

} // ros_http_video_streamer

#endif // HTTP_SERVER3_REQUEST_HPP
