//
// header.hpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef HTTP_SERVER3_HEADER_H
#define HTTP_SERVER3_HEADER_H

#include <string>

namespace ros_http_video_streamer
{

struct header
{
  std::string name;
  std::string value;
};

} // ros_http_video_streamer

#endif // HTTP_SERVER3_HEADER_HPP
