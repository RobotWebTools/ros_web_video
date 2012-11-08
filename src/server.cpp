//
// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "server.h"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <vector>

namespace ros_http_video_streamer
{

server::server(const ServerConfiguration& server_conf, std::size_t thread_pool_size)
  : server_conf_(server_conf),
    thread_pool_size_(thread_pool_size),
    acceptor_(io_service_),
    encoder_manager_(),
    new_connection_(new connection(io_service_, encoder_manager_, server_conf))
{
  // Open the acceptor with the option to reuse the address (i.e. SO_REUSEADDR).
  boost::asio::ip::tcp::resolver resolver(io_service_);
  boost::asio::ip::tcp::resolver::query query("0.0.0.0",
                                              boost::lexical_cast<std::string>(server_conf.port_));
  boost::asio::ip::tcp::endpoint endpoint = *resolver.resolve(query);
  acceptor_.open(endpoint.protocol());
  acceptor_.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
  acceptor_.bind(endpoint);
  acceptor_.listen();
  acceptor_.async_accept(new_connection_->socket(),
      boost::bind(&server::handle_accept, this,
        boost::asio::placeholders::error));
}

void server::run()
{
  // Create a pool of threads to run all of the io_services.
  std::vector<boost::shared_ptr<boost::thread> > threads;
  for (std::size_t i = 0; i < thread_pool_size_; ++i)
  {
    boost::shared_ptr<boost::thread> thread(new boost::thread(
          boost::bind(&boost::asio::io_service::run, &io_service_)));
    threads.push_back(thread);
  }

  // Wait for all threads in the pool to exit.
  for (std::size_t i = 0; i < threads.size(); ++i)
    threads[i]->join();
}

void server::stop()
{
  io_service_.stop();
}

void server::handle_accept(const boost::system::error_code& e)
{
  if (!e)
  {

   // connection_threads_.push_back( boost::shared_ptr<boost::thread>( new boost::thread( boost::bind(&connection::start, new_connection_ )  ) ) );

    new_connection_->start();
    new_connection_.reset(new connection(io_service_, encoder_manager_, server_conf_));
    acceptor_.async_accept(new_connection_->socket(),
        boost::bind(&server::handle_accept, this,
          boost::asio::placeholders::error));
  }
}

} // ros_http_video_streamer

