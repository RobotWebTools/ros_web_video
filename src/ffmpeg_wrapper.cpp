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
 * ffmpeg_wrapper.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: jkammerl
 */

#include <stdlib.h>
#include <stdio.h>
#include <string>

#include <set>

#include "ffmpeg_wrapper.h"
#include <boost/bind.hpp>

#ifdef HAVE_AV_CONFIG_H
#undef HAVE_AV_CONFIG_H
#endif

#ifdef __cplusplus
 #define __STDC_CONSTANT_MACROS
 #ifdef _STDINT_H
  #undef _STDINT_H
 #endif
 # include <stdint.h>
#endif

#define MILLISEC_FOR_FFMPEG_INIT (5*1000)

// include ffmpeg C headers

namespace ros_http_video_streamer
{

extern "C"
{
        #include <libavcodec/avcodec.h>
        #include <libavformat/avformat.h>
        #include "libavutil/intreadwrite.h"
        #include "libavformat/avio.h"
        #include <libswscale/swscale.h>
        #include "libavutil/opt.h"
}


FFMPEG_Wrapper::FFMPEG_Wrapper() :
    ffmpeg_codec_(0),
    ffmpeg_output_format_(0),
    ffmpeg_codec_context_(0),
    ffmpeg_format_context_(0),
    ffmpeg_src_picture_(new AVPicture),
    ffmpeg_dst_picture_(new AVPicture),
    ffmpeg_video_st_(0),
    ffmpeg_frame_(0),
    ffmpeg_video_pts_(0.0),
    ffmpeg_sws_ctx_(0),
    config_(),
    input_width_(-1),
    input_height_(-1),
    output_width_(-1),
    output_height_(-1),
    init_(false)
{
}

FFMPEG_Wrapper::~FFMPEG_Wrapper()
{
  shutdown();
}

static int ff_lockmgr(void **mutex, enum AVLockOp op)
{
  if (NULL == mutex)
    return -1;

  switch (op)
  {
    case AV_LOCK_CREATE:
    {
      *mutex = NULL;
      boost::mutex * m = new boost::mutex();
      *mutex = static_cast<void*>(m);
      break;
    }
    case AV_LOCK_OBTAIN:
    {
      boost::mutex * m = static_cast<boost::mutex*>(*mutex);
      m->lock();
      break;
    }
    case AV_LOCK_RELEASE:
    {
      boost::mutex * m = static_cast<boost::mutex*>(*mutex);
      m->unlock();
      break;
    }
    case AV_LOCK_DESTROY:
    {
      boost::mutex * m = static_cast<boost::mutex*>(*mutex);
      m->lock();
      m->unlock();
      delete m;
      break;
    }
    default:
      break;
  }
  return 0;
}



int FFMPEG_Wrapper::init(int input_width,
                          int input_height,
                          const ServerConfiguration& config)
{
  boost::mutex::scoped_lock lock(frame_mutex_);

  time_started_ = boost::posix_time::microsec_clock::local_time();

  config_  = config;

  input_width_ = input_width;
  input_height_ = input_height;

  output_width_ = config.frame_width_;
  output_height_ = config.frame_height_;

  if (output_width_<0)
    output_width_ = input_width_;

  if (output_height_<0)
    output_height_ = input_height_;

  av_lockmgr_register(&ff_lockmgr);

  /* register all the codecs */
  avcodec_register_all();
  av_register_all();

  // lookup webm codec
  avformat_alloc_output_context2(&ffmpeg_format_context_, NULL, config_.codec_.c_str(), NULL);
  if (!ffmpeg_format_context_) {
    return -1;
  }

  ffmpeg_output_format_ = ffmpeg_format_context_->oformat;

  /* Add the audio and video streams using the default format codecs
   * and initialize the codecs. */
  ffmpeg_video_st_ = NULL;
  if (ffmpeg_output_format_->video_codec != AV_CODEC_ID_NONE)
  {

    /* find the video encoder */
    ffmpeg_codec_ = avcodec_find_encoder(ffmpeg_output_format_->video_codec);
    if (!(ffmpeg_codec_))
    {
      fprintf(stderr, "Codec not found (%s)\n",config_.codec_.c_str());
      return -1;
    }

    ffmpeg_video_st_ = avformat_new_stream(ffmpeg_format_context_, ffmpeg_codec_);
    if (!ffmpeg_video_st_)
    {
      fprintf(stderr, "Could not alloc stream\n");
      return -1;
    }

    ffmpeg_codec_context_ = ffmpeg_video_st_->codec;



    avcodec_get_context_defaults3(ffmpeg_codec_context_, ffmpeg_codec_);

    //////////////////////////////////////////////
    // ffmpeg codec configuration
    //////////////////////////////////////////////

    ffmpeg_codec_context_->codec_id = ffmpeg_output_format_->video_codec;
    ffmpeg_codec_context_->bit_rate = config_.bitrate_;

    ffmpeg_codec_context_->width = output_width_;
    ffmpeg_codec_context_->height = output_height_;
    ffmpeg_codec_context_->delay = 0;

    ffmpeg_codec_context_->time_base.den = config_.framerate_+3; //increased framerate to compensate playback delay
    ffmpeg_codec_context_->time_base.num = 1;
    ffmpeg_codec_context_->gop_size = config_.gop_; /* emit one intra ffmpeg_frame_ every twelve frames at most */
    ffmpeg_codec_context_->pix_fmt = PIX_FMT_YUV420P;
    ffmpeg_codec_context_->max_b_frames = 0;

    av_opt_set(ffmpeg_codec_context_->priv_data, "quality", config_.profile_.c_str(), 0);

    av_opt_set(ffmpeg_codec_context_->priv_data, "deadline", "1", 0);
    av_opt_set(ffmpeg_codec_context_->priv_data, "auto-alt-ref", "0", 0);

    // lag in frames
    av_opt_set(ffmpeg_codec_context_->priv_data, "lag-in-frames", "1", 0);
    av_opt_set(ffmpeg_codec_context_->priv_data, "rc_lookahead", "1", 0);

    av_opt_set(ffmpeg_codec_context_->priv_data, "drop_frame", "1", 0);

    // enable error-resilient coding
    av_opt_set(ffmpeg_codec_context_->priv_data, "error-resilient", "1", 0);

    // buffer size of rate controller (length: rc_buffer_size/bitrate * 1000) ms
    int bufsize = 10;//ffmpeg_codec_context_->bit_rate/10;
    ffmpeg_codec_context_->rc_buffer_size = bufsize;
    // prebuffering at decoder
    ffmpeg_codec_context_->rc_initial_buffer_occupancy = bufsize ;//bitrate/3;  

    av_opt_set_int(ffmpeg_codec_context_->priv_data, "bufsize", bufsize, 0);
    av_opt_set_int(ffmpeg_codec_context_->priv_data, "buf-initial", bufsize, 0);
    av_opt_set_int(ffmpeg_codec_context_->priv_data, "buf-optimal", bufsize, 0);

    // buffer agressivity
    ffmpeg_codec_context_->rc_buffer_aggressivity = 0.5;

    // Quality settings
    //ffmpeg_codec_context_->qmin = 50;
    //ffmpeg_codec_context_->qmax = 62;
    if (config_.quality_>0)
      ffmpeg_codec_context_->qmin = config_.quality_;

     //ffmpeg_codec_context_->frame_skip_threshold = 100;

    /* Some formats want stream headers to be separate. */
    if (ffmpeg_format_context_->oformat->flags & AVFMT_GLOBALHEADER)
      ffmpeg_codec_context_->flags |= CODEC_FLAG_GLOBAL_HEADER;
  }

  if (ffmpeg_video_st_)
  {
    int ret;

    /* open the codec */
      {
         boost::mutex::scoped_lock lock(codec_mutex_);
         if (avcodec_open2(ffmpeg_codec_context_, ffmpeg_codec_, NULL) < 0) {
             fprintf(stderr, "Could not open video codec\n");
             return -1;
         }
      }  

      /* allocate and init a re-usable ffmpeg_frame_ */
      ffmpeg_frame_ = avcodec_alloc_frame();
      if (!ffmpeg_frame_) {
          fprintf(stderr, "Could not allocate video ffmpeg_frame_\n");
          return -1;
      }

      /* Allocate the encoded raw picture. */
      ret = avpicture_alloc(ffmpeg_dst_picture_, ffmpeg_codec_context_->pix_fmt, output_width_, output_height_);
      if (ret < 0) {
          fprintf(stderr, "Could not allocate picture\n");
          return -1;
      }

      /* If the output format is not YUV420P, then a temporary YUV420P
       * picture is needed too. It is then converted to the required
       * output format. */
          ret = avpicture_alloc(ffmpeg_src_picture_, AV_PIX_FMT_RGB24, input_width_, input_height_);
          if (ret < 0) {
              fprintf(stderr, "Could not allocate temporary picture\n");
              return -1;
          }

      /* copy data and linesize picture pointers to ffmpeg_frame_ */
      *((AVPicture *)ffmpeg_frame_) = *ffmpeg_dst_picture_;

      av_dump_format(ffmpeg_format_context_, 0, "", 1);

      ffmpeg_output_format_->flags |= AVFMT_NOFILE;

      if (ffmpeg_frame_)
          ffmpeg_frame_->pts = 0;
  }

  init_ = true;

  return 0;
}


void FFMPEG_Wrapper::shutdown()
{
  boost::mutex::scoped_lock lock(frame_mutex_);

  if (init_)
  {

    // give ffmpeg enough time to initialize before shutdown
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration diff = now - time_started_;
    unsigned int milisec_used = std::max(100u,(unsigned int)diff.total_milliseconds());

    if (milisec_used < MILLISEC_FOR_FFMPEG_INIT)
    {
      // if encoder worked faster than the desired frame rate -> go sleeping
      boost::this_thread::sleep(boost::posix_time::milliseconds(MILLISEC_FOR_FFMPEG_INIT - milisec_used));
    }


    // Close the codec
    if (ffmpeg_codec_context_)
    {
      boost::mutex::scoped_lock lock(codec_mutex_);
      avcodec_close(ffmpeg_codec_context_);
    }


    if (ffmpeg_frame_)
      avcodec_free_frame(&ffmpeg_frame_);

    if (ffmpeg_format_context_)
    {
      /* Free the streams. */
   /*   for (i = 0; i < ffmpeg_format_context_->nb_streams; i++)
      {
        avcodec_close(ffmpeg_format_context_->streams[i]->codec);
        av_freep(&ffmpeg_format_context_->streams[i]->metadata);
        av_freep(&ffmpeg_format_context_->streams[i]->codec->extradata);
        av_freep(&ffmpeg_format_context_->streams[i]->codec);
        av_freep(&ffmpeg_format_context_->streams[i]);

      }*/

       avformat_free_context(ffmpeg_format_context_);
    }

   // av_free(ffmpeg_src_picture_->data[0]);
    if (ffmpeg_src_picture_)
      av_free(ffmpeg_src_picture_);
  //  av_free(ffmpeg_dst_picture_->data[0]);
    if (ffmpeg_dst_picture_)
      av_free(ffmpeg_dst_picture_);

    if (ffmpeg_sws_ctx_)
      sws_freeContext(ffmpeg_sws_ctx_);

    init_ = false;

  }
}

void FFMPEG_Wrapper::encode_bgr_frame(uint8_t *bgr_data, std::vector<uint8_t>& encoded_frame)
{
  encode_frame<PIX_FMT_BGR24>(bgr_data, encoded_frame);
}

void FFMPEG_Wrapper::encode_rgb_frame(uint8_t *rgb_data, std::vector<uint8_t>& encoded_frame)
{
  encode_frame<PIX_FMT_RGB24>(rgb_data, encoded_frame);
}

void FFMPEG_Wrapper::encode_mono8_frame(uint8_t *gray8_data, std::vector<uint8_t>& encoded_frame)
{
  encode_frame<PIX_FMT_GRAY8>(gray8_data, encoded_frame);
}

void FFMPEG_Wrapper::encode_mono16_frame(uint8_t *gray16_data, std::vector<uint8_t>& encoded_frame)
{
  encode_frame<PIX_FMT_GRAY16LE>(gray16_data, encoded_frame);
  }

template <int CODING_FORMAT>
void FFMPEG_Wrapper::encode_frame(uint8_t *image_data, std::vector<uint8_t>& encoded_frame)
{

  boost::mutex::scoped_lock lock(frame_mutex_);

  avpicture_fill(ffmpeg_src_picture_, image_data,  (AVPixelFormat)CODING_FORMAT, input_width_,  input_height_);

  // get format conversion
  if (!ffmpeg_sws_ctx_) {
    static int sws_flags = SWS_BICUBIC;
      ffmpeg_sws_ctx_ = sws_getContext(input_width_, input_height_, (AVPixelFormat)CODING_FORMAT,
                                       output_width_, output_height_, ffmpeg_codec_context_->pix_fmt,
                                sws_flags, NULL, NULL, NULL);
      if (!ffmpeg_sws_ctx_) {
          fprintf(stderr,
                  "Could not initialize the conversion context\n");
          return;
      }
  }

  sws_scale(ffmpeg_sws_ctx_,
            (const uint8_t * const *)ffmpeg_src_picture_->data,
            ffmpeg_src_picture_->linesize, 0, input_height_, ffmpeg_dst_picture_->data, ffmpeg_dst_picture_->linesize);

  /* encode the image */
  AVPacket pkt;
  int got_output;

  av_init_packet(&pkt);
  pkt.data = NULL; // packet data will be allocated by the encoder
  pkt.size = 0;

  if (avcodec_encode_video2(ffmpeg_codec_context_, &pkt, ffmpeg_frame_, &got_output) < 0)
  {
    fprintf(stderr, "Error encoding video ffmpeg_frame_\n");
    return;
  }

  /* If size is zero, it means the image was buffered. */
  if (got_output)
  {
    std::size_t size;
    uint8_t* output_buf;

    if (ffmpeg_codec_context_->coded_frame->pts != AV_NOPTS_VALUE)
      pkt.pts = av_rescale_q(ffmpeg_codec_context_->coded_frame->pts, ffmpeg_codec_context_->time_base, ffmpeg_video_st_->time_base);

    pkt.pts= ffmpeg_codec_context_->coded_frame->pts;

    if (ffmpeg_codec_context_->coded_frame->key_frame)
      pkt.flags |= AV_PKT_FLAG_KEY;

    pkt.stream_index = ffmpeg_video_st_->index;

    if (avio_open_dyn_buf(&ffmpeg_format_context_->pb) >= 0)
     {
     ffmpeg_format_context_->pb->seekable = 0;

     if (av_write_frame(ffmpeg_format_context_, &pkt)) //av_write_frame
     {
     fprintf(stderr, "Error occurred when opening output file\n");
     return;
     }

     size = avio_close_dyn_buf(ffmpeg_format_context_->pb, &output_buf);

     encoded_frame.resize(size);
     memcpy(&encoded_frame[0], output_buf, size);

     av_free(output_buf);
     }
  }
  else
  {
    encoded_frame.clear();
  }

  av_free_packet(&pkt);

  AVRational timebase = ffmpeg_codec_context_->time_base;

  timebase.den += 2; //increased framerate to compensate playback delay

  //ffmpeg_frame_->pts++;
  ffmpeg_frame_->pts += av_rescale_q(1, timebase, ffmpeg_video_st_->time_base);
}

void FFMPEG_Wrapper::get_header(std::vector<uint8_t>& header)
{
  boost::mutex::scoped_lock lock(frame_mutex_);

  std::size_t size;
  uint8_t* output_buf;

  // define meta data
  av_dict_set(&ffmpeg_format_context_->metadata, "author"   , "ROS topic http streamer"   , 0);
  av_dict_set(&ffmpeg_format_context_->metadata, "comment"  , "this is awesome"  , 0);
  av_dict_set(&ffmpeg_format_context_->metadata, "copyright", "BSD", 0);
  av_dict_set(&ffmpeg_format_context_->metadata, "title"    , "ROS Topic Stream" , 0);

  if (avio_open_dyn_buf(&ffmpeg_format_context_->pb) >= 0)
  {
    ffmpeg_format_context_->pb->seekable = 0;

    if (avformat_write_header(ffmpeg_format_context_, NULL) < 0)
    {
      fprintf(stderr, "Error occurred when opening output file\n");
      return;
    }

    //av_dict_free(&ffmpeg_format_context_->metadata);

    size = avio_close_dyn_buf(ffmpeg_format_context_->pb, &output_buf);

    // copy header buffer to vector
    header.resize(size);
    memcpy(&header[0], output_buf, size);

    av_free(output_buf);
  }
}

void FFMPEG_Wrapper::get_trailer(std::vector<uint8_t>& trailer)
{
  std::size_t size;
  uint8_t* output_buf;

  boost::mutex::scoped_lock lock(frame_mutex_);

  if (avio_open_dyn_buf(&ffmpeg_format_context_->pb) >= 0)
  {
    ffmpeg_format_context_->pb->seekable = 0;

    if (av_write_trailer(ffmpeg_format_context_) < 0)
    {
      fprintf(stderr, "Error occurred when opening output file\n");
      return;
    }

    size = avio_close_dyn_buf(ffmpeg_format_context_->pb, &output_buf);

    // copy header buffer to vector
    trailer.resize(size);
    memcpy(&trailer[0], output_buf, size);

    av_free(output_buf);
  }
}

} // ros_http_video_streamer
