cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include(include)

include_directories(include)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/ffmpeg/)


add_library(avcodec STATIC IMPORTED)
set_property(TARGET avcodec PROPERTY
             IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/3rdparty/ffmpeg/libavcodec/libavcodec.a)

add_library(avformat STATIC IMPORTED)
set_property(TARGET avformat PROPERTY
             IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/3rdparty/ffmpeg/libavformat/libavformat.a)
           
add_library(avutil STATIC IMPORTED)  
set_property(TARGET avutil PROPERTY
             IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/3rdparty/ffmpeg/libavutil/libavutil.a)

add_library(avfilter STATIC IMPORTED)  
set_property(TARGET avfilter PROPERTY
             IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/3rdparty/ffmpeg/libavfilter/libavfilter.a)
             
add_library(swscale STATIC IMPORTED)  
set_property(TARGET swscale PROPERTY
             IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/3rdparty/ffmpeg/libswscale/libswscale.a)
             
add_library(vpx STATIC IMPORTED)  
set_property(TARGET vpx PROPERTY
             IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/3rdparty/libvpx/libvpx.a)


rosbuild_add_executable(${PROJECT_NAME}  src/ffmpeg_wrapper.cpp 
                                         src/server.cpp
                                         src/connection.cpp
                                         src/topic_streamer.cpp
                                         src/reply.cpp
                                         src/request_parser.cpp
                                         src/image_subscriber.cpp
                                         src/ffmpeg_encoder.cpp
                                         )
                                         
target_link_libraries(${PROJECT_NAME} avcodec avformat avutil avfilter swscale vpx bz2 z)
rosbuild_link_boost(${PROJECT_NAME} thread signals system)       

                                         
#common commands for building c++ executables and libraries 
rosbuild_add_executable(webGLPointCloudTopicConverter tools/depth_to_webGL_pointclouds.cpp)                                       
rosbuild_link_boost(webGLPointCloudTopicConverter thread signals system)       

    