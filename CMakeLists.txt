cmake_minimum_required(VERSION 2.8.3)
project(ros_web_video)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp rostime sensor_msgs image_transport mk) 
find_package(Boost COMPONENTS system filesystem thread REQUIRED)
find_package(BZip2 REQUIRED)
find_package(ZLIB REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS sensor_msgs image_transport roscpp rostime mk
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${PROJECT_SOURCE_DIR}/FFmpeg/
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
  ${ZLIB_INCLUDE_DIRS}
  ${BZIP2_INCLUDE_DIR}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  ${ZLIB_LIBRARY_DIRS}
  ${BZIP2_LIBRARY_DIR}
)

add_definitions(
  ${ZLIB_CFLAGS_OTHER}
  ${BZIP2_CFLAGS_OTHER}
)

## Grab external libraries
execute_process(
  COMMAND git clone https://chromium.googlesource.com/webm/libvpx
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/
)
execute_process(
  COMMAND git checkout v1.2.0
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/libvpx
)
execute_process(
  COMMAND git clone https://github.com/FFmpeg/FFmpeg.git
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/
)
execute_process(
  COMMAND git checkout n1.1
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/FFmpeg
)

## Setup libvpx
execute_process(
  COMMAND ./configure --enable-vp8
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/libvpx/
)
execute_process(
  COMMAND make
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/libvpx/
)

## Setup FFmpeg
execute_process(
  COMMAND ./configure --enable-libvpx --extra-cflags="-I../libvpx" --extra-ldflags="-L../libvpx" --enable-gpl
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/FFmpeg/
)
execute_process(
  COMMAND make WORKING_DIRECTORY
  ${PROJECT_SOURCE_DIR}/FFmpeg/
)

## Declare a cpp executables and libraries
add_executable(${PROJECT_NAME}
  src/ffmpeg_wrapper.cpp 
  src/server.cpp
  src/connection.cpp
  src/topic_streamer.cpp
  src/reply.cpp
  src/request_parser.cpp
  src/image_subscriber.cpp
  src/ffmpeg_encoder.cpp
)

add_library(avcodec STATIC IMPORTED)
set_property(TARGET avcodec
  PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/FFmpeg/libavcodec/libavcodec.a
)

add_library(avformat STATIC IMPORTED)
set_property(TARGET avformat PROPERTY
  IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/FFmpeg/libavformat/libavformat.a
)
           
add_library(avutil STATIC IMPORTED)  
set_property(TARGET avutil PROPERTY
  IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/FFmpeg/libavutil/libavutil.a
)

add_library(avfilter STATIC IMPORTED)  
set_property(TARGET avfilter PROPERTY
  IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/FFmpeg/libavfilter/libavfilter.a
)
             
add_library(swscale STATIC IMPORTED)  
set_property(TARGET swscale PROPERTY
  IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/FFmpeg/libswscale/libswscale.a
)
             
add_library(vpx STATIC IMPORTED)  
set_property(TARGET vpx PROPERTY
  IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/libvpx/libvpx.a
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  avcodec
  avformat
  avutil
  avfilter
  swscale
  vpx
  ${BZIP2_LIBRARIES}
  ${ZLIB_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
