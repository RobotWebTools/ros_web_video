cmake_minimum_required(VERSION 2.8.3)
project(ros_http_video_streamer)

find_package(catkin REQUIRED roscpp rostime sensor_msgs image_transport)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)

find_package ( BZip2 REQUIRED )
find_package ( ZLIB REQUIRED )

include_directories(include ${catkin_INCLUDE_DIRS})

set_source_files_properties( third_party_make PROPERTIES GENERATED true)
add_custom_command ( OUTPUT third_party_make
    COMMAND make  
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/3rdparty/ )

include_directories(include ${PROJECT_SOURCE_DIR}/3rdparty/ffmpeg/ ${ZLIB_INCLUDE_DIRS} ${BZIP2_INCLUDE_DIR}) 
link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${ZLIB_LIBRARY_DIRS} ${BZIP2_LIBRARY_DIR})
add_definitions(${ZLIB_CFLAGS_OTHER} ${BZIP2_CFLAGS_OTHER})

execute_process(COMMAND make WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/3rdparty/ )
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS sensor_msgs image_transport roscpp rostime
)

add_executable(${PROJECT_NAME}  src/ffmpeg_wrapper.cpp 
                                src/server.cpp
                                src/connection.cpp
                                src/topic_streamer.cpp
                                src/reply.cpp
                                src/request_parser.cpp
                                src/image_subscriber.cpp
                                src/ffmpeg_encoder.cpp
                                )


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


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} avcodec avformat avutil avfilter swscale vpx
${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES})

install(TARGETS ${PROJECT_NAME} 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

