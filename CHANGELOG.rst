^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_web_video
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.7 (2014-04-16)
------------------
* submodule moved to HTTPS
* git submodules now used instead of willow servers for ffmpeg
* v1.2.0 libvpx
* libvpx added
* moved to submodule
* cmake cleanup
* basic cleanup
* adding travis.yml
* adding travis.yml
* Contributors: Julius Kammerl, Russell Toris

0.1.6 (2013-05-16)
------------------
* 0.1.5 -> 0.1.6
* adding switch to disable file serving
* adding file_server ros parameter
* package.xml
* renaming project to ros_web_video
* Contributors: Julius Kammerl

0.1.5 (2013-04-22)
------------------
* 0.1.4 -> 0.1.5
* patching "codec not found" crash
* Contributors: Julius Kammerl

0.1.4 (2013-04-17)
------------------
* 0.1.3 -> 0.1.4
* patching ffmpeg bug
* Contributors: Julius Kammerl

0.1.3 (2013-02-15)
------------------
* 0.1.2 -> 0.1.3
* adding download_checkmd5.py to 3rdparty
* Contributors: Julius Kammerl

0.1.2 (2013-02-14 16:23)
------------------------
* 0.1.1 -> 0.1.2
* adding mk package dependency
* Contributors: Julius Kammerl

0.1.1 (2013-02-14 14:21)
------------------------
* version 0.1.1
* integrating download_unpack_build.mk
* Contributors: Julius Kammerl

0.1.0 (2013-02-13)
------------------
* version 0.1.0
* adding download script to 3rdparty dir
* Removed the '?' prefix requirement for URLs
* making http headers optional
* removing depthcloud proof-of-concept code
* removed bad bzip2 dependecy
* adding missing bzip2 dependency
* more catkinization
* adding zlib dependency to CMakeLists.txt and package.xml
* Merge branch 'groovy-devel' of github.com:RobotWebTools/ros_http_video_streamer into groovy-devel
* adding zlib to package.xml
* minor.. added comments
* adding MIME types for css and javascript files
* fixing wwwroot bug
* adding debug output to web file server
* renaming webGL_pointcloud_image_encoder in CMakeList
* renaming web_gl pointlcoud converter node
* switching to local rosparam nodehandle + additional debug output
* Merge pull request `#2 <https://github.com/RobotWebTools/ros_web_video/issues/2>`_ from jon-weisz/cmake_fixcmake-fix-for-3rdparty
  fixed 3rdparty library build problem in ros_http_video_streamer by expli...
* fixed 3rdparty library build problem in ros_http_video_streamer by explicitly executing the make command in CMakeLists.txt
* more ffmpeg tuning
* adding rosparam parameter in order to define the ROS image transport
* adding filter in order to remove raw image topics from topic list
* fixed encoding <-> data transmission synchronization
* adding additional parameters to server configuration
* adding roscpp and rostime deps
* catkinizing image streamer
* Adding 3rdparty checkout&compilation to CMake
* Merge pull request `#1 <https://github.com/RobotWebTools/ros_web_video/issues/1>`_ from KaijenHsiao/master
  added rosdep for yasm
* added rosdep for yasm
* adding mutex lock manager
* adding additional mutext to protect av_open/av_close
* explicit ffmpeg_wrapper shutdown
* adding ROS makefile
* cleanup
* restructured javascript code
* ffmepg initialization startup protection
* transfercoding header fix
* minor
* shared pointers seem to be reused in openni_launch
* more thread security
* more mleak fixing
* fixing memory leak
* switching back to image transport
* moving OrbitControls.js to js folder
* Adding orbit control to webgl pointcloud viewer
* unsubscribe from image topics in deconstructor
* Merge branch 'master' of github.com:ros-interactive-manipulation/ros_http_video_streamer
* major commit: improved pointcloud rendering, fixed image subscription bug
* major commit: improved pointcloud rendering, fixed image subscription bug
* fixing http headers
* Merge branch 'master' of github.com:ros-interactive-manipulation/ros_http_video_streamer
* adding .webm to URLs to make Firefox happy
* fixing multithreading
* Merge branch 'master' of github.com:ros-interactive-manipulation/ros_http_video_streamer
* fixing race condition
* missing include
* improved depth encoding for webgl-pointcloud streamer
* Adding ROS makefile
* minor
* removed depth encoding functionality from streaming server
* adding webgl-based pointcloud streaming html page
* adding file server functionality to streamer
* adding webGL-based http frontend for pointcloud streaming + first working version of depth_to_webGL streamer
* Merge branch 'master' of github.com:jkammerl/ros_http_video_streamer
* more on depth_to_webGL_pointclouds..
* ffmpeg tuning
* minor
* adding additional parameters to the streaming URL request
* added depth_to_webGL_pointclouds.cpp tool
* adding global encoding configuration
* more depth coding
* Merge branch 'master' of github.com:jkammerl/ros_http_video_streamer
* extended depthmap encoding
* revisions, added image rescaling
* improved parameter handling, added experimental depth_to_rgb encoding tests
* adding webgl_pointcloud_streaming file
* minor
* adding ros_http_video_streamer namespace
* initial commit
* Contributors: Interactive Manipulation, Julius Kammerl, Kaijen Hsiao, jon-weisz
