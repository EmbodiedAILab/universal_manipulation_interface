cmake_minimum_required(VERSION 3.0.2)
project(record_data)

add_compile_options(-std=c++11)
find_package (OpenCV 4.0.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  aruco_ros
  cv_bridge
  image_transport
  tf2_ros
)

set(ffmpeg_libs_DIR /usr/lib/x86_64-linux-gnu)
set(ffmpeg_headers_DIR /usr/include/x86_64-linux-gnu)
add_library(avcodec SHARED IMPORTED)
add_library(avfilter SHARED IMPORTED)
add_library(swresample SHARED IMPORTED)
add_library(swscale SHARED IMPORTED)
add_library(avformat SHARED IMPORTED)
add_library(avutil SHARED IMPORTED)
set_target_properties(avcodec PROPERTIES IMPORTED_LOCATION ${ffmpeg_libs_DIR}/libavcodec.so)
set_target_properties(avfilter PROPERTIES IMPORTED_LOCATION ${ffmpeg_libs_DIR}/libavfilter.so)
set_target_properties(swresample PROPERTIES IMPORTED_LOCATION ${ffmpeg_libs_DIR}/libswresample.so)
set_target_properties(swscale PROPERTIES IMPORTED_LOCATION ${ffmpeg_libs_DIR}/libswscale.so)
set_target_properties(avformat PROPERTIES IMPORTED_LOCATION ${ffmpeg_libs_DIR}/libavformat.so)
set_target_properties(avutil PROPERTIES IMPORTED_LOCATION ${ffmpeg_libs_DIR}/libavutil.so)
catkin_package(
 #INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp aruco_ros 
)
#include_directories ("/usr/include/opencv4/")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  ${FFMPEG_INCLUDE_DIR}
  "/usr/include/opencv4/"
)

add_executable(video_traj_recorder src/video_traj_recorder.cpp)
add_dependencies(video_traj_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(video_traj_recorder ${catkin_LIBRARIES} avcodec avfilter swresample swscale avformat avutil)