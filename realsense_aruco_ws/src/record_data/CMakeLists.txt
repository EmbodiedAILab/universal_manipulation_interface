cmake_minimum_required(VERSION 3.0.2)
project(record_data)

add_compile_options(-std=c++11)
find_package (OpenCV 4.0.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  sensor_msgs
  aruco_ros
  cv_bridge
  image_transport
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp aruco_ros message_runtime
)
include_directories ("/usr/include/opencv4/")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(video_and_traj_recorde src/video_and_traj_recorde.cpp)
add_dependencies(video_and_traj_recorde ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(video_and_traj_recorde ${catkin_LIBRARIES})