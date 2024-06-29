cmake_minimum_required(VERSION 3.0.2)
project(umi_control)

add_compile_options(-std=c++11)

find_package(CURL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFFI REQUIRED libffi)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES umi_control
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${LIBFFI_INCLUDE_DIRS}
)

add_executable(umi_control_from_moveit src/umi_control_from_moveit.cpp)
add_dependencies(umi_control_from_moveit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(umi_control_from_moveit ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CURL_LIBRARIES} ${LIBFFI_LIBRARIES}) 
target_include_directories(umi_control_from_moveit PUBLIC include)

add_executable(umi_control_from_spacemouse src/umi_control_from_spacemouse.cpp)
add_dependencies(umi_control_from_spacemouse ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(umi_control_from_spacemouse ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CURL_LIBRARIES} ${LIBFFI_LIBRARIES}) 
target_include_directories(umi_control_from_spacemouse PUBLIC include)
