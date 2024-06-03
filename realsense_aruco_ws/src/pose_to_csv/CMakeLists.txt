cmake_minimum_required(VERSION 3.0.2)
project(pose_to_csv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_to_csv
#  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(pose_to_csv src/pose_to_csv.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pose_to_csv ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(pose_to_csv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
