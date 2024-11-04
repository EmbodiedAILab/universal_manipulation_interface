cmake_minimum_required(VERSION 3.8)
project(umi_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
#find_package(tinyxml2_vendor REQUIRED)

#include_directories(
#  /usr/include
#  /usr/local/sbin
#)

find_library(TINYXML2_LIBRARIES NAMES tinyxml2 PATHS /usr/lib/aarch64-linux-gnu)
#find_include_directories(TINYXML2_INCLUDE_DIRS NAMES tinyxml2.h PATHS /usr/local/sbin)
find_path(TINYXML2_INCLUDE_DIRS tinyxml2.h PATHS /usr/include)

add_executable(arm_joint_controller src/arm_joint_controller.cpp)
target_include_directories(arm_joint_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(arm_joint_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  arm_joint_controller
  "moveit_ros_planning_interface"
  "rclcpp"
  rclcpp_action control_msgs sensor_msgs trajectory_msgs
)
target_include_directories(arm_joint_controller PUBLIC ${TINYXML2_INCLUDE_DIRS})
target_link_libraries(arm_joint_controller ${TINYXML2_LIBRARIES})

add_executable(arm_eef_controller src/arm_eef_controller.cpp)
target_include_directories(arm_eef_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(arm_eef_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  arm_eef_controller
  "moveit_ros_planning_interface"
  "rclcpp"
  control_msgs
)
#target_link_libraries(arm_eef_controller tinyxml2_vendor)
target_include_directories(arm_eef_controller PUBLIC ${TINYXML2_INCLUDE_DIRS})
target_link_libraries(arm_eef_controller ${TINYXML2_LIBRARIES})

add_executable(arm_eef_controller_test src/arm_eef_controller_test.cpp)
target_include_directories(arm_eef_controller_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(arm_eef_controller_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  arm_eef_controller_test
  "moveit_ros_planning_interface"
  "rclcpp"
)
#target_link_libraries(arm_eef_controller_test tinyxml2_vendor)
target_include_directories(arm_eef_controller_test PUBLIC ${TINYXML2_INCLUDE_DIRS})
target_link_libraries(arm_eef_controller_test ${TINYXML2_LIBRARIES})

add_executable(eef_pose_gripper_width_pub src/eef_pose_gripper_width_pub.cpp)
target_include_directories(eef_pose_gripper_width_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(eef_pose_gripper_width_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
#target_link_libraries(eef_pose_gripper_width_pub tinyxml2_vendor)
ament_target_dependencies(
  eef_pose_gripper_width_pub
  "moveit_ros_planning_interface"
  "rclcpp"
)
target_include_directories(eef_pose_gripper_width_pub PUBLIC ${TINYXML2_INCLUDE_DIRS})
target_link_libraries(eef_pose_gripper_width_pub ${TINYXML2_LIBRARIES})

install(TARGETS arm_joint_controller arm_eef_controller arm_eef_controller_test eef_pose_gripper_width_pub
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
