################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(test_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)


################################################################################
# Build
################################################################################
include_directories(
  include
)

add_executable(test_rpm_publisher src/test_rpm_publisher.cpp)
ament_target_dependencies(
  test_rpm_publisher
  rclcpp
  robot_msgs
)

add_executable(test_rpm_subcriber src/test_rpm_subcriber.cpp)
ament_target_dependencies(
  test_rpm_subcriber
  rclcpp
  robot_msgs
)

################################################################################
# Install
################################################################################
install(TARGETS
   test_rpm_publisher
   test_rpm_subcriber
   DESTINATION lib/${PROJECT_NAME})

ament_package()
