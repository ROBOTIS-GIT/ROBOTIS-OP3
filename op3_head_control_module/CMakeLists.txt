################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(op3_head_control_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  robotis_device
  robotis_math
  dynamixel_sdk
  robotis_controller_msgs
  robotis_framework_common
  roscpp
  std_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES op3_head_control_module
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(op3_head_control_module
  src/head_control_module.cpp
)

add_dependencies(op3_head_control_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(op3_head_control_module
  ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################