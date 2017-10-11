################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(op3_walking_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamixel_sdk
  eigen_conversions
  op3_kinematics_dynamics
  op3_walking_module_msgs
  robotis_controller_msgs
  robotis_device
  robotis_framework_common
  robotis_math
  op3_walking_module_msgs
  roscpp
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
  LIBRARIES op3_walking_module
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(op3_walking_module
  src/op3_walking_module.cpp
)

add_dependencies(op3_walking_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(op3_walking_module
  yaml-cpp
  ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
