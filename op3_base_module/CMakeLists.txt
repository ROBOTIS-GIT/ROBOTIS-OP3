################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(op3_base_module)

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
  geometry_msgs
  cmake_modules
  op3_kinematics_dynamics
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
  LIBRARIES op3_base_module
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(op3_base_module
  src/base_module.cpp
  src/base_module_state.cpp
)

add_dependencies(op3_base_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(op3_base_module
  yaml-cpp
  ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################