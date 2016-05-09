cmake_minimum_required(VERSION 2.8.3)
project(op2_base_module)

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
  op2_kinematics_dynamics
  #message_generation
)

find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  sensor_msgs
#  geometry_msgs
#)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES op2_base_module
#  CATKIN_DEPENDS robotis_device roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(op2_base_module
  src/BaseModule.cpp
  src/RobotisState.cpp
  #src/JointState.cpp
  #src/Transformation.cpp
  #src/Trajectory.cpp
)

add_dependencies(op2_base_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(op2_base_module
  yaml-cpp
  ${catkin_LIBRARIES}
)
