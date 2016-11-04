cmake_minimum_required(VERSION 2.8.3)
project(op3_offset_tuner_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  cmake_modules
  dynamixel_sdk
  robotis_device
  robotis_framework_common
  robotis_math
  op3_base_module
  op3_kinematics_dynamics
  robotis_controller_msgs
  robotis_controller
)

find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(offset_tuner_server_node 
   src/op3_offset_tuner_server.cpp
   src/main.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(offset_tuner_server_node
  yaml-cpp
  ${catkin_LIBRARIES}
)


