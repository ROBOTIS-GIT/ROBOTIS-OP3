cmake_minimum_required(VERSION 2.8.3)
project(op3_walking_module)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamixel_sdk
  eigen_conversions
  op3_kinematics_dynamics
  robotis_controller_msgs
  robotis_device
  robotis_framework_common
  robotis_math
  roscpp
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES op3_walking_module
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(op3_walking_module
  src/op3_walking_module.cpp
)

## Add cmake target dependencies of the library
add_dependencies(op3_walking_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(op3_walking_module
  yaml-cpp
  ${catkin_LIBRARIES}
)
