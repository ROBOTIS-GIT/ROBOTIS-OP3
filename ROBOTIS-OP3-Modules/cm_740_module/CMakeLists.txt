cmake_minimum_required(VERSION 2.8.3)
project(cm_740_module)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamixel_sdk
  robotis_device
  robotis_controller_msgs
  robotis_framework_common
  robotis_math
  roscpp
)

find_package(Eigen REQUIRED)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cm_740_module
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
add_library(cm_740_module
  src/cm_740_module.cpp
)

## Add cmake target dependencies of the library
add_dependencies(cm_740_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})