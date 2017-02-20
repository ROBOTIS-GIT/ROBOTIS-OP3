cmake_minimum_required(VERSION 2.8.3)
project(op3_head_control_module)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES op3_head_control_module
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
add_library(op3_head_control_module
  src/head_control_module.cpp
)

## Add cmake target dependencies of the library
add_dependencies(op3_head_control_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable

## Add cmake target dependencies of the executable

## Specify libraries to link a library or executable target against
target_link_libraries(op3_head_control_module
  ${catkin_LIBRARIES}
)
