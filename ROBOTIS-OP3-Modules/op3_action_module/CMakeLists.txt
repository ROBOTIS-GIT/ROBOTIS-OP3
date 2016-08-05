cmake_minimum_required(VERSION 2.8.3)
project(op3_action_module)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_sdk
  robotis_device
  robotis_controller_msgs
  robotis_framework_common
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES op3_action_module
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(op3_action_module
  src/action_module.cpp
)

## Add cmake target dependencies of the library
add_dependencies(op3_action_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(op3_action_module
  ${catkin_LIBRARIES}
)

