cmake_minimum_required(VERSION 2.8.3)
project(op3_action_editor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_controller_msgs
  robotis_math
  
  op3_action_module
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES op3_action_editor
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(op3_action_editor 
  src/main.cpp
  src/action_editor.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(op3_action_editor
   ncurses
   ${catkin_LIBRARIES}
)
