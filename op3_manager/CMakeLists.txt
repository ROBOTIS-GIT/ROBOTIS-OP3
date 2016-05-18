cmake_minimum_required(VERSION 2.8.3)
project(op3_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_controller_msgs
  robotis_math
    
  cm_740_module
  
  op3_base_module
  op3_head_control_module
  op3_action_module
  op3_walking_module
  op3_kinematics_dynamics
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)

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
#  INCLUDE_DIRS include
  LIBRARIES op3_manager
#  CATKIN_DEPENDS robotis_controller roscpp
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
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(op3_manager src/op3_manager.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(thormang3_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(op3_manager
  ${catkin_LIBRARIES}
)
