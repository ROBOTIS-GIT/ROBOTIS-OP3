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
  open_cr_module
  
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
catkin_package(
  LIBRARIES op3_manager
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(op3_manager src/op3_manager.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(op3_manager
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(op3_test_manager src/test_manager.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(op3_test_manager
  ${catkin_LIBRARIES}
)