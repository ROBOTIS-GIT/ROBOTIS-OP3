cmake_minimum_required(VERSION 2.8.3)
project(op3_kinematics_dynamics)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotis_math
  cmake_modules
)

find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES op3_kinematics_dynamics
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
add_library(op3_kinematics_dynamics
  src/link_data.cpp
  src/op3_kinematics_dynamics.cpp
)

## Add cmake target dependencies of the library
add_dependencies(op3_kinematics_dynamics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(op3_kinematics_dynamics
  ${catkin_LIBRARIES}
)
