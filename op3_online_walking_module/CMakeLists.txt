cmake_minimum_required(VERSION 2.8.3)
project(op3_online_walking_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  cmake_modules
  robotis_math
  robotis_controller_msgs
  robotis_framework_common
#  op3_kinematics_dynamics
  op3_balance_control
  op3_online_walking_module_msgs
)

find_package(Eigen REQUIRED)
find_package(orocos_kdl)
find_package(kdl_parser)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES op3_online_walking_module
  CATKIN_DEPENDS roscpp robotis_math robotis_framework_common
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(op3_online_walking_module
  src/online_walking_module.cpp
  src/joint_control.cpp
  src/walking_control.cpp
  src/wholebody_control.cpp
  src/op3_kdl.cpp
)

add_dependencies(op3_online_walking_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(op3_online_walking_module
  yaml-cpp
  ${catkin_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(TARGETS op3_online_walking_module
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
