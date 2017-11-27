cmake_minimum_required(VERSION 2.8.3)
project(op3_optimization)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  op3_online_walking_module_msgs
  robotis_math
  scilab_optimization
)

find_package(Eigen3 REQUIRED)

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
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  DEPENDS EIGEN3
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(op3_optimization
  src/op3_optimization.cpp
)

add_dependencies(op3_optimization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(op3_optimization
  ${catkin_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(TARGETS op3_optimization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################
# Test
################################################################################
