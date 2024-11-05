cmake_minimum_required(VERSION 3.5)
project(op3_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(op3_action_module_msgs REQUIRED)
find_package(op3_walking_module_msgs REQUIRED)
find_package(op3_online_walking_module_msgs REQUIRED)
find_package(op3_tuning_module_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(robotis_device REQUIRED)
find_package(robotis_controller REQUIRED)
find_package(robotis_math REQUIRED)
find_package(op3_kinematics_dynamics REQUIRED)
find_package(op3_balance_control REQUIRED)
find_package(open_cr_module REQUIRED)
find_package(op3_base_module REQUIRED)
find_package(op3_head_control_module REQUIRED)
find_package(op3_walking_module REQUIRED)
find_package(op3_action_module REQUIRED)
find_package(op3_direct_control_module REQUIRED)
find_package(op3_online_walking_module REQUIRED)
find_package(op3_tuning_module REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${robotis_controller_msgs_INCLUDE_DIRS}
  ${op3_action_module_msgs_INCLUDE_DIRS}
  ${op3_walking_module_msgs_INCLUDE_DIRS}
  ${op3_online_walking_module_msgs_INCLUDE_DIRS}
  ${op3_tuning_module_msgs_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIRS}
  ${robotis_framework_common_INCLUDE_DIRS}
  ${robotis_device_INCLUDE_DIRS}
  ${robotis_controller_INCLUDE_DIRS}
  ${robotis_math_INCLUDE_DIRS}
  ${op3_kinematics_dynamics_INCLUDE_DIRS}
  ${op3_balance_control_INCLUDE_DIRS}
  ${open_cr_module_INCLUDE_DIRS}
  ${op3_base_module_INCLUDE_DIRS}
  ${op3_head_control_module_INCLUDE_DIRS}
  ${op3_walking_module_INCLUDE_DIRS}
  ${op3_action_module_INCLUDE_DIRS}
  ${op3_direct_control_module_INCLUDE_DIRS}
  ${op3_online_walking_module_INCLUDE_DIRS}
  ${op3_tuning_module_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/op3_manager.cpp)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp 
  rclcpp
  robotis_controller_msgs
  op3_action_module_msgs
  op3_walking_module_msgs
  op3_online_walking_module_msgs
  op3_tuning_module_msgs
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_math
  op3_kinematics_dynamics
  op3_balance_control
  open_cr_module
  op3_base_module
  op3_head_control_module
  op3_walking_module
  op3_action_module
  op3_direct_control_module
  op3_online_walking_module
  op3_tuning_module
)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
