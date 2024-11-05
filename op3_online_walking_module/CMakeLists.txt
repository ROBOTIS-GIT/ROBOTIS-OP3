cmake_minimum_required(VERSION 3.5)
project(op3_online_walking_module)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(op3_online_walking_module_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(robotis_device REQUIRED)
find_package(robotis_math REQUIRED)
find_package(op3_balance_control REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
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

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_compile_definitions(HAVE_NEW_YAMLCPP)
endif()

include_directories(
  include
  ${robotis_framework_common_INCLUDE_DIRS}
  ${robotis_device_INCLUDE_DIRS}
  ${robotis_math_INCLUDE_DIRS}
  ${op3_balance_control_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/online_walking_module.cpp
  src/joint_control.cpp
  src/walking_control.cpp
  src/wholebody_control.cpp
  src/op3_kdl.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  robotis_controller_msgs
  op3_online_walking_module_msgs
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_math
  op3_balance_control
  Eigen3
  orocos_kdl
)

target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs robotis_controller_msgs op3_online_walking_module_msgs dynamixel_sdk robotis_framework_common robotis_device robotis_math op3_balance_control Eigen3 orocos_kdl)

ament_package()
