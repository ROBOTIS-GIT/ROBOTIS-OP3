cmake_minimum_required(VERSION 3.5)
project(op3_head_control_module)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(robotis_device REQUIRED)
find_package(robotis_math REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${robotis_math_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/head_control_module.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  dynamixel_sdk
  robotis_controller_msgs
  robotis_framework_common
  robotis_device
  robotis_math
  Eigen3
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs sensor_msgs dynamixel_sdk robotis_controller_msgs robotis_framework_common robotis_device robotis_math Eigen3)

ament_package()
