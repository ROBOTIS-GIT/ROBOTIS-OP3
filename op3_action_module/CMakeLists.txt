cmake_minimum_required(VERSION 3.5)
project(op3_action_module)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(op3_action_module_msgs REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(robotis_device REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${robotis_framework_common_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/action_module.cpp)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  std_msgs
  dynamixel_sdk
  robotis_controller_msgs
  op3_action_module_msgs
  robotis_framework_common
  robotis_device
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ament_index_cpp rclcpp std_msgs dynamixel_sdk robotis_controller_msgs op3_action_module_msgs robotis_framework_common robotis_device)

ament_package()
