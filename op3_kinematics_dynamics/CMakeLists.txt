cmake_minimum_required(VERSION 3.5)
project(op3_kinematics_dynamics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotis_math REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${robotis_math_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/link_data.cpp src/op3_kinematics_dynamics.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp robotis_math Eigen3)

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
ament_export_dependencies(rclcpp robotis_math Eigen3)

ament_package()
