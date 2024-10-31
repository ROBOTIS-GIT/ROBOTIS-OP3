cmake_minimum_required(VERSION 3.5)
project(op3_balance_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robotis_math REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${robotis_math_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/op3_balance_control.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp Eigen3 robotis_math)

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
ament_export_dependencies(rclcpp Eigen3 robotis_math)

ament_package()
