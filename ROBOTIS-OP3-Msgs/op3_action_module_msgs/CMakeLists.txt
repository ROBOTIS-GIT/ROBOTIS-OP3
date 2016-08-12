cmake_minimum_required(VERSION 2.8.3)
project(op3_action_module_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  StartAction.msg
)

add_service_files(
  FILES
  IsRunning.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


