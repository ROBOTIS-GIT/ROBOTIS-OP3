cmake_minimum_required(VERSION 2.8.3)
project(op3_offset_tuner_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   JointOffsetData.msg
   JointOffsetPositionData.msg
   JointTorqueOnOff.msg
   JointTorqueOnOffArray.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   GetPresentJointOffsetData.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)


