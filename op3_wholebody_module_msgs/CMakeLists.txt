cmake_minimum_required(VERSION 2.8.3)
project(op3_wholebody_module_msgs)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  JointPose.msg
  KinematicsPose.msg
  FootStepCommand.msg
  FootStepArray.msg
  PreviewRequest.msg
  PreviewResponse.msg
  WalkingParam.msg
  Step2D.msg
  Step2DArray.msg
)

add_service_files(
  FILES
  GetJointPose.srv
  GetKinematicsPose.srv
  GetPreviewMatrix.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
