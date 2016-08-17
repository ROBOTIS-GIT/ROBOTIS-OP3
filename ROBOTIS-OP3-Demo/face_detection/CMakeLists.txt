cmake_minimum_required(VERSION 2.8.3)
project(face_detection)


set(CMAKE_CXX_FLAGS "-std=gnu++0x ${CMAKE_CXX_FLAGS}")




## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -o0")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )
generate_dynamic_reconfigure_options(
  cfg/face_det.cfg
  cfg/face_det_dlib.cfg
  cfg/face_det_cuda.cfg
  cfg/face_track.cfg
  #cfg/face_track_cuda.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES face_detection
   CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs dynamic_reconfigure
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a cpp library
# add_library(face_detection
#   src/${PROJECT_NAME}/face_detection.cpp
# )

##############################
####### DLIB SECTION #########
##############################

# set this value to the directory of you DLib installation
# for example: include(/home/user/dlib-X.X/dlib/cmake)
#include(../dlib/cmake)

#set(USE_AVX_INSTRUCTIONS 1)


#add_executable(face_detection_dlib src/face_detection_dlib.cpp)
#add_dependencies(face_detection_dlib ${PROJECT_NAME}_gencfg)
#target_link_libraries(face_detection_dlib dlib ${catkin_LIBRARIES})


## Declare a cpp executable
# add_executable(face_detection_node src/face_detection_node.cpp)
add_executable(face_detection src/face_detection.cpp)

add_executable(face_detection_cuda src/face_detection_cuda.cpp)
add_executable(face_tracking src/face_tracking.cpp)
#add_executable(face_tracking_cuda src/face_tracking_cuda.cpp)
add_executable(face_listener src/face_listener.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(face_detection ${PROJECT_NAME}_gencfg)
add_dependencies(face_tracking ${PROJECT_NAME}_gencfg)
# add_dependencies(face_detection_node face_detection_generate_messages_cpp)
add_dependencies(face_detection_cuda ${PROJECT_NAME}_gencfg)
#add_dependencies(face_tracking_cuda ${PROJECT_NAME}_gencfg)


## Specify libraries to link a library or executable target against
# target_link_libraries(face_detection_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(face_detection
   ${catkin_LIBRARIES}
)
target_link_libraries(face_detection_cuda
   ${catkin_LIBRARIES}
)
target_link_libraries(face_tracking
   ${catkin_LIBRARIES}
)
#target_link_libraries(face_tracking_cuda
#   ${catkin_LIBRARIES}
#)
target_link_libraries(face_listener
   ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS face_detection face_detection_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_face_detection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
