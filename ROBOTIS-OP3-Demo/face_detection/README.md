**DISCLAMER:**

This project was created within an academic research setting, and thus should
be considered as EXPERIMENTAL code. There may be bugs and deficiencies in the
code, so please adjust expectations accordingly. With that said, we are
intrinsically motivated to ensure its correctness (and often its performance).
Please use the corresponding web repository tool (e.g. github, bitbucket, etc)
to file bugs, suggestions, pull requests; we will do our best to address them
in a timely manner.


**LAYOUT:**
- face_detection/
  - cfg/:                 dynamic_reconfigure configuration files
  - include/:             cascade files for the detection
  - launch/:              roslaunch files
  - src/:                 source files
  - CMakeLists.txt:       CMake project configuration file
  - LICENSES:             license agreement
  - package.xml:          ROS/Catkin package file
  - README.txt:           this file


**Why use this package:**

These ROS nodes are designed to detect and track faces in images coming from a
ROS image topic. The nodes display or publish an image with the resulting
detections drawn on top of the image. The settings of the detection system can
be easily adapted using ROS rqt_reconfigure (rosrun rqt_reconfigure
rqt_reconfigure). The nodes also publish the coordinates of the faces. To work for a wide variaty of scenarios, the following features are included:
  - OpenCV Face Detection
  - Dlib Face Detection
  - OpenCV CUDA support
  - OpenCV tracking (Optical flow)

**How to Use this package:**

To run the package, use the provided RosLaunch file as follows:
  - roslaunch face_detection face_detection.launch
  - roslaunch face_detection face_tracking.launch
  - roslaunch face_detection face_detection_cuda.launch

To see the coordinates published by the nodes, launch the listener node:
  - rosrun face_detection face_listener

The settings of the program can be changed with the ROS rqt_reconfigure setup.
  - rosrun rqt_reconfigure rqt_reconfigure

Once you have the rqt_reconfigure open, change the input image default
(/camera/image_raw) to your desired input. Additional information about the
different settings are annotated in the dynamic reconfigure setup (hover over
the setting in the rqt_reconfigure for additional information)

**Dlib:**

Since it is uncommon to have Dlib installed, the face_detection_dlib node is
disabled by default. The Dlib face detection system has a higher accuracy to
detect faces, especially faces which are tilted sideways. The system is a bit
slower than OpenCV, mainly because of the lack of additional configuration
settings (min max searchwindow_scale ...). It also has a lower accuracy with
smaller faces. The detection is using a HOG cascade.

If you want to use this node, you need to enable it in the CMakeLists file under
the section "DLIB". You will also need to link the installation directory in the
CMakeLists (include(../dlib/cmake)). The Dlib library can be found on their
website (http://dlib.net/). If your CPU does not support the AVX instruction
set, you might have to change the line "#set(USE_AVX_INSTRUCTIONS 1)"

The node can be launched with:
  - roslaunch face_detection face_detection_dlib.launch

Settings can be changed with:
  - rosrun rqt_reconfigure rqt_reconfigure

**OpenCV CUDA:**

To enable faster detection, CUDA support has been enabled for the detection process. Keep in mind that the CUDA detection is slightly different to the original OpenCV detection and will deliver different, slightly less accurate results. Currently only the detection process has been implemented for CUDA. The tracking part is still under development.


**Requirements:**

This node was designed for ROS Indigo and requires a catkin workspace. The node
also makes use of OpenCV. The node has been tested under OpenCV 2.4.8 and 3.0.0.
To be able to get your images into the node, you will need to ROS Vision_OpenCV
package (http://wiki.ros.org/vision_opencv). You will also need a driver which
can read the images from your camera and which can publish these images inside
ROS, like for example ueye_cam (http://wiki.ros.org/ueye_cam) or usb_cam
(http://wiki.ros.org/usb_cam).

**Multi-threading:**

In order to get the best performance out of the detection, you should compile
your OpenCV package with Multi-threading support. OpenCV makes use of TBB
(WITH_TBB=ON)to to enable Multi-threading, for both versions 2.4 and 3.0. Here
is a tutorial on how to install OpenCV with Multi-threading for version 3.0
(http://rodrigoberriel.com/2014/10/installing-opencv-3-0-0-on-ubuntu-14-04/).

WARNING: Reinstalling OpenCV might break some of you other ROS packages, so you
might want to stick to your current version. This node will also run without
Multi-threading support, but especially the Face_Tracking node will run
considerably slower.

**Future plans:**

This package is currently still under development and will be updated
continuously. Please report any problems or desired features, feedback is
always welcome.



Copyright (c) 2015, Philippe Ludivig

All rights reserved.

BSD2 license: see LICENSE file
