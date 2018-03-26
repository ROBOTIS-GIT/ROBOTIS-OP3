^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotis_op3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2018-03-26)
------------------
* modified preview_response to fix operator matching bug on i386 machine
* Contributors: SCH

0.2.0 (2018-03-26)
------------------
* first release of op3_online_walking_module package
* first release of op3_localization package
* first release of op3_balance_control package
* added C++11 build option
* deleted op3_optimization
* changed package.xml to use format v2
* op3_base_module:
  changed method to setting module(msg -> srv)
  fixed a bug to jump to init pose
  fixed a bug in head_control_module that wrong init vel or accel is set for generating trajectory in middle of moving
  changed a method of setting module in base_module
  added C++11 build option
* op3_head_control_module:
  increased scanning speed
  fixed bug that the garbage value entered the initial value of velocity/acceleration to generate a trajectory
  fixed a bug in head_control_module that wrong init vel or accel is set for generating trajectory in middle of moving.
  changed a method of setting module in base_module
* op3_manager:
  added wholebody module 
  added online walking module
  added body offset z
  added C++11 build option
  deleted op3_optimization node in op3_manager.launch
  changed dynamixel code for deprecated function
* op3_walking_module:
  added debug print
* refactoring to release
* Contributors: Kayman, SCH, Pyo

0.1.1 (2017-10-31)
------------------
* added op3_direct_control_module package
* added missing package in find_package()
* added op3_walking_module_msgs in find_package() function
* fixed missing dependence
* fixed action_module bug
* changed License from BSD to Apache 2.0
* Contributors: Kayman, Pyo

0.1.0 (2017-10-27)
------------------
* added new metapackage for ROBOTIS OP3
* added new function to action_editor(mirror, copy step value)
* added the function of recovery after reset
* added function of reset dxl power
* added action, modified dxl init file
* added function to play mp3 with playing action
* added debug setting to the init.yaml
* added motoin_data
* added ceremony motion after kick [soccer_demo]
* applied latest framework to robot files
* applied advance of soccer demo
* applied ROS C++ Coding Style
* applying framework
* clear offset file
* modified start button function : restart -> restart or go init pose [op3_manager]
* modified the OP3.robot
* modified gain
* updated for humanoid 2016
* changed control cycle to 8ms
* changed setting for walking and dxl
* changed mp3 files to new voice
* changed package name : op2 -> op3
* Contributors: Kayman
