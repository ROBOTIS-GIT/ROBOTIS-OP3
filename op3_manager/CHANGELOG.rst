^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package op3_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2017-10-31)
-----------
* Changed License from BSD to Apache 2.0
* Added a launch file for gazebo
* fixed typo.
* added test code about collision check
* added direct_control_module
* added direct_control_module
* deleted offset.yaml
* fixed action_module bug.
* Contributors: Kayman, Pyo

0.1.0 (2017-10-27)
-----------
* clear offset file
* Update offset.yaml
  clear offset.yaml
* cleanup the code.
  added the function of recovery after reset.
* head_control_module : reduced moving time and check angle limits
* added function of reset dxl power
* added action, modified dxl init file
* updated gain
  changed control cycle to 8ms
* changed setting for walking and dxl.
* cleaned up the code.
* added meta packages
* added new function to action_editor(mirror, copy step value)
  changed mp3 files to new voice
* added function to play mp3 with playing action
* ball_detector :
  changed parameters of config files.
  added debug code.
  op3_demo :
  added debug code.
  added pd gain to ball_tracker
  added pd gain to face_tracker
  deleted unused files.
* change minor setting
* edit action file
  add debug setting to the init.yaml
* clean up launch files
  change minor
  - RGB led control
* op3_demo : replaced cm-740 to open-cr
  walking_module : fixed sensory feedback
  op3_gui : add button of init gyro
  op3_manager : code revision
* Merge remote-tracking branch 'remotes/origin/feature_default_demo_revision' into feature_open_cr
  # Conflicts:
  #	op3_manager/config/offset.yaml
* modify open_cr_module
  modify OPEN-CR device file
  etc
* edit minor
  delete sound_play
* clean up
  apply latest framework to robot files
* demo revision
* middle of making advance of soccer demo
* for new leg structure
* update for humanoid 2016
* test for open-cr
* WIP on feature_open_cr: 875cd6d add manager for testing open-cr clean up unnecessary files
* add manager for testing open-cr
  clean up unnecessary files
* make soccer demo, action demo
  modify op3_action_module
  add op3_action_module_msgs
* applied ROS C++ Coding Style
* applying framework
* Merge branch 'feature_soccer_demo' of https://github.com/ROBOTIS-GIT/ROBOTIS-OP3 into feature_soccer_demo
* edit soccer_demo
  edit op3_offset_tuner config
* add motoin_data
  change init_file
* add ceremony motion after kick [soccer_demo]
  modify start button function : restart -> restart or go init pose [op3_manager]
* Start OP3 soccer demo
* refactoring
* change files name : op3_description
* apply framework revision
* Organize folder structure
* modify the OP3.robot
* change op3_manager
* cm-740 : edit method for gyro
  base_module : change init pose
  walking module : fix bug(yaml)
  change kinematics for op3
* refactoring
* action_module : OnEnable
* walking_module : init pose
* walking_module : refactoring
  walking_tuner
* add walking_module
  edit op3_demo (walking_tuner)
  use op3_kinematics_dynamics(for IK)
* dev walking module for op3 (frame)
  remove io module
  modify op3_manager
* add Tool
  change directory structure
* sensor module for cm_740
* change package name : op2 -> op3
  add package for CM-740
* Contributors: Kayman
