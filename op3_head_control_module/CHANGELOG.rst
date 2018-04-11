^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package op3_head_control_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2018-03-26)
------------------
* none

0.2.0 (2018-03-26)
------------------
* increased scanning speed
* fixed bug that the garbage value entered the initial value of velocity/acceleration to generate a trajectory
* fixed a bug in head_control_module that wrong init vel or accel is set for generating trajectory in middle of moving
* changed a method of setting module in base_module
* changed package.xml to use format v2
* refactoring to release
* Contributors: Kayman, Pyo

0.1.1 (2017-10-31)
------------------
* fixed missing dependence
* Changed License from BSD to Apache 2.0
* Contributors: Kayman

0.1.0 (2017-10-27)
------------------
* added the function of recovery after reset
* reduced moving time and check angle limits
* cleaned up the code
* change package name : op2 -> op3
* Contributors: Kayman
