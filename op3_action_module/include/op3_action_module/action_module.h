/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Kayman, Jay Song */

#ifndef ACTION_MOTION_MODULE_H_
#define ACTION_MOTION_MODULE_H_

#define _USE_MATH_DEFINES

#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include "robotis_controller_msgs/StatusMsg.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "op3_action_module_msgs/StartAction.h"
#include "robotis_framework_common/motion_module.h"
#include "action_file_define.h"

namespace robotis_op
{

class ActionModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<ActionModule>
{
 public:
  ActionModule();
  virtual ~ActionModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  bool loadFile(std::string file_name);
  bool createFile(std::string file_name);

  bool start(int page_number);
  bool start(std::string page_name);
  bool start(int page_number, action_file_define::Page* page);

  void onModuleEnable();
  void onModuleDisable();

  void brake();
  bool isRunning(int* playing_page_num, int* playing_step_num);
  bool loadPage(int page_number, action_file_define::Page* page);
  bool savePage(int page_number, action_file_define::Page* page);
  void resetPage(action_file_define::Page* page);

  void enableAllJoints();
  void actionPlayProcess(std::map<std::string, robotis_framework::Dynamixel *> dxls);

private:
  const int PRE_SECTION;
  const int MAIN_SECTION;
  const int POST_SECTION;
  const int PAUSE_SECTION;
  const int ZERO_FINISH;
  const int NONE_ZERO_FINISH;
  const bool DEBUG_PRINT;

  void queueThread();

  bool verifyChecksum( action_file_define::Page* page );
  void setChecksum( action_file_define::Page* page );

  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(std::string msg);

  bool isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request  &req,
                                op3_action_module_msgs::IsRunning::Response &res);

  void pageNumberCallback(const std_msgs::Int32::ConstPtr& msg);
  void startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr& msg);

  int convertRadTow4095(double rad);
  double convertw4095ToRad(int w4095);
  std::string convertIntToString(int n);

  std::map<std::string, bool> action_joints_enable_;
  std::map<std::string, robotis_framework::DynamixelState *> action_result_;
  int             control_cycle_msec_;
  boost::thread   queue_thread_;

  /* sample subscriber & publisher */
  ros::Publisher status_msg_pub_;
  ros::Publisher  done_msg_pub_;
  /////////////////////////////////////////////////////////////////////////
  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;
  FILE* action_file_;
  action_file_define::Page play_page_;
  action_file_define::Page next_play_page_;
  action_file_define::Step current_step_;

  int play_page_idx_;
  bool first_driving_start_;
  int page_step_count_;

  bool playing_;
  bool stop_playing_;
  bool playing_finished_;

  bool action_module_enabled_;
  bool previous_running_;
  bool present_running_;
};

}

#endif /* OP3_ACTION_MOTION_MODULE_H_ */
