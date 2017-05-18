/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Kayman Jung, Jay Song */

#ifndef ACTION_MOTION_MODULE_H_
#define ACTION_MOTION_MODULE_H_

#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>
#include <boost/thread.hpp>
#include <std_msgs/String.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "op3_action_module_msgs/StartAction.h"
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
