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


/* Author: Kayman Jung */

#ifndef ACTION_MOTION_MODULE_H_
#define ACTION_MOTION_MODULE_H_

#define _USE_MATH_DEFINES
#include <cmath>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "action_file_define.h"

namespace ROBOTIS
{

class ActionModule : public MotionModule, public Singleton<ActionModule>
{
 private:
  int             control_cycle_msec_;
  boost::thread   queue_thread_;

  /* sample subscriber & publisher */
  ros::Subscriber action_page_sub_;
  ros::Publisher  status_msg_pub_;

  void QueueThread();


  /////////////////////////////////////////////////////////////////////////
  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;
  FILE* action_file_;
  ACTION_FILE::PAGE play_page_;
  ACTION_FILE::PAGE next_play_page_;
  ACTION_FILE::STEP current_step_;

  int  play_page_idx_;
  bool first_driving_start_;
  int  page_step_count_;

  bool playing_;
  bool stop_playing_;
  bool playing_finished;

  bool VerifyChecksum( ACTION_FILE::PAGE* _page );
  void SetChecksum( ACTION_FILE::PAGE* _page );

  int  RadTow4095(double _rad);
  double w4095ToRad(int _w4095);

  void PublishStatusMsg(unsigned int type, std::string msg);

  std::string IntToString(int _n);

  bool previous_enable_;
  bool present_enable_;
  bool previous_running_;
  bool present_running_;

  //////////////////////////////////////////////////////////////
 public:
  ActionModule();
  virtual ~ActionModule();

  void Initialize(const int control_cycle_msec, Robot *robot);
  void Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors);

  void Stop();
  bool IsRunning();

  void OnModuleEnable();
  void OnModuleDisable();

  ////////////////////////////////////////////////////////////////
  void PageNumberCallback(const std_msgs::Int32::ConstPtr& _msg);

  bool LoadFile(char* _file_name);
  bool CreateFile(char* _file_name);

  bool Start(int _page_number);
  bool Start(char* _page_name);
  bool Start(int _page_number, ACTION_FILE::PAGE *_page);

  void Brake();
  bool IsRunning(int* _playing_page_num, int* _playing_step_num);
  bool LoadPage(int _page_number, ACTION_FILE::PAGE* _page);
  bool SavePage(int _page_number, ACTION_FILE::PAGE* _page);
  void ResetPage(ACTION_FILE::PAGE* _page);

  void ActionPlayProcess(std::map<std::string, Dynamixel *> dxls);
};

}



#endif /* OP3_ACTION_MOTION_MODULE_H_ */
