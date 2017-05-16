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

#ifndef ACTION_DEMO_H_
#define ACTION_DEMO_H_

#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "op3_demo/op_demo.h"
#include "robotis_controller_msgs/JointCtrlModule.h"
#include "op3_action_module_msgs/IsRunning.h"

namespace robotis_op
{

class ActionDemo : public OPDemo
{
 public:
  ActionDemo();
  ~ActionDemo();

  void setDemoEnable();
  void setDemoDisable();

 protected:
  enum ActionCommandIndex
  {
    BrakeActionCommand = -2,
    StopActionCommand = -1,
  };

  enum ActionStatus
  {
    PlayAction = 1,
    PauseAction = 2,
    StopAction = 3,
  };

  const int SPIN_RATE;
  const int DEMO_INIT_POSE;

  void processThread();
  void callbackThread();

  void process();
  void startProcess(const std::string &set_name = "default");
  void resumeProcess();
  void pauseProcess();
  void stopProcess();

  void handleStatus();

  void parseActionScript(const std::string &path);
  bool parseActionScriptSetName(const std::string &path, const std::string &set_name);

  bool playActionWithSound(int motion_index);

  void playMP3(std::string &path);
  void stopMP3();

  void playAction(int motion_index);
  void stopAction();
  void brakeAction();
  bool isActionRunning();

  void setModuleToDemo(const std::string &module_name);

  void actionSetNameCallback(const std_msgs::String::ConstPtr& msg);
  void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);

  ros::Publisher module_control_pub_;
  ros::Publisher motion_index_pub_;
  ros::Publisher play_sound_pub_;

  ros::Subscriber buttuon_sub_;

  ros::ServiceClient is_running_client_;

  std::map<int, std::string> action_sound_table_;
  std::vector<int> play_list_;

  std::string script_path_;
  std::string play_list_name_;
  int play_index_;

  bool start_play_;
  bool stop_play_;
  bool pause_play_;

  int play_status_;
};

} /* namespace robotis_op */

#endif /* ACTION_DEMO_H_ */
