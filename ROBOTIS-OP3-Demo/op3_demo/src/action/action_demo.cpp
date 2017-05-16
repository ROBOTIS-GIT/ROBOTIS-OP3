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

#include "op3_demo/action_demo.h"

namespace robotis_op
{

ActionDemo::ActionDemo()
    : SPIN_RATE(30),
      DEMO_INIT_POSE(8),
      play_index_(0),
      start_play_(false),
      stop_play_(false),
      pause_play_(false),
      play_status_(StopAction)
{
  enable_ = false;

  ros::NodeHandle nh(ros::this_node::getName());

  std::string default_path = ros::package::getPath("op3_demo") + "/script/action_script.yaml";
  script_path_ = nh.param<std::string>("action_script", default_path);

  std::string default_play_list = "default";
  play_list_name_ = nh.param<std::string>("action_script_play_list", default_play_list);

  parseActionScript (script_path_);

  boost::thread queue_thread = boost::thread(boost::bind(&ActionDemo::callbackThread, this));
  boost::thread process_thread = boost::thread(boost::bind(&ActionDemo::processThread, this));
}

ActionDemo::~ActionDemo()
{
  // TODO Auto-generated destructor stub
}

void ActionDemo::setDemoEnable()
{
  setModuleToDemo("action_module");

  usleep(10 * 1000);

  enable_ = true;

  ROS_INFO("Start ActionScript Demo");

  playAction(DEMO_INIT_POSE);

  startProcess(play_list_name_);
}

void ActionDemo::setDemoDisable()
{
  stopProcess();

  enable_ = false;

  play_list_.resize(0);
}

void ActionDemo::process()
{
  // check current status
  //handleStatus();

  switch (play_status_)
  {
    case PlayAction:
    {
      if (play_list_.size() == 0)
      {
        ROS_WARN("Play List is empty.");
        return;
      }

      // action is not running
      if (isActionRunning() == false)
      {
        // play
        bool result_play = playActionWithSound(play_list_.at(play_index_));

        ROS_INFO_COND(!result_play, "Fail to play action script.");

        // add play index
        int index_to_play = (play_index_ + 1) % play_list_.size();
        play_index_ = index_to_play;
      }
      else
      {
        // wait
        return;
      }
      break;
    }

    case PauseAction:
    {
      stopMP3();
      stopAction();

      break;
    }

    case StopAction:
    {
      stopMP3();
      stopAction();

      break;
    }

    default:
      break;
  }
}

void ActionDemo::handleStatus()
{
  if (start_play_ == true)
  {
    play_status_ = PlayAction;
    start_play_ = false;
  }

  if (pause_play_ == true)
  {
    play_status_ = PauseAction;
    pause_play_ = false;
  }

  if (stop_play_ == true)
  {
    play_status_ = StopAction;
    stop_play_ = false;
  }
}

void ActionDemo::startProcess(const std::string &set_name)
{
  parseActionScriptSetName(script_path_, set_name);

  start_play_ = true;
  play_status_ = PlayAction;
}

void ActionDemo::resumeProcess()
{
  start_play_ = true;
  play_status_ = PlayAction;
}

void ActionDemo::pauseProcess()
{
  pause_play_ = true;
  play_status_ = PauseAction;
}

void ActionDemo::stopProcess()
{
  stop_play_ = true;
  play_index_ = 0;
  play_status_ = StopAction;
}

void ActionDemo::processThread()
{
  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  //node loop
  while (ros::ok())
  {
    if (enable_ == true)
      process();

    //relax to fit output rate
    loop_rate.sleep();
  }
}

void ActionDemo::callbackThread()
{
  ros::NodeHandle nh(ros::this_node::getName());

  // subscriber & publisher
  module_control_pub_ = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  play_sound_pub_ = nh.advertise<std_msgs::String>("/play_sound_file", 0);

  buttuon_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &ActionDemo::buttonHandlerCallback, this);

  is_running_client_ = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");

  while (nh.ok())
  {
    ros::spinOnce();

    usleep(1000);
  }
}

void ActionDemo::parseActionScript(const std::string &path)
{
  YAML::Node doc;

  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Fail to load action script yaml. - " << e.what());
    ROS_ERROR_STREAM("Script Path : " << path);
    return;
  }

  // parse action_sound table
  YAML::Node sub_node = doc["action_and_sound"];
  for (YAML::iterator yaml_it = sub_node.begin(); yaml_it != sub_node.end(); ++yaml_it)
  {
    int action_index = yaml_it->first.as<int>();
    std::string mp3_path = yaml_it->second.as<std::string>();

    action_sound_table_[action_index] = mp3_path;
  }

  // default action set
  if (doc["default"])
    play_list_ = doc["default"].as<std::vector<int> >();
}

bool ActionDemo::parseActionScriptSetName(const std::string &path, const std::string &set_name)
{

  YAML::Node doc;

  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml.");
    return false;
  }

  // parse action_sound table
  if (doc[set_name])
  {
    play_list_ = doc[set_name].as<std::vector<int> >();
    return true;
  }
  else
    return false;
}

bool ActionDemo::playActionWithSound(int motion_index)
{
  std::map<int, std::string>::iterator map_it = action_sound_table_.find(motion_index);
  if (map_it == action_sound_table_.end())
    return false;

  playAction(motion_index);
  playMP3(map_it->second);

  ROS_INFO_STREAM("action : " << motion_index << ", mp3 path : " << map_it->second);

  return true;
}

void ActionDemo::playMP3(std::string &path)
{
  std_msgs::String sound_msg;
  sound_msg.data = path;

  play_sound_pub_.publish(sound_msg);
}

void ActionDemo::stopMP3()
{
  std_msgs::String sound_msg;
  sound_msg.data = "";

  play_sound_pub_.publish(sound_msg);
}

void ActionDemo::playAction(int motion_index)
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_.publish(motion_msg);
}

void ActionDemo::stopAction()
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = StopActionCommand;

  motion_index_pub_.publish(motion_msg);
}

void ActionDemo::brakeAction()
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = BrakeActionCommand;

  motion_index_pub_.publish(motion_msg);
}

// check running of action
bool ActionDemo::isActionRunning()
{
  op3_action_module_msgs::IsRunning is_running_srv;

  if (is_running_client_.call(is_running_srv) == false)
  {
    ROS_ERROR("Failed to get action status");
    return true;
  }
  else
  {
    if (is_running_srv.response.is_running == true)
    {
      return true;
    }
  }

  return false;
}

void ActionDemo::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    switch (play_status_)
      {
        case PlayAction:
        {
          pauseProcess();
          break;
        }

        case PauseAction:
        {
          resumeProcess();
          break;
        }

        case StopAction:
        {
          resumeProcess();
          break;
        }

        default:
          break;
      }
  }
  else if (msg->data == "mode")
  {

  }
}

void ActionDemo::setModuleToDemo(const std::string &module_name)
{
//  robotis_controller_msgs::JointCtrlModule control_msg;
//
//  // todo : remove hard coding
//  for (int ix = 1; ix <= 20; ix++)
//  {
//    std::string joint_name;
//
//    if (getJointNameFromID(ix, joint_name) == false)
//      continue;
//
//    control_msg.joint_name.push_back(joint_name);
//    control_msg.module_name.push_back(module_name);
//  }
//
//  // no control
//  if (control_msg.joint_name.size() == 0)
//    return;
//
//  module_control_pub_.publish(control_msg);
//  std::cout << "enable module : " << module_name << std::endl;

  std_msgs::String control_msg;
  control_msg.data = "action_module";

  module_control_pub_.publish(control_msg);
  std::cout << "enable module : " << module_name << std::endl;
}

} /* namespace robotis_op */
