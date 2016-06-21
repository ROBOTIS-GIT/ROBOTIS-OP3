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

#include "ball_tracker/ball_tracker.h"

namespace robotis_op
{

BallTracker::BallTracker()
: nh_(ros::this_node::getName())
, FOV_WIDTH(30 * M_PI / 180)
, FOV_HEIGHT(23 * M_PI / 180)
, NOT_FOUND_THRESHOLD(50)
, count_not_found_(0)
, on_tracking_(false)
, approach_ball_position_(0)
, kick_motion_index_(83)
, MAX_FB_STEP(35.0 * 0.001)
, MAX_RL_TURN(15.0 * M_PI / 180)
, MIN_FB_STEP(5.0 * 0.001)
, MIN_RL_TURN(5.0 * M_PI / 180)
, UNIT_FB_STEP(1.0 * 0.001)
, UNIT_RL_TURN(0.5 * M_PI / 180)
, current_head_pan_(-10)
, current_head_tilt_(-10)
, current_x_move_(0.005)
, current_r_angle_(0)
{
  //module_control_pub_  = nh_.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  //module_control_pub_  = nh_.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
  head_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states_offset", 0);
  head_scan_pub_ = nh_.advertise<std_msgs::String>("/robotis/head_control/scan_command", 0);
  //motion_index_pub_ = nh_.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);

  ball_position_sub_ = nh_.subscribe("/ball_detector_node/circle_set", 1, &BallTracker::ballPositionCallback, this);
  ball_tracking_command_sub_ = nh_.subscribe("/ball_tracker/command", 1, &BallTracker::ballTrackerCommandCallback, this);
  current_joint_states_sub_ = nh_.subscribe("/robotis/goal_joint_states", 10, &BallTracker::currentJointStatesCallback, this);

  set_walking_command_pub_ = nh_.advertise<std_msgs::String>("/robotis/walking/command", 0);
  set_walking_param_pub_ = nh_.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 0);
  get_walking_param_client_ = nh_.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/robotis/walking/get_params");

  std::string _default_path = ros::package::getPath("op3_demo") +"/config/demo_config.yaml";
  std::string _path = nh_.param<std::string>("demo_config", _default_path);
  parseJointNameFromYaml(_path);
}

BallTracker::~BallTracker()
{

}

void BallTracker::ballPositionCallback(const ball_detector::circleSetStamped::ConstPtr &msg)
{
  for(int idx = 0; idx < msg->circles.size(); idx++ )
  {
    if(ball_position_.z >= msg->circles[idx].z)
      continue;

    ball_position_ = msg->circles[idx];
  }
}

void BallTracker::ballTrackerCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if(msg->data == "start")
  {
    startTracking();
  }
  else if(msg->data == "stop")
  {
    stopTracking();
  }
  else if(msg->data == "toggle_start")
  {
    if(on_tracking_ == false)
      startTracking();
    else
      stopTracking();
  }
}

void BallTracker::startTracking()
{
  on_tracking_ = true;
  ROS_INFO("Start Ball tracking");
//  setModuleToDemo("walking_module");
//
//  usleep(10 * 1000);

//  setWalkingCommand("start");
}

void BallTracker::stopTracking()
{
  on_tracking_ = false;
  approach_ball_position_ = 0;
  ROS_INFO("Stop Ball tracking");

//  setWalkingCommand("stop");
}

void BallTracker::currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  double pan, tilt;
  int _get = 0;

  for(int ix = 0; ix < msg->name.size(); ix++)
  {
    if(msg->name[ix] == "head_pan")
    {
      pan = - msg->position[ix];
      _get += 1;
    }
    else if(msg->name[ix] == "head_tilt")
    {
      tilt = msg->position[ix];
      _get += 1;
    }

    if(_get == 2) break;
  }

  // check variation
  if(current_head_pan_ == -10 || fabs(pan - current_head_pan_) < 5 * M_PI / 180 )
    current_head_pan_ = pan;
  if(current_head_tilt_ == -10 || fabs(tilt - current_head_tilt_) < 5 * M_PI / 180 )
    current_head_tilt_ = tilt;
}

bool BallTracker::processTracking()
{
  if(on_tracking_ == false)
  {
//    if(approach_ball_position_ != 0)
//    {
//      usleep(100 * 1000);
//
//
//      kick_motion_index_ = (approach_ball_position_ == -1) ? 84 : 83;
//      approach_ball_position_ = 0;
//
//      // kick
//      setModuleToDemo("action_module");
//
//      return false;
//    }

    ball_position_.z = 0;
    count_not_found_ = 0;
    current_head_pan_ = -10;
    current_head_tilt_ = -10;
    return false;
  }

  // check ball position
  if(ball_position_.z <= 0)
  {
    count_not_found_++;

//    if(count_not_found_ > NOT_FOUND_THRESHOLD * 0.5)
//      setWalkingParam(MIN_FB_STEP, 0, 0);

    if(count_not_found_ > NOT_FOUND_THRESHOLD)
    {
      scanBall();
      count_not_found_ = 0;
    }

    return false;
  }

  // if ball is found
  double x_offset_rad = - atan(ball_position_.x * tan(FOV_WIDTH));
  double y_offset_rad = - atan(ball_position_.y * tan(FOV_HEIGHT));

  ball_position_.z = 0;
  count_not_found_ = 0;

  // std::cout << "Target angle : " << x_offset_rad << " | " << y_offset_rad << std::endl;

  // move head joint
  publishHeadJoint(x_offset_rad, y_offset_rad);

  current_ball_pan_ = x_offset_rad;
  current_ball_tilt_ = y_offset_rad;

  // move to target position
  //approachBall(x_offset_rad, y_offset_rad);

  return true;
}

bool BallTracker::processActing()
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = kick_motion_index_;

  motion_index_pub_.publish(motion_msg);

  std::cout << "Kick Motion : " << kick_motion_index_ << std::endl;

  return true;
}

void BallTracker::publishHeadJoint(double pan, double tilt)
{
  double min_angle = 1 * M_PI / 180;
  if(fabs(pan) < min_angle && fabs(tilt) < min_angle)
    return;

  sensor_msgs::JointState _head_angle_msg;

  _head_angle_msg.name.push_back("head_pan");
  _head_angle_msg.name.push_back("head_tilt");

  _head_angle_msg.position.push_back(pan);
  _head_angle_msg.position.push_back(tilt);

  head_joint_pub_.publish(_head_angle_msg);
}

void BallTracker::scanBall()
{
  // check head control module enabled
  // ...

  // send message to head control module
  std_msgs::String _scan_msg;
  _scan_msg.data = "scan";

  head_scan_pub_.publish(_scan_msg);
  ROS_INFO("Scan the ball");
}

void BallTracker::approachBall(double pan, double tilt)
{
  // check walking module enabled
  // ...

  // generate navigation
  // check right/left

  // check to stop
  if((fabs(current_head_tilt_ + 70 * M_PI / 180) < 5 * M_PI / 180)
      && (fabs(current_head_pan_) < 3 * M_PI / 180))
  {
    ROS_INFO_STREAM("tilt : " << (current_head_tilt_ * 180 / M_PI) << " | pan : " << (current_head_pan_ * 180 / M_PI));

    setWalkingCommand("stop");
    on_tracking_ = false;

    // check direction of the ball
    if(current_head_pan_ > 0)
    {
      ROS_INFO("Ready to kick : left"); // left
      approach_ball_position_ = -1;
    }
    else
    {
      ROS_INFO("Ready to kick : right");  // right
      approach_ball_position_ = 1;
    }

    return;
  }

  approach_ball_position_ = 0;

  ROS_INFO("   ==============================================   ");

  // clac fb
  //double x_offset = 0.56 * (tan((17 + 70) * M_PI / 180 + current_tilt_) - tan(17 * M_PI / 180));
  double x_offset = 0.56 * tan(M_PI * 0.5 + current_head_tilt_ + tilt - 3 * M_PI / 180);

  if(x_offset < 0)
    x_offset *= (-1);
  x_offset -= 0.05;

  ROS_INFO_STREAM("goal offset : " << x_offset << " | " << (current_head_tilt_ * 180 / M_PI) << " | tile : " << (tilt * 180 / M_PI));
  double fb_goal, fb_move;


  if(x_offset < 0.2 && (fabs(current_head_pan_) < 3 * M_PI / 180))
  {
    ROS_INFO_STREAM("offest stop - tilt : " << (current_head_tilt_ * 180 / M_PI) << " | pan : " << (current_head_pan_ * 180 / M_PI));

    setWalkingCommand("stop");
    on_tracking_ = false;

    // check direction of the ball
    if(current_head_pan_ > 0)
    {
      ROS_INFO("Ready to kick : left"); // left
      approach_ball_position_ = -1;
    }
    else
    {
      ROS_INFO("Ready to kick : right");  // right
      approach_ball_position_ = 1;
    }

    return;
  }

  // fb_goal = fmin(x_offset * 0.2, MAX_FB_STEP);
  // fb_goal = fmax(fb_goal, MIN_FB_STEP);
  // fb_move = fmin(current_x_move_ + UNIT_FB_STEP, fb_goal);
  // ROS_INFO_STREAM("== x  offset : " << fb_move << " | " << fb_goal);

  fb_goal = fmin(x_offset * 0.2, MAX_FB_STEP);
  if(x_offset < 0.4)
  {
    fb_goal = fmin(current_x_move_ - UNIT_FB_STEP, fb_goal);
    fb_move = fmax(fb_goal, MIN_FB_STEP * 1.5);
  }
  else
  {
    fb_goal = fmin(current_x_move_ + UNIT_FB_STEP, fb_goal);
    fb_move = fmax(fb_goal, MIN_FB_STEP * 1.5);
  }


  // calc rl
  double rl_offset = fabs(current_head_pan_) * 0.3;
  double rl_goal, rl_angle;
  rl_goal = fmin(rl_offset, MAX_RL_TURN);
  rl_goal = fmax(rl_goal, MIN_RL_TURN);
  rl_angle = fmin(fabs(current_r_angle_) + UNIT_RL_TURN, rl_goal);

  if(current_head_pan_ > 0) rl_angle *= (-1);

  // ROS_INFO_STREAM("== rl  offset : " << (current_pan_ * 180 / M_PI) << " | " << (rl_angle * 180 / M_PI));
  ROS_INFO_STREAM("x_move : " << fb_move << ", rotation : " << (rl_angle * 180 / M_PI));

  // check to stop
  //  if((fabs(fb_move) < (MIN_FB_STEP * 1.5)) && (fabs(rl_angle) < (MIN_RL_TURN * 1.2)))
  //  {
  //    ROS_INFO("Approached the ball");
  //    setWalkingCommand("stop");
  //
  //    return;
  //  }

  // send message
  setWalkingParam(fb_move, 0, rl_angle);
}

void BallTracker::setWalkingCommand(const std::string &command)
{
  // get param
  if(command == "start")
  {
    getWalkingParam();
    setWalkingParam(0.005, 0, 0, true);
  }

  std_msgs::String _command_msg;
  _command_msg.data = command;
  set_walking_command_pub_.publish(_command_msg);

  ROS_INFO_STREAM("Send Walking command : " << command);
}

void BallTracker::setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance)
{
  current_walking_param_.balance_enable = balance;
  current_walking_param_.x_move_amplitude = x_move;
  current_walking_param_.y_move_amplitude = y_move;
  current_walking_param_.angle_move_amplitude = rotation_angle;

  set_walking_param_pub_.publish(current_walking_param_);
  // ROS_INFO("Change walking param");

  current_x_move_ = x_move;
  current_r_angle_ = rotation_angle;
}

void BallTracker::getWalkingParam()
{
  op3_walking_module_msgs::GetWalkingParam walking_param_msg;

  if(get_walking_param_client_.call(walking_param_msg))
  {
    current_walking_param_ = walking_param_msg.response.parameters;

    // update ui
    ROS_INFO("Get walking parameters");
  }
  else
    ROS_ERROR("Fail to get walking parameters.");
}

void BallTracker::setModuleToDemo(const std::string &body_module)
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  //std::string body_module = "action_module";
  std::string head_module = "head_control_module";

  for(int ix = 1; ix <= 20; ix++)
  {
    std::string joint_name;

    if(getJointNameFromID(ix, joint_name) ==false)
      continue;

    control_msg.joint_name.push_back(joint_name);
    if(ix <= 18)
      control_msg.module_name.push_back(body_module);
    else
      control_msg.module_name.push_back(head_module);

  }

  // no control
  if(control_msg.joint_name.size() == 0) return;

  module_control_pub_.publish(control_msg);
  std::cout << "enable module" << std::endl;
}

void BallTracker::parseJointNameFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load id_joint table yaml.");
    return;
  }

  // parse id_joint table
  YAML::Node _id_sub_node = doc["id_joint"];
  for(YAML::iterator _it = _id_sub_node.begin() ; _it != _id_sub_node.end() ; ++_it)
  {
    int _id;
    std::string _joint_name;

    _id = _it->first.as<int>();
    _joint_name = _it->second.as<std::string>();

    id_joint_table_[_id] = _joint_name;
    joint_id_table_[_joint_name] = _id;
  }
}

// joint id -> joint name
bool BallTracker::getJointNameFromID(const int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator _iter;

  _iter = id_joint_table_.find(id);
  if(_iter == id_joint_table_.end()) return false;

  joint_name = _iter->second;
  return true;
}

// joint name -> joint id
bool BallTracker::getIDFromJointName(const std::string &joint_name, int &id)
{
  std::map<std::string, int>::iterator _iter;

  _iter = joint_id_table_.find(joint_name);
  if(_iter == joint_id_table_.end()) return false;

  id = _iter->second;
  return true;
}

}

