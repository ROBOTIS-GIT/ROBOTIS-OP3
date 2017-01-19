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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>

#include "op3_demo/ball_tracker.h"
#include "op3_demo/ball_follower.h"
#include "robotis_math/robotis_linear_algebra.h"

enum Motion_Index
{
  GetUpFront = 81,
  GetUpBack = 82,
  RightKick = 83,
  LeftKick = 84,
  Ceremony = 85,
};

enum Stand_Status
{
  Stand = 0,
  Fallen_Forward = 1,
  Fallen_Behind = 2,
};

enum Robot_Status
{
  Waited = 0,
  TrackingAndFollowing = 1,
  ReadyToKick = 2,
  ReadyToCeremony = 3,
  ReadyToGetup = 4,
};

const double FALLEN_FORWARD_LIMIT = -60;
const double FALLEN_BEHIND_LIMIT = 60;
const int SPIN_RATE = 30;

void callbackThread();

void setBodyModuleToDemo(const std::string &body_module);
void setModuleToDemo(const std::string &module_name);
void parseJointNameFromYaml(const std::string &path);
bool getJointNameFromID(const int &id, std::string &joint_name);
bool getIDFromJointName(const std::string &joint_name, int &id);
void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
void demoCommandCallback(const std_msgs::String::ConstPtr& msg);
void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg);

void startSoccerMode();
void stopSoccerMode();

void handleKick(int ball_position);
bool handleFallen(int fallen_status);

void playMotion(int motion_index);

ros::Publisher module_control_pub;
ros::Publisher motion_index_pub;
ros::Subscriber buttuon_sub;
ros::Subscriber demo_command_sub;
ros::Subscriber imu_data_sub;
std::map<int, std::string> id_joint_table;
std::map<std::string, int> joint_id_table;

int wait_count = 0;
bool on_following_ball = false;
bool restart_soccer = false;
bool start_following = false;
bool stop_following = false;
bool stop_fallen_check = false;
int robot_status = Waited;
int stand_state = Stand;
double present_pitch = 0;

bool debug_code = false;

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "soccer_demo_node");

  //create ros wrapper object
  robotis_op::BallTracker tracker;
  robotis_op::BallFollower follower;

  ros::NodeHandle nh(ros::this_node::getName());

  std::string _default_path = ros::package::getPath("op3_gui_demo") + "/config/demo_config.yaml";
  std::string _path = nh.param<std::string>("demo_config", _default_path);
  parseJointNameFromYaml(_path);

  boost::thread queue_thread = boost::thread(boost::bind(&callbackThread));

  bool result = false;

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  tracker.startTracking();

  //node loop
  while (ros::ok())
  {
    // ball tracking
    bool is_tracked;
    is_tracked = tracker.processTracking();

    if (start_following == true)
    {
      tracker.startTracking();
      follower.startFollowing();
      start_following = false;

      wait_count = 1 * SPIN_RATE;  // wait 1 sec
    }

    if (stop_following == true)
    {
      follower.stopFollowing();
      stop_following = false;

      wait_count = 0;
    }

    if (wait_count <= 0)
    {
      // ball following
      if (on_following_ball == true)
      {
        if (is_tracked)
          follower.processFollowing(tracker.getPanOfBall(), tracker.getTiltOfBall(), tracker.getBallSize());
        else
          follower.waitFollowing();
      }

      // check fallen states
      switch (stand_state)
      {
        case Stand:
        {
          // check restart soccer
          if (restart_soccer == true)
          {
            restart_soccer = false;
            startSoccerMode();
            break;
          }

          // check states for kick
          int ball_position = follower.getBallPosition();
          if (ball_position != robotis_op::BallFollower::NotFound)
          {
            follower.stopFollowing();
            handleKick(ball_position);
          }
          break;
        }

          // fallen state : Fallen_Forward, Fallen_Behind
        default:
        {
          follower.stopFollowing();
          handleFallen(stand_state);
          break;
        }
      }
    }
    else
    {
      wait_count -= 1;
    }

    //execute pending callbacks
    ros::spinOnce();

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  return 0;
}

void callbackThread()
{
  ros::NodeHandle nh(ros::this_node::getName());

  // subscriber & publisher
  module_control_pub = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
  motion_index_pub = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  buttuon_sub = nh.subscribe("/robotis/open_cr/button", 1, buttonHandlerCallback);
  demo_command_sub = nh.subscribe("/ball_tracker/command", 1, demoCommandCallback);
  imu_data_sub = nh.subscribe("/robotis/open_cr/imu", 1, imuDataCallback);

  while (nh.ok())
  {
    ros::spinOnce();

    usleep(1000);
  }
}

void setBodyModuleToDemo(const std::string &body_module)
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  //std::string body_module = "action_module";
  std::string head_module = "head_control_module";

  // todo : remove hard coding
  for (int ix = 1; ix <= 20; ix++)
  {
    std::string joint_name;

    if (getJointNameFromID(ix, joint_name) == false)
      continue;

    control_msg.joint_name.push_back(joint_name);
    if (ix <= 18)
      control_msg.module_name.push_back(body_module);
    else
      control_msg.module_name.push_back(head_module);

  }

  // no control
  if (control_msg.joint_name.size() == 0)
    return;

  module_control_pub.publish(control_msg);
  std::cout << "enable module of body : " << body_module << std::endl;
}

void setModuleToDemo(const std::string &module_name)
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  // todo : remove hard coding
  for (int ix = 1; ix <= 20; ix++)
  {
    std::string joint_name;

    if (getJointNameFromID(ix, joint_name) == false)
      continue;

    control_msg.joint_name.push_back(joint_name);
    control_msg.module_name.push_back(module_name);
  }

  // no control
  if (control_msg.joint_name.size() == 0)
    return;

  module_control_pub.publish(control_msg);
  std::cout << "enable module : " << module_name << std::endl;
}

void parseJointNameFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load id_joint table yaml.");
    return;
  }

  // parse id_joint table
  YAML::Node _id_sub_node = doc["id_joint"];
  for (YAML::iterator _it = _id_sub_node.begin(); _it != _id_sub_node.end(); ++_it)
  {
    int _id;
    std::string _joint_name;

    _id = _it->first.as<int>();
    _joint_name = _it->second.as<std::string>();

    id_joint_table[_id] = _joint_name;
    joint_id_table[_joint_name] = _id;
  }
}

// joint id -> joint name
bool getJointNameFromID(const int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator _iter;

  _iter = id_joint_table.find(id);
  if (_iter == id_joint_table.end())
    return false;

  joint_name = _iter->second;
  return true;
}

// joint name -> joint id
bool getIDFromJointName(const std::string &joint_name, int &id)
{
  std::map<std::string, int>::iterator _iter;

  _iter = joint_id_table.find(joint_name);
  if (_iter == joint_id_table.end())
    return false;

  id = _iter->second;
  return true;
}

void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "start")
  {
    if (on_following_ball == true)
      stopSoccerMode();
    else
      startSoccerMode();
  }
}

void demoCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if (msg->data == "start")
  {
    if (on_following_ball == true)
      stopSoccerMode();
    else
      startSoccerMode();
  }
  else if (msg->data == "stop")
  {
    stopSoccerMode();
  }
}

// check fallen states
void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (stop_fallen_check == true)
    return;

  Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  Eigen::MatrixXd rpy_orientation = robotis_framework::convertQuaternionToRPY(orientation);
  rpy_orientation *= (180 / M_PI);

  // ROS_INFO("Roll : %3.2f, Pitch : %2.2f", rpy_orientation.coeff(0, 0), rpy_orientation.coeff(1, 0));

  double pitch = rpy_orientation.coeff(1, 0);

  if (present_pitch == 0)
    present_pitch = pitch;
  else
    present_pitch = present_pitch * 0.5 + pitch * 0.5;

  if (present_pitch < FALLEN_FORWARD_LIMIT)
    stand_state = Fallen_Forward;
  else if (present_pitch > FALLEN_BEHIND_LIMIT)
    stand_state = Fallen_Behind;
  else
    stand_state = Stand;
}

void startSoccerMode()
{
  setBodyModuleToDemo("walking_module");

  usleep(10 * 1000);

  ROS_INFO("Start Soccer Demo");
  on_following_ball = true;
  start_following = true;
}

void stopSoccerMode()
{
  ROS_INFO("Stop Soccer Demo");
  on_following_ball = false;
  stop_following = true;
}

void handleKick(int ball_position)
{
  usleep(500 * 1000);

  // change to motion module
  // setBodyModuleToDemo("action_module");
  setModuleToDemo("action_module");

  usleep(1000 * 1000);

  if (handleFallen(stand_state) == true)
    return;

  // kick motion
  switch (ball_position)
  {
    case robotis_op::BallFollower::OnRight:
      std::cout << "Kick Motion [R]: " << ball_position << std::endl;
      playMotion(RightKick);
      break;

    case robotis_op::BallFollower::OnLeft:
      std::cout << "Kick Motion [L]: " << ball_position << std::endl;
      playMotion(LeftKick);
      break;

    default:
      break;
  }

  on_following_ball = false;

  usleep(2000 * 1000);

  if (handleFallen(stand_state) == true)
    return;

  // ceremony
  std::cout << "Go Ceremony!!!" << std::endl;
  playMotion(Ceremony);
}

bool handleFallen(int fallen_status)
{
  if (fallen_status == Stand)
  {
    return false;
  }

  // change to motion module
  setModuleToDemo("action_module");

  usleep(600 * 1000);

  // getup motion
  switch (fallen_status)
  {
    case Fallen_Forward:
      std::cout << "Getup Motion [F]: " << std::endl;
      playMotion(GetUpFront);
      break;

    case Fallen_Behind:
      std::cout << "Getup Motion [B]: " << std::endl;
      playMotion(GetUpBack);
      break;

    default:
      break;
  }

  usleep(500 * 1000);

  if (on_following_ball == true)
    restart_soccer = true;

  // reset state
  //stand_state = Stand;
  on_following_ball = false;

  return true;
}

void playMotion(int motion_index)
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub.publish(motion_msg);
}
