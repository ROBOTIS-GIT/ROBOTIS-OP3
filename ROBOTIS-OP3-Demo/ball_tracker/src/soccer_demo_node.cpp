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

#include "ball_tracker/ball_tracker.h"
#include "ball_tracker/ball_follower.h"
#include "robotis_math/RobotisLinearAlgebra.h"

void setModuleToDemo(const std::string &body_module);
void parseJointNameFromYaml(const std::string &path);
bool getJointNameFromID(const int &id, std::string &joint_name);
bool getIDFromJointName(const std::string &joint_name, int &id);
void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
void demoCommandCallback(const std_msgs::String::ConstPtr& msg);
void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg);

void startSoccerMode();
void stopSoccerMode();

void playMotion(int motion_index);

ros::Publisher module_control_pub_;
ros::Publisher motion_index_pub_;
ros::Subscriber buttuon_sub_;
ros::Subscriber demo_command_sub_;
ros::Subscriber imu_data_sub_;
std::map<int, std::string> id_joint_table_;
std::map<std::string, int> joint_id_table_;

bool on_following_ball = false;
bool start_following = false;
bool stop_following = false;

enum Motion_Index
{
  GetUpFront = 81,
  GetUpBack = 82,
  RightKick = 83,
  LeftKick = 84,
  Ceremony = 85,
};

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "soccer_demo_node");

  //create ros wrapper object
  robotis_op::BallTracker tracker;
  robotis_op::BallFollower follower;

  ros::NodeHandle nh(ros::this_node::getName());

  std::string _default_path = ros::package::getPath("op3_demo") +"/config/demo_config.yaml";
  std::string _path = nh.param<std::string>("demo_config", _default_path);
  parseJointNameFromYaml(_path);

  // subscriber & publisher
  module_control_pub_  = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
  motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  buttuon_sub_ = nh.subscribe("/robotis/cm_740/button", 1, buttonHandlerCallback);
  demo_command_sub_ = nh.subscribe("/ball_tracker/command", 1, demoCommandCallback);

  int wait_count = 0;
  bool result = false;

  //set node loop rate
  ros::Rate loop_rate(30);

  tracker.startTracking();

  //node loop
  while ( ros::ok() )
  {
    // ball tracking
    bool is_tracked;
    is_tracked = tracker.processTracking();

    if(start_following == true)
    {
      tracker.startTracking();
      follower.startFollowing();
      start_following = false;

      wait_count = 1 * 30;  // wait 1 sec
    }

    if(stop_following == true)
    {
      follower.stopFollowing();
      stop_following = false;

      wait_count = 0;
    }

    if(wait_count <= 0)
    {
      // ball following
      if(on_following_ball == true)
      {
        if(is_tracked)
          follower.processFollowing(tracker.getPanOfBall(), tracker.getTiltOfBall());
        else
          follower.waitFollowing();
      }

      // check states
      int ball_position = follower.getBallPosition();


      // kick or getup motion
      if(ball_position != robotis_op::BallFollower::NotFound)
      {
        usleep(500 * 1000);

        setModuleToDemo("action_module");

        usleep(1000 * 1000);

        switch(ball_position)
        {
          case robotis_op::BallFollower::BallIsRight:
            std::cout << "Kick Motion [R]: " << ball_position << std::endl;
            playMotion(RightKick);
            break;

          case robotis_op::BallFollower::BallIsLeft:
            std::cout << "Kick Motion [L]: " << ball_position << std::endl;
            playMotion(LeftKick);
            break;

          default:
            break;
        }

        follower.stopFollowing();
        on_following_ball = false;

        usleep(2000 * 1000);

        playMotion(Ceremony);
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

void setModuleToDemo(const std::string &body_module)
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

void parseJointNameFromYaml(const std::string &path)
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
bool getJointNameFromID(const int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator _iter;

  _iter = id_joint_table_.find(id);
  if(_iter == id_joint_table_.end()) return false;

  joint_name = _iter->second;
  return true;
}

// joint name -> joint id
bool getIDFromJointName(const std::string &joint_name, int &id)
{
  std::map<std::string, int>::iterator _iter;

  _iter = joint_id_table_.find(joint_name);
  if(_iter == joint_id_table_.end()) return false;

  id = _iter->second;
  return true;
}

void buttonHandlerCallback( const std_msgs::String::ConstPtr& msg )
{
  if (msg->data == "start")
  {
    if(on_following_ball == true)
      stopSoccerMode();
    else
      startSoccerMode();
  }
}

void demoCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if(msg->data == "start")
  {
    if(on_following_ball == true)
      stopSoccerMode();
    else
      startSoccerMode();
  }
  else if(msg->data == "stop")
  {
    stopSoccerMode();
  }
}

void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{

}

void startSoccerMode()
{
  setModuleToDemo("walking_module");

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

void playMotion(int motion_index)
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_.publish(motion_msg);
}
