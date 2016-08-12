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

#ifndef BALL_TRACKING_H_
#define BALL_TRACKING_H_

#include <math.h>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "ball_detector/circleSetStamped.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"

namespace robotis_op
{

// head tracking for looking the ball
class BallTracker
{
 public:
  BallTracker();
  ~BallTracker();

  bool processTracking();
  bool processActing();

  void startTracking();
  void stopTracking();

  double getPanOfBall()
  {
    return current_ball_pan_;
  }
  double getTiltOfBall()
  {
    return current_ball_tilt_;
  }

 protected:
  const double FOV_WIDTH;
  const double FOV_HEIGHT;
  const int NOT_FOUND_THRESHOLD;
  const double MAX_FB_STEP;
  const double MAX_RL_TURN;
  const double MIN_FB_STEP;
  const double MIN_RL_TURN;
  const double UNIT_FB_STEP;
  const double UNIT_RL_TURN;

  void ballPositionCallback(const ball_detector::circleSetStamped::ConstPtr &msg);
  void ballTrackerCommandCallback(const std_msgs::String::ConstPtr &msg);
  void currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void publishHeadJoint(double pan, double tilt);
  void scanBall();
  void approachBall(double pan, double tilt);
  void setWalkingCommand(const std::string &command);
  void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance = true);
  void getWalkingParam();

  void setModuleToDemo(const std::string &body_module);
  void parseJointNameFromYaml(const std::string &path);
  bool getJointNameFromID(const int &id, std::string &joint_name);
  bool getIDFromJointName(const std::string &joint_name, int &id);

  //ros node handle
  ros::NodeHandle nh_;

  //image publisher/subscriber
  ros::Publisher module_control_pub_;
  ros::Publisher head_joint_pub_;
  ros::Publisher head_scan_pub_;
  ros::Publisher set_walking_command_pub_;
  ros::Publisher set_walking_param_pub_;
  ;
  ros::Publisher motion_index_pub_;
  ros::ServiceClient get_walking_param_client_;

  ros::Subscriber ball_position_sub_;
  ros::Subscriber ball_tracking_command_sub_;
  ros::Subscriber current_joint_states_sub_;

  std::map<int, std::string> id_joint_table_;
  std::map<std::string, int> joint_id_table_;

  // (x, y) is the center position of the ball in image coordinates
  // z is the ball radius
  geometry_msgs::Point ball_position_;
  op3_walking_module_msgs::WalkingParam current_walking_param_;

  int count_not_found_;
  bool on_tracking_;
  int approach_ball_position_;
  double current_head_pan_, current_head_tilt_;
  double current_ball_pan_, current_ball_tilt_;
  double current_x_move_, current_r_angle_;
  int kick_motion_index_;

};
}

#endif /* BALL_TRACKING_H_ */
