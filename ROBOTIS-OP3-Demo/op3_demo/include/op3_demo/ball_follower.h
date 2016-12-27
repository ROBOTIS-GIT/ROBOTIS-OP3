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

#ifndef BALL_FOLLOWER_H_
#define BALL_FOLLOWER_H_

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

// following the ball using walking
class BallFollower
{
 public:
  enum
  {
    NotFound = 0,
    OnRight = 1,
    OnLeft = 2,
  };

  BallFollower();
  ~BallFollower();

  bool processFollowing(double x_angle, double y_angle, double ball_size);
  void waitFollowing();
  void startFollowing();
  void stopFollowing();

  int getBallPosition()
  {
    return approach_ball_position_;
  }

 protected:
  const double CAMERA_HEIGHT;
  const int NOT_FOUND_THRESHOLD;
  const double FOV_WIDTH;
  const double FOV_HEIGHT;
  const double MAX_FB_STEP;
  const double MAX_RL_TURN;
  const double MIN_FB_STEP;
  const double MIN_RL_TURN;
  const double UNIT_FB_STEP;
  const double UNIT_RL_TURN;

  const double SPOT_FB_OFFSET;
  const double SPOT_RL_OFFSET;
  const double SPOT_ANGLE_OFFSET;

  void currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void setWalkingCommand(const std::string &command);
  void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance = true);
  void getWalkingParam();

  bool debug_print_;

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

  // (x, y) is the center position of the ball in image coordinates
  // z is the ball radius
  geometry_msgs::Point ball_position_;
  op3_walking_module_msgs::WalkingParam current_walking_param_;

  int count_not_found_;
  int count_to_kick_;
  int accum_ball_position_;
  bool on_tracking_;
  int approach_ball_position_;
  double current_pan_, current_tilt_;
  double current_x_move_, current_r_angle_;
  int kick_motion_index_;

};
}

#endif /* BALL_FOLLOWER_H_ */
