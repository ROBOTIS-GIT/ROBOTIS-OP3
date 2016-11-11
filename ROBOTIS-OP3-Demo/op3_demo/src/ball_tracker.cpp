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

#include "op3_demo/ball_tracker.h"

namespace robotis_op
{

BallTracker::BallTracker()
    : nh_(ros::this_node::getName()),
      FOV_WIDTH(30 * M_PI / 180),
      FOV_HEIGHT(23 * M_PI / 180),
      NOT_FOUND_THRESHOLD(50),
      use_head_scan_(true),
      count_not_found_(0),
      on_tracking_(false),
      current_head_pan_(-10),
      current_head_tilt_(-10)
{
  head_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states_offset", 0);
  head_scan_pub_ = nh_.advertise<std_msgs::String>("/robotis/head_control/scan_command", 0);

  ball_position_sub_ = nh_.subscribe("/ball_detector_node/circle_set", 1, &BallTracker::ballPositionCallback, this);
  ball_tracking_command_sub_ = nh_.subscribe("/ball_tracker/command", 1, &BallTracker::ballTrackerCommandCallback,
                                             this);
  current_joint_states_sub_ = nh_.subscribe("/robotis/goal_joint_states", 10, &BallTracker::currentJointStatesCallback,
                                            this);

}

BallTracker::~BallTracker()
{

}

void BallTracker::ballPositionCallback(const ball_detector::circleSetStamped::ConstPtr &msg)
{
  for (int idx = 0; idx < msg->circles.size(); idx++)
  {
    if (ball_position_.z >= msg->circles[idx].z)
      continue;

    ball_position_ = msg->circles[idx];
  }
}

void BallTracker::ballTrackerCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if (msg->data == "start")
  {
    startTracking();
  }
  else if (msg->data == "stop")
  {
    stopTracking();
  }
  else if (msg->data == "toggle_start")
  {
    if (on_tracking_ == false)
      startTracking();
    else
      stopTracking();
  }
}

void BallTracker::startTracking()
{
  on_tracking_ = true;
  ROS_INFO("Start Ball tracking");
}

void BallTracker::stopTracking()
{
  on_tracking_ = false;
  ROS_INFO("Stop Ball tracking");
}

void BallTracker::setUsingHeadScan(bool use_scan)
{
  use_head_scan_ = use_scan;
}

void BallTracker::currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  double pan, tilt;
  int get_count = 0;

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    if (msg->name[ix] == "head_pan")
    {
      pan = -msg->position[ix];
      get_count += 1;
    }
    else if (msg->name[ix] == "head_tilt")
    {
      tilt = msg->position[ix];
      get_count += 1;
    }

    if (get_count == 2)
      break;
  }

  // check variation
  if (current_head_pan_ == -10 || fabs(pan - current_head_pan_) < 5 * M_PI / 180)
    current_head_pan_ = pan;
  if (current_head_tilt_ == -10 || fabs(tilt - current_head_tilt_) < 5 * M_PI / 180)
    current_head_tilt_ = tilt;
}

bool BallTracker::processTracking()
{
  if (on_tracking_ == false)
  {
    ball_position_.z = 0;
    count_not_found_ = 0;
    current_head_pan_ = -10;
    current_head_tilt_ = -10;
    return false;
  }

  // check ball position
  if (ball_position_.z <= 0)
  {
    count_not_found_++;

    if (count_not_found_ > NOT_FOUND_THRESHOLD)
    {
      scanBall();
      count_not_found_ = 0;
    }

    return false;
  }

  // if ball is found
  // convert ball position to desired angle(rad) of head
  double x_offset_rad = -atan(ball_position_.x * tan(FOV_WIDTH));
  double y_offset_rad = -atan(ball_position_.y * tan(FOV_HEIGHT));

  ball_position_.z = 0;
  count_not_found_ = 0;

  // std::cout << "Target angle : " << x_offset_rad << " | " << y_offset_rad << std::endl;

  // move head joint
  publishHeadJoint(x_offset_rad, y_offset_rad);

  current_ball_pan_ = x_offset_rad;
  current_ball_tilt_ = y_offset_rad;

  double ball_x_angle = (current_head_pan_ + x_offset_rad) * (180 / M_PI);
  double ball_y_angle = (current_head_tilt_ + y_offset_rad) * (180 / M_PI);

  // Pan : left(-), right(+) / Tilt : bottom(-), top(+)
  /*ROS_INFO("   ============== Head | Ball ==============   ");
  ROS_INFO_STREAM(
      "== Head Pan : " << (current_head_pan_ * 180 / M_PI) << " | Ball X : " << (x_offset_rad * 180 / M_PI) << " | " << ball_x_angle);
  ROS_INFO_STREAM(
      "== Head Tilt : " << (current_head_tilt_ * 180 / M_PI) << " | Ball Y : " << (y_offset_rad * 180 / M_PI) << " | " << ball_y_angle);
*/
  return true;
}

void BallTracker::publishHeadJoint(double pan, double tilt)
{
  double min_angle = 1 * M_PI / 180;
  if (fabs(pan) < min_angle && fabs(tilt) < min_angle)
    return;

  sensor_msgs::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");

  head_angle_msg.position.push_back(pan);
  head_angle_msg.position.push_back(tilt);

  head_joint_pub_.publish(head_angle_msg);
}

void BallTracker::scanBall()
{
  if (use_head_scan_ == false)
    return;

  // check head control module enabled
  // ...

  // send message to head control module
  std_msgs::String scan_msg;
  scan_msg.data = "scan";

  head_scan_pub_.publish(scan_msg);
  // ROS_INFO("Scan the ball");
}

}

