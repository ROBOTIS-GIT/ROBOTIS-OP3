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

#include "ball_tracker/face_tracker.h"

namespace robotis_op
{

FaceTracker::FaceTracker()
    : nh_(ros::this_node::getName()),
      FOV_WIDTH(30 * M_PI / 180),
      FOV_HEIGHT(23 * M_PI / 180),
      NOT_FOUND_THRESHOLD(50),
      use_head_scan_(false),
      count_not_found_(0),
      on_tracking_(false),
      current_head_pan_(-10),
      current_head_tilt_(-10)
{
  head_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states_offset", 0);
  head_scan_pub_ = nh_.advertise<std_msgs::String>("/robotis/head_control/scan_command", 0);

  face_position_sub_ = nh_.subscribe("/face_position", 1, &FaceTracker::facePositionCallback, this);
  face_tracking_command_sub_ = nh_.subscribe("/face_tracker/command", 1, &FaceTracker::faceTrackerCommandCallback,
                                             this);
  current_joint_states_sub_ = nh_.subscribe("/robotis/goal_joint_states", 10, &FaceTracker::currentJointStatesCallback,
                                            this);

}

FaceTracker::~FaceTracker()
{

}

void FaceTracker::facePositionCallback(const geometry_msgs::Point::ConstPtr &msg)
{
  if (msg->z < 0)
    return;

  face_position_ = *msg;
}

void FaceTracker::faceTrackerCommandCallback(const std_msgs::String::ConstPtr &msg)
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

void FaceTracker::startTracking()
{
  on_tracking_ = true;
  ROS_INFO("Start Face tracking");
}

void FaceTracker::stopTracking()
{
  on_tracking_ = false;
  ROS_INFO("Stop Face tracking");
}

void FaceTracker::setUsingHeadScan(bool use_scan)
{
  use_head_scan_ = use_scan;
}

void FaceTracker::setFacePosition(geometry_msgs::Point &face_position)
{
  if (face_position.z > 0)
  {
    face_position_ = face_position;
  }
}

void FaceTracker::currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
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

bool FaceTracker::processTracking()
{
  if (on_tracking_ == false)
  {
    face_position_.z = 0;
    count_not_found_ = 0;
    current_head_pan_ = -10;
    current_head_tilt_ = -10;
    return false;
  }

  // check ball position
  if (face_position_.z <= 0)
  {
    count_not_found_++;

    if (count_not_found_ > NOT_FOUND_THRESHOLD)
    {
      scanFace();
      count_not_found_ = 0;
    }

    return false;
  }

  // if face is detected
  double x_offset_rad = -atan(face_position_.x * tan(FOV_WIDTH));
  double y_offset_rad = -atan(face_position_.y * tan(FOV_HEIGHT));

  face_position_.z = 0;
  count_not_found_ = 0;

  std::cout << "Target angle : " << x_offset_rad << " | " << y_offset_rad << std::endl;

  // move head joint
  publishHeadJoint(x_offset_rad, y_offset_rad);

  current_face_pan_ = x_offset_rad;
  current_face_tilt_ = y_offset_rad;

  return true;
}

void FaceTracker::publishHeadJoint(double pan, double tilt)
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

void FaceTracker::scanFace()
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

