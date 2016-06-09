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
{
  head_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states_offset", 0);
  ball_position_sub_ = nh_.subscribe("/ball_detector_node/circle_set", 1, &BallTracker::ballPositionCallback, this);
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

void BallTracker::processTracking()
{
  // check ball position
  if(ball_position_.z <= 0)
    return;

  // if ball is found
  double x_offset_rad = - atan(ball_position_.x * tan(FOV_WIDTH));
  double y_offset_rad = - atan(ball_position_.y * tan(FOV_HEIGHT));

  ball_position_.z = 0;

  std::cout << "Target angle : " << x_offset_rad << " | " << y_offset_rad << std::endl;

  // move head joint
  publishHeadJoint(x_offset_rad, y_offset_rad);
}

void BallTracker::publishHeadJoint(double pan, double tilt)
{
  sensor_msgs::JointState _head_angle_msg;

  _head_angle_msg.name.push_back("head_pan");
  _head_angle_msg.name.push_back("head_tilt");

  _head_angle_msg.position.push_back(pan);
  _head_angle_msg.position.push_back(tilt);

  head_joint_pub_.publish(_head_angle_msg);
}

}

