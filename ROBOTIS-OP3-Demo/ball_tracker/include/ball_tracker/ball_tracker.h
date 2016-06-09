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

#ifndef ROBOTIS_OP3_ROBOTIS_OP3_DEMO_BALL_TRACKING_INCLUDE_BALL_TRACKING_BALL_TRACKING_H_
#define ROBOTIS_OP3_ROBOTIS_OP3_DEMO_BALL_TRACKING_INCLUDE_BALL_TRACKING_BALL_TRACKING_H_

#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "ball_detector/circleSetStamped.h"

namespace robotis_op {

class BallTracker
{
 public:
  BallTracker();
  ~BallTracker();

  void processTracking();

 protected:
  const double FOV_WIDTH = 30 * M_PI / 180;
  const double FOV_HEIGHT = 23 * M_PI / 180;

  void ballPositionCallback(const ball_detector::circleSetStamped::ConstPtr &msg);
  void publishHeadJoint(double pan, double tilt);

  //ros node handle
  ros::NodeHandle nh_;

  //image publisher/subscriber
  ros::Publisher head_joint_pub_;
  ros::Subscriber ball_position_sub_;

  // (x, y) is the center position of the ball in image coordinates
  // z is the ball radius
  geometry_msgs::Point ball_position_;

};
}

#endif /* SRC_ROBOTIS_OP3_ROBOTIS_OP3_DEMO_BALL_TRACKING_INCLUDE_BALL_TRACKING_BALL_TRACKING_H_ */
