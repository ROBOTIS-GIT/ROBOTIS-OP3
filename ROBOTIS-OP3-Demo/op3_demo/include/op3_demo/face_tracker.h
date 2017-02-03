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

#ifndef FACE_TRACKING_H_
#define FACE_TRACKING_H_

#include <math.h>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>

namespace robotis_op
{

// head tracking for looking the ball
class FaceTracker
{
 public:
  enum TrackingStatus
  {
    NotFound = -1,
    Waiting = 0,
    Found = 1,

  };

  FaceTracker();
  ~FaceTracker();

  int processTracking();

  void startTracking();
  void stopTracking();

  void setUsingHeadScan(bool use_scan);
  void setFacePosition(geometry_msgs::Point &face_position);

  double getPanOfFace()
  {
    return current_face_pan_;
  }
  double getTiltOfFace()
  {
    return current_face_tilt_;
  }

 protected:
  const double FOV_WIDTH;
  const double FOV_HEIGHT;
  const int NOT_FOUND_THRESHOLD;

  void facePositionCallback(const geometry_msgs::Point::ConstPtr &msg);
  void faceTrackerCommandCallback(const std_msgs::String::ConstPtr &msg);
  void currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void publishHeadJoint(double pan, double tilt);
  void scanFace();

  //ros node handle
  ros::NodeHandle nh_;

  //image publisher/subscriber
  ros::Publisher module_control_pub_;
  ros::Publisher head_joint_pub_;
  ros::Publisher head_scan_pub_;

  ros::Subscriber face_position_sub_;
  ros::Subscriber face_tracking_command_sub_;
  ros::Subscriber current_joint_states_sub_;

  // (x, y) is the center position of the ball in image coordinates
  // z is the face size
  geometry_msgs::Point face_position_;

  bool use_head_scan_;
  int count_not_found_;
  bool on_tracking_;
  double current_head_pan_, current_head_tilt_;
  double current_face_pan_, current_face_tilt_;

  int dismissed_count_;

};
}

#endif /* FACE_TRACKING_H_ */
