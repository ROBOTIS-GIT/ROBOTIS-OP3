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

#ifndef VISION_DEMO_H_
#define VISION_DEMO_H_

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Point.h>

#include "op3_demo/op_demo.h"
#include "op3_demo/face_tracker.h"

namespace robotis_op
{

class VisionDemo : public OPDemo
{
 public:
  VisionDemo();
  ~VisionDemo();

  void setDemoEnable();
  void setDemoDisable();

 protected:
  const int SPIN_RATE;

  void processThread();
  void callbackThread();

  void process();

  void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
  //void ballPositionCallback(const ball_detector::circleSetStamped::ConstPtr &msg);
  //void currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void facePositionCallback(const std_msgs::Int32MultiArray::ConstPtr &msg);

  //void publishHeadJoint(double pan, double tilt);

  void setModuleToDemo(const std::string &module_name);

  FaceTracker face_tracker_;

  ros::Publisher module_control_pub_;
  ros::Subscriber buttuon_sub_;
  ros::Subscriber faceCoord_sub_;
  geometry_msgs::Point face_position_;
};

} /* namespace robotis_op */

#endif /* VISION_DEMO_H_ */
