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

#include "op3_demo/vision_demo.h"

namespace robotis_op
{

VisionDemo::VisionDemo()
    : SPIN_RATE(30)
{
  enable_ = false;

  ros::NodeHandle nh(ros::this_node::getName());

  boost::thread queue_thread = boost::thread(boost::bind(&VisionDemo::callbackThread, this));
  boost::thread process_thread = boost::thread(boost::bind(&VisionDemo::processThread, this));
}

VisionDemo::~VisionDemo()
{
  // TODO Auto-generated destructor stub
}

void VisionDemo::setDemoEnable()
{
  // change to motion module
  setModuleToDemo("action_module");

  usleep(100 * 1000);

  playMotion(WalkingReady);

  usleep(1500 * 1000);

  setModuleToDemo("head_control_module");

  usleep(10 * 1000);

  enable_ = true;
  face_tracker_.startTracking();

  ROS_INFO("Start Vision Demo");

}

void VisionDemo::setDemoDisable()
{

  face_tracker_.stopTracking();
  enable_ = false;
}

void VisionDemo::process()
{
  bool is_tracked = face_tracker_.processTracking();
}

void VisionDemo::processThread()
{
  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  //node loop
  while (ros::ok())
  {
    if (enable_ == true)
      process();

    //relax to fit output rate
    loop_rate.sleep();
  }
}

void VisionDemo::callbackThread()
{
  ros::NodeHandle nh(ros::this_node::getName());

  // subscriber & publisher
  module_control_pub_ = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);

  buttuon_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &VisionDemo::buttonHandlerCallback, this);
  faceCoord_sub_ = nh.subscribe("/faceCoord", 1, &VisionDemo::facePositionCallback, this);

  while (nh.ok())
  {
    ros::spinOnce();

    usleep(1000);
  }
}

void VisionDemo::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
//    switch (play_status_)
//      {
//        case PlayAction:
//        {
//          pauseProcess();
//          break;
//        }
//
//        case PauseAction:
//        {
//          resumeProcess();
//          break;
//        }
//
//        case StopAction:
//        {
//          resumeProcess();
//          break;
//        }
//
//        default:
//          break;
//      }
  }
  else if (msg->data == "mode")
  {

  }
}

void VisionDemo::setModuleToDemo(const std::string &module_name)
{
  std_msgs::String control_msg;
  control_msg.data = module_name;

  module_control_pub_.publish(control_msg);
  std::cout << "enable module : " << module_name << std::endl;
}

void VisionDemo::facePositionCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  if (enable_ == false)
    return;

  // face is detected
  if (msg->data.size() >= 10)
  {
    // center of face
    face_position_.x = (msg->data[6] + msg->data[8] * 0.5) / msg->data[2] * 2 - 1;
    face_position_.y = (msg->data[7] + msg->data[9] * 0.5) / msg->data[3] * 2 - 1;
    face_position_.z = msg->data[8] * 0.5 + msg->data[9] * 0.5;

    face_tracker_.setFacePosition(face_position_);
  }
  else
  {
    face_position_.x = 0;
    face_position_.y = 0;
    face_position_.z = 0;
    return;
  }
}

void VisionDemo::playMotion(int motion_index)
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_.publish(motion_msg);
}

} /* namespace robotis_op */
