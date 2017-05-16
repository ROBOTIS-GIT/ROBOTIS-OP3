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

#include "op3_demo/button_test.h"

namespace robotis_op
{

ButtonTest::ButtonTest()
    : SPIN_RATE(30),
      led_count_(0),
      rgb_led_count_(0)
//      is_tracking_(false),
//      tracking_status_(FaceTracker::Waiting)
{
  enable_ = false;

  ros::NodeHandle nh(ros::this_node::getName());

  boost::thread queue_thread = boost::thread(boost::bind(&ButtonTest::callbackThread, this));
  boost::thread process_thread = boost::thread(boost::bind(&ButtonTest::processThread, this));

  default_mp3_path_ = ros::package::getPath("op3_demo") + "/Data/mp3/";
}

ButtonTest::~ButtonTest()
{
  // TODO Auto-generated destructor stub
}

void ButtonTest::setDemoEnable()
{
  // change to motion module
//  setModuleToDemo("action_module");

//  usleep(100 * 1000);

//  playMotion(InitPose);

//  usleep(1500 * 1000);

//  setModuleToDemo("head_control_module");

//  usleep(10 * 1000);

  enable_ = true;
//  face_tracker_.startTracking();

  ROS_INFO("Start Button Test");

}

void ButtonTest::setDemoDisable()
{

//  face_tracker_.stopTracking();
//  is_tracking_ = false;
//  tracking_status_ = FaceTracker::Waiting;
  enable_ = false;
}

void ButtonTest::process()
{
  //bool is_tracked = face_tracker_.processTracking();
//  int tracking_status = face_tracker_.processTracking();

//if(is_tracking_ != is_tracked)
//  if(tracking_status_ != tracking_status)
//  {
//    switch(tracking_status)
//    {
//      case FaceTracker::Found:
//        setRGBLED(0x1F, 0x1F, 0x1F);
//        break;
//
//      case FaceTracker::NotFound:
//        setRGBLED(0, 0, 0);
//        break;
//
//      default:
//        break;
//    }
//  }

//  if(tracking_status != FaceTracker::Waiting)
//    tracking_status_ = tracking_status;

//is_tracking_ = is_tracked;
//  std::cout << "Tracking : " << tracking_status << std::endl;
}

void ButtonTest::processThread()
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

void ButtonTest::callbackThread()
{
  ros::NodeHandle nh(ros::this_node::getName());

  // subscriber & publisher
//  module_control_pub_ = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
//  motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  rgb_led_pub_ = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);

  buttuon_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &ButtonTest::buttonHandlerCallback, this);
//  faceCoord_sub_ = nh.subscribe("/faceCoord", 1, &ButtonTest::facePositionCallback, this);

  while (nh.ok())
  {
    ros::spinOnce();

    usleep(1 * 1000);
  }
}

// button test
void ButtonTest::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "mode")
  {
    playSound(default_mp3_path_ + "Mode button.mp3");
  }
  else if (msg->data == "start")
  {
    playSound(default_mp3_path_ + "Start button.mp3");
    int rgb_selector[3] = { 1, 0, 0 };
    setRGBLED(0x1F * rgb_selector[rgb_led_count_ % 3], 0x1F * rgb_selector[(rgb_led_count_ + 1) % 3],
              0x1F * rgb_selector[(rgb_led_count_ + 2) % 3]);
    rgb_led_count_ += 1;
  }
  else if (msg->data == "user")
  {
    playSound(default_mp3_path_ + "User button.mp3");
    setLED(0x01 << (led_count_++ % 3));
  }

  else if (msg->data == "mode_long")
  {
    playSound(default_mp3_path_ + "Mode button long press.mp3");
  }
  else if (msg->data == "start_long")
  {
    playSound(default_mp3_path_ + "Start button long press.mp3");
  }
  else if (msg->data == "user_long")
  {
    playSound(default_mp3_path_ + "User button long press.mp3");
  }
}

//void ButtonTest::setModuleToDemo(const std::string &module_name)
//{
//  std_msgs::String control_msg;
//  control_msg.data = module_name;
//
//  module_control_pub_.publish(control_msg);
//  std::cout << "enable module : " << module_name << std::endl;
//}
//
//void ButtonTest::facePositionCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
//{
//  if (enable_ == false)
//    return;
//
//  // face is detected
//  if (msg->data.size() >= 10)
//  {
//    // center of face
//    face_position_.x = (msg->data[6] + msg->data[8] * 0.5) / msg->data[2] * 2 - 1;
//    face_position_.y = (msg->data[7] + msg->data[9] * 0.5) / msg->data[3] * 2 - 1;
//    face_position_.z = msg->data[8] * 0.5 + msg->data[9] * 0.5;
//
//    face_tracker_.setFacePosition(face_position_);
//  }
//  else
//  {
//    face_position_.x = 0;
//    face_position_.y = 0;
//    face_position_.z = 0;
//    return;
//  }
//}
//
//void ButtonTest::playMotion(int motion_index)
//{
//  std_msgs::Int32 motion_msg;
//  motion_msg.data = motion_index;
//
//  motion_index_pub_.publish(motion_msg);
//}

void ButtonTest::setRGBLED(int blue, int green, int red)
{
  int led_full_unit = 0x1F;
  int led_value = (blue & led_full_unit) << 10 | (green & led_full_unit) << 5 | (red & led_full_unit);
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED_RGB";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led_value);

  rgb_led_pub_.publish(syncwrite_msg);
}

void ButtonTest::setLED(int led)
{
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led);

  rgb_led_pub_.publish(syncwrite_msg);
}

void ButtonTest::playSound(const std::string &path)
{
  std_msgs::String sound_msg;
  sound_msg.data = path;

  play_sound_pub_.publish(sound_msg);
}

} /* namespace robotis_op */
