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

enum Demo_Status
{
  Ready = 0,
  DesireToTrack = 1,
  DesireToStop = 2,
  Tracking = 3,
  DemoCount = 4,
};
int current_status = Ready;

void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);


//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "ball_tracker_node");
  ros::NodeHandle nh("~");
  ros::Publisher module_control_pub_ = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  ros::Subscriber buttuon_sub = nh.subscribe("/robotis/cm_740/button", 1, buttonHandlerCallback);

  //create ros wrapper object
  robotis_op::BallTracker tracker;

  // set head_control_module
  std_msgs::String control_msg;
  control_msg.data = "head_control_module";

  usleep(1000 * 1000);

  module_control_pub_.publish(control_msg);

  // start ball tracking
  tracker.startTracking();
  current_status = Tracking;

  //set node loop rate
  ros::Rate loop_rate(30);

  //node loop
  while (ros::ok())
  {
    switch (current_status)
    {
      case DesireToTrack:
        tracker.startTracking();
        current_status = Tracking;
        break;

      case DesireToStop:
        tracker.stopTracking();
        current_status = Ready;
        break;

      case Tracking:
        tracker.processTracking();
        break;

      default:
        break;
    }

    //execute pending callback
    ros::spinOnce();

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  return 0;
}

void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "mode_long")
  {

  }
  else if (msg->data == "start_long")
  {
    // it's using in op3_manager
    // torque on and going to init pose
  }

  if (msg->data == "start")
  {
    if (current_status == Ready)
      current_status = DesireToTrack;
    else if (current_status == Tracking)
      current_status = DesireToStop;
  }
  else if (msg->data == "mode")
  {

  }

}
