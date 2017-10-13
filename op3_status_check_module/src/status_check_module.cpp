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

#include <stdio.h>
#include "op3_status_check_module/status_check_module.h"

using namespace robotis_op;

StatusCheckModule::StatusCheckModule()
  : control_cycle_msec_(8)
{
  module_name_ = "op3_status_check_module"; // set unique module name
}

StatusCheckModule::~StatusCheckModule()
{
  queue_thread_.join();
}

void StatusCheckModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&StatusCheckModule::queueThread, this));
}

void StatusCheckModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publisher */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void StatusCheckModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
    std::map<std::string, robotis_framework::Sensor *> sensors)
{
  static int cnt = 0;

  if (++cnt < (1000/control_cycle_msec_))
    return;
  cnt = 0;

  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type         = robotis_controller_msgs::StatusMsg::STATUS_ERROR;
  status.module_name  = "StatusCheck";
  std::string msg     = "";

  for (auto& dxl_it : dxls)
  {
    auto ctrl_table_it = dxl_it.second->dxl_state_->bulk_read_table_.find("hardware_error_status");
    if(ctrl_table_it != dxl_it.second->dxl_state_->bulk_read_table_.end())
    {
      uint8_t hw_error = ctrl_table_it->second;
      if(hw_error != 0)
      {
        // add hardware error status message
        msg += "[" + dxl_it.first + "]";

        if (hw_error & 0x01)  // input voltage error
        {
          msg += " - Input Voltage Error";
        }

        //  if (hw_error & 0x02)  // motor hall sensor error
        //  {
        //    msg += " - Motor Hall Sensor Error";
        //  }

        if (hw_error & 0x04)  // overheating error
        {
          msg += " - Overheating Error";
        }

        if (hw_error & 0x08)  // motor encoder error
        {
          msg += " - Motor Encoder Error";
        }

        if (hw_error & 0x10)  // electronical shock error
        {
          msg += " - Electronical Shock Error";
        }

        if (hw_error & 0x20)  // overload error
        {
          msg += " - Overload Error";
        }
        msg += "]\n";
      }
    }
  }

  if (msg != "")
  {
    status.status_msg = msg;
    status_msg_pub_.publish(status);
  }
}

