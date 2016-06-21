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

#include <stdio.h>
#include "cm_740_module/cm_740_module.h"

using namespace ROBOTIS;

CM740Module::CM740Module()
: control_cycle_msec_(8)
, DEBUG(false)
, button_mode_(false)
, button_start_(false)
, present_volt_(0.0)
, volt_count_(0)
{
  module_name     = "cm_740_module"; // set unique module name

  result["gyro_x"] = 0.0;
  result["gyro_y"] = 0.0;
  result["gyro_z"] = 0.0;

  result["acc_x"] = 0.0;
  result["acc_y"] = 0.0;
  result["acc_z"] = 0.0;

  result["button_mode"] = 0;
  result["button_start"] = 0;

  result["present_voltage"] = 0.0;
}

CM740Module::~CM740Module()
{
  queue_thread_.join();
}

void CM740Module::Initialize(const int control_cycle_msec, Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_       = boost::thread(boost::bind(&CM740Module::QueueThread, this));
}

void CM740Module::QueueThread()
{
  ros::NodeHandle     _ros_node;
  ros::CallbackQueue  _callback_queue;

  _ros_node.setCallbackQueue(&_callback_queue);

  /* subscriber */
  //sub1_ = _ros_node.subscribe("/tutorial_topic", 10, &CM740Module::TopicCallback, this);

  /* publisher */
  status_msg_pub_         = _ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  imu_pub_                = _ros_node.advertise<sensor_msgs::Imu>("/robotis/cm_740/imu", 1);
  reset_dxl_pub_          = _ros_node.advertise<std_msgs::String>("/robotis/cm_740/button", 1);

  while(_ros_node.ok())
  {
    _callback_queue.callAvailable();

    usleep(100);
  }
}

void CM740Module::Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, Sensor *> sensors)
{
  if(sensors["cm-740"] == NULL) return;

  UINT16_T gyro_x = sensors["cm-740"]->sensor_state->bulk_read_table["gyro_x"];
  UINT16_T gyro_y = sensors["cm-740"]->sensor_state->bulk_read_table["gyro_y"];
  UINT16_T gyro_z = sensors["cm-740"]->sensor_state->bulk_read_table["gyro_z"];

  UINT16_T acc_x = sensors["cm-740"]->sensor_state->bulk_read_table["acc_x"];
  UINT16_T acc_y = sensors["cm-740"]->sensor_state->bulk_read_table["acc_y"];
  UINT16_T acc_z = sensors["cm-740"]->sensor_state->bulk_read_table["acc_z"];

  UINT8_T present_volt = sensors["cm-740"]->sensor_state->bulk_read_table["present_voltage"];

  result["gyro_x"] = getGyroValue(gyro_x);
  result["gyro_y"] = getGyroValue(gyro_y);
  result["gyro_z"] = getGyroValue(gyro_z);

  if(DEBUG) ROS_INFO("Gyro : %f, %f, %f", result["gyro_x"], result["gyro_y"], result["gyro_z"]);

  // align axis of Accelerometer to robot
  result["acc_x"] = - getAccValue(acc_y);
  result["acc_y"] = getAccValue(acc_x);
  result["acc_z"] = - getAccValue(acc_z);

  if(DEBUG) ROS_INFO("Acc : %f, %f, %f", result["acc_x"], result["acc_y"], result["acc_z"]);

  UINT8_T button_flag = sensors["cm-740"]->sensor_state->bulk_read_table["button"];
  result["button_mode"] = button_flag & 0x01;
  result["button_start"] = (button_flag & 0x02) >> 1;

  buttonMode(result["button_mode"] == 1.0);
  buttonStart(result["button_start"] == 1.0);

  result["present_voltage"] = present_volt * 0.1;
  handleVoltage(result["present_voltage"]);

  fusionIMU();
}

// -500 ~ 500dps, dps -> rps
double CM740Module::getGyroValue(int dxl_value)
{
  return (dxl_value - 512) * 500.0 * 2.0 / 1023 * deg2rad;
}

// -4.0 ~ 4.0g, 1g = 9.8 m/s^2
double CM740Module::getAccValue(int dxl_value)
{
  return (dxl_value - 512) * 4.0 * 2.0 / 1023;
}

void CM740Module::fusionIMU()
{
  // fusion imu data
  imu_msg_.header.stamp = ros::Time::now();
  imu_msg_.header.frame_id = "body_link";

  double filter_alpha = 0.4;

  //in rad/s
  long int _value = 0;
  int _arrd_length = 2;
  imu_msg_.angular_velocity.x = lowPassFilter(filter_alpha, result["gyro_x"], imu_msg_.angular_velocity.x);
  imu_msg_.angular_velocity.y = lowPassFilter(filter_alpha, result["gyro_y"], imu_msg_.angular_velocity.y);
  imu_msg_.angular_velocity.z = lowPassFilter(filter_alpha, result["gyro_z"], imu_msg_.angular_velocity.z);
  // ROS_INFO("angular velocity : %f, %f, %f", imu_angular_velocity[0], imu_angular_velocity[1], imu_angular_velocity[2]);

  //in m/s^2
  double _const = 1;
  imu_msg_.linear_acceleration.x = lowPassFilter(filter_alpha, result["acc_x"] * G_ACC, imu_msg_.linear_acceleration.x);
  imu_msg_.linear_acceleration.y = lowPassFilter(filter_alpha, result["acc_y"] * G_ACC, imu_msg_.linear_acceleration.y);
  imu_msg_.linear_acceleration.z = lowPassFilter(filter_alpha, result["acc_z"] * G_ACC, imu_msg_.linear_acceleration.z);
  // ROS_INFO("linear_acceleration : %f, %f, %f", imu_linear_acceleration[0], imu_linear_acceleration[1], imu_linear_acceleration[2]);

  //Estimation of roll and pitch based on accelometer data, see http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
  double roll = atan2(- result["acc_x"], result["acc_z"]);
  double pitch = atan2( result["acc_y"], sqrt( result["acc_x"] * result["acc_x"] + result["acc_z"] * result["acc_z"]));
  double yaw = 0.0;

  Eigen::Quaterniond orientation = rpy2quaternion(roll, pitch, yaw);

  imu_msg_.orientation.x = orientation.x();
  imu_msg_.orientation.y = orientation.y();
  imu_msg_.orientation.z = orientation.z();
  imu_msg_.orientation.w = orientation.w();

  imu_pub_.publish(imu_msg_);
}

void CM740Module::buttonMode(bool pushed)
{
  if(button_mode_ == pushed)
    return;

  button_mode_ = pushed;

  if(pushed == true)
    handleButton("mode");
}

void CM740Module::buttonStart(bool pushed)
{
  if(button_start_ == pushed)
    return;

  button_start_ = pushed;

  if(pushed == true)
    handleButton("start");
}

void CM740Module::handleButton(const std::string &button_name)
{
  std_msgs::String _button_msg;
  _button_msg.data = button_name;

  reset_dxl_pub_.publish(_button_msg);
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Button : " + button_name);
}

void CM740Module::handleVoltage(double present_volt)
{
  double _ratio = 0.4;
  previous_volt_ = (previous_volt_ != 0) ? previous_volt_ * (1 - _ratio) + present_volt * _ratio : present_volt;

  if(fabs(present_volt_ - previous_volt_) >= 0.1)
  {
    present_volt_ = previous_volt_;
    std::stringstream _ss;
    _ss << "Present Volt : " << present_volt_ << "V";
    publishStatusMsg((present_volt_ < 11 ? robotis_controller_msgs::StatusMsg::STATUS_WARN : robotis_controller_msgs::StatusMsg::STATUS_INFO), _ss.str());
    if(DEBUG) ROS_INFO("Present Volt : %fV, Read Volt : %fV", previous_volt_, result["present_voltage"]);
  }
}

void CM740Module::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg _status;
  _status.header.stamp = ros::Time::now();
  _status.type = type;
  _status.module_name = "SENSOR";
  _status.status_msg = msg;

  status_msg_pub_.publish(_status);
}

double CM740Module::lowPassFilter(double alpha, double x_new, double x_old)
{
  return alpha*x_new + (1.0-alpha)*x_old;
}
