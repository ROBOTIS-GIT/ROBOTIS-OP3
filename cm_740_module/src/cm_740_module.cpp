/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman */

#include <stdio.h>
#include "cm_740_module/cm_740_module.h"

namespace robotis_op
{

CM740Module::CM740Module()
    : control_cycle_msec_(8),
      DEBUG_PRINT(false),
      present_volt_(0.0)
{
  module_name_ = "cm_740_module";  // set unique module name

  result_["gyro_x"] = 0.0;
  result_["gyro_y"] = 0.0;
  result_["gyro_z"] = 0.0;

  result_["acc_x"] = 0.0;
  result_["acc_y"] = 0.0;
  result_["acc_z"] = 0.0;

  result_["button_mode"] = 0;
  result_["button_start"] = 0;

  result_["present_voltage"] = 0.0;
  buttons_["button_mode"] = false;
  buttons_["button_start"] = false;
  buttons_["published_mode"] = false;
  buttons_["published_start"] = false;

  last_msg_time_ = ros::Time::now();
}

CM740Module::~CM740Module()
{
  queue_thread_.join();
}

void CM740Module::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&CM740Module::queueThread, this));
}

void CM740Module::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publisher */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  imu_pub_ = ros_node.advertise<sensor_msgs::Imu>("/robotis/cm_740/imu", 1);
  button_pub_ = ros_node.advertise<std_msgs::String>("/robotis/cm_740/button", 1);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

void CM740Module::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                          std::map<std::string, robotis_framework::Sensor *> sensors)
{
  if (sensors["cm-740"] == NULL)
    return;

  uint16_t gyro_x = sensors["cm-740"]->sensor_state_->bulk_read_table_["gyro_x"];
  uint16_t gyro_y = sensors["cm-740"]->sensor_state_->bulk_read_table_["gyro_y"];
  uint16_t gyro_z = sensors["cm-740"]->sensor_state_->bulk_read_table_["gyro_z"];

  uint16_t acc_x = sensors["cm-740"]->sensor_state_->bulk_read_table_["acc_x"];
  uint16_t acc_y = sensors["cm-740"]->sensor_state_->bulk_read_table_["acc_y"];
  uint16_t acc_z = sensors["cm-740"]->sensor_state_->bulk_read_table_["acc_z"];

  uint16_t present_volt = sensors["cm-740"]->sensor_state_->bulk_read_table_["present_voltage"];

  result_["gyro_x"] = getGyroValue(gyro_x);
  result_["gyro_y"] = getGyroValue(gyro_y);
  result_["gyro_z"] = getGyroValue(gyro_z);

  ROS_INFO_COND(DEBUG_PRINT, "Gyro : %f, %f, %f", result_["gyro_x"], result_["gyro_y"], result_["gyro_z"]);

  // align axis of Accelerometer to robot
  result_["acc_x"] = -getAccValue(acc_y);
  result_["acc_y"] = getAccValue(acc_x);
  result_["acc_z"] = -getAccValue(acc_z);

  ROS_INFO_COND(DEBUG_PRINT, "Acc : %f, %f, %f", result_["acc_x"], result_["acc_y"], result_["acc_z"]);

  uint8_t button_flag = sensors["cm-740"]->sensor_state_->bulk_read_table_["button"];
  result_["button_mode"] = button_flag & 0x01;
  result_["button_start"] = (button_flag & 0x02) >> 1;

  handleButton("mode");
  handleButton("start");

  result_["present_voltage"] = present_volt * 0.1;
  handleVoltage(result_["present_voltage"]);

  fusionIMU();
}

// -500 ~ 500dps, dps -> rps
double CM740Module::getGyroValue(int raw_value)
{
  return (raw_value - 512) * 500.0 * 2.0 / 1023 * DEGREE2RADIAN;
}

// -4.0 ~ 4.0g, 1g = 9.8 m/s^2
double CM740Module::getAccValue(int raw_value)
{
  return (raw_value - 512) * 4.0 * 2.0 / 1023;
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
  imu_msg_.angular_velocity.x = lowPassFilter(filter_alpha, result_["gyro_x"], imu_msg_.angular_velocity.x);
  imu_msg_.angular_velocity.y = lowPassFilter(filter_alpha, result_["gyro_y"], imu_msg_.angular_velocity.y);
  imu_msg_.angular_velocity.z = lowPassFilter(filter_alpha, result_["gyro_z"], imu_msg_.angular_velocity.z);

  //in m/s^2
  imu_msg_.linear_acceleration.x = lowPassFilter(filter_alpha, result_["acc_x"] * G_ACC,
                                                 imu_msg_.linear_acceleration.x);
  imu_msg_.linear_acceleration.y = lowPassFilter(filter_alpha, result_["acc_y"] * G_ACC,
                                                 imu_msg_.linear_acceleration.y);
  imu_msg_.linear_acceleration.z = lowPassFilter(filter_alpha, result_["acc_z"] * G_ACC,
                                                 imu_msg_.linear_acceleration.z);

  //Estimation of roll and pitch based on accelometer data, see http://www.nxp.com/files/sensors/doc/app_note/AN3461.pdf
  double mui = 0.01;
  double sign = copysignf(1.0, result_["acc_z"]);
  double roll = atan2(result_["acc_y"],
                      sign * sqrt(result_["acc_z"] * result_["acc_z"] + mui * result_["acc_x"] * result_["acc_x"]));
  double pitch = atan2(-result_["acc_x"],
                       sqrt(result_["acc_y"] * result_["acc_y"] + result_["acc_z"] * result_["acc_z"]));
  double yaw = 0.0;

  Eigen::Quaterniond orientation = robotis_framework::convertRPYToQuaternion(roll, pitch, yaw);

  imu_msg_.orientation.x = orientation.x();
  imu_msg_.orientation.y = orientation.y();
  imu_msg_.orientation.z = orientation.z();
  imu_msg_.orientation.w = orientation.w();

  imu_pub_.publish(imu_msg_);
}

void CM740Module::handleButton(const std::string &button_name)
{
  std::string button_key = "button_" + button_name;
  std::string button_published = "published_" + button_name;

  bool pushed = (result_[button_key] == 1.0);
  // same state
  if (buttons_[button_key] == pushed)
  {
    if (pushed == true && buttons_[button_published] == false)
    {
      // check long press
      ros::Duration button_duration = ros::Time::now() - buttons_press_time_[button_name];
      if (button_duration.toSec() > 2.0)
      {
        publishButtonMsg(button_name + "_long");
        buttons_[button_published] = true;
      }
    }
  }
  else    // state is changed
  {
    buttons_[button_key] = pushed;

    if (pushed == true)
    {
      buttons_press_time_[button_name] = ros::Time::now();
      buttons_[button_published] = false;
    }
    else
    {
      ros::Duration button_duration = ros::Time::now() - buttons_press_time_[button_name];

      if (button_duration.toSec() < 2)     // short press
        publishButtonMsg(button_name);
      else
        // long press
        ;

    }
  }
}

void CM740Module::publishButtonMsg(const std::string &button_name)
{
  std_msgs::String button_msg;
  button_msg.data = button_name;

  button_pub_.publish(button_msg);
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Button : " + button_name);
}

void CM740Module::handleVoltage(double present_volt)
{
  double voltage_ratio = 0.4;
  previous_volt_ =
      (previous_volt_ != 0) ? previous_volt_ * (1 - voltage_ratio) + present_volt * voltage_ratio : present_volt;

  if (fabs(present_volt_ - previous_volt_) >= 0.1)
  {
    // check last published time
    ros::Time now = ros::Time::now();
    ros::Duration dur = now - last_msg_time_;
    if (dur.sec < 1)
      return;

    last_msg_time_ = now;

    present_volt_ = previous_volt_;
    std::stringstream log_stream;
    log_stream << "Present Volt : " << present_volt_ << "V";
    publishStatusMsg(
        (present_volt_ < 11.0 ?
            robotis_controller_msgs::StatusMsg::STATUS_WARN : robotis_controller_msgs::StatusMsg::STATUS_INFO),
        log_stream.str());
    ROS_INFO_COND(DEBUG_PRINT, "Present Volt : %fV, Read Volt : %fV", previous_volt_, result_["present_voltage"]);
  }
}

void CM740Module::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "SENSOR";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

double CM740Module::lowPassFilter(double alpha, double x_new, double x_old)
{
  return alpha * x_new + (1.0 - alpha) * x_old;
}

}
