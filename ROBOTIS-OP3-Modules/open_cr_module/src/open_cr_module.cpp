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
#include "open_cr_module/open_cr_module.h"

namespace robotis_op
{

OpenCRModule::OpenCRModule()
    : control_cycle_msec_(8),
      debug_print_(false),
      button_mode_(false),
      button_start_(false),
      present_volt_(0.0),
      button_count_for_debug_(0)
{
  module_name_ = "open_cr_module";  // set unique module name

  result_["gyro_x"] = 0.0;
  result_["gyro_y"] = 0.0;
  result_["gyro_z"] = 0.0;

  result_["acc_x"] = 0.0;
  result_["acc_y"] = 0.0;
  result_["acc_z"] = 0.0;

  result_["button_mode"] = 0;
  result_["button_start"] = 0;
  result_["button_user"] = 0;

  result_["present_voltage"] = 0.0;

  buttons_["button_mode"] = false;
  buttons_["button_start"] = false;
  buttons_["button_user"] = false;

  // buttons_press_time_["button_mode"] = ros::Time::now();
  // buttons_press_time_["button_start"] = ros::Time::now();
  // buttons_press_time_["button_user"] = ros::Time::now();

  last_msg_time_ = ros::Time::now();
}

OpenCRModule::~OpenCRModule()
{
  queue_thread_.join();
}

void OpenCRModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&OpenCRModule::queueThread, this));
}

void OpenCRModule::queueThread()
{
  ros::NodeHandle _ros_node;
  ros::CallbackQueue _callback_queue;

  _ros_node.setCallbackQueue(&_callback_queue);

  /* subscriber */
  //sub1_ = _ros_node.subscribe("/tutorial_topic", 10, &OpenCRModule::TopicCallback, this);
  /* publisher */
  status_msg_pub_ = _ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  imu_pub_ = _ros_node.advertise<sensor_msgs::Imu>("/robotis/open_cr/imu", 1);
  imu_pub_2_ = _ros_node.advertise<sensor_msgs::Imu>("/robotis/open_cr/imu2", 1);
  reset_dxl_pub_ = _ros_node.advertise<std_msgs::String>("/robotis/open_cr/button", 1);

  while (_ros_node.ok())
  {
    _callback_queue.callAvailable();

    usleep(100);
  }
}

void OpenCRModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                           std::map<std::string, robotis_framework::Sensor *> sensors)
{
  if (sensors["open-cr"] == NULL)
    return;

  int16_t gyro_x = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_x"];
  int16_t gyro_y = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_y"];
  int16_t gyro_z = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_z"];

  int16_t acc_x = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_x"];
  int16_t acc_y = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_y"];
  int16_t acc_z = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_z"];

  uint16_t present_volt = sensors["open-cr"]->sensor_state_->bulk_read_table_["present_voltage"];

  result_["gyro_x"] = - getGyroValue(gyro_x);
  result_["gyro_y"] = - getGyroValue(gyro_y);
  result_["gyro_z"] = getGyroValue(gyro_z);

  ROS_INFO_COND(debug_print_, "Gyro Raw =============================================== ");
  ROS_INFO_COND(debug_print_, "Raw : %d, %d, %d", gyro_x, gyro_y, gyro_z);
  ROS_INFO_COND(debug_print_, "Gyro : %f, %f, %f", result_["gyro_x"], result_["gyro_y"], result_["gyro_z"]);

  // align axis of Accelerometer to robot
  result_["acc_x"] = - getAccValue(acc_x);
  result_["acc_y"] = - getAccValue(acc_y);
  result_["acc_z"] = getAccValue(acc_z);

  ROS_INFO_COND(debug_print_, "Acc Raw =============================================== ");
  ROS_INFO_COND(debug_print_, "Raw : %d, %d, %d", acc_x, acc_y, acc_z);
  ROS_INFO_COND(debug_print_, "Acc : %f, %f, %f", result_["acc_x"], result_["acc_y"], result_["acc_z"]);

  uint8_t button_flag = sensors["open-cr"]->sensor_state_->bulk_read_table_["button"];
  result_["button_mode"] = button_flag & 0x01;
  result_["button_start"] = (button_flag & 0x02) >> 1;
  result_["button_user"] = (button_flag & 0x04) >> 2;

  // pushedModeButton(result_["button_mode"] == 1.0);
  // pushedStartButton(result_["button_start"] == 1.0);
  handleButton("mode");
  handleButton("start");
  handleButton("user");

  result_["present_voltage"] = present_volt * 0.1;
  handleVoltage(result_["present_voltage"]);

  fusionIMU();

  double roll = DEGREE2RADIAN * sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_x"];
  double pitch = DEGREE2RADIAN * sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_y"];
  double yaw = DEGREE2RADIAN * sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_z"];

  publishIMU(roll, pitch, yaw);
}

// -2000 ~ 2000dps(-32800 ~ 32800), scale factor : 16.4, dps -> rps
double OpenCRModule::getGyroValue(int raw_value)
{
  return (double) raw_value * GYRO_FACTOR * DEGREE2RADIAN;
}

// -2.0 ~ 2.0g(-32768 ~ 32768), 1g = 9.8 m/s^2
double OpenCRModule::getAccValue(int raw_value)
{
  return (double) raw_value * ACCEL_FACTOR;
}

void OpenCRModule::fusionIMU()
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
  // ROS_INFO("angular velocity : %f, %f, %f", imu_angular_velocity[0], imu_angular_velocity[1], imu_angular_velocity[2]);

  //in m/s^2
  imu_msg_.linear_acceleration.x = lowPassFilter(filter_alpha, result_["acc_x"] * G_ACC,
                                                 imu_msg_.linear_acceleration.x);
  imu_msg_.linear_acceleration.y = lowPassFilter(filter_alpha, result_["acc_y"] * G_ACC,
                                                 imu_msg_.linear_acceleration.y);
  imu_msg_.linear_acceleration.z = lowPassFilter(filter_alpha, result_["acc_z"] * G_ACC,
                                                 imu_msg_.linear_acceleration.z);
  // ROS_INFO("linear_acceleration : %f, %f, %f", imu_linear_acceleration[0], imu_linear_acceleration[1], imu_linear_acceleration[2]);

  //Estimation of roll and pitch based on accelometer data, see http://www.nxp.com/files/sensors/doc/app_note/AN3461.pdf
  double mui = 0.01;
  double sign = copysignf(1.0, result_["acc_z"]);
  double roll = atan2(result_["acc_y"],
                      sign * sqrt(result_["acc_z"] * result_["acc_z"] + mui * result_["acc_x"] * result_["acc_x"]));
  double pitch = atan2(-result_["acc_x"],
                       sqrt(result_["acc_y"] * result_["acc_y"] + result_["acc_z"] * result_["acc_z"]));
  double yaw = 0.0;

  // ROS_INFO("Roll : %3.2f, Pitch : %2.2f", (roll * 180 / M_PI), (pitch * 180 / M_PI));

  Eigen::Quaterniond orientation = robotis_framework::convertRPYToQuaternion(roll, pitch, yaw);

  imu_msg_.orientation.x = orientation.x();
  imu_msg_.orientation.y = orientation.y();
  imu_msg_.orientation.z = orientation.z();
  imu_msg_.orientation.w = orientation.w();

  imu_pub_.publish(imu_msg_);
}

void OpenCRModule::publishIMU(double roll, double pitch, double yaw)
{
  // fusion imu data
  imu_msg_2_.header.stamp = ros::Time::now();
  imu_msg_2_.header.frame_id = "body_link";

  double filter_alpha = 0.4;

  //in rad/s
  long int _value = 0;
  int _arrd_length = 2;
  imu_msg_2_.angular_velocity.x = lowPassFilter(filter_alpha, result_["gyro_x"], imu_msg_2_.angular_velocity.x);
  imu_msg_2_.angular_velocity.y = lowPassFilter(filter_alpha, result_["gyro_y"], imu_msg_2_.angular_velocity.y);
  imu_msg_2_.angular_velocity.z = lowPassFilter(filter_alpha, result_["gyro_z"], imu_msg_2_.angular_velocity.z);

  //in m/s^2
  imu_msg_2_.linear_acceleration.x = lowPassFilter(filter_alpha, result_["acc_x"] * G_ACC,
                                                   imu_msg_2_.linear_acceleration.x);
  imu_msg_2_.linear_acceleration.y = lowPassFilter(filter_alpha, result_["acc_y"] * G_ACC,
                                                   imu_msg_2_.linear_acceleration.y);
  imu_msg_2_.linear_acceleration.z = lowPassFilter(filter_alpha, result_["acc_z"] * G_ACC,
                                                   imu_msg_2_.linear_acceleration.z);

  Eigen::Quaterniond orientation = robotis_framework::convertRPYToQuaternion(roll, pitch, yaw);

  imu_msg_2_.orientation.x = orientation.x();
  imu_msg_2_.orientation.y = orientation.y();
  imu_msg_2_.orientation.z = orientation.z();
  imu_msg_2_.orientation.w = orientation.w();

  imu_pub_2_.publish(imu_msg_);
}

void OpenCRModule::pushedModeButton(bool pushed)
{
  if (button_mode_ == pushed)
    return;

  button_mode_ = pushed;

  if (pushed == true)
  {
    button_press_time_ = ros::Time::now();
  }
  else
  {
    ros::Duration button_duration = ros::Time::now() - button_press_time_;
    if (button_duration.sec < 2)     // short press
      publishButtonMsg("mode");
    else
      // long press
      publishButtonMsg("mode_long");
  }
}

void OpenCRModule::pushedStartButton(bool pushed)
{
  if (button_start_ == pushed)
    return;

  button_start_ = pushed;

  if (pushed == true)
  {
    button_press_time_ = ros::Time::now();
  }
  else
  {
    ros::Duration button_duration = ros::Time::now() - button_press_time_;

    if (button_duration.sec < 2)     // short press
      publishButtonMsg("start");
    else
      // long press
      publishButtonMsg("start_long");
  }
}

void OpenCRModule::handleButton(const std::string &button_name)
{
  std::string button_key = "button_" + button_name;

  bool pushed = (result_[button_key] == 1.0);
  // same state
  if (buttons_[button_key] == pushed)
    return;

  buttons_[button_key] = pushed;

  if (pushed == true)
  {
    buttons_press_time_[button_name] = ros::Time::now();
  }
  else
  {
    ros::Duration button_duration = ros::Time::now() - buttons_press_time_[button_name];

    // for debug
    if(button_duration.toSec() < 0.01)
      return;

    if (button_duration.toSec() < 2)     // short press
      publishButtonMsg(button_name);
    else
      // long press
      publishButtonMsg(button_name + "_long");

    // for debug
    std::cout << "Button Pressed : " << button_name << ", time : " << button_duration.toSec() << std::endl;
  }
}

void OpenCRModule::publishButtonMsg(const std::string &button_name)
{
  std_msgs::String button_msg;
  button_msg.data = button_name;

  reset_dxl_pub_.publish(button_msg);
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Button : " + button_name);
}

void OpenCRModule::handleVoltage(double present_volt)
{
  double voltage_ratio = 0.4;
  previous_volt_ =
      (previous_volt_ != 0) ? previous_volt_ * (1 - voltage_ratio) + present_volt * voltage_ratio : present_volt;

  if (fabs(present_volt_ - previous_volt_) >= 0.1)
  {
    // check last publised time
    ros::Time now = ros::Time::now();
    ros::Duration dur = now - last_msg_time_;
    if (dur.sec < 1)
      return;

    last_msg_time_ = now;

    present_volt_ = previous_volt_;
    std::stringstream log_stream;
    log_stream << "Present Volt : " << present_volt_ << "V";
    publishStatusMsg(
        (present_volt_ < 11 ?
            robotis_controller_msgs::StatusMsg::STATUS_WARN : robotis_controller_msgs::StatusMsg::STATUS_INFO),
        log_stream.str());
    ROS_INFO_COND(debug_print_, "Present Volt : %fV, Read Volt : %fV", previous_volt_, result_["present_voltage"]);
    // ROS_INFO_THROTTLE(0.1, "Present Volt : %fV, Read Volt : %fV", previous_volt_, result_["present_voltage"]);
  }
}

void OpenCRModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "SENSOR";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

double OpenCRModule::lowPassFilter(double alpha, double x_new, double x_old)
{
  return alpha * x_new + (1.0 - alpha) * x_old;
}

}
