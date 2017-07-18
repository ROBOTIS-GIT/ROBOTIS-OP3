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

#ifndef OP3_OPEN_CR_MODULE_H_
#define OP3_OPEN_CR_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include "robotis_math/robotis_math_base.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "robotis_framework_common/sensor_module.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

namespace robotis_op
{

class OpenCRModule : public robotis_framework::SensorModule, public robotis_framework::Singleton<OpenCRModule>
{
 public:
  OpenCRModule();
  virtual ~OpenCRModule();

  /* ROS Topic Callback Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
               std::map<std::string, robotis_framework::Sensor *> sensors);

 private:
  const double G_ACC = 9.80665;
  const double GYRO_FACTOR = 2000.0 / 32800.0;
  const double ACCEL_FACTOR = 2.0 / 32768.0;
  const bool DEBUG_PRINT;

  void queueThread();

  double getGyroValue(int raw_value);
  double getAccValue(int raw_value);
  void publishIMU();

  void handleButton(const std::string &button_name);
  void publishButtonMsg(const std::string &button_name);
  void handleVoltage(double present_volt);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDXLPowerMsg(unsigned int value);
  double lowPassFilter(double alpha, double x_new, double &x_old);

  int control_cycle_msec_;
  boost::thread queue_thread_;
  std::map<std::string, bool> buttons_;
  std::map<std::string, ros::Time> buttons_press_time_;
  ros::Time button_press_time_;
  ros::Time last_msg_time_;
  std::map<std::string, double> previous_result_;
  double previous_volt_;
  double present_volt_;

  sensor_msgs::Imu imu_msg_;

  /* subscriber & publisher */
  ros::Publisher imu_pub_;
  ros::Publisher button_pub_;
  ros::Publisher status_msg_pub_;
  ros::Publisher dxl_power_msg_pub_;
};

}

#endif /* OP3_OPEN_CR_MODULE_H_ */
