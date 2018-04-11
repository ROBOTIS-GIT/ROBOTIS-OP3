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

#ifndef HEAD_CONTROL_MODULE_H_
#define HEAD_CONTROL_MODULE_H_

#include <cstdlib>
#include <ctime>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

namespace robotis_op
{

class HeadControlModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<HeadControlModule>
{
 public:
  HeadControlModule();
  virtual ~HeadControlModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  void onModuleEnable();
  void onModuleDisable();

 private:
  enum
  {
    NoScan = 0,
    TopLeft = 1,
    BottomRight = 2,
    BottomLeft = 3,
    TopRight = 4,
  };

  /* ROS Topic Callback Functions */
  void setHeadJointCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void setHeadJointOffsetCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void setHeadScanCallback(const std_msgs::String::ConstPtr &msg);

  void queueThread();
  void jointTraGeneThread();
  void setHeadJoint(const sensor_msgs::JointState::ConstPtr &msg, bool is_offset);
  bool checkAngleLimit(const int joint_index, double &goal_position);
  void generateScanTra(const int head_direction);

  void startMoving();
  void finishMoving();
  void stopMoving();

  void publishStatusMsg(unsigned int type, std::string msg);

  Eigen::MatrixXd calcMinimumJerkTraPVA(double pos_start, double vel_start, double accel_start, double pos_end,
                                        double vel_end, double accel_end, double smp_time, double mov_time);

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::thread *tra_gene_thread_;
  boost::mutex tra_lock_;
  ros::Publisher status_msg_pub_;
  const bool DEBUG;
  bool stop_process_;
  bool is_moving_;
  bool is_direct_control_;
  int tra_count_, tra_size_;
  double moving_time_;
  int scan_state_;
  bool has_goal_position_;
  double angle_unit_;

  Eigen::MatrixXd target_position_;
  Eigen::MatrixXd current_position_;
  Eigen::MatrixXd goal_position_;
  Eigen::MatrixXd goal_velocity_;
  Eigen::MatrixXd goal_acceleration_;
  Eigen::MatrixXd calc_joint_tra_;
  Eigen::MatrixXd calc_joint_vel_tra_;
  Eigen::MatrixXd calc_joint_accel_tra_;

  std::map<std::string, int> using_joint_name_;
  std::map<int, double> max_angle_;
  std::map<int, double> min_angle_;

  ros::Time last_msg_time_;
  std::string last_msg_;
};

}

#endif /* HEAD_CONTROL_MODULE_H_ */
