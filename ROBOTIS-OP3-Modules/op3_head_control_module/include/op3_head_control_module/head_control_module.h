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

#ifndef HEAD_CONTROL_MODULE_H_
#define HEAD_CONTROL_MODULE_H_

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include "robotis_framework_common/MotionModule.h"
#include "robotis_math/RobotisMath.h"
#include "robotis_controller_msgs/StatusMsg.h"

namespace ROBOTIS
{

class HeadControlModule : public MotionModule, public Singleton<
HeadControlModule>
{
 public:
  HeadControlModule();
  virtual ~HeadControlModule();

  void Initialize(const int control_cycle_msec, Robot *robot);
  void Process(std::map<std::string, Dynamixel *> dxls,
               std::map<std::string, double> sensors);

  void Stop();
  bool IsRunning();

 private:
  enum
  {
    NoScan = 0,
    BottomToTop = 1,
    RightToLeft = 2,
    TopToBottom = 3,
    LeftToRight = 4,
  };

  /* ROS Topic Callback Functions */
  void SetHeadJointCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void SetHeadJointOffsetCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void setHeadScanCallback(const std_msgs::String::ConstPtr &msg);

  void QueueThread();
  void JointTraGeneThread();
  void setHeadJoint(const sensor_msgs::JointState::ConstPtr &msg, bool is_offset);
  bool checkAngleLimit(const int joint_index, double &goal_position);
  void generateScanTra(const int head_direction);

  void StartMoving();
  void FinishMoving();
  void StopMoving();

  void PublishStatusMsg(unsigned int type, std::string msg);

  Eigen::MatrixXd MinimumJerkTraPVA(double pos_start, double vel_start,
                                    double accel_start, double pos_end,
                                    double vel_end, double accel_end,
                                    double smp_time, double mov_time);

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
};

}

#endif /* HEAD_CONTROL_MODULE_H_ */
