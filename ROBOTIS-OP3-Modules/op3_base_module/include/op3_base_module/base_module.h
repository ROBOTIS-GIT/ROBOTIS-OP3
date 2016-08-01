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

#ifndef BASEMODULE_H_
#define BASEMODULE_H_

#include <map>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_math/robotis_math.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

#include "robotis_state.h"

namespace ROBOTIS
{

class BaseJointData
{

 public:
  double position;
  double velocity;
  double effort;

  int p_gain;
  int i_gain;
  int d_gain;

};

class BaseJointState
{

 public:
  BaseJointData curr_joint_state[ MAX_JOINT_ID + 1 ];
  BaseJointData goal_joint_state[ MAX_JOINT_ID + 1 ];
  BaseJointData fake_joint_state[ MAX_JOINT_ID + 1 ];

};

class BaseModule : public MotionModule, public Singleton<BaseModule>
{    
 public:
  BaseModule();
  virtual ~BaseModule();

  /* ROS Framework Functions */
  void Initialize(const int control_cycle_msec, Robot *robot);
  void Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors);

  void Stop();
  bool IsRunning();

  void OnModuleEnable();
  void OnModuleDisable();

  /* ROS Topic Callback Functions */
  void iniPoseMsgCallback( const std_msgs::String::ConstPtr& msg );

  /* ROS Calculation Functions */
  void iniposeTraGeneProc();

  void poseGenProc(Eigen::MatrixXd _joint_angle_pose);
  void poseGenProc(std::map<std::string, double>& joint_angle_pose);

  /* Parameter */
  ROBOTIS_BASE::RobotisState *Robotis;
  BaseJointState *JointState;

 private:
  void queueThread();
  void setCtrlModule(std::string module);
  void parseIniPoseData(const std::string &path);
  void publishStatusMsg(unsigned int type, std::string msg);

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::thread tra_gene_tread_;

  ros::Publisher status_msg_pub_;
  ros::Publisher set_ctrl_module_pub_;

  std::map<std::string, int> joint_name_to_id_;

  bool has_goal_joints_;
  bool ini_pose_only_;
};

}


#endif /* BASEMODULE_H_ */
