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

#ifndef OP3_WALKING_MODULE_INCLUDE_OP3_WALKING_MODULE_OP3_WALKING_MODULE_H_
#define OP3_WALKING_MODULE_INCLUDE_OP3_WALKING_MODULE_OP3_WALKING_MODULE_H_

#include "op3_walking_parameter.h"

#include <stdio.h>
#include <math.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <boost/thread.hpp>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <eigen_conversions/eigen_msg.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "robotis_math/robotis_trajectory_calculator.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/SetWalkingParam.h"

namespace ROBOTIS
{

typedef struct
{
  double x, y, z;
} Position3D;

typedef struct
{
  double x, y, z, roll, pitch, yaw;
} Pose3D;

class WalkingMotionModule : public MotionModule, public Singleton<WalkingMotionModule>
{

 public:
  enum
  {
    PHASE0 = 0,
    PHASE1 = 1,
    PHASE2 = 2,
    PHASE3 = 3
  };

  WalkingMotionModule();
  virtual ~WalkingMotionModule();

  void Initialize(const int control_cycle_msec, Robot *robot);
  void Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors);
  void Stop();
  bool IsRunning();
  void OnModuleEnable();
  void OnModuleDisable();

  int getCurrentPhase()
  {
    return m_Phase;
  }
  double getBodySwingY()
  {
    return m_Body_Swing_Y;
  }
  double getBodySwingZ()
  {
    return m_Body_Swing_Z;
  }

 private:
  enum
  {
    WalkingDisable = 0,
    WalkingEnable = 1,
    WalkingInitPose = 2,
    WalkingReady = 3
  };

  void queueThread();

  /* ROS Topic Callback Functions */
  void walkingCommandCallback(const std_msgs::String::ConstPtr &msg);
  void walkingParameterCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr &msg);
  bool getWalkigParameterCallback(op3_walking_module_msgs::GetWalkingParam::Request &req,
                                  op3_walking_module_msgs::GetWalkingParam::Response &res);

  /* ROS Service Callback Functions */
  void processPhase(const double &time_unit);
  bool computeLegAngle(double *leg_angle);
  void computeArmAngle(double *arm_angle);
  void sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle);

  void publishStatusMsg(unsigned int type, std::string msg);
  double wSin(double time, double period, double period_shift, double mag, double mag_shift);
  bool computeIK(double *out, double x, double y, double z, double a, double b, double c);
  void updateTimeParam();
  void updateMovementParam();
  void updatePoseParam();
  void startWalking();
  void loadWalkingParam(const std::string &path);
  void saveWalkingParam(std::string &path);
  void iniPoseTraGene(double mov_time);

  OP3KinematicsDynamics* op3_kd_;
  int control_cycle_msec_;
  std::string param_path_;
  boost::thread queue_thread_;
  boost::mutex publish_mutex_;

  /* ROS Topic Publish Functions */
  ros::Publisher robot_pose_pub_;
  ros::Publisher status_msg_pub_;

  Eigen::MatrixXd calc_joint_tra_;

  Eigen::MatrixXd target_position_;
  Eigen::MatrixXd goal_position_;
  Eigen::MatrixXd init_position_;
  Eigen::MatrixXi joint_axis_direction_;
  std::map<std::string, int> joint_table_;
  int walking_state_;
  bool DEBUG_;
  int init_count_;
  op3_walking_module_msgs::WalkingParam walking_param_;
  double previous_X_Move_Amplitude;

  // variable for walking
  double m_PeriodTime;
  double m_DSP_Ratio;
  double m_SSP_Ratio;
  double m_X_Swap_PeriodTime;
  double m_X_Move_PeriodTime;
  double m_Y_Swap_PeriodTime;
  double m_Y_Move_PeriodTime;
  double m_Z_Swap_PeriodTime;
  double m_Z_Move_PeriodTime;
  double m_A_Move_PeriodTime;
  double m_SSP_Time;
  double m_SSP_Time_Start_L;
  double m_SSP_Time_End_L;
  double m_SSP_Time_Start_R;
  double m_SSP_Time_End_R;
  double m_Phase_Time1;
  double m_Phase_Time2;
  double m_Phase_Time3;

  double m_X_Offset;
  double m_Y_Offset;
  double m_Z_Offset;
  double m_R_Offset;
  double m_P_Offset;
  double m_A_Offset;

  double m_X_Swap_Phase_Shift;
  double m_X_Swap_Amplitude;
  double m_X_Swap_Amplitude_Shift;
  double m_X_Move_Phase_Shift;
  double m_X_Move_Amplitude;
  double m_X_Move_Amplitude_Shift;
  double m_Y_Swap_Phase_Shift;
  double m_Y_Swap_Amplitude;
  double m_Y_Swap_Amplitude_Shift;
  double m_Y_Move_Phase_Shift;
  double m_Y_Move_Amplitude;
  double m_Y_Move_Amplitude_Shift;
  double m_Z_Swap_Phase_Shift;
  double m_Z_Swap_Amplitude;
  double m_Z_Swap_Amplitude_Shift;
  double m_Z_Move_Phase_Shift;
  double m_Z_Move_Amplitude;
  double m_Z_Move_Amplitude_Shift;
  double m_A_Move_Phase_Shift;
  double m_A_Move_Amplitude;
  double m_A_Move_Amplitude_Shift;

  double m_Pelvis_Offset;
  double m_Pelvis_Swing;
  double m_Hip_Pitch_Offset;
  double m_Arm_Swing_Gain;

  bool m_Ctrl_Running;
  bool m_Real_Running;
  double m_Time;

  int m_Phase;
  double m_Body_Swing_Y;
  double m_Body_Swing_Z;
};

}

#endif /* OP3_WALKING_MODULE_INCLUDE_OP3_WALKING_MODULE_OP3_WALKING_MODULE_H_ */
