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

/* Author: SCH */

#ifndef OP3_ONLINE_WALKING_MODULE_WALKING_CONTROL_
#define OP3_ONLINE_WALKING_MODULE_WALKING_CONTROL_

#pragma once

#include <math.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>
#include <geometry_msgs/Pose2D.h>
#include <eigen3/Eigen/Eigen>
#include "op3_online_walking_module_msgs/FootStepCommand.h"
#include "op3_online_walking_module_msgs/FootStepArray.h"
#include "op3_online_walking_module_msgs/PreviewResponse.h"
#include "op3_online_walking_module_msgs/Step2D.h"
#include "op3_online_walking_module_msgs/Step2DArray.h"
//#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"
#include "robotis_math/robotis_math.h"

enum WALKING_LEG {
  LEFT_LEG = 0,
  RIGHT_LEG = 1,
  LEG_COUNT = 2
};

enum WALKING_PHASE {
  DSP = 0, // Double Support Phase
  SSP = 1, // Single Support Phase
  PHASE_COUNT = 2
};

class WalkingControl
{
public:
  WalkingControl(double control_cycle,
                 double dsp_ratio, double lipm_height, double foot_height_max, double zmp_offset_x, double zmp_offset_y,
                 std::vector<double_t> x_lipm, std::vector<double_t> y_lipm,
                 double foot_distance);
  virtual ~WalkingControl();

  void initialize(op3_online_walking_module_msgs::FootStepCommand foot_step_command,
                  std::vector<double_t> init_body_pos, std::vector<double_t> init_body_Q,
                  std::vector<double_t> init_r_foot_pos, std::vector<double_t> init_r_foot_Q,
                  std::vector<double_t> init_l_foot_pos, std::vector<double_t> init_l_foot_Q);
  void initialize(op3_online_walking_module_msgs::Step2DArray foot_step_2d,
                  std::vector<double_t> init_body_pos, std::vector<double_t> init_body_Q,
                  std::vector<double_t> init_r_foot_pos, std::vector<double_t> init_r_foot_Q,
                  std::vector<double_t> init_l_foot_pos, std::vector<double_t> init_l_foot_Q);
  void next();
  void finalize();
  void set(double time, int step, bool foot_step_2d);

  double getLipmHeight();

  void calcFootStepParam();

  void transformFootStep2D();

  void calcFootTrajectory(int step);
  void calcFootStepPose(double time,  int step);
  void calcRefZMP(int step);
  void calcPreviewParam(std::vector<double_t> K, int K_row, int K_col,
                        std::vector<double_t> P, int P_row, int P_col);
  void calcPreviewControl(double time, int step);

  void calcGoalFootPose();

  double calcRefZMPx(int step);
  double calcRefZMPy(int step);

  void getWalkingPosition(std::vector<double_t> &l_foot_pos,
                          std::vector<double_t> &r_foot_pos,
                          std::vector<double_t> &body_pos);
  void getWalkingVelocity(std::vector<double_t> &l_foot_vel,
                          std::vector<double_t> &r_foot_vel,
                          std::vector<double_t> &body_vel);
  void getWalkingAccleration(std::vector<double_t> &l_foot_accel,
                             std::vector<double_t> &r_foot_accel,
                             std::vector<double_t> &body_accel);
  void getWalkingOrientation(std::vector<double_t> &l_foot_Q,
                             std::vector<double_t> &r_foot_Q,
                             std::vector<double_t> &body_Q);
  void getLIPM(std::vector<double_t> &x_lipm, std::vector<double_t> &y_lipm);
  void getWalkingState(int &walking_leg, int &walking_phase);

protected:
//  thormang3::KinematicsDynamics *robot_;

  robotis_framework::MinimumJerk *body_trajectory_;
  robotis_framework::MinimumJerkViaPoint *r_foot_tra_;
  robotis_framework::MinimumJerkViaPoint *l_foot_tra_;

  robotis_framework::PreviewControl *preview_control_;

  double init_time_, fin_time_;
  double control_cycle_;

  int walking_leg_;
  int walking_phase_;

  // Foot Trajectory
  double foot_size_x_;
  double foot_size_y_;
  double foot_origin_shift_x_;
  double foot_origin_shift_y_;

  double dsp_ratio_;
  double foot_tra_max_z_;

  int foot_step_size_;
  op3_online_walking_module_msgs::FootStepCommand foot_step_command_;
  op3_online_walking_module_msgs::FootStepArray foot_step_param_;
  op3_online_walking_module_msgs::PreviewResponse preview_response_;

  op3_online_walking_module_msgs::Step2DArray foot_step_2d_;

  // Preview Control
  int preview_size_;
  double preview_time_;
  double lipm_height_;
  double sum_of_zmp_x_, sum_of_zmp_y_ ;
  double sum_of_cx_, sum_of_cy_ ;
  Eigen::MatrixXd A_, b_, c_;
  Eigen::MatrixXd k_x_;
  double k_s_;
  Eigen::MatrixXd f_;
  Eigen::MatrixXd u_x_, u_y_;
  Eigen::MatrixXd x_lipm_, y_lipm_;

  Eigen::MatrixXd K_, P_;

  double ref_zmp_x_, ref_zmp_y_;
  double preview_sum_zmp_x_, preview_sum_zmp_y_;
  double zmp_offset_x_, zmp_offset_y_;

  Eigen::MatrixXd goal_r_foot_pos_buffer_, goal_l_foot_pos_buffer_;
  Eigen::MatrixXd ref_zmp_buffer_;

  // Pose Information
  double init_body_yaw_angle_;

  std::vector<double_t> init_body_pos_, init_body_vel_, init_body_accel_;
  std::vector<double_t> des_body_pos_, des_body_vel_, des_body_accel_;
  std::vector<double_t> goal_body_pos_, goal_body_vel_, goal_body_accel_;
  Eigen::Quaterniond    init_body_Q_, des_body_Q_, goal_body_Q_;

  std::vector<double_t> init_l_foot_pos_, init_l_foot_vel_, init_l_foot_accel_;
  std::vector<double_t> des_l_foot_pos_, des_l_foot_vel_, des_l_foot_accel_;
  std::vector<double_t> goal_l_foot_pos_, goal_l_foot_vel_, goal_l_foot_accel_;
  Eigen::Quaterniond    init_l_foot_Q_, des_l_foot_Q_, goal_l_foot_Q_;

  std::vector<double_t> init_r_foot_pos_, init_r_foot_vel_, init_r_foot_accel_;
  std::vector<double_t> des_r_foot_pos_, des_r_foot_vel_, des_r_foot_accel_;
  std::vector<double_t> goal_r_foot_pos_, goal_r_foot_vel_, goal_r_foot_accel_;
  Eigen::Quaterniond    init_r_foot_Q_, des_r_foot_Q_, goal_r_foot_Q_;
};

#endif
