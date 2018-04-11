/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

#include "op3_balance_control/op3_balance_control.h"

#include <iostream>

using namespace robotis_op;

DampingController::DampingController()
{
  desired_ = 0.0;

  gain_ = 0.0;
  time_constant_sec_ = 1.0;
  output_ = 0.0;
  control_cycle_sec_ = 0.008;

  previous_result_ = 0.0;
}

DampingController::DampingController(double time_unit_sec)
{
  desired_ = 0.0;

  gain_ = 0.0;
  time_constant_sec_ = 1.0;
  output_ = 0.0;
  control_cycle_sec_ = time_unit_sec;

  previous_result_ = 0.0;
}

DampingController::~DampingController()
{  }

double DampingController::getDampingControllerOutput(double present_sensor_output)
{
  double cut_off_freq = 1.0/time_constant_sec_;
  double alpha = 1.0;
  alpha = (2.0*M_PI*cut_off_freq*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq*control_cycle_sec_);

  previous_result_ = alpha*(desired_ - present_sensor_output) + (1.0 - alpha)*previous_result_;
  output_ = gain_*previous_result_;

  return output_;
}


BalanceLowPassFilter::BalanceLowPassFilter()
{
  cut_off_freq_ = 1.0;
  control_cycle_sec_ = 0.008;
  prev_output_ = 0;

  alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
}

BalanceLowPassFilter::BalanceLowPassFilter(double control_cycle_sec, double cut_off_frequency)
{
  cut_off_freq_ = cut_off_frequency;
  control_cycle_sec_ = control_cycle_sec;
  prev_output_ = 0;

  if(cut_off_frequency > 0)
    alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
  else
    alpha_ = 1;
}

BalanceLowPassFilter::~BalanceLowPassFilter()
{ }

void BalanceLowPassFilter::initialize(double control_cycle_sec, double cut_off_frequency)
{
  cut_off_freq_ = cut_off_frequency;
  control_cycle_sec_ = control_cycle_sec;
  prev_output_ = 0;

  if(cut_off_frequency > 0)
    alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
  else
    alpha_ = 1;
}

void BalanceLowPassFilter::setCutOffFrequency(double cut_off_frequency)
{
  cut_off_freq_ = cut_off_frequency;

  if(cut_off_frequency > 0)
    alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
  else
    alpha_ = 1;
}

double BalanceLowPassFilter::getCutOffFrequency(void)
{
  return cut_off_freq_;
}

double BalanceLowPassFilter::getFilteredOutput(double present_raw_value)
{
  prev_output_ = alpha_*present_raw_value + (1.0 - alpha_)*prev_output_;
  return prev_output_;
}


BalancePDController::BalancePDController()
{
  desired_ = 0;
  p_gain_ = 0;
  d_gain_ = 0;
  curr_err_ = 0;
  prev_err_ = 0;
}

BalancePDController::~BalancePDController()
{ }

double BalancePDController::getFeedBack(double present_sensor_output)
{
  prev_err_ = curr_err_;
  curr_err_ = desired_ - present_sensor_output;

  return (p_gain_*curr_err_ + d_gain_*(curr_err_ - prev_err_));
}


BalanceControlUsingDampingConroller::BalanceControlUsingDampingConroller()
{
  balance_control_error_ = BalanceControlError::NoError;
  control_cycle_sec_ = 0.008;

  // balance enable
  gyro_enable_ = 1.0;
  orientation_enable_ = 1.0;
  ft_enable_ = 1.0;

  desired_robot_to_cob_         = Eigen::MatrixXd::Identity(4,4);
  desired_robot_to_right_foot_  = Eigen::MatrixXd::Identity(4,4);
  desired_robot_to_left_foot_   = Eigen::MatrixXd::Identity(4,4);

  gyro_cut_off_freq_  = 10.0;
  gyro_lpf_alpha_     =  2.0*M_PI*gyro_cut_off_freq_*control_cycle_sec_/(1.0 + 2.0*M_PI*gyro_cut_off_freq_*control_cycle_sec_);
  gyro_roll_filtered_ = gyro_pitch_filtered_ = 0;
  desired_gyro_roll_  = desired_gyro_pitch_ = 0;

  //sensed values
  current_gyro_roll_rad_per_sec_ = current_gyro_pitch_rad_per_sec_ = 0;

  current_orientation_roll_rad_ = current_orientation_pitch_rad_ = 0;

  current_right_fx_N_  = current_right_fy_N_  = current_right_fz_N_  = 0;
  current_right_tx_Nm_ = current_right_ty_Nm_ = current_right_tz_Nm_ = 0;
  current_left_fx_N_   = current_left_fy_N_   = current_left_fz_N_   = 0;
  current_left_tx_Nm_  = current_left_ty_Nm_  = current_left_tz_Nm_  = 0;

  // balance algorithm result
  foot_roll_adjustment_by_gyro_roll_ = 0;
  foot_pitch_adjustment_by_gyro_pitch_ = 0;

  foot_roll_adjustment_by_orientation_roll_ = 0;
  foot_pitch_adjustment_by_orientation_pitch_ = 0;

  foot_z_adjustment_by_force_z_difference_ = 0;
  r_foot_z_adjustment_by_force_z_ = 0;
  l_foot_z_adjustment_by_force_z_ = 0;

  r_foot_x_adjustment_by_force_x_ = 0;
  r_foot_y_adjustment_by_force_y_ = 0;
  r_foot_roll_adjustment_by_torque_roll_ = 0;
  r_foot_pitch_adjustment_by_torque_pitch_ = 0;

  l_foot_x_adjustment_by_force_x_ = 0;
  l_foot_y_adjustment_by_force_y_ = 0;
  l_foot_roll_adjustment_by_torque_roll_ = 0;
  l_foot_pitch_adjustment_by_torque_pitch_ = 0;

  // manual cob adjustment
  cob_x_manual_adjustment_m_ = 0;
  cob_y_manual_adjustment_m_ = 0;
  cob_z_manual_adjustment_m_ = 0;

  // gyro gain
  gyro_balance_gain_ratio_ = 0.0;
  gyro_balance_roll_gain_  = -0.10*0.75*gyro_balance_gain_ratio_;
  gyro_balance_pitch_gain_ = -0.10*0.5 *gyro_balance_gain_ratio_;


  // maximum adjustment
  cob_x_adjustment_abs_max_m_ = 0.05;
  cob_y_adjustment_abs_max_m_ = 0.05;
  cob_z_adjustment_abs_max_m_ = 0.05;
  cob_roll_adjustment_abs_max_rad_  = 15.0*DEGREE2RADIAN;
  cob_pitch_adjustment_abs_max_rad_ = 15.0*DEGREE2RADIAN;
  cob_yaw_adjustment_abs_max_rad_   = 15.0*DEGREE2RADIAN;
  foot_x_adjustment_abs_max_m_ = 0.05;
  foot_y_adjustment_abs_max_m_ = 0.05;
  foot_z_adjustment_abs_max_m_ = 0.05;
  foot_roll_adjustment_abs_max_rad_  = 15.0*DEGREE2RADIAN;
  foot_pitch_adjustment_abs_max_rad_ = 15.0*DEGREE2RADIAN;
  foot_yaw_adjustment_abs_max_rad_   = 15.0*DEGREE2RADIAN;

  mat_robot_to_cob_modified_        = Eigen::MatrixXd::Identity(4,4);
  mat_robot_to_right_foot_modified_ = Eigen::MatrixXd::Identity(4,4);
  mat_robot_to_left_foot_modified_  = Eigen::MatrixXd::Identity(4,4);
  pose_cob_adjustment_         = Eigen::VectorXd::Zero(6);
  pose_right_foot_adjustment_  = Eigen::VectorXd::Zero(6);;
  pose_left_foot_adjustment_   = Eigen::VectorXd::Zero(6);;
}

BalanceControlUsingDampingConroller::~BalanceControlUsingDampingConroller()
{  }

void BalanceControlUsingDampingConroller::initialize(const int control_cycle_msec)
{
  balance_control_error_ = BalanceControlError::NoError;

  control_cycle_sec_ = control_cycle_msec * 0.001;

  gyro_cut_off_freq_ = 10.0;
  gyro_lpf_alpha_ = 2.0*M_PI*gyro_cut_off_freq_*control_cycle_sec_/(1.0 + 2.0*M_PI*gyro_cut_off_freq_*control_cycle_sec_);

  pose_cob_adjustment_.fill(0);
  pose_right_foot_adjustment_.fill(0);
  pose_left_foot_adjustment_.fill(0);

  foot_roll_angle_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  foot_pitch_angle_ctrl_.control_cycle_sec_ = control_cycle_sec_;

  foot_force_z_diff_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  right_foot_force_z_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  left_foot_force_z_ctrl_.control_cycle_sec_ = control_cycle_sec_;

  right_foot_force_x_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  right_foot_force_y_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  right_foot_torque_roll_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  right_foot_torque_pitch_ctrl_.control_cycle_sec_ = control_cycle_sec_;

  left_foot_force_x_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  left_foot_force_y_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  left_foot_torque_roll_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  left_foot_torque_pitch_ctrl_.control_cycle_sec_ = control_cycle_sec_;
}

void BalanceControlUsingDampingConroller::setGyroBalanceEnable(bool enable)
{
  if(enable)
    gyro_enable_ = 1.0;
  else
    gyro_enable_ = 0.0;
}

void BalanceControlUsingDampingConroller::setOrientationBalanceEnable(bool enable)
{
  if(enable)
    orientation_enable_ = 1.0;
  else
    orientation_enable_ = 0.0;
}

void BalanceControlUsingDampingConroller::setForceTorqueBalanceEnable(bool enable)
{
  if(enable)
    ft_enable_ = 1.0;
  else
    ft_enable_ = 0.0;
}

void BalanceControlUsingDampingConroller::process(int *balance_error, Eigen::MatrixXd *robot_to_cob_modified, Eigen::MatrixXd *robot_to_right_foot_modified, Eigen::MatrixXd *robot_to_left_foot_modified)
{
  balance_control_error_ = BalanceControlError::NoError;

  pose_cob_adjustment_.fill(0);
  pose_right_foot_adjustment_.fill(0);
  pose_left_foot_adjustment_.fill(0);

  // gyro
  gyro_roll_filtered_  = current_gyro_roll_rad_per_sec_*gyro_lpf_alpha_  + (1.0 - gyro_lpf_alpha_)*gyro_roll_filtered_;
  gyro_pitch_filtered_ = current_gyro_pitch_rad_per_sec_*gyro_lpf_alpha_ + (1.0 - gyro_lpf_alpha_)*gyro_pitch_filtered_;

  foot_roll_adjustment_by_gyro_roll_   = gyro_enable_ * (desired_gyro_roll_  - gyro_roll_filtered_)  * gyro_balance_roll_gain_;
  foot_pitch_adjustment_by_gyro_pitch_ = gyro_enable_ * (desired_gyro_pitch_ - gyro_pitch_filtered_) * gyro_balance_pitch_gain_;

  // z by imu
  foot_roll_adjustment_by_orientation_roll_   = orientation_enable_ * foot_roll_angle_ctrl_.getDampingControllerOutput(current_orientation_roll_rad_);
  foot_pitch_adjustment_by_orientation_pitch_ = orientation_enable_ * foot_pitch_angle_ctrl_.getDampingControllerOutput(current_orientation_pitch_rad_);

  Eigen::MatrixXd mat_orientation_adjustment_by_imu = robotis_framework::getRotation4d(foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_, foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_, 0.0);
  Eigen::MatrixXd mat_r_xy, mat_l_xy;
  mat_r_xy.resize(4,1); mat_l_xy.resize(4,1);
  mat_r_xy.coeffRef(0,0) = desired_robot_to_right_foot_.coeff(0,3) - 0.5*(desired_robot_to_right_foot_.coeff(0,3) + desired_robot_to_left_foot_.coeff(0,3));
  mat_r_xy.coeffRef(1,0) = desired_robot_to_right_foot_.coeff(1,3) - 0.5*(desired_robot_to_right_foot_.coeff(1,3) + desired_robot_to_left_foot_.coeff(1,3));
  mat_r_xy.coeffRef(2,0) = 0.0;
  mat_r_xy.coeffRef(3,0) = 1;

  mat_l_xy.coeffRef(0,0) = desired_robot_to_left_foot_.coeff(0,3) - 0.5*(desired_robot_to_right_foot_.coeff(0,3) + desired_robot_to_left_foot_.coeff(0,3));
  mat_l_xy.coeffRef(1,0) = desired_robot_to_left_foot_.coeff(1,3) - 0.5*(desired_robot_to_right_foot_.coeff(1,3) + desired_robot_to_left_foot_.coeff(1,3));
  mat_l_xy.coeffRef(2,0) = 0.0;
  mat_l_xy.coeffRef(3,0) = 1;

  mat_r_xy = mat_orientation_adjustment_by_imu * mat_r_xy;
  mat_l_xy = mat_orientation_adjustment_by_imu * mat_l_xy;

  // ft sensor
  foot_z_adjustment_by_force_z_difference_ = ft_enable_*0.001*foot_force_z_diff_ctrl_.getDampingControllerOutput(current_left_fz_N_ - current_right_fz_N_);
  r_foot_z_adjustment_by_force_z_ = ft_enable_*0.001*right_foot_force_z_ctrl_.getDampingControllerOutput(current_right_fz_N_);
  l_foot_z_adjustment_by_force_z_ = ft_enable_*0.001*left_foot_force_z_ctrl_.getDampingControllerOutput(current_left_fz_N_);

  r_foot_x_adjustment_by_force_x_ = ft_enable_*0.001*right_foot_force_x_ctrl_.getDampingControllerOutput(current_right_fx_N_);
  r_foot_y_adjustment_by_force_y_ = ft_enable_*0.001*right_foot_force_y_ctrl_.getDampingControllerOutput(current_right_fy_N_);
  r_foot_roll_adjustment_by_torque_roll_   = ft_enable_*right_foot_torque_roll_ctrl_.getDampingControllerOutput(current_right_tx_Nm_);
  r_foot_pitch_adjustment_by_torque_pitch_ = ft_enable_*right_foot_torque_pitch_ctrl_.getDampingControllerOutput(current_right_ty_Nm_);

  l_foot_x_adjustment_by_force_x_ = ft_enable_*0.001*left_foot_force_x_ctrl_.getDampingControllerOutput(current_left_fx_N_);
  l_foot_y_adjustment_by_force_y_ = ft_enable_*0.001*left_foot_force_y_ctrl_.getDampingControllerOutput(current_left_fy_N_);
  l_foot_roll_adjustment_by_torque_roll_   = ft_enable_*left_foot_torque_roll_ctrl_.getDampingControllerOutput(current_left_tx_Nm_);
  l_foot_pitch_adjustment_by_torque_pitch_ = ft_enable_*left_foot_torque_pitch_ctrl_.getDampingControllerOutput(current_left_ty_Nm_);

  r_foot_z_adjustment_by_force_z_ += ft_enable_*0.001*0.0*(right_foot_force_z_ctrl_.desired_ - current_right_fz_N_);
  l_foot_z_adjustment_by_force_z_ += ft_enable_*0.001*0.0*(left_foot_force_z_ctrl_.desired_ - current_left_fz_N_);
  // sum of sensory balance result
  pose_cob_adjustment_.coeffRef(0) = cob_x_manual_adjustment_m_;
  pose_cob_adjustment_.coeffRef(1) = cob_y_manual_adjustment_m_;
  pose_cob_adjustment_.coeffRef(2) = cob_z_manual_adjustment_m_;

  pose_right_foot_adjustment_.coeffRef(0) = r_foot_x_adjustment_by_force_x_;
  pose_right_foot_adjustment_.coeffRef(1) = r_foot_y_adjustment_by_force_y_;
  pose_right_foot_adjustment_.coeffRef(2) = 0.5*0.0*foot_z_adjustment_by_force_z_difference_ + mat_r_xy.coeff(2, 0) + r_foot_z_adjustment_by_force_z_*1.0;
  pose_right_foot_adjustment_.coeffRef(3) = (foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_ + r_foot_roll_adjustment_by_torque_roll_);
  pose_right_foot_adjustment_.coeffRef(4) = (foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_ + r_foot_pitch_adjustment_by_torque_pitch_);

  pose_left_foot_adjustment_.coeffRef(0) = l_foot_x_adjustment_by_force_x_;
  pose_left_foot_adjustment_.coeffRef(1) = l_foot_y_adjustment_by_force_y_;
  pose_left_foot_adjustment_.coeffRef(2) = -0.5*0.0*foot_z_adjustment_by_force_z_difference_ + mat_l_xy.coeff(2, 0) + l_foot_z_adjustment_by_force_z_*1.0;
  pose_left_foot_adjustment_.coeffRef(3) = (foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_ + l_foot_roll_adjustment_by_torque_roll_);
  pose_left_foot_adjustment_.coeffRef(4) = (foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_ + l_foot_pitch_adjustment_by_torque_pitch_);

  // check limitation
  if((fabs(pose_cob_adjustment_.coeff(0)) == cob_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(1)) == cob_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(2)) == cob_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(3)) == cob_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_cob_adjustment_.coeff(4)) == cob_pitch_adjustment_abs_max_rad_) ||
     (fabs(pose_right_foot_adjustment_.coeff(0)) == foot_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(1)) == foot_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(2)) == foot_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(3)) == foot_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_right_foot_adjustment_.coeff(4)) == foot_pitch_adjustment_abs_max_rad_) ||
     (fabs(pose_left_foot_adjustment_.coeff(0)) == foot_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(1)) == foot_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(2)) == foot_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(3)) == foot_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_left_foot_adjustment_.coeff(4)) == foot_pitch_adjustment_abs_max_rad_))
    balance_control_error_ &= BalanceControlError::BalanceLimit;

  pose_cob_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(0)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(0));
  pose_cob_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(1)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(1));
  pose_cob_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(2)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(2));
  pose_cob_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(3)), cob_roll_adjustment_abs_max_rad_ ), pose_cob_adjustment_.coeff(3));
  pose_cob_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(4)), cob_pitch_adjustment_abs_max_rad_), pose_cob_adjustment_.coeff(4));
  pose_cob_adjustment_.coeffRef(5) = 0;

  pose_right_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(0));
  pose_right_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(1));
  pose_right_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(2));
  pose_right_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_right_foot_adjustment_.coeff(3));
  pose_right_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_right_foot_adjustment_.coeff(4));
  pose_right_foot_adjustment_.coeffRef(5) = 0;

  pose_left_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(0));
  pose_left_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(1));
  pose_left_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(2));
  pose_left_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_left_foot_adjustment_.coeff(3));
  pose_left_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_left_foot_adjustment_.coeff(4));
  pose_left_foot_adjustment_.coeffRef(5) = 0;

  Eigen::MatrixXd cob_rotation_adj = robotis_framework::getRotationZ(pose_cob_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_cob_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_cob_adjustment_.coeff(3));
  Eigen::MatrixXd rf_rotation_adj = robotis_framework::getRotationZ(pose_right_foot_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_right_foot_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_right_foot_adjustment_.coeff(3));
  Eigen::MatrixXd lf_rotation_adj = robotis_framework::getRotationZ(pose_left_foot_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_left_foot_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_left_foot_adjustment_.coeff(3));
  mat_robot_to_cob_modified_.block<3,3>(0,0) = cob_rotation_adj * desired_robot_to_cob_.block<3,3>(0,0);
  mat_robot_to_right_foot_modified_.block<3,3>(0,0) = rf_rotation_adj * desired_robot_to_right_foot_.block<3,3>(0,0);;
  mat_robot_to_left_foot_modified_.block<3,3>(0,0) = lf_rotation_adj * desired_robot_to_left_foot_.block<3,3>(0,0);;

  mat_robot_to_cob_modified_.coeffRef(0,3) = desired_robot_to_cob_.coeff(0,3) + pose_cob_adjustment_.coeff(0);
  mat_robot_to_cob_modified_.coeffRef(1,3) = desired_robot_to_cob_.coeff(1,3) + pose_cob_adjustment_.coeff(1);
  mat_robot_to_cob_modified_.coeffRef(2,3) = desired_robot_to_cob_.coeff(2,3) + pose_cob_adjustment_.coeff(2);

  mat_robot_to_right_foot_modified_.coeffRef(0,3) = desired_robot_to_right_foot_.coeff(0,3) + pose_right_foot_adjustment_.coeff(0);
  mat_robot_to_right_foot_modified_.coeffRef(1,3) = desired_robot_to_right_foot_.coeff(1,3) + pose_right_foot_adjustment_.coeff(1);
  mat_robot_to_right_foot_modified_.coeffRef(2,3) = desired_robot_to_right_foot_.coeff(2,3) + pose_right_foot_adjustment_.coeff(2);

  mat_robot_to_left_foot_modified_.coeffRef(0,3) = desired_robot_to_left_foot_.coeff(0,3) + pose_left_foot_adjustment_.coeff(0);
  mat_robot_to_left_foot_modified_.coeffRef(1,3) = desired_robot_to_left_foot_.coeff(1,3) + pose_left_foot_adjustment_.coeff(1);
  mat_robot_to_left_foot_modified_.coeffRef(2,3) = desired_robot_to_left_foot_.coeff(2,3) + pose_left_foot_adjustment_.coeff(2);

  if(balance_error != 0)
    *balance_error = balance_control_error_;

  *robot_to_cob_modified        = mat_robot_to_cob_modified_;
  *robot_to_right_foot_modified = mat_robot_to_right_foot_modified_;
  *robot_to_left_foot_modified  = mat_robot_to_left_foot_modified_;
}

void BalanceControlUsingDampingConroller::setDesiredPose(const Eigen::MatrixXd &robot_to_cob, const Eigen::MatrixXd &robot_to_right_foot, const Eigen::MatrixXd &robot_to_left_foot)
{
  desired_robot_to_cob_        = robot_to_cob;
  desired_robot_to_right_foot_ = robot_to_right_foot;
  desired_robot_to_left_foot_  = robot_to_left_foot;
}

void BalanceControlUsingDampingConroller::setDesiredCOBGyro(double gyro_roll, double gyro_pitch)\
{
  desired_gyro_roll_  = gyro_roll;
  desired_gyro_pitch_ = gyro_pitch;
}

void BalanceControlUsingDampingConroller::setDesiredCOBOrientation(double cob_orientation_roll, double cob_orientation_pitch)
{
  foot_roll_angle_ctrl_.desired_  = cob_orientation_roll;
  foot_pitch_angle_ctrl_.desired_ = cob_orientation_pitch;
}

void BalanceControlUsingDampingConroller::setDesiredFootForceTorque(double r_force_x_N,      double r_force_y_N,       double r_force_z_N,
                                               double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                               double l_force_x_N,      double l_force_y_N,       double l_force_z_N,
                                               double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm)
{
  foot_force_z_diff_ctrl_.desired_ = l_force_z_N - r_force_z_N;
  right_foot_force_z_ctrl_.desired_ = r_force_z_N;
  left_foot_force_z_ctrl_.desired_ = l_force_z_N;

  right_foot_force_x_ctrl_.desired_      = r_force_x_N;
  right_foot_force_y_ctrl_.desired_      = r_force_y_N;
  right_foot_torque_roll_ctrl_.desired_  = r_torque_roll_Nm;
  right_foot_torque_pitch_ctrl_.desired_ = r_torque_pitch_Nm;

  left_foot_force_x_ctrl_.desired_      = l_force_x_N;
  left_foot_force_y_ctrl_.desired_      = l_force_y_N;
  left_foot_torque_roll_ctrl_.desired_  = l_torque_roll_Nm;
  left_foot_torque_pitch_ctrl_.desired_ = l_torque_pitch_Nm;
}


void BalanceControlUsingDampingConroller::setCurrentGyroSensorOutput(double gyro_roll, double gyro_pitch)
{
  current_gyro_roll_rad_per_sec_  = gyro_roll;
  current_gyro_pitch_rad_per_sec_ = gyro_pitch;
}

void BalanceControlUsingDampingConroller::setCurrentOrientationSensorOutput(double cob_orientation_roll, double cob_orientation_pitch)
{
  current_orientation_roll_rad_  = cob_orientation_roll;
  current_orientation_pitch_rad_ = cob_orientation_pitch;
}

void BalanceControlUsingDampingConroller::setCurrentFootForceTorqueSensorOutput(double r_force_x_N,      double r_force_y_N,       double r_force_z_N,
                                                           double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                                           double l_force_x_N,      double l_force_y_N,       double l_force_z_N,
                                                           double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm)
{
  current_right_fx_N_  = r_force_x_N;
  current_right_fy_N_  = r_force_y_N;
  current_right_fz_N_  = r_force_z_N;
  current_right_tx_Nm_ = r_torque_roll_Nm;
  current_right_ty_Nm_ = r_torque_pitch_Nm;
  current_right_tz_Nm_ = r_torque_yaw_Nm;

  current_left_fx_N_  = l_force_x_N;
  current_left_fy_N_  = l_force_y_N;
  current_left_fz_N_  = l_force_z_N;
  current_left_tx_Nm_ = l_torque_roll_Nm;
  current_left_ty_Nm_ = l_torque_pitch_Nm;
  current_left_tz_Nm_ = l_torque_yaw_Nm;
}

// set maximum adjustment
void BalanceControlUsingDampingConroller::setMaximumAdjustment(double cob_x_max_adjustment_m,  double cob_y_max_adjustment_m,  double cob_z_max_adjustment_m,
                                          double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
                                          double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
                                          double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad)
{
  cob_x_adjustment_abs_max_m_        = cob_x_max_adjustment_m;
  cob_y_adjustment_abs_max_m_        = cob_y_max_adjustment_m;
  cob_z_adjustment_abs_max_m_        = cob_z_max_adjustment_m;
  cob_roll_adjustment_abs_max_rad_   = cob_roll_max_adjustment_rad;
  cob_pitch_adjustment_abs_max_rad_  = cob_pitch_max_adjustment_rad;
  cob_yaw_adjustment_abs_max_rad_    = cob_yaw_max_adjustment_rad;
  foot_x_adjustment_abs_max_m_       = foot_x_max_adjustment_m;
  foot_y_adjustment_abs_max_m_       = foot_y_max_adjustment_m;
  foot_z_adjustment_abs_max_m_       = foot_z_max_adjustment_m;
  foot_roll_adjustment_abs_max_rad_  = foot_roll_max_adjustment_rad;
  foot_pitch_adjustment_abs_max_rad_ = foot_pitch_max_adjustment_rad;
  foot_yaw_adjustment_abs_max_rad_   = foot_yaw_max_adjustment_rad;
}

//Manual Adjustment
void BalanceControlUsingDampingConroller::setCOBManualAdjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m)
{
  cob_x_manual_adjustment_m_ = cob_x_adjustment_m;
  cob_y_manual_adjustment_m_ = cob_y_adjustment_m;
  cob_z_manual_adjustment_m_ = cob_z_adjustment_m;
}

double BalanceControlUsingDampingConroller::getCOBManualAdjustmentX()
{
  return cob_x_manual_adjustment_m_;
}

double BalanceControlUsingDampingConroller::getCOBManualAdjustmentY()
{
  return cob_y_manual_adjustment_m_;
}

double BalanceControlUsingDampingConroller::getCOBManualAdjustmentZ()
{
  return cob_z_manual_adjustment_m_;
}

void BalanceControlUsingDampingConroller::setGyroBalanceGainRatio(double gyro_balance_gain_ratio)
{
  gyro_balance_gain_ratio_ = gyro_balance_gain_ratio;
  gyro_balance_roll_gain_  = -0.10*0.75*gyro_balance_gain_ratio_;
  gyro_balance_pitch_gain_ = -0.10*0.5 *gyro_balance_gain_ratio_;
}

double BalanceControlUsingDampingConroller::getGyroBalanceGainRatio(void)
{
  return gyro_balance_gain_ratio_;
}


BalanceControlUsingPDController::BalanceControlUsingPDController()
{
  balance_control_error_ = BalanceControlError::NoError;
  control_cycle_sec_ = 0.008;

  // balance enable
  gyro_enable_ = 1.0;
  orientation_enable_ = 1.0;
  ft_enable_ = 1.0;

  desired_robot_to_cob_         = Eigen::MatrixXd::Identity(4,4);
  desired_robot_to_right_foot_  = Eigen::MatrixXd::Identity(4,4);
  desired_robot_to_left_foot_   = Eigen::MatrixXd::Identity(4,4);

  //sensed values
  current_gyro_roll_rad_per_sec_ = current_gyro_pitch_rad_per_sec_ = 0;

  current_orientation_roll_rad_ = current_orientation_pitch_rad_ = 0;

  current_right_fx_N_  = current_right_fy_N_  = current_right_fz_N_  = 0;
  current_right_tx_Nm_ = current_right_ty_Nm_ = current_right_tz_Nm_ = 0;
  current_left_fx_N_   = current_left_fy_N_   = current_left_fz_N_   = 0;
  current_left_tx_Nm_  = current_left_ty_Nm_  = current_left_tz_Nm_  = 0;

  // balance algorithm result
  foot_roll_adjustment_by_gyro_roll_ = 0;
  foot_pitch_adjustment_by_gyro_pitch_ = 0;

  foot_roll_adjustment_by_orientation_roll_ = 0;
  foot_pitch_adjustment_by_orientation_pitch_ = 0;

  r_foot_z_adjustment_by_force_z_ = 0;
  l_foot_z_adjustment_by_force_z_ = 0;

  r_foot_x_adjustment_by_force_x_ = 0;
  r_foot_y_adjustment_by_force_y_ = 0;
  r_foot_roll_adjustment_by_torque_roll_ = 0;
  r_foot_pitch_adjustment_by_torque_pitch_ = 0;

  l_foot_x_adjustment_by_force_x_ = 0;
  l_foot_y_adjustment_by_force_y_ = 0;
  l_foot_roll_adjustment_by_torque_roll_ = 0;
  l_foot_pitch_adjustment_by_torque_pitch_ = 0;

  // manual cob adjustment
  cob_x_manual_adjustment_m_ = 0;
  cob_y_manual_adjustment_m_ = 0;
  cob_z_manual_adjustment_m_ = 0;

  // maximum adjustment
  cob_x_adjustment_abs_max_m_ = 0.05;
  cob_y_adjustment_abs_max_m_ = 0.05;
  cob_z_adjustment_abs_max_m_ = 0.05;
  cob_roll_adjustment_abs_max_rad_  = 30.0*DEGREE2RADIAN;
  cob_pitch_adjustment_abs_max_rad_ = 30.0*DEGREE2RADIAN;
  cob_yaw_adjustment_abs_max_rad_   = 30.0*DEGREE2RADIAN;
  foot_x_adjustment_abs_max_m_ = 0.1;
  foot_y_adjustment_abs_max_m_ = 0.1;
  foot_z_adjustment_abs_max_m_ = 0.1;
  foot_roll_adjustment_abs_max_rad_  = 30.0*DEGREE2RADIAN;
  foot_pitch_adjustment_abs_max_rad_ = 30.0*DEGREE2RADIAN;
  foot_yaw_adjustment_abs_max_rad_   = 30.0*DEGREE2RADIAN;

  mat_robot_to_cob_modified_        = Eigen::MatrixXd::Identity(4,4);
  mat_robot_to_right_foot_modified_ = Eigen::MatrixXd::Identity(4,4);
  mat_robot_to_left_foot_modified_  = Eigen::MatrixXd::Identity(4,4);
  pose_cob_adjustment_         = Eigen::VectorXd::Zero(6);
  pose_right_foot_adjustment_  = Eigen::VectorXd::Zero(6);;
  pose_left_foot_adjustment_   = Eigen::VectorXd::Zero(6);;
}

BalanceControlUsingPDController::~BalanceControlUsingPDController()
{  }

void BalanceControlUsingPDController::initialize(const int control_cycle_msec)
{
  balance_control_error_ = BalanceControlError::NoError;

  control_cycle_sec_ = control_cycle_msec * 0.001;

  pose_cob_adjustment_.fill(0);
  pose_right_foot_adjustment_.fill(0);
  pose_left_foot_adjustment_.fill(0);

  roll_gyro_lpf_.initialize(control_cycle_sec_, 1.0);
  pitch_gyro_lpf_.initialize(control_cycle_sec_, 1.0);

  roll_angle_lpf_.initialize(control_cycle_sec_, 1.0);
  pitch_angle_lpf_.initialize(control_cycle_sec_, 1.0);

  right_foot_force_x_lpf_.initialize(control_cycle_sec_, 1.0);
  right_foot_force_y_lpf_.initialize(control_cycle_sec_, 1.0);
  right_foot_force_z_lpf_.initialize(control_cycle_sec_, 1.0);
  right_foot_torque_roll_lpf_.initialize(control_cycle_sec_, 1.0);
  right_foot_torque_pitch_lpf_.initialize(control_cycle_sec_, 1.0);

  left_foot_force_x_lpf_.initialize(control_cycle_sec_, 1.0);
  left_foot_force_y_lpf_.initialize(control_cycle_sec_, 1.0);
  left_foot_force_z_lpf_.initialize(control_cycle_sec_, 1.0);
  left_foot_torque_roll_lpf_.initialize(control_cycle_sec_, 1.0);
  left_foot_torque_pitch_lpf_.initialize(control_cycle_sec_, 1.0);
}

void BalanceControlUsingPDController::setGyroBalanceEnable(bool enable)
{
  if(enable)
    gyro_enable_ = 1.0;
  else
    gyro_enable_ = 0.0;
}

void BalanceControlUsingPDController::setOrientationBalanceEnable(bool enable)
{
  if(enable)
    orientation_enable_ = 1.0;
  else
    orientation_enable_ = 0.0;
}

void BalanceControlUsingPDController::setForceTorqueBalanceEnable(bool enable)
{
  if(enable)
    ft_enable_ = 1.0;
  else
    ft_enable_ = 0.0;
}

void BalanceControlUsingPDController::process(int *balance_error, Eigen::MatrixXd *robot_to_cob_modified, Eigen::MatrixXd *robot_to_right_foot_modified, Eigen::MatrixXd *robot_to_left_foot_modified)
{
  balance_control_error_ = BalanceControlError::NoError;

  pose_cob_adjustment_.fill(0);
  pose_right_foot_adjustment_.fill(0);
  pose_left_foot_adjustment_.fill(0);

  //get filtered value
  double roll_gyro_filtered  = roll_gyro_lpf_.getFilteredOutput(current_gyro_roll_rad_per_sec_);
  double pitch_gyro_filtered = pitch_gyro_lpf_.getFilteredOutput(current_gyro_pitch_rad_per_sec_);

  double roll_angle_filtered  = roll_angle_lpf_.getFilteredOutput(current_orientation_roll_rad_);
  double pitch_angle_filtered = pitch_angle_lpf_.getFilteredOutput(current_orientation_pitch_rad_);

  double right_foot_force_x_filtered      = right_foot_force_x_lpf_.getFilteredOutput(current_right_fx_N_);
  double right_foot_force_y_filtered      = right_foot_force_y_lpf_.getFilteredOutput(current_right_fy_N_);
  double right_foot_force_z_filtered      = right_foot_force_z_lpf_.getFilteredOutput(current_right_fz_N_);
  double right_foot_torque_roll_filtered  = right_foot_torque_roll_lpf_.getFilteredOutput(current_right_tx_Nm_);
  double right_foot_torque_pitch_filtered = right_foot_torque_pitch_lpf_.getFilteredOutput(current_right_ty_Nm_);;

  double left_foot_force_x_filtered      = left_foot_force_x_lpf_.getFilteredOutput(current_left_fx_N_);
  double left_foot_force_y_filtered      = left_foot_force_y_lpf_.getFilteredOutput(current_left_fy_N_);
  double left_foot_force_z_filtered      = left_foot_force_z_lpf_.getFilteredOutput(current_left_fz_N_);
  double left_foot_torque_roll_filtered  = left_foot_torque_roll_lpf_.getFilteredOutput(current_left_tx_Nm_);
  double left_foot_torque_pitch_filtered = left_foot_torque_pitch_lpf_.getFilteredOutput(current_left_ty_Nm_);


  // gyro
  foot_roll_adjustment_by_gyro_roll_   = -0.1*gyro_enable_*foot_roll_gyro_ctrl_.getFeedBack(roll_gyro_filtered);
  foot_pitch_adjustment_by_gyro_pitch_ = -0.1*gyro_enable_*foot_pitch_gyro_ctrl_.getFeedBack(pitch_gyro_filtered);

  // z by imu
  foot_roll_adjustment_by_orientation_roll_   = -1.0*orientation_enable_ * foot_roll_angle_ctrl_.getFeedBack(roll_angle_filtered);
  foot_pitch_adjustment_by_orientation_pitch_ = -1.0*orientation_enable_ * foot_pitch_angle_ctrl_.getFeedBack(pitch_angle_filtered);

  Eigen::MatrixXd mat_orientation_adjustment_by_imu = robotis_framework::getRotation4d(foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_, foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_, 0.0);
  Eigen::MatrixXd mat_r_xy, mat_l_xy;
  mat_r_xy.resize(4,1); mat_l_xy.resize(4,1);
  mat_r_xy.coeffRef(0,0) = desired_robot_to_right_foot_.coeff(0,3) - 0.5*(desired_robot_to_right_foot_.coeff(0,3) + desired_robot_to_left_foot_.coeff(0,3));
  mat_r_xy.coeffRef(1,0) = desired_robot_to_right_foot_.coeff(1,3) - 0.5*(desired_robot_to_right_foot_.coeff(1,3) + desired_robot_to_left_foot_.coeff(1,3));
  mat_r_xy.coeffRef(2,0) = 0.0;
  mat_r_xy.coeffRef(3,0) = 1;

  mat_l_xy.coeffRef(0,0) = desired_robot_to_left_foot_.coeff(0,3) - 0.5*(desired_robot_to_right_foot_.coeff(0,3) + desired_robot_to_left_foot_.coeff(0,3));
  mat_l_xy.coeffRef(1,0) = desired_robot_to_left_foot_.coeff(1,3) - 0.5*(desired_robot_to_right_foot_.coeff(1,3) + desired_robot_to_left_foot_.coeff(1,3));
  mat_l_xy.coeffRef(2,0) = 0.0;
  mat_l_xy.coeffRef(3,0) = 1;

  mat_r_xy = mat_orientation_adjustment_by_imu * mat_r_xy;
  mat_l_xy = mat_orientation_adjustment_by_imu * mat_l_xy;

  // ft sensor
  r_foot_x_adjustment_by_force_x_ = ft_enable_*0.001*right_foot_force_x_ctrl_.getFeedBack(right_foot_force_x_filtered);
  r_foot_y_adjustment_by_force_y_ = ft_enable_*0.001*right_foot_force_y_ctrl_.getFeedBack(right_foot_force_y_filtered);
  r_foot_z_adjustment_by_force_z_ = ft_enable_*0.001*right_foot_force_z_ctrl_.getFeedBack(right_foot_force_z_filtered);
  r_foot_roll_adjustment_by_torque_roll_   = ft_enable_*right_foot_torque_roll_ctrl_.getFeedBack(right_foot_torque_roll_filtered);
  r_foot_pitch_adjustment_by_torque_pitch_ = ft_enable_*right_foot_torque_pitch_ctrl_.getFeedBack(right_foot_torque_pitch_filtered);

  l_foot_x_adjustment_by_force_x_ = ft_enable_*0.001*left_foot_force_x_ctrl_.getFeedBack(left_foot_force_x_filtered);
  l_foot_y_adjustment_by_force_y_ = ft_enable_*0.001*left_foot_force_y_ctrl_.getFeedBack(left_foot_force_y_filtered);
  l_foot_z_adjustment_by_force_z_ = ft_enable_*0.001*left_foot_force_z_ctrl_.getFeedBack(left_foot_force_z_filtered);
  l_foot_roll_adjustment_by_torque_roll_   = ft_enable_*left_foot_torque_roll_ctrl_.getFeedBack(left_foot_torque_roll_filtered);
  l_foot_pitch_adjustment_by_torque_pitch_ = ft_enable_*left_foot_torque_pitch_ctrl_.getFeedBack(left_foot_torque_pitch_filtered);

  // sum of sensory balance result
  pose_cob_adjustment_.coeffRef(0) = cob_x_manual_adjustment_m_;
  pose_cob_adjustment_.coeffRef(1) = cob_y_manual_adjustment_m_;
  pose_cob_adjustment_.coeffRef(2) = cob_z_manual_adjustment_m_;

  pose_right_foot_adjustment_.coeffRef(0) = r_foot_x_adjustment_by_force_x_;
  pose_right_foot_adjustment_.coeffRef(1) = r_foot_y_adjustment_by_force_y_;
  pose_right_foot_adjustment_.coeffRef(2) = mat_r_xy.coeff(2, 0) + r_foot_z_adjustment_by_force_z_*1.0;
  pose_right_foot_adjustment_.coeffRef(3) = (foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_ + r_foot_roll_adjustment_by_torque_roll_);
  pose_right_foot_adjustment_.coeffRef(4) = (foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_ + r_foot_pitch_adjustment_by_torque_pitch_);

  pose_left_foot_adjustment_.coeffRef(0) = l_foot_x_adjustment_by_force_x_;
  pose_left_foot_adjustment_.coeffRef(1) = l_foot_y_adjustment_by_force_y_;
  pose_left_foot_adjustment_.coeffRef(2) = mat_l_xy.coeff(2, 0) + l_foot_z_adjustment_by_force_z_*1.0;
  pose_left_foot_adjustment_.coeffRef(3) = (foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_ + l_foot_roll_adjustment_by_torque_roll_);
  pose_left_foot_adjustment_.coeffRef(4) = (foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_ + l_foot_pitch_adjustment_by_torque_pitch_);

  // check limitation
  if((fabs(pose_cob_adjustment_.coeff(0)) == cob_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(1)) == cob_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(2)) == cob_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(3)) == cob_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_cob_adjustment_.coeff(4)) == cob_pitch_adjustment_abs_max_rad_) ||
     (fabs(pose_right_foot_adjustment_.coeff(0)) == foot_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(1)) == foot_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(2)) == foot_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(3)) == foot_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_right_foot_adjustment_.coeff(4)) == foot_pitch_adjustment_abs_max_rad_) ||
     (fabs(pose_left_foot_adjustment_.coeff(0)) == foot_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(1)) == foot_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(2)) == foot_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(3)) == foot_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_left_foot_adjustment_.coeff(4)) == foot_pitch_adjustment_abs_max_rad_))
    balance_control_error_ &= BalanceControlError::BalanceLimit;

  pose_cob_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(0)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(0));
  pose_cob_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(1)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(1));
  pose_cob_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(2)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(2));
  pose_cob_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(3)), cob_roll_adjustment_abs_max_rad_ ), pose_cob_adjustment_.coeff(3));
  pose_cob_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(4)), cob_pitch_adjustment_abs_max_rad_), pose_cob_adjustment_.coeff(4));
  pose_cob_adjustment_.coeffRef(5) = 0;

  pose_right_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(0));
  pose_right_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(1));
  pose_right_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(2));
  pose_right_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_right_foot_adjustment_.coeff(3));
  pose_right_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_right_foot_adjustment_.coeff(4));
  pose_right_foot_adjustment_.coeffRef(5) = 0;

  pose_left_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(0));
  pose_left_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(1));
  pose_left_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(2));
  pose_left_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_left_foot_adjustment_.coeff(3));
  pose_left_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_left_foot_adjustment_.coeff(4));
  pose_left_foot_adjustment_.coeffRef(5) = 0;

  Eigen::MatrixXd cob_rotation_adj = robotis_framework::getRotationZ(pose_cob_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_cob_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_cob_adjustment_.coeff(3));
  Eigen::MatrixXd rf_rotation_adj = robotis_framework::getRotationZ(pose_right_foot_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_right_foot_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_right_foot_adjustment_.coeff(3));
  Eigen::MatrixXd lf_rotation_adj = robotis_framework::getRotationZ(pose_left_foot_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_left_foot_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_left_foot_adjustment_.coeff(3));
  mat_robot_to_cob_modified_.block<3,3>(0,0) = cob_rotation_adj * desired_robot_to_cob_.block<3,3>(0,0);
  mat_robot_to_right_foot_modified_.block<3,3>(0,0) = rf_rotation_adj * desired_robot_to_right_foot_.block<3,3>(0,0);;
  mat_robot_to_left_foot_modified_.block<3,3>(0,0) = lf_rotation_adj * desired_robot_to_left_foot_.block<3,3>(0,0);;

  mat_robot_to_cob_modified_.coeffRef(0,3) = desired_robot_to_cob_.coeff(0,3) + pose_cob_adjustment_.coeff(0);
  mat_robot_to_cob_modified_.coeffRef(1,3) = desired_robot_to_cob_.coeff(1,3) + pose_cob_adjustment_.coeff(1);
  mat_robot_to_cob_modified_.coeffRef(2,3) = desired_robot_to_cob_.coeff(2,3) + pose_cob_adjustment_.coeff(2);

  mat_robot_to_right_foot_modified_.coeffRef(0,3) = desired_robot_to_right_foot_.coeff(0,3) + pose_right_foot_adjustment_.coeff(0);
  mat_robot_to_right_foot_modified_.coeffRef(1,3) = desired_robot_to_right_foot_.coeff(1,3) + pose_right_foot_adjustment_.coeff(1);
  mat_robot_to_right_foot_modified_.coeffRef(2,3) = desired_robot_to_right_foot_.coeff(2,3) + pose_right_foot_adjustment_.coeff(2);

  mat_robot_to_left_foot_modified_.coeffRef(0,3) = desired_robot_to_left_foot_.coeff(0,3) + pose_left_foot_adjustment_.coeff(0);
  mat_robot_to_left_foot_modified_.coeffRef(1,3) = desired_robot_to_left_foot_.coeff(1,3) + pose_left_foot_adjustment_.coeff(1);
  mat_robot_to_left_foot_modified_.coeffRef(2,3) = desired_robot_to_left_foot_.coeff(2,3) + pose_left_foot_adjustment_.coeff(2);

  if(balance_error != 0)
    *balance_error = balance_control_error_;

  *robot_to_cob_modified        = mat_robot_to_cob_modified_;
  *robot_to_right_foot_modified = mat_robot_to_right_foot_modified_;
  *robot_to_left_foot_modified  = mat_robot_to_left_foot_modified_;
}

void BalanceControlUsingPDController::setDesiredPose(const Eigen::MatrixXd &robot_to_cob, const Eigen::MatrixXd &robot_to_right_foot, const Eigen::MatrixXd &robot_to_left_foot)
{
  desired_robot_to_cob_        = robot_to_cob;
  desired_robot_to_right_foot_ = robot_to_right_foot;
  desired_robot_to_left_foot_  = robot_to_left_foot;
}

void BalanceControlUsingPDController::setDesiredCOBGyro(double gyro_roll, double gyro_pitch)\
{
  foot_roll_gyro_ctrl_.desired_  = gyro_roll;
  foot_pitch_gyro_ctrl_.desired_ = gyro_pitch;
}

void BalanceControlUsingPDController::setDesiredCOBOrientation(double cob_orientation_roll, double cob_orientation_pitch)
{
  foot_roll_angle_ctrl_.desired_  = cob_orientation_roll;
  foot_pitch_angle_ctrl_.desired_ = cob_orientation_pitch;
}

void BalanceControlUsingPDController::setDesiredFootForceTorque(double r_force_x_N,      double r_force_y_N,       double r_force_z_N,
                                               double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                               double l_force_x_N,      double l_force_y_N,       double l_force_z_N,
                                               double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm)
{
  right_foot_force_x_ctrl_.desired_      = r_force_x_N;
  right_foot_force_y_ctrl_.desired_      = r_force_y_N;
  right_foot_force_z_ctrl_.desired_      = r_force_z_N;
  right_foot_torque_roll_ctrl_.desired_  = r_torque_roll_Nm;
  right_foot_torque_pitch_ctrl_.desired_ = r_torque_pitch_Nm;

  left_foot_force_x_ctrl_.desired_      = l_force_x_N;
  left_foot_force_y_ctrl_.desired_      = l_force_y_N;
  left_foot_force_z_ctrl_.desired_      = l_force_z_N;
  left_foot_torque_roll_ctrl_.desired_  = l_torque_roll_Nm;
  left_foot_torque_pitch_ctrl_.desired_ = l_torque_pitch_Nm;
}


void BalanceControlUsingPDController::setCurrentGyroSensorOutput(double gyro_roll, double gyro_pitch)
{
  current_gyro_roll_rad_per_sec_  = gyro_roll;
  current_gyro_pitch_rad_per_sec_ = gyro_pitch;
}

void BalanceControlUsingPDController::setCurrentOrientationSensorOutput(double cob_orientation_roll, double cob_orientation_pitch)
{
  current_orientation_roll_rad_  = cob_orientation_roll;
  current_orientation_pitch_rad_ = cob_orientation_pitch;
}

void BalanceControlUsingPDController::setCurrentFootForceTorqueSensorOutput(double r_force_x_N,      double r_force_y_N,       double r_force_z_N,
                                                           double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                                           double l_force_x_N,      double l_force_y_N,       double l_force_z_N,
                                                           double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm)
{
  current_right_fx_N_  = r_force_x_N;
  current_right_fy_N_  = r_force_y_N;
  current_right_fz_N_  = r_force_z_N;
  current_right_tx_Nm_ = r_torque_roll_Nm;
  current_right_ty_Nm_ = r_torque_pitch_Nm;
  current_right_tz_Nm_ = r_torque_yaw_Nm;

  current_left_fx_N_  = l_force_x_N;
  current_left_fy_N_  = l_force_y_N;
  current_left_fz_N_  = l_force_z_N;
  current_left_tx_Nm_ = l_torque_roll_Nm;
  current_left_ty_Nm_ = l_torque_pitch_Nm;
  current_left_tz_Nm_ = l_torque_yaw_Nm;
}

// set maximum adjustment
void BalanceControlUsingPDController::setMaximumAdjustment(double cob_x_max_adjustment_m,  double cob_y_max_adjustment_m,  double cob_z_max_adjustment_m,
                                          double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
                                          double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
                                          double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad)
{
  cob_x_adjustment_abs_max_m_        = cob_x_max_adjustment_m;
  cob_y_adjustment_abs_max_m_        = cob_y_max_adjustment_m;
  cob_z_adjustment_abs_max_m_        = cob_z_max_adjustment_m;
  cob_roll_adjustment_abs_max_rad_   = cob_roll_max_adjustment_rad;
  cob_pitch_adjustment_abs_max_rad_  = cob_pitch_max_adjustment_rad;
  cob_yaw_adjustment_abs_max_rad_    = cob_yaw_max_adjustment_rad;
  foot_x_adjustment_abs_max_m_       = foot_x_max_adjustment_m;
  foot_y_adjustment_abs_max_m_       = foot_y_max_adjustment_m;
  foot_z_adjustment_abs_max_m_       = foot_z_max_adjustment_m;
  foot_roll_adjustment_abs_max_rad_  = foot_roll_max_adjustment_rad;
  foot_pitch_adjustment_abs_max_rad_ = foot_pitch_max_adjustment_rad;
  foot_yaw_adjustment_abs_max_rad_   = foot_yaw_max_adjustment_rad;
}

//Manual Adjustment
void BalanceControlUsingPDController::setCOBManualAdjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m)
{
  cob_x_manual_adjustment_m_ = cob_x_adjustment_m;
  cob_y_manual_adjustment_m_ = cob_y_adjustment_m;
  cob_z_manual_adjustment_m_ = cob_z_adjustment_m;
}

double BalanceControlUsingPDController::getCOBManualAdjustmentX()
{
  return cob_x_manual_adjustment_m_;
}

double BalanceControlUsingPDController::getCOBManualAdjustmentY()
{
  return cob_y_manual_adjustment_m_;
}

double BalanceControlUsingPDController::getCOBManualAdjustmentZ()
{
  return cob_z_manual_adjustment_m_;
}
