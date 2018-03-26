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

#ifndef OP3_BALANCE_CONTROL_OP3_BALANCE_CONTROL_H_
#define OP3_BALANCE_CONTROL_OP3_BALANCE_CONTROL_H_

#include <eigen3/Eigen/Eigen>

#include "robotis_math/robotis_math.h"

namespace robotis_op
{

class BalanceControlError
{
public:
  static const int NoError = 0;
  static const int BalanceLimit = 2;
};

class DampingController
{
public:
  DampingController();
  DampingController(double time_unit_sec);
  ~DampingController();

  double getDampingControllerOutput(double present_sensor_output);

  double desired_;

  double gain_;
  double time_constant_sec_;
  double output_;

  double control_cycle_sec_;

private:
  double previous_result_;
};

class BalancePDController
{
public:
  BalancePDController();
  ~BalancePDController();

  double desired_;

  double p_gain_;
  double d_gain_;

  double getFeedBack(double present_sensor_output);

private:
  double curr_err_;
  double prev_err_;
};

class BalanceLowPassFilter
{
public:
  BalanceLowPassFilter();
  BalanceLowPassFilter(double control_cycle_sec, double cut_off_frequency);
  ~BalanceLowPassFilter();

  void initialize(double control_cycle_sec_, double cut_off_frequency);
  void setCutOffFrequency(double cut_off_frequency);
  double getCutOffFrequency(void);
  double getFilteredOutput(double present_raw_value);

private:
  double cut_off_freq_;
  double control_cycle_sec_;
  double alpha_;

  double prev_output_;
};

class BalanceControlUsingDampingConroller
{
public:
  BalanceControlUsingDampingConroller();
  ~BalanceControlUsingDampingConroller();

  void initialize(const int control_cycle_msec);

  void setGyroBalanceEnable(bool enable);
  void setOrientationBalanceEnable(bool enable);
  void setForceTorqueBalanceEnable(bool enable);

  void process(int *balance_error, Eigen::MatrixXd *robot_to_cob_modified, Eigen::MatrixXd *robot_to_right_foot_modified, Eigen::MatrixXd *robot_to_left_foot_modified);

  void setDesiredPose(const Eigen::MatrixXd &robot_to_cob, const Eigen::MatrixXd &robot_to_right_foot, const Eigen::MatrixXd &robot_to_left_foot);

  // all arguments are with respect to robot coordinate.
  void setDesiredCOBGyro(double gyro_roll, double gyro_pitch);
  void setDesiredCOBOrientation(double cob_orientation_roll, double cob_orientation_pitch);
  void setDesiredFootForceTorque(double r_force_x_N,      double r_force_y_N,       double r_force_z_N,
                                 double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                 double l_force_x_N,      double l_force_y_N,       double l_force_z_N,
                                 double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm);

  // with respect to robot coordinate.
  void setCurrentGyroSensorOutput(double gyro_roll, double gyro_pitch);
  void setCurrentOrientationSensorOutput(double cob_orientation_roll, double cob_orientation_pitch);

  // with respect to robot coordinate.
  void setCurrentFootForceTorqueSensorOutput(double r_force_x_N,      double r_force_y_N,       double r_force_z_N,
                                             double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                             double l_force_x_N,      double l_force_y_N,       double l_force_z_N,
                                             double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm);


  // set maximum adjustment
  void setMaximumAdjustment(double cob_x_max_adjustment_m,  double cob_y_max_adjustment_m,  double cob_z_max_adjustment_m,
                            double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
                            double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
                            double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad);

  //Manual Adjustment
  void setCOBManualAdjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m);
  double getCOBManualAdjustmentX();
  double getCOBManualAdjustmentY();
  double getCOBManualAdjustmentZ();

  void setGyroBalanceGainRatio(double gyro_balance_gain_ratio);
  double getGyroBalanceGainRatio(void);

  // damping controllers
  DampingController foot_roll_angle_ctrl_;
  DampingController foot_pitch_angle_ctrl_;

  DampingController foot_force_z_diff_ctrl_;
  DampingController right_foot_force_z_ctrl_;
  DampingController left_foot_force_z_ctrl_;

  DampingController right_foot_force_x_ctrl_;
  DampingController right_foot_force_y_ctrl_;
  DampingController right_foot_torque_roll_ctrl_;
  DampingController right_foot_torque_pitch_ctrl_;

  DampingController left_foot_force_x_ctrl_;
  DampingController left_foot_force_y_ctrl_;
  DampingController left_foot_torque_roll_ctrl_;
  DampingController left_foot_torque_pitch_ctrl_;

private:
  int balance_control_error_;
  double control_cycle_sec_;

  // balance enable
  double gyro_enable_;
  double orientation_enable_;
  double ft_enable_;


  // desired pose
  Eigen::MatrixXd desired_robot_to_cob_;
  Eigen::MatrixXd desired_robot_to_right_foot_;
  Eigen::MatrixXd desired_robot_to_left_foot_;

  // for gyro balancing
  double gyro_balance_gain_ratio_;
  double gyro_balance_roll_gain_;
  double gyro_balance_pitch_gain_;
  double gyro_cut_off_freq_;
  double gyro_lpf_alpha_;
  double gyro_roll_filtered_, gyro_pitch_filtered_;
  double desired_gyro_roll_, desired_gyro_pitch_;

  // sensed values
  double current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_;

  double current_orientation_roll_rad_, current_orientation_pitch_rad_;

  double current_right_fx_N_,  current_right_fy_N_,  current_right_fz_N_;
  double current_right_tx_Nm_, current_right_ty_Nm_, current_right_tz_Nm_;
  double current_left_fx_N_,   current_left_fy_N_,   current_left_fz_N_;
  double current_left_tx_Nm_,  current_left_ty_Nm_,  current_left_tz_Nm_;

  // manual cob adjustment
  double cob_x_manual_adjustment_m_;
  double cob_y_manual_adjustment_m_;
  double cob_z_manual_adjustment_m_;

  // result of balance control
  double foot_roll_adjustment_by_gyro_roll_;
  double foot_pitch_adjustment_by_gyro_pitch_;

  double foot_roll_adjustment_by_orientation_roll_;
  double foot_pitch_adjustment_by_orientation_pitch_;

  double foot_z_adjustment_by_force_z_difference_;
  double r_foot_z_adjustment_by_force_z_;
  double l_foot_z_adjustment_by_force_z_;

  double r_foot_x_adjustment_by_force_x_;
  double r_foot_y_adjustment_by_force_y_;
  double r_foot_roll_adjustment_by_torque_roll_;
  double r_foot_pitch_adjustment_by_torque_pitch_;

  double l_foot_x_adjustment_by_force_x_;
  double l_foot_y_adjustment_by_force_y_;
  double l_foot_roll_adjustment_by_torque_roll_;
  double l_foot_pitch_adjustment_by_torque_pitch_;

  // sum of results of balance control
  Eigen::VectorXd pose_cob_adjustment_;
  Eigen::VectorXd pose_right_foot_adjustment_;
  Eigen::VectorXd pose_left_foot_adjustment_;

  Eigen::MatrixXd mat_robot_to_cob_modified_;
  Eigen::MatrixXd mat_robot_to_right_foot_modified_;
  Eigen::MatrixXd mat_robot_to_left_foot_modified_;

  // maximum adjustment
  double cob_x_adjustment_abs_max_m_;
  double cob_y_adjustment_abs_max_m_;
  double cob_z_adjustment_abs_max_m_;
  double cob_roll_adjustment_abs_max_rad_;
  double cob_pitch_adjustment_abs_max_rad_;
  double cob_yaw_adjustment_abs_max_rad_;

  double foot_x_adjustment_abs_max_m_;
  double foot_y_adjustment_abs_max_m_;
  double foot_z_adjustment_abs_max_m_;
  double foot_roll_adjustment_abs_max_rad_;
  double foot_pitch_adjustment_abs_max_rad_;
  double foot_yaw_adjustment_abs_max_rad_;

};

class BalanceControlUsingPDController
{
public:
  BalanceControlUsingPDController();
  ~BalanceControlUsingPDController();

  void initialize(const int control_cycle_msec);

  void setGyroBalanceEnable(bool enable);
  void setOrientationBalanceEnable(bool enable);
  void setForceTorqueBalanceEnable(bool enable);

  void process(int *balance_error, Eigen::MatrixXd *robot_to_cob_modified, Eigen::MatrixXd *robot_to_right_foot_modified, Eigen::MatrixXd *robot_to_left_foot_modified);

  void setDesiredPose(const Eigen::MatrixXd &robot_to_cob, const Eigen::MatrixXd &robot_to_right_foot, const Eigen::MatrixXd &robot_to_left_foot);

  // all arguments are with respect to robot coordinate.
  void setDesiredCOBGyro(double gyro_roll, double gyro_pitch);
  void setDesiredCOBOrientation(double cob_orientation_roll, double cob_orientation_pitch);
  void setDesiredFootForceTorque(double r_force_x_N,      double r_force_y_N,       double r_force_z_N,
                                 double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                 double l_force_x_N,      double l_force_y_N,       double l_force_z_N,
                                 double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm);

  // with respect to robot coordinate.
  void setCurrentGyroSensorOutput(double gyro_roll, double gyro_pitch);
  void setCurrentOrientationSensorOutput(double cob_orientation_roll, double cob_orientation_pitch);

  // with respect to robot coordinate.
  void setCurrentFootForceTorqueSensorOutput(double r_force_x_N,      double r_force_y_N,       double r_force_z_N,
                                             double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                             double l_force_x_N,      double l_force_y_N,       double l_force_z_N,
                                             double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm);


  // set maximum adjustment
  void setMaximumAdjustment(double cob_x_max_adjustment_m,  double cob_y_max_adjustment_m,  double cob_z_max_adjustment_m,
                            double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
                            double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
                            double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad);

  //Manual Adjustment
  void setCOBManualAdjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m);
  double getCOBManualAdjustmentX();
  double getCOBManualAdjustmentY();
  double getCOBManualAdjustmentZ();

  // damping controllers
  BalancePDController foot_roll_gyro_ctrl_;
  BalancePDController foot_pitch_gyro_ctrl_;
  BalancePDController foot_roll_angle_ctrl_;
  BalancePDController foot_pitch_angle_ctrl_;

  BalancePDController right_foot_force_z_ctrl_;
  BalancePDController left_foot_force_z_ctrl_;

  BalancePDController right_foot_force_x_ctrl_;
  BalancePDController right_foot_force_y_ctrl_;
  BalancePDController right_foot_torque_roll_ctrl_;
  BalancePDController right_foot_torque_pitch_ctrl_;

  BalancePDController left_foot_force_x_ctrl_;
  BalancePDController left_foot_force_y_ctrl_;
  BalancePDController left_foot_torque_roll_ctrl_;
  BalancePDController left_foot_torque_pitch_ctrl_;

  BalanceLowPassFilter roll_gyro_lpf_;
  BalanceLowPassFilter pitch_gyro_lpf_;

  BalanceLowPassFilter roll_angle_lpf_;
  BalanceLowPassFilter pitch_angle_lpf_;

  BalanceLowPassFilter right_foot_force_x_lpf_;
  BalanceLowPassFilter right_foot_force_y_lpf_;
  BalanceLowPassFilter right_foot_force_z_lpf_;
  BalanceLowPassFilter right_foot_torque_roll_lpf_;
  BalanceLowPassFilter right_foot_torque_pitch_lpf_;

  BalanceLowPassFilter left_foot_force_x_lpf_;
  BalanceLowPassFilter left_foot_force_y_lpf_;
  BalanceLowPassFilter left_foot_force_z_lpf_;
  BalanceLowPassFilter left_foot_torque_roll_lpf_;
  BalanceLowPassFilter left_foot_torque_pitch_lpf_;

private:
  int balance_control_error_;
  double control_cycle_sec_;

  // balance enable
  double gyro_enable_;
  double orientation_enable_;
  double ft_enable_;

  // desired pose
  Eigen::MatrixXd desired_robot_to_cob_;
  Eigen::MatrixXd desired_robot_to_right_foot_;
  Eigen::MatrixXd desired_robot_to_left_foot_;

  // sensed values
  double current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_;

  double current_orientation_roll_rad_, current_orientation_pitch_rad_;

  double current_right_fx_N_,  current_right_fy_N_,  current_right_fz_N_;
  double current_right_tx_Nm_, current_right_ty_Nm_, current_right_tz_Nm_;
  double current_left_fx_N_,   current_left_fy_N_,   current_left_fz_N_;
  double current_left_tx_Nm_,  current_left_ty_Nm_,  current_left_tz_Nm_;

  // manual cob adjustment
  double cob_x_manual_adjustment_m_;
  double cob_y_manual_adjustment_m_;
  double cob_z_manual_adjustment_m_;

  // result of balance control
  double foot_roll_adjustment_by_gyro_roll_;
  double foot_pitch_adjustment_by_gyro_pitch_;

  double foot_roll_adjustment_by_orientation_roll_;
  double foot_pitch_adjustment_by_orientation_pitch_;

  double r_foot_z_adjustment_by_force_z_;
  double l_foot_z_adjustment_by_force_z_;

  double r_foot_x_adjustment_by_force_x_;
  double r_foot_y_adjustment_by_force_y_;
  double r_foot_roll_adjustment_by_torque_roll_;
  double r_foot_pitch_adjustment_by_torque_pitch_;

  double l_foot_x_adjustment_by_force_x_;
  double l_foot_y_adjustment_by_force_y_;
  double l_foot_roll_adjustment_by_torque_roll_;
  double l_foot_pitch_adjustment_by_torque_pitch_;

  // sum of results of balance control
  Eigen::VectorXd pose_cob_adjustment_;
  Eigen::VectorXd pose_right_foot_adjustment_;
  Eigen::VectorXd pose_left_foot_adjustment_;

  Eigen::MatrixXd mat_robot_to_cob_modified_;
  Eigen::MatrixXd mat_robot_to_right_foot_modified_;
  Eigen::MatrixXd mat_robot_to_left_foot_modified_;

  // maximum adjustment
  double cob_x_adjustment_abs_max_m_;
  double cob_y_adjustment_abs_max_m_;
  double cob_z_adjustment_abs_max_m_;
  double cob_roll_adjustment_abs_max_rad_;
  double cob_pitch_adjustment_abs_max_rad_;
  double cob_yaw_adjustment_abs_max_rad_;

  double foot_x_adjustment_abs_max_m_;
  double foot_y_adjustment_abs_max_m_;
  double foot_z_adjustment_abs_max_m_;
  double foot_roll_adjustment_abs_max_rad_;
  double foot_pitch_adjustment_abs_max_rad_;
  double foot_yaw_adjustment_abs_max_rad_;
};

}

#endif
