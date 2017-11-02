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

#ifndef OP3_WHOLEBODY_MODULE_WHOLEBODY_MODULE_H_
#define OP3_WHOLEBODY_MODULE_WHOLEBODY_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "joint_control.h"
#include "wholebody_control.h"
#include "walking_control.h"
#include "op3_kdl.h"

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

//#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"
#include "op3_balance_control/op3_balance_control.h"

#include "op3_wholebody_module_msgs/JointPose.h"
#include "op3_wholebody_module_msgs/KinematicsPose.h"
#include "op3_wholebody_module_msgs/FootStepCommand.h"
#include "op3_wholebody_module_msgs/PreviewRequest.h"
#include "op3_wholebody_module_msgs/PreviewResponse.h"
#include "op3_wholebody_module_msgs/WalkingParam.h"

#include "op3_wholebody_module_msgs/GetJointPose.h"
#include "op3_wholebody_module_msgs/GetKinematicsPose.h"
#include "op3_wholebody_module_msgs/GetPreviewMatrix.h"

namespace robotis_op
{

enum CONTROL_TYPE {
  JOINT_CONTROL,
  WHOLEBODY_CONTROL,
  WALKING_CONTROL,
  OFFSET_CONTROL,
  NONE
};

enum BALANCE_TYPE {
  ON,
  OFF
};

class WholebodyModule: public robotis_framework::MotionModule,
                       public robotis_framework::Singleton<WholebodyModule>
{
public:
  WholebodyModule();
  virtual ~WholebodyModule();

  /* ROS Topic Callback Functions */
  void setResetBodyCallback(const std_msgs::Bool::ConstPtr& msg);
  void setWholebodyBalanceMsgCallback(const std_msgs::String::ConstPtr& msg);
  void setBodyOffsetCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void setFootDistanceCallback(const std_msgs::Float64::ConstPtr& msg);

  void goalJointPoseCallback(const op3_wholebody_module_msgs::JointPose &msg);
  void goalKinematicsPoseCallback(const op3_wholebody_module_msgs::KinematicsPose& msg);
  void footStepCommandCallback(const op3_wholebody_module_msgs::FootStepCommand& msg);
  void walkingParamCallback(const op3_wholebody_module_msgs::WalkingParam& msg);

  void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void leftFootForceTorqueOutputCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  void rightFootForceTorqueOutputCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);

  /* ROS Service Functions */
  bool getJointPoseCallback(op3_wholebody_module_msgs::GetJointPose::Request &req,
                            op3_wholebody_module_msgs::GetJointPose::Response &res);
  bool getKinematicsPoseCallback(op3_wholebody_module_msgs::GetKinematicsPose::Request &req,
                                 op3_wholebody_module_msgs::GetKinematicsPose::Response &res);
  bool getPreviewMatrix(op3_wholebody_module_msgs::PreviewRequest msg);

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();

  /* yaml Functions */
  void parseBalanceGainData(const std::string &path);
  void parseJointFeedbackGainData(const std::string &path);
  void parseJointFeedforwardGainData(const std::string &path);

  /* ROS Publish Functions */
  void publishStatusMsg(unsigned int type, std::string msg);

  /* Parameter */
  WholebodyControl  *wholebody_control_;
  WalkingControl    *walking_control_;

  OP3Kinematics *op3_kdl_;

private:
  void queueThread();

  void initJointControl();
  void calcJointControl();
  void initWholebodyControl();
  void calcWholebodyControl();
  void initOffsetControl();
  void calcOffsetControl();
  void initWalkingControl();
  void calcWalkingControl();
  void initBalanceControl();
  void calcBalanceControl();

  void initFeedforwardControl();
  void setFeedforwardControl();

  void sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle);

  void calcRobotPose();

  void setTargetForceTorque();
  void setBalanceControlGain();
  bool setBalanceControl();
  void setFeedbackControl();
  void resetBodyPose();

  std::map<std::string, int> joint_name_to_id_;

  double          control_cycle_sec_;
  boost::thread   queue_thread_;
  boost::mutex    queue_mutex_;
  boost::mutex    imu_data_mutex_lock_;

  std_msgs::String movement_done_msg_;

  ros::Publisher  status_msg_pub_;
  ros::Publisher  movement_done_pub_;
  ros::Publisher  goal_joint_state_pub_;

  ros::ServiceClient get_preview_matrix_client_;

  CONTROL_TYPE control_type_;

  bool    is_moving_;
  int     mov_size_, mov_step_;
  double  mov_time_;

  bool goal_initialize_;
  bool joint_control_initialize_;
  bool wholebody_initialize_;
  bool walking_initialize_;
  bool balance_control_initialize_;
  bool body_offset_initialize_;

  int walking_leg_, walking_phase_;
  int walking_size_, walking_step_;

  robotis_framework::MinimumJerk *joint_tra_;
  robotis_framework::MinimumJerk *balance_tra_;
  robotis_framework::MinimumJerk *body_offset_tra_;
  robotis_framework::MinimumJerkViaPoint *feed_forward_tra_;

  size_t number_of_joints_;
  std::vector<std::string> joint_name_;
  std::string wholegbody_control_group_;

  // Joint Command
  std::vector<double_t> curr_joint_accel_, curr_joint_vel_, curr_joint_pos_;
  std::vector<double_t> des_joint_accel_,  des_joint_vel_,  des_joint_pos_;
  std::vector<double_t> goal_joint_accel_, goal_joint_vel_, goal_joint_pos_;

  std::vector<double_t> des_joint_feedback_;
  std::vector<double_t> des_joint_feedforward_;
  std::vector<double_t> des_joint_pos_to_robot_;

  std::vector<double_t> des_l_arm_pos_, des_l_arm_vel_, des_l_arm_accel_, des_l_arm_Q_;
  std::vector<double_t> des_r_arm_pos_, des_r_arm_vel_, des_r_arm_accel_, des_r_arm_Q_;
  std::vector<double_t> des_l_leg_pos_, des_l_leg_vel_, des_l_leg_accel_, des_l_leg_Q_;
  std::vector<double_t> des_r_leg_pos_, des_r_leg_vel_, des_r_leg_accel_, des_r_leg_Q_;
  std::vector<double_t> des_body_pos_,  des_body_vel_,  des_body_accel_,  des_body_Q_;

  // Walking Control
  std::vector<double_t> x_lipm_, y_lipm_;

  op3_wholebody_module_msgs::FootStepCommand foot_step_command_;
  op3_wholebody_module_msgs::PreviewRequest preview_request_;
  op3_wholebody_module_msgs::PreviewResponse preview_response_;
  op3_wholebody_module_msgs::WalkingParam walking_param_;

  // Wholebody Control
  geometry_msgs::Pose wholebody_goal_msg_;

  // Balance Control
  BALANCE_TYPE balance_type_;

  bool  is_balancing_;
  int   balance_step_, balance_size_;

  BalanceControlUsingPDController balance_control_;
  BalancePDController joint_feedback_[12];

  std::vector<double_t> joint_feedforward_gain_;

  std::vector<double_t> des_balance_gain_ratio_;
  std::vector<double_t> goal_balance_gain_ratio_;

  // Body Offset
  std::vector<double_t> des_body_offset_;
  std::vector<double_t> goal_body_offset_;

  bool  is_offset_updating_;
  int   body_offset_step_, body_offset_size_;

  //
  double foot_distance_;

  // Balance Gain
  double foot_roll_gyro_p_gain_;
  double foot_roll_gyro_d_gain_;
  double foot_pitch_gyro_p_gain_;
  double foot_pitch_gyro_d_gain_;

  double foot_roll_angle_p_gain_;
  double foot_roll_angle_d_gain_;
  double foot_pitch_angle_p_gain_;
  double foot_pitch_angle_d_gain_;

  double foot_x_force_p_gain_;
  double foot_x_force_d_gain_;
  double foot_y_force_p_gain_;
  double foot_y_force_d_gain_;
  double foot_z_force_p_gain_;
  double foot_z_force_d_gain_;

  double foot_roll_torque_p_gain_;
  double foot_roll_torque_d_gain_;
  double foot_pitch_torque_p_gain_;
  double foot_pitch_torque_d_gain_;

  double roll_gyro_cut_off_frequency_;
  double pitch_gyro_cut_off_frequency_;

  double roll_angle_cut_off_frequency_;
  double pitch_angle_cut_off_frequency_;

  double foot_x_force_cut_off_frequency_;
  double foot_y_force_cut_off_frequency_;
  double foot_z_force_cut_off_frequency_;

  double foot_roll_torque_cut_off_frequency_;
  double foot_pitch_torque_cut_off_frequency_;

  double balance_hip_roll_gain_;
  double balance_knee_gain_;
  double balance_ankle_roll_gain_;
  double balance_ankle_pitch_gain_;

  // Balance Control : Desired Force
  double balance_l_foot_force_x_;
  double balance_l_foot_force_y_;
  double balance_l_foot_force_z_;
  double balance_l_foot_torque_x_;
  double balance_l_foot_torque_y_;
  double balance_l_foot_torque_z_;

  double balance_r_foot_force_x_;
  double balance_r_foot_force_y_;
  double balance_r_foot_force_z_;
  double balance_r_foot_torque_x_;
  double balance_r_foot_torque_y_;
  double balance_r_foot_torque_z_;

  Eigen::MatrixXd g_to_r_leg_, g_to_l_leg_;

  // Sensor msgs
  sensor_msgs::Imu imu_data_msg_;
  geometry_msgs::Wrench l_foot_ft_data_msg_;
  geometry_msgs::Wrench r_foot_ft_data_msg_;

  double total_mass_;
};

}

#endif
