#include <stdio.h>
#include "op3_wholebody_module/wholebody_control.h"

WholebodyControl::WholebodyControl(std::string control_group,
                                   double init_time, double fin_time,
                                   geometry_msgs::Pose goal_msg)
{
  control_group_ = control_group;

  init_time_ = init_time;
  fin_time_ = fin_time;

  goal_msg_ = goal_msg;

  // Initialization
  init_body_pos_.resize(3, 0.0);
  init_body_vel_.resize(3, 0.0);
  init_body_accel_.resize(3, 0.0);
  des_body_pos_.resize(3, 0.0);
  des_body_vel_.resize(3, 0.0);
  des_body_accel_.resize(3, 0.0);
  goal_body_pos_.resize(3, 0.0);
  goal_body_vel_.resize(3, 0.0);
  goal_body_accel_.resize(3, 0.0);

  init_l_foot_pos_.resize(3, 0.0);
  init_l_foot_vel_.resize(3, 0.0);
  init_l_foot_accel_.resize(3, 0.0);
  des_l_foot_pos_.resize(3, 0.0);
  des_l_foot_vel_.resize(3, 0.0);
  des_l_foot_accel_.resize(3, 0.0);
  goal_l_foot_pos_.resize(3, 0.0);
  goal_l_foot_vel_.resize(3, 0.0);
  goal_l_foot_accel_.resize(3, 0.0);

  init_r_foot_pos_.resize(3, 0.0);
  init_r_foot_vel_.resize(3, 0.0);
  init_r_foot_accel_.resize(3, 0.0);
  des_r_foot_pos_.resize(3, 0.0);
  des_r_foot_vel_.resize(3, 0.0);
  des_r_foot_accel_.resize(3, 0.0);
  goal_r_foot_pos_.resize(3, 0.0);
  goal_r_foot_vel_.resize(3, 0.0);
  goal_r_foot_accel_.resize(3, 0.0);

  goal_task_pos_.resize(3, 0.0);
  goal_task_vel_.resize(3, 0.0);
  goal_task_accel_.resize(3, 0.0);

  goal_task_pos_[0] = goal_msg_.position.x;
  goal_task_pos_[1] = goal_msg_.position.y;
  goal_task_pos_[2] = goal_msg_.position.z;

  Eigen::Quaterniond goal_task_Q(goal_msg_.orientation.w,goal_msg_.orientation.x,
                                 goal_msg_.orientation.y,goal_msg_.orientation.z);
  goal_task_Q_ = goal_task_Q;
}

WholebodyControl::~WholebodyControl()
{

}

void WholebodyControl::initialize(std::vector<double_t> init_body_pos, std::vector<double_t> init_body_rot,
                                  std::vector<double_t> init_r_foot_pos, std::vector<double_t> init_r_foot_Q,
                                  std::vector<double_t> init_l_foot_pos, std::vector<double_t> init_l_foot_Q)
{
  init_body_pos_ = init_body_pos;
  des_body_pos_ = init_body_pos;

  Eigen::Quaterniond body_Q(init_body_rot[3],init_body_rot[0],init_body_rot[1],init_body_rot[2]);
  init_body_Q_ = body_Q;
  des_body_Q_ = body_Q;

  init_r_foot_pos_ = init_r_foot_pos;
  init_l_foot_pos_ = init_l_foot_pos;

  des_l_foot_pos_ = init_l_foot_pos_;
  des_r_foot_pos_ = init_r_foot_pos_;

  Eigen::Quaterniond l_foot_Q(init_l_foot_Q[3],init_l_foot_Q[0],init_l_foot_Q[1],init_l_foot_Q[2]);
  init_l_foot_Q_ = l_foot_Q;
  des_l_foot_Q_ = l_foot_Q;

  Eigen::Quaterniond r_foot_Q(init_r_foot_Q[3],init_r_foot_Q[0],init_r_foot_Q[1],init_r_foot_Q[2]);
  init_r_foot_Q_ = r_foot_Q;
  des_r_foot_Q_ = r_foot_Q;

  if (control_group_ == "body")
  {
    task_trajectory_ =
        new robotis_framework::MinimumJerk(init_time_, fin_time_,
                                           init_body_pos_, init_body_vel_, init_body_accel_,
                                           goal_task_pos_, goal_task_vel_, goal_task_accel_);
    init_task_Q_ = body_Q;
  }
  else if (control_group_ == "right_leg")
  {
    task_trajectory_ =
        new robotis_framework::MinimumJerk(init_time_, fin_time_,
                                           init_r_foot_pos_, init_r_foot_vel_, init_r_foot_accel_,
                                           goal_task_pos_, goal_task_vel_, goal_task_accel_);
    init_task_Q_ = r_foot_Q;
  }
  else if (control_group_ == "left_leg")
  {
    task_trajectory_ =
        new robotis_framework::MinimumJerk(init_time_, fin_time_,
                                           init_l_foot_pos_, init_l_foot_vel_, init_l_foot_accel_,
                                           goal_task_pos_, goal_task_vel_, goal_task_accel_);
    init_task_Q_ = l_foot_Q;
  }
}

//void WholebodyControl::update(std::vector<double_t> init_body_pos, std::vector<double_t> init_body_rot,
//                              std::vector<double_t> init_task_pos, std::vector<double_t> init_task_vel, std::vector<double_t> init_task_accel)
//{
//  robot_->op3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = init_body_pos[0];
//  robot_->op3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = init_body_pos[1];
//  robot_->op3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = init_body_pos[2];

//  Eigen::Quaterniond init_body_quat(init_body_rot[3],init_body_rot[0],init_body_rot[1],init_body_rot[2]);
//  Eigen::MatrixXd init_body_rpy = robotis_framework::convertQuaternionToRPY(init_body_quat);

//  robot_->op3_link_data_[ID_PELVIS_ROT_X]->joint_angle_  = init_body_rpy.coeff(0,0);
//  robot_->op3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = init_body_rpy.coeff(1,0);
//  robot_->op3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_   = init_body_rpy.coeff(2,0);

//  for (int id=1; id<=MAX_JOINT_ID; id++)
//    robot_->op3_link_data_[id]->joint_angle_ = init_joint_pos_[id-1];

//  robot_->calcForwardKinematics(0);

//  if (control_group_ == "body")
//    end_link_ = ID_PELVIS;
//  else if (control_group_ == "right_leg")
//    end_link_ = ID_R_LEG_END;
//  else if (control_group_ == "left_leg")
//    end_link_ = ID_L_LEG_END;
//  else
//    end_link_ = ID_BASE;

//  Eigen::MatrixXd rot = robot_->op3_link_data_[end_link_]->orientation_;
//  init_task_quaternion_ = robotis_framework::convertRotationToQuaternion(rot);

//  init_task_pos_ = init_task_pos;
//  init_task_vel_ = init_task_vel;
//  init_task_accel_ = init_task_accel;

//  task_trajectory_ =
//      new robotis_framework::MinimumJerk(init_time_, fin_time_,
//                                         init_task_pos_, init_task_vel_, init_task_accel_,
//                                         goal_task_pos_, goal_task_vel_, goal_task_accel_);

//  init_body_pos_ = robot_->op3_link_data_[ID_PELVIS]->position_;
//  init_body_rot_ = robot_->op3_link_data_[ID_PELVIS]->orientation_;

//  init_rleg_pos_ = robot_->op3_link_data_[ID_R_LEG_END]->position_;
//  init_rleg_rot_ = robot_->op3_link_data_[ID_R_LEG_END]->orientation_;

//  init_lleg_pos_ = robot_->op3_link_data_[ID_L_LEG_END]->position_;
//  init_lleg_rot_ = robot_->op3_link_data_[ID_L_LEG_END]->orientation_;
//}

void WholebodyControl::finalize()
{
  delete task_trajectory_;
}

void WholebodyControl::set(double time)
{
  std::vector<double_t> des_task_pos = task_trajectory_->getPosition(time);

//  Eigen::MatrixXd ik_pos = Eigen::MatrixXd::Zero(3,1);
//  for (int i=0; i<3; i++)
//    ik_pos.coeffRef(i,0) = desired_task_pos[i];

  double count = time / fin_time_;
  des_task_Q_ = init_task_Q_.slerp(count, goal_task_Q_);

//  Eigen::MatrixXd ik_rot = robotis_framework::convertQuaternionToRotation(ik_quat);

//  desired_task_quaternion_ = ik_quat;

//  int     max_iter    = 30;
//  double  ik_tol      = 1e-5;

//  bool ik_success = false;
//  bool ik_success = true;

  if (control_group_ == "left_leg")
  {
    des_body_pos_ = init_body_pos_;
    des_body_Q_ = init_body_Q_;

    des_l_foot_pos_ = des_task_pos;
    des_l_foot_Q_ = des_task_Q_;

    des_r_foot_pos_ = init_r_foot_pos_;
    des_r_foot_Q_ = init_r_foot_Q_;
  }
  else if (control_group_ == "right_leg")
  {
    des_body_pos_ = init_body_pos_;
    des_body_Q_ = init_body_Q_;

    des_l_foot_pos_ = init_l_foot_pos_;
    des_l_foot_Q_ = init_l_foot_Q_;

    des_r_foot_pos_ = des_task_pos;
    des_r_foot_Q_ = des_task_Q_;

//    ik_success = robot_->calcInverseKinematics(ID_PELVIS, end_link_,
//                                               ik_pos, ik_rot,
//                                               max_iter, ik_tol);
  }
  else if (control_group_ == "body")
  {
    des_body_pos_ = des_task_pos;
    des_body_Q_ = des_task_Q_;

    des_l_foot_pos_ = init_l_foot_pos_;
    des_l_foot_Q_ = init_l_foot_Q_;

    des_r_foot_pos_ = init_r_foot_pos_;
    des_r_foot_Q_ = init_r_foot_Q_;

//    robot_->op3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = ik_pos.coeff(0,0);
//    robot_->op3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = ik_pos.coeff(1,0);
//    robot_->op3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = ik_pos.coeff(2,0);

//    Eigen::MatrixXd ik_rpy = robotis_framework::convertQuaternionToRPY(ik_quat);

//    robot_->op3_link_data_[ID_PELVIS_ROT_X]->joint_angle_ = ik_rpy.coeff(0,0);
//    robot_->op3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = ik_rpy.coeff(1,0);
//    robot_->op3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_ = ik_rpy.coeff(2,0);

//    bool ik_rleg_success = robot_->calcInverseKinematics(ID_PELVIS, ID_R_LEG_END,
//                                                         init_rleg_pos_, init_rleg_rot_,
//                                                         max_iter, ik_tol);

//    bool ik_lleg_success = robot_->calcInverseKinematics(ID_PELVIS, ID_L_LEG_END,
//                                                         init_lleg_pos_, init_lleg_rot_,
//                                                         max_iter, ik_tol);

//    if (ik_rleg_success == true && ik_lleg_success == true)
//      ik_success = true;
//    else
//      ik_success = false;
  }

//  return ik_success;
}

std::vector<double_t> WholebodyControl::getJointPosition(double time)
{

}

std::vector<double_t> WholebodyControl::getJointVelocity(double time)
{

}

std::vector<double_t> WholebodyControl::getJointAcceleration(double time)
{

}

void WholebodyControl::getTaskPosition(std::vector<double_t> &l_foot_pos,
                                       std::vector<double_t> &r_foot_pos,
                                       std::vector<double_t> &body_pos)
{
  l_foot_pos = des_l_foot_pos_;
  r_foot_pos = des_r_foot_pos_;
  body_pos   = des_body_pos_;
}

std::vector<double_t> WholebodyControl::getTaskVelocity(double time)
{

}

std::vector<double_t> WholebodyControl::getTaskAcceleration(double time)
{

}

void WholebodyControl::getTaskOrientation(std::vector<double_t> &l_foot_Q,
                                          std::vector<double_t> &r_foot_Q,
                                          std::vector<double_t> &body_Q)
{
  l_foot_Q[0] = des_l_foot_Q_.x();
  l_foot_Q[1] = des_l_foot_Q_.y();
  l_foot_Q[2] = des_l_foot_Q_.z();
  l_foot_Q[3] = des_l_foot_Q_.w();

  r_foot_Q[0] = des_r_foot_Q_.x();
  r_foot_Q[1] = des_r_foot_Q_.y();
  r_foot_Q[2] = des_r_foot_Q_.z();
  r_foot_Q[3] = des_r_foot_Q_.w();

  body_Q[0] = des_body_Q_.x();
  body_Q[1] = des_body_Q_.y();
  body_Q[2] = des_body_Q_.z();
  body_Q[3] = des_body_Q_.w();
}
void WholebodyControl::getGroupPose(std::string name, geometry_msgs::Pose *msg)
{

}
