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

#include <stdio.h>
#include "op3_online_walking_module/op3_kdl.h"

OP3Kinematics::OP3Kinematics()
{
  rleg_joint_position_.resize(LEG_JOINT_NUM);

  for (int i=0; i<LEG_JOINT_NUM; i++)
    rleg_joint_position_(i) = 0.0;
}

OP3Kinematics::~OP3Kinematics()
{

}

void OP3Kinematics::initialize(Eigen::MatrixXd pelvis_position, Eigen::MatrixXd pelvis_orientation)
{
  KDL::Chain rleg_chain, lleg_chain;

  double pelvis_x = pelvis_position.coeff(0,0);
  double pelvis_y = pelvis_position.coeff(1,0);
  double pelvis_z = pelvis_position.coeff(2,0);

  double pelvis_Xx = pelvis_orientation.coeff(0,0);
  double pelvis_Yx = pelvis_orientation.coeff(0,1);
  double pelvis_Zx = pelvis_orientation.coeff(0,2);

  double pelvis_Xy = pelvis_orientation.coeff(1,0);
  double pelvis_Yy = pelvis_orientation.coeff(1,1);
  double pelvis_Zy = pelvis_orientation.coeff(1,2);

  double pelvis_Xz = pelvis_orientation.coeff(2,0);
  double pelvis_Yz = pelvis_orientation.coeff(2,1);
  double pelvis_Zz = pelvis_orientation.coeff(2,2);

  // Set Kinematics Tree

  // Right Leg Chain
  rleg_chain.addSegment(KDL::Segment("base",
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Rotation(pelvis_Xx, pelvis_Yx, pelvis_Zx,
                                                              pelvis_Xy, pelvis_Yy, pelvis_Zy,
                                                              pelvis_Xz, pelvis_Yz, pelvis_Zz),
                                                KDL::Vector(pelvis_x , pelvis_y , pelvis_z)),
                                     KDL::RigidBodyInertia(0.0,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain.addSegment(KDL::Segment("pelvis",
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Vector(0.0, -0.035, -0.0907)),
//                                     KDL::Frame(KDL::Vector(-0.005, -0.035, -0.0907)),
                                     KDL::RigidBodyInertia(0.72235,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain.addSegment(KDL::Segment("r_hip_yaw",
                                     KDL::Joint("minus_RotZ", KDL::Vector(0,0,0), KDL::Vector(0,0,-1), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.000, 0.000, -0.0285)),
                                     KDL::RigidBodyInertia(0.01181,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain.addSegment(KDL::Segment("r_leg_hip_r",
                                     KDL::Joint("minus_RotX", KDL::Vector(0,0,0), KDL::Vector(-1,0,0), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.17886,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain.addSegment(KDL::Segment("r_leg_hip_p",
                                     KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, -0.11)),
                                     KDL::RigidBodyInertia(0.11543,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain.addSegment(KDL::Segment("r_leg_kn_p",
                                     KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, -0.11)),
                                     KDL::RigidBodyInertia(0.04015,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain.addSegment(KDL::Segment("r_leg_an_p",
                                     KDL::Joint(KDL::Joint::RotY),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.17886,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain.addSegment(KDL::Segment("r_leg_an_r",
                                     KDL::Joint(KDL::Joint::RotX),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, -0.0305)),
                                     KDL::RigidBodyInertia(0.06934,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  rleg_chain.addSegment(KDL::Segment("r_leg_end",
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Vector(0.0 , 0.0 , 0.0)),
                                     KDL::RigidBodyInertia(0.0,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );

  // Left Leg Chain
  lleg_chain.addSegment(KDL::Segment("base",
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Rotation(pelvis_Xx, pelvis_Yx, pelvis_Zx,
                                                              pelvis_Xy, pelvis_Yy, pelvis_Zy,
                                                              pelvis_Xz, pelvis_Yz, pelvis_Zz),
                                                KDL::Vector(pelvis_x , pelvis_y , pelvis_z)),
                                     KDL::RigidBodyInertia(0.0,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain.addSegment(KDL::Segment("pelvis",
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Vector(0.0, 0.035, -0.0907)),
//                                     KDL::Frame(KDL::Vector(-0.005, 0.035, -0.0907)),
                                     KDL::RigidBodyInertia(0.72235,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain.addSegment(KDL::Segment("l_leg_hip_y",
                                     KDL::Joint("minus_RotZ", KDL::Vector(0,0,0), KDL::Vector(0,0,-1), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.000, 0.000, -0.0285)),
                                     KDL::RigidBodyInertia(0.01181,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain.addSegment(KDL::Segment("l_leg_hip_r",
                                     KDL::Joint("minus_RotX", KDL::Vector(0,0,0), KDL::Vector(-1,0,0), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.17886,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain.addSegment(KDL::Segment("l_leg_hip_p",
                                     KDL::Joint(KDL::Joint::RotY),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, -0.11)),
                                     KDL::RigidBodyInertia(0.11543,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain.addSegment(KDL::Segment("l_leg_kn_p",
                                     KDL::Joint(KDL::Joint::RotY),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, -0.11)),
                                     KDL::RigidBodyInertia(0.04015,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain.addSegment(KDL::Segment("l_leg_an_p",
                                     KDL::Joint("minus_RotY", KDL::Vector(0,0,0), KDL::Vector(0,-1,0), KDL::Joint::RotAxis),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.17886,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain.addSegment(KDL::Segment("l_leg_an_r",
                                     KDL::Joint(KDL::Joint::RotX),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, -0.0305)),
                                     KDL::RigidBodyInertia(0.06934,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  lleg_chain.addSegment(KDL::Segment("l_leg_end",
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Vector(0.0 , 0.0 , 0.0)),
                                     KDL::RigidBodyInertia(0.0,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );

  // Set Joint Limits
  std::vector<double> min_position_limit, max_position_limit;
  min_position_limit.push_back(-180.0); max_position_limit.push_back(180.0); // r_leg_hip_y
  min_position_limit.push_back(-180.0);	max_position_limit.push_back(180.0); // r_leg_hip_r
  min_position_limit.push_back(-180.0); max_position_limit.push_back(180.0); // r_leg_hip_p
  min_position_limit.push_back(-180.0);	max_position_limit.push_back(180.0); // r_leg_kn_p
  min_position_limit.push_back(-180.0);	max_position_limit.push_back(180.0); // r_leg_an_p
  min_position_limit.push_back(-180.0);	max_position_limit.push_back(180.0); // r_leg_an_r

  KDL::JntArray min_joint_position_limit(LEG_JOINT_NUM), max_joint_position_limit(LEG_JOINT_NUM);
  for (int index=0; index<LEG_JOINT_NUM; index++)
  {
    min_joint_position_limit(index) = min_position_limit[index]*D2R;
    max_joint_position_limit(index) = max_position_limit[index]*D2R;
  }

  /* KDL Solver Initialization */
  //  rleg_dyn_param_ = new KDL::ChainDynParam(rleg_chain_, KDL::Vector(0.0, 0.0, -9.81)); // kinematics & dynamics parameter
  //  rleg_jacobian_solver_ = new KDL::ChainJntToJacSolver(rleg_chain__); // jabocian solver
  rleg_fk_solver_ = new KDL::ChainFkSolverPos_recursive(rleg_chain); // forward kinematics solver

  // inverse kinematics solver
  rleg_ik_vel_solver_ = new KDL::ChainIkSolverVel_pinv(rleg_chain);
  rleg_ik_pos_solver_ = new KDL::ChainIkSolverPos_NR_JL(rleg_chain,
                                                        min_joint_position_limit, max_joint_position_limit,
                                                        *rleg_fk_solver_,
                                                        *rleg_ik_vel_solver_);

  //  lleg_dyn_param_ = new KDL::ChainDynParam(lleg_chain_, KDL::Vector(0.0, 0.0, -9.81)); // kinematics & dynamics parameter
  //  lleg_jacobian_solver_ = new KDL::ChainJntToJacSolver(lleg_chain__); // jabocian solver
  lleg_fk_solver_ = new KDL::ChainFkSolverPos_recursive(lleg_chain); // forward kinematics solver

  // inverse kinematics solver
  lleg_ik_vel_solver_ = new KDL::ChainIkSolverVel_pinv(lleg_chain);
  lleg_ik_pos_solver_ = new KDL::ChainIkSolverPos_NR_JL(lleg_chain,
                                                        min_joint_position_limit, max_joint_position_limit,
                                                        *lleg_fk_solver_,
                                                        *lleg_ik_vel_solver_);
}

void OP3Kinematics::setJointPosition(Eigen::VectorXd rleg_joint_position, Eigen::VectorXd lleg_joint_position)
{
  rleg_joint_position_ = rleg_joint_position;
  lleg_joint_position_ = lleg_joint_position;

  //  for (int i=0; i<LEG_JOINT_NUM; i++)
  //    ROS_INFO("rleg_joint_position_(%d): %f", i, rleg_joint_position_(i));

  //  for (int i=0; i<LEG_JOINT_NUM; i++)
  //    ROS_INFO("lleg_joint_position_(%d): %f", i, lleg_joint_position_(i));
}

void OP3Kinematics::solveForwardKinematics(std::vector<double_t> &rleg_position, std::vector<double_t> &rleg_orientation,
                                                 std::vector<double_t> &lleg_position, std::vector<double_t> &lleg_orientation)
{
  // rleg
  KDL::JntArray rleg_joint_position;
  rleg_joint_position.data = rleg_joint_position_;

  KDL::Frame rleg_pose;
  rleg_fk_solver_->JntToCart(rleg_joint_position, rleg_pose);

  rleg_pose_.position.x = rleg_pose.p.x();
  rleg_pose_.position.y = rleg_pose.p.y();
  rleg_pose_.position.z = rleg_pose.p.z();

  rleg_pose.M.GetQuaternion(rleg_pose_.orientation.x,
                            rleg_pose_.orientation.y,
                            rleg_pose_.orientation.z,
                            rleg_pose_.orientation.w);

//  ROS_INFO("rleg position x : %f y: %f, z: %f", rleg_pose_.position.x, rleg_pose_.position.y, rleg_pose_.position.z);

  rleg_position.resize(3,0.0);
  rleg_position[0] = rleg_pose_.position.x;
  rleg_position[1] = rleg_pose_.position.y;
  rleg_position[2] = rleg_pose_.position.z;

  rleg_orientation.resize(4,0.0);
  rleg_orientation[0] = rleg_pose_.orientation.x;
  rleg_orientation[1] = rleg_pose_.orientation.y;
  rleg_orientation[2] = rleg_pose_.orientation.z;
  rleg_orientation[3] = rleg_pose_.orientation.w;

  // lleg
  KDL::JntArray lleg_joint_position;
  lleg_joint_position.data = lleg_joint_position_;

  KDL::Frame lleg_pose;
  lleg_fk_solver_->JntToCart(lleg_joint_position, lleg_pose);

  lleg_pose_.position.x = lleg_pose.p.x();
  lleg_pose_.position.y = lleg_pose.p.y();
  lleg_pose_.position.z = lleg_pose.p.z();

  lleg_pose.M.GetQuaternion(lleg_pose_.orientation.x,
                            lleg_pose_.orientation.y,
                            lleg_pose_.orientation.z,
                            lleg_pose_.orientation.w);

//  ROS_INFO("lleg position x : %f y: %f, z: %f", lleg_pose_.position.x, lleg_pose_.position.y, lleg_pose_.position.z);

  lleg_position.resize(3,0.0);
  lleg_position[0] = lleg_pose_.position.x;
  lleg_position[1] = lleg_pose_.position.y;
  lleg_position[2] = lleg_pose_.position.z;

  lleg_orientation.resize(4,0.0);
  lleg_orientation[0] = lleg_pose_.orientation.x;
  lleg_orientation[1] = lleg_pose_.orientation.y;
  lleg_orientation[2] = lleg_pose_.orientation.z;
  lleg_orientation[3] = lleg_pose_.orientation.w;
}

bool OP3Kinematics::solveInverseKinematics(std::vector<double_t> &rleg_output,
                                                 Eigen::MatrixXd rleg_target_position, Eigen::Quaterniond rleg_target_orientation,
                                                 std::vector<double_t> &lleg_output,
                                                 Eigen::MatrixXd lleg_target_position, Eigen::Quaterniond lleg_target_orientation)
{
  //  ROS_INFO("right x: %f, y: %f, z: %f", rleg_target_position(0), rleg_target_position(1), rleg_target_position(2));
  //  ROS_INFO("left x: %f, y: %f, z: %f", lleg_target_position(0), lleg_target_position(1), lleg_target_position(2));

  // rleg
  KDL::JntArray rleg_joint_position;
  rleg_joint_position.data = rleg_joint_position_;

  KDL::Frame rleg_desired_pose;
  rleg_desired_pose.p.x(rleg_target_position.coeff(0,0));
  rleg_desired_pose.p.y(rleg_target_position.coeff(1,0));
  rleg_desired_pose.p.z(rleg_target_position.coeff(2,0));

  rleg_desired_pose.M = KDL::Rotation::Quaternion(rleg_target_orientation.x(),
                                                  rleg_target_orientation.y(),
                                                  rleg_target_orientation.z(),
                                                  rleg_target_orientation.w());

  KDL::JntArray rleg_desired_joint_position;
  rleg_desired_joint_position.resize(LEG_JOINT_NUM);

  int rleg_err = rleg_ik_pos_solver_->CartToJnt(rleg_joint_position, rleg_desired_pose, rleg_desired_joint_position);

  if (rleg_err < 0)
  {
    ROS_WARN("RLEG IK ERR : %d", rleg_err);
    return false;
  }

  // lleg
  KDL::JntArray lleg_joint_position;
  lleg_joint_position.data = lleg_joint_position_;

  KDL::Frame lleg_desired_pose;
  lleg_desired_pose.p.x(lleg_target_position.coeff(0,0));
  lleg_desired_pose.p.y(lleg_target_position.coeff(1,0));
  lleg_desired_pose.p.z(lleg_target_position.coeff(2,0));

  lleg_desired_pose.M = KDL::Rotation::Quaternion(lleg_target_orientation.x(),
                                                  lleg_target_orientation.y(),
                                                  lleg_target_orientation.z(),
                                                  lleg_target_orientation.w());

  KDL::JntArray lleg_desired_joint_position;
  lleg_desired_joint_position.resize(LEG_JOINT_NUM);

  int lleg_err = lleg_ik_pos_solver_->CartToJnt(lleg_joint_position, lleg_desired_pose, lleg_desired_joint_position);

  if (lleg_err < 0)
  {
    ROS_WARN("LLEG IK ERR : %d", lleg_err);
    return false;
  }

  // output
  rleg_output.resize(LEG_JOINT_NUM);
  lleg_output.resize(LEG_JOINT_NUM);

  for (int i=0; i<LEG_JOINT_NUM; i++)
  {
    rleg_output[i] = rleg_desired_joint_position(i);
    lleg_output[i] = lleg_desired_joint_position(i);
  }

  return true;
}

void OP3Kinematics::finalize()
{
  //  delete rleg_chain_;
  //  delete rleg_dyn_param_;
  //  delete rleg_jacobian_solver_;
  delete rleg_fk_solver_;
  delete rleg_ik_vel_solver_;
  delete rleg_ik_pos_solver_;

  //  delete lleg_chain_;
  //  delete lleg_dyn_param_;
  //  delete lleg_jacobian_solver_;
  delete lleg_fk_solver_;
  delete lleg_ik_vel_solver_;
  delete lleg_ik_pos_solver_;
}
