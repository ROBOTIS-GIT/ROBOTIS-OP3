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

#include "op3_localization/op3_localization.h"

namespace robotis_op
{

OP3Localization::OP3Localization()
 : ros_node_(),
   transform_tolerance_(0.0),
   err_tol_(0.2),
   is_moving_walking_(false)
{
  initialize();

  pelvis_pose_base_walking_.pose.position.x = 0.0;
  pelvis_pose_base_walking_.pose.position.y = 0.0;
  pelvis_pose_base_walking_.pose.position.z = 0.0;
  pelvis_pose_base_walking_.pose.orientation.x = 0.0;
  pelvis_pose_base_walking_.pose.orientation.y = 0.0;
  pelvis_pose_base_walking_.pose.orientation.z = 0.0;
  pelvis_pose_base_walking_.pose.orientation.w = 1.0;

  pelvis_pose_offset_.pose.position.x = 0.0;
  pelvis_pose_offset_.pose.position.y = 0.0;
  pelvis_pose_offset_.pose.position.z = 0.2495256; //0.3402256 - 0.0907;
  pelvis_pose_offset_.pose.orientation.x = 0.0;
  pelvis_pose_offset_.pose.orientation.y = 0.0;
  pelvis_pose_offset_.pose.orientation.z = 0.0;
  pelvis_pose_offset_.pose.orientation.w = 1.0;

  pelvis_pose_old_.pose.position.x = 0.0;
  pelvis_pose_old_.pose.position.y = 0.0;
  pelvis_pose_old_.pose.position.z = 0.0;
  pelvis_pose_old_.pose.orientation.x = 0.0;
  pelvis_pose_old_.pose.orientation.y = 0.0;
  pelvis_pose_old_.pose.orientation.z = 0.0;
  pelvis_pose_old_.pose.orientation.w = 1.0;

  update();
}

OP3Localization::~OP3Localization()
{

}

void OP3Localization::initialize()
{
  // subscriber
  pelvis_pose_msg_sub_ = ros_node_.subscribe("/robotis/pelvis_pose", 5,
                                             &OP3Localization::pelvisPoseCallback, this);
//  pelvis_base_walking_msg_sub_ = ros_node_.subscribe("/robotis/pelvis_pose_base_walking", 5,
//                                                               &OP3Localization::pelvisPoseBaseWalkingCallback, this);

  pelvis_reset_msg_sub_ = ros_node_.subscribe("/robotis/pelvis_pose_reset", 5,
                                                       &OP3Localization::pelvisPoseResetCallback, this);

}

void OP3Localization::pelvisPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  mutex_.lock();

  pelvis_pose_offset_ = *msg;
  pelvis_pose_.header.stamp = pelvis_pose_offset_.header.stamp;

  mutex_.unlock();
}

//void OP3Localization::pelvisPoseBaseWalkingCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
//{
//  mutex_.lock();

//  Eigen::Quaterniond msg_q;
//  tf::quaternionMsgToEigen(msg->pose.orientation, msg_q);

//  Eigen::MatrixXd rpy = robotis_framework::convertQuaternionToRPY(msg_q);
//  double yaw = rpy.coeff(2,0);

//  if ( fabs(msg->pose.position.x) <= 1e-3 &&
//       fabs(msg->pose.position.y) <= 1e-3 &&
//       fabs(yaw) <= 1e-3 )
//  {
//    if (is_moving_walking_ == true)
//    {
//      ROS_INFO("Walking Pelvis Pose Update");
//      pelvis_pose_old_.pose.position.x += pelvis_pose_base_walking_new_.pose.position.x;
//      pelvis_pose_old_.pose.position.y += pelvis_pose_base_walking_new_.pose.position.y;

//      Eigen::Quaterniond pose_old_quaternion(pelvis_pose_old_.pose.orientation.w,
//                                             pelvis_pose_old_.pose.orientation.x,
//                                             pelvis_pose_old_.pose.orientation.y,
//                                             pelvis_pose_old_.pose.orientation.z);

//      Eigen::Quaterniond pose_base_quaternion(pelvis_pose_base_walking_.pose.orientation.w,
//                                              pelvis_pose_base_walking_.pose.orientation.x,
//                                              pelvis_pose_base_walking_.pose.orientation.y,
//                                              pelvis_pose_base_walking_.pose.orientation.z);

//      Eigen::Quaterniond q = pose_old_quaternion * pose_base_quaternion;
//      tf::quaternionEigenToMsg(q, pelvis_pose_old_.pose.orientation);

//      is_moving_walking_ = false;
//    }
//  }
//  else
//  {
//    is_moving_walking_ = true;
//  }

//  pelvis_pose_base_walking_ = *msg;
//  pelvis_pose_.header.stamp = pelvis_pose_base_walking_.header.stamp;

//  mutex_.unlock();
//}

void OP3Localization::pelvisPoseResetCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "reset")
  {
    ROS_INFO("Pelvis Pose Reset");

    pelvis_pose_old_.pose.position.x = 0.0;
    pelvis_pose_old_.pose.position.y = 0.0;
    pelvis_pose_old_.pose.orientation.x = 0.0;
    pelvis_pose_old_.pose.orientation.y = 0.0;
    pelvis_pose_old_.pose.orientation.z = 0.0;
    pelvis_pose_old_.pose.orientation.w = 1.0;
  }
}

void OP3Localization::process()
{
  update();

  pelvis_trans_.setOrigin(tf::Vector3(pelvis_pose_.pose.position.x,
                                      pelvis_pose_.pose.position.y,
                                      pelvis_pose_.pose.position.z)
                          );

  tf::Quaternion q(pelvis_pose_.pose.orientation.x,
                   pelvis_pose_.pose.orientation.y,
                   pelvis_pose_.pose.orientation.z,
                   pelvis_pose_.pose.orientation.w);

  pelvis_trans_.setRotation(q);

  ros::Duration transform_tolerance(transform_tolerance_);
  ros::Time transform_expiration = (pelvis_pose_.header.stamp + transform_tolerance);

  tf::StampedTransform tmp_tf_stamped(pelvis_trans_, transform_expiration, "world", "body_link");

  broadcaster_.sendTransform(tmp_tf_stamped);
}

void OP3Localization::update()
{
  mutex_.lock();

  Eigen::Quaterniond pose_old_quaternion(pelvis_pose_old_.pose.orientation.w,
                                         pelvis_pose_old_.pose.orientation.x,
                                         pelvis_pose_old_.pose.orientation.y,
                                         pelvis_pose_old_.pose.orientation.z);

//  Eigen::Quaterniond pose_base_walking_quaternion(pelvis_pose_base_walking_.pose.orientation.w,
//                                                  pelvis_pose_base_walking_.pose.orientation.x,
//                                                  pelvis_pose_base_walking_.pose.orientation.y,
//                                                  pelvis_pose_base_walking_.pose.orientation.z);

  Eigen::Quaterniond pose_offset_quaternion(pelvis_pose_offset_.pose.orientation.w,
                                            pelvis_pose_offset_.pose.orientation.x,
                                            pelvis_pose_offset_.pose.orientation.y,
                                            pelvis_pose_offset_.pose.orientation.z);

  Eigen::Quaterniond pose_quaternion =
      pose_old_quaternion *
//      pose_base_walking_quaternion *
      pose_offset_quaternion;

//  Eigen::MatrixXd position_walking = Eigen::MatrixXd::Zero(3,1);
//  position_walking.coeffRef(0,0) =
//      pelvis_pose_base_walking_.pose.position.x;
//  position_walking.coeffRef(1,0) =
//      pelvis_pose_base_walking_.pose.position.y;

  Eigen::MatrixXd position_offset = Eigen::MatrixXd::Zero(3,1);
  position_offset.coeffRef(0,0) =
      pelvis_pose_offset_.pose.position.x;
  position_offset.coeffRef(1,0) =
      pelvis_pose_offset_.pose.position.y;

  Eigen::MatrixXd orientation = robotis_framework::convertQuaternionToRotation(pose_old_quaternion);
//  Eigen::MatrixXd position_walking_new = orientation * position_walking;
  Eigen::MatrixXd position_offset_new = orientation * position_offset;

//  pelvis_pose_base_walking_new_.pose.position.x = position_walking_new.coeff(0,0);
//  pelvis_pose_base_walking_new_.pose.position.y = position_walking_new.coeff(1,0);

  pelvis_pose_offset_new_.pose.position.x = position_offset_new.coeff(0,0);
  pelvis_pose_offset_new_.pose.position.y = position_offset_new.coeff(1,0);

  pelvis_pose_.pose.position.x =
      pelvis_pose_old_.pose.position.x +
//      pelvis_pose_base_walking_new_.pose.position.x +
      pelvis_pose_offset_new_.pose.position.x;

  pelvis_pose_.pose.position.y =
      pelvis_pose_old_.pose.position.y +
//      pelvis_pose_base_walking_new_.pose.position.y +
      pelvis_pose_offset_new_.pose.position.y;

  pelvis_pose_.pose.position.z =
      pelvis_pose_offset_.pose.position.z;

  tf::quaternionEigenToMsg(pose_quaternion, pelvis_pose_.pose.orientation);

  mutex_.unlock();
}

} // namespace robotis_op
