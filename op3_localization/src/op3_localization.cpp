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
#include "robotis_math/robotis_math.h"

namespace robotis_op
{

OP3Localization::OP3Localization()
 : Node("op3_localization"),
   transform_tolerance_(0.0),
   err_tol_(0.2),
   is_moving_walking_(false)
{
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
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
  pelvis_pose_msg_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/robotis/pelvis_pose", 5, std::bind(&OP3Localization::pelvisPoseCallback, this, std::placeholders::_1));
//  pelvis_base_walking_msg_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//      "/robotis/pelvis_pose_base_walking", 5, std::bind(&OP3Localization::pelvisPoseBaseWalkingCallback, this, std::placeholders::_1));

  pelvis_reset_msg_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/robotis/pelvis_pose_reset", 5, std::bind(&OP3Localization::pelvisPoseResetCallback, this, std::placeholders::_1));

}

void OP3Localization::pelvisPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  pelvis_pose_offset_ = *msg;
  pelvis_pose_.header.stamp = pelvis_pose_offset_.header.stamp;
}

//void OP3Localization::pelvisPoseBaseWalkingCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//{
//  std::lock_guard<std::mutex> lock(mutex_);

//  Eigen::Quaterniond msg_q;
//  tf2::fromMsg(msg->pose.orientation, msg_q);

//  Eigen::MatrixXd rpy = robotis_framework::convertQuaternionToRPY(msg_q);
//  double yaw = rpy.coeff(2,0);

//  if ( fabs(msg->pose.position.x) <= 1e-3 &&
//       fabs(msg->pose.position.y) <= 1e-3 &&
//       fabs(yaw) <= 1e-3 )
//  {
//    if (is_moving_walking_ == true)
//    {
//      RCLCPP_INFO(this->get_logger(), "Walking Pelvis Pose Update");
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
//      pelvis_pose_old_.pose.orientation = tf2::toMsg(q);

//      is_moving_walking_ = false;
//    }
//  }
//  else
//  {
//    is_moving_walking_ = true;
//  }

//  pelvis_pose_base_walking_ = *msg;
//  pelvis_pose_.header.stamp = pelvis_pose_base_walking_.header.stamp;
//}

void OP3Localization::pelvisPoseResetCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "reset")
  {
   RCLCPP_INFO(this->get_logger(), "Pelvis Pose Reset");

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

  pelvis_trans_.transform.translation.x = pelvis_pose_.pose.position.x;
  pelvis_trans_.transform.translation.y = pelvis_pose_.pose.position.y;
  pelvis_trans_.transform.translation.z = pelvis_pose_.pose.position.z;

  pelvis_trans_.transform.rotation = pelvis_pose_.pose.orientation;

  rclcpp::Duration transform_tolerance = rclcpp::Duration::from_seconds(transform_tolerance_);
  rclcpp::Time transform_expiration = (pelvis_pose_.header.stamp + transform_tolerance);

  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.stamp = transform_expiration;
  tmp_tf_stamped.header.frame_id = "world";
  tmp_tf_stamped.child_frame_id = "body_link";
  tmp_tf_stamped.transform.translation.x = pelvis_trans_.transform.translation.x;
  tmp_tf_stamped.transform.translation.y = pelvis_trans_.transform.translation.y;
  tmp_tf_stamped.transform.translation.z = pelvis_trans_.transform.translation.z;
  tmp_tf_stamped.transform.rotation = pelvis_trans_.transform.rotation;

  broadcaster_->sendTransform(tmp_tf_stamped);
}

void OP3Localization::update()
{
  std::lock_guard<std::mutex> lock(mutex_);

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

  pelvis_pose_.pose.orientation = tf2::toMsg(pose_quaternion);
}

} // namespace robotis_op
