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

#ifndef OP3_LOCALIZATION_H_
#define OP3_LOCALIZATION_H_

// std
#include <string>

// ros dependencies
#include <rclcpp/rclcpp.hpp>

// ros msg, srv
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

// eigen
#include <eigen3/Eigen/Eigen>

#include "robotis_math/robotis_math.h"

namespace robotis_op
{

class OP3Localization : public rclcpp::Node
{

private:
    //ros node handle
    rclcpp::Node::SharedPtr ros_node_;

    //subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pelvis_pose_msg_sub_;
//    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pelvis_base_walking_msg_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pelvis_reset_msg_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    geometry_msgs::msg::TransformStamped pelvis_trans_;

    geometry_msgs::msg::PoseStamped pelvis_pose_;
    geometry_msgs::msg::PoseStamped pelvis_pose_old_;
    geometry_msgs::msg::PoseStamped pelvis_pose_base_walking_;
    geometry_msgs::msg::PoseStamped pelvis_pose_offset_;

    geometry_msgs::msg::PoseStamped pelvis_pose_base_walking_new_;
    geometry_msgs::msg::PoseStamped pelvis_pose_offset_new_;

    double transform_tolerance_;
    double err_tol_;

    bool is_moving_walking_;

    std::mutex mutex_;

public:
    void initialize();
    void pelvisPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
//    void pelvisPoseBaseWalkingCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void pelvisPoseResetCallback(const std_msgs::msg::String::SharedPtr msg);
    Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                              Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

    //constructor
    OP3Localization();
    //destructor
    ~OP3Localization();

    void update();
    void process();

};

}       // namespace

#endif  // OP3_LOCALIZATION_H_
