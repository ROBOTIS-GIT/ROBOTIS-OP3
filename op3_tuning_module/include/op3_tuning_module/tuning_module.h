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

/* Authors: Kayman, SCH */

#ifndef TuningModule_H_
#define TuningModule_H_

#include <map>
#include <thread>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <numeric>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/msg/sync_write_item.hpp"
#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "robotis_controller_msgs/srv/set_module.hpp"
#include "robotis_controller_msgs/srv/load_offset.hpp"
#include "robotis_math/robotis_math.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

#include "op3_tuning_module_msgs/msg/joint_offset_data.hpp"
#include "op3_tuning_module_msgs/msg/joint_torque_on_off_array.hpp"
#include "op3_tuning_module_msgs/srv/get_present_joint_offset_data.hpp"

#include "tuning_module_state.h"
#include "tuning_data.h"

namespace robotis_op
{

class TuningJointData
{
 public:
  double position_;
  double velocity_;
  double effort_;

  int p_gain_;
  int i_gain_;
  int d_gain_;

};

class TuneJointState
{
 public:
  TuningJointData curr_joint_state_[ MAX_JOINT_ID + 1];
  TuningJointData goal_joint_state_[ MAX_JOINT_ID + 1];
  TuningJointData fake_joint_state_[ MAX_JOINT_ID + 1];

};

class TuningModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<TuningModule>, public rclcpp::Node
{
 public:
  TuningModule();
  virtual ~TuningModule();

  /* ROS2 Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  void onModuleEnable();
  void onModuleDisable();

  /* ROS2 Topic Callback Functions */
  void tunePoseMsgCallback(const std_msgs::msg::String::SharedPtr msg);
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);
  void jointOffsetDataCallback(const op3_tuning_module_msgs::msg::JointOffsetData::SharedPtr msg);
  void jointGainDataCallback(const op3_tuning_module_msgs::msg::JointOffsetData::SharedPtr msg);
  void jointTorqueOnOffCallback(const op3_tuning_module_msgs::msg::JointTorqueOnOffArray::SharedPtr msg);
  bool getPresentJointOffsetDataServiceCallback(const std::shared_ptr<op3_tuning_module_msgs::srv::GetPresentJointOffsetData::Request> req,
                                                std::shared_ptr<op3_tuning_module_msgs::srv::GetPresentJointOffsetData::Response> res);

  /* ROS2 Calculation Functions */
  void targetPoseTrajGenerateProc();

  void poseGenerateProc(Eigen::MatrixXd joint_angle_pose);
  void poseGenerateProc(std::map<std::string, double>& joint_angle_pose);

  /* Parameter */
  // state for generating trajectory
  TuningModuleState *tuning_module_state_;
  // state of the joints
  TuneJointState *joint_state_;

 private:
  const int NONE_GAIN = 65535;
  void queueThread();
  void setCtrlModule(std::string module);
  void callServiceSettingModule(const std::string &module_name);
  void moveToInitPose();
  void moveToTunePose(const std::string &pose_name);
  bool parseOffsetData(const std::string &path);
  bool parseInitPoseData(const std::string &path);
  bool parseTunePoseData(const std::string &path, const std::string &pose_name);
  void publishStatusMsg(unsigned int type, std::string msg);
  bool turnOnOffOffset(bool turn_on);
  bool loadOffsetToController(const std::string &path);
  void saveOffsetToYaml(const std::string &path);
  void parseDxlInit(const std::string &path);
  void saveDxlInit(const std::string &path);

  int control_cycle_msec_;
  std::thread queue_thread_;
  std::thread tra_gene_thread_;
  std::mutex data_mutex_;

  // init pose
  rclcpp::Publisher<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_msg_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_ctrl_module_pub_;
  rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedPtr set_module_client_;

  // offset tuner
  rclcpp::Publisher<robotis_controller_msgs::msg::SyncWriteItem>::SharedPtr sync_write_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_offset_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr send_tra_sub_;
  rclcpp::Subscription<op3_tuning_module_msgs::msg::JointOffsetData>::SharedPtr joint_offset_data_sub_;
  rclcpp::Subscription<op3_tuning_module_msgs::msg::JointOffsetData>::SharedPtr joint_gain_data_sub_;
  rclcpp::Subscription<op3_tuning_module_msgs::msg::JointTorqueOnOffArray>::SharedPtr joint_torque_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Service<op3_tuning_module_msgs::srv::GetPresentJointOffsetData>::SharedPtr offset_data_server_;
  rclcpp::Client<robotis_controller_msgs::srv::LoadOffset>::SharedPtr load_offset_client_;

  std::map<std::string, int> joint_name_to_id_;
  // data set for tuner client
  std::map<std::string, JointOffsetData*> robot_tuning_data_;
  std::map<std::string, bool> robot_torque_enable_data_;

  std::string tune_pose_path_;
  std::string offset_path_;
  std::string init_file_path_;
  // data set for write the dxls
  TuningData tuning_data_;

  bool has_goal_joints_;
  bool ini_pose_only_;
  bool get_tuning_data_;
};

}

#endif /* TuningModule_H_ */
