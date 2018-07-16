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

/* Author: Kayman, SCH */

#include <stdio.h>
#include "op3_base_module/base_module.h"

namespace robotis_op
{

BaseModule::BaseModule()
  : control_cycle_msec_(0),
    has_goal_joints_(false),
    ini_pose_only_(false),
    init_pose_file_path_("")
{
  enable_ = false;
  module_name_ = "base_module";
  control_mode_ = robotis_framework::PositionControl;

  base_module_state_ = new BaseModuleState();
  joint_state_ = new BaseJointState();
}

BaseModule::~BaseModule()
{
  queue_thread_.join();
}

void BaseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&BaseModule::queueThread, this));

  // init result, joint_id_table
  for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
       it != robot->dxls_.end(); it++)
  {
    std::string joint_name = it->first;
    robotis_framework::Dynamixel* dxl_info = it->second;

    joint_name_to_id_[joint_name] = dxl_info->id_;
    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
  }

  ros::NodeHandle ros_node;

  /* Load ROS Parameter */
  ros_node.param<std::string>("init_pose_file_path", init_pose_file_path_, ros::package::getPath("op3_base_module") + "/data/ini_pose.yaml");

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
}

void BaseModule::parseInitPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // parse movement time
  double mov_time;
  mov_time = doc["mov_time"].as<double>();

  base_module_state_->mov_time_ = mov_time;

  // parse via-point number
  int via_num;
  via_num = doc["via_num"].as<int>();

  base_module_state_->via_num_ = via_num;

  // parse via-point time
  std::vector<double> via_time;
  via_time = doc["via_time"].as<std::vector<double> >();

  base_module_state_->via_time_.resize(via_num, 1);
  for (int num = 0; num < via_num; num++)
    base_module_state_->via_time_.coeffRef(num, 0) = via_time[num];

  // parse via-point pose
  base_module_state_->joint_via_pose_.resize(via_num, MAX_JOINT_ID + 1);
  base_module_state_->joint_via_dpose_.resize(via_num, MAX_JOINT_ID + 1);
  base_module_state_->joint_via_ddpose_.resize(via_num, MAX_JOINT_ID + 1);

  base_module_state_->joint_via_pose_.fill(0.0);
  base_module_state_->joint_via_dpose_.fill(0.0);
  base_module_state_->joint_via_ddpose_.fill(0.0);

  YAML::Node via_pose_node = doc["via_pose"];
  for (YAML::iterator yaml_it = via_pose_node.begin(); yaml_it != via_pose_node.end(); ++yaml_it)
  {
    int id;
    std::vector<double> value;

    id = yaml_it->first.as<int>();
    value = yaml_it->second.as<std::vector<double> >();

    for (int num = 0; num < via_num; num++)
      base_module_state_->joint_via_pose_.coeffRef(num, id) = value[num] * DEGREE2RADIAN;
  }

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator yaml_it = tar_pose_node.begin(); yaml_it != tar_pose_node.end(); ++yaml_it)
  {
    int id;
    double value;

    id = yaml_it->first.as<int>();
    value = yaml_it->second.as<double>();

    base_module_state_->joint_ini_pose_.coeffRef(id, 0) = value * DEGREE2RADIAN;
  }

  base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;
  base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);
}

void BaseModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscribe topics */
  ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/robotis/base/ini_pose", 5, &BaseModule::initPoseMsgCallback,
                                                        this);
  set_module_client_ = ros_node.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

void BaseModule::initPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (base_module_state_->is_moving_ == false)
  {
    if (msg->data == "ini_pose")
    {
      // set module of all joints -> this module
      callServiceSettingModule(module_name_);

      // wait for changing the module to base_module and getting the goal position
      while (enable_ == false || has_goal_joints_ == false)
        usleep(8 * 1000);

      // parse initial pose
      parseInitPoseData(init_pose_file_path_);

      // generate trajectory
      tra_gene_tread_ = boost::thread(boost::bind(&BaseModule::initPoseTrajGenerateProc, this));
    }
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void BaseModule::initPoseTrajGenerateProc()
{
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = base_module_state_->joint_ini_pose_.coeff(id, 0);

    Eigen::MatrixXd tra;

    if (base_module_state_->via_num_ == 0)
    {
      tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                  base_module_state_->smp_time_, base_module_state_->mov_time_);
    }
    else
    {
      Eigen::MatrixXd via_value = base_module_state_->joint_via_pose_.col(id);
      Eigen::MatrixXd d_via_value = base_module_state_->joint_via_dpose_.col(id);
      Eigen::MatrixXd dd_via_value = base_module_state_->joint_via_ddpose_.col(id);

      tra = robotis_framework::calcMinimumJerkTraWithViaPoints(base_module_state_->via_num_, ini_value, 0.0, 0.0,
                                                               via_value, d_via_value, dd_via_value, tar_value, 0.0,
                                                               0.0, base_module_state_->smp_time_,
                                                               base_module_state_->via_time_,
                                                               base_module_state_->mov_time_);
    }

    base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
  }

  base_module_state_->is_moving_ = true;
  base_module_state_->cnt_ = 0;
  ROS_INFO("[start] send trajectory");
}

void BaseModule::poseGenerateProc(Eigen::MatrixXd joint_angle_pose)
{
  callServiceSettingModule(module_name_);

  while (enable_ == false || has_goal_joints_ == false)
    usleep(8 * 1000);

  base_module_state_->mov_time_ = 5.0;
  base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;

  base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  base_module_state_->joint_pose_ = joint_angle_pose;

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = base_module_state_->joint_pose_.coeff(id, 0);

    ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                base_module_state_->smp_time_,
                                                                base_module_state_->mov_time_);

    base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
  }

  base_module_state_->is_moving_ = true;
  base_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  ROS_INFO("[start] send trajectory");
}

void BaseModule::poseGenerateProc(std::map<std::string, double>& joint_angle_pose)
{
  callServiceSettingModule(module_name_);

  while (enable_ == false || has_goal_joints_ == false)
    usleep(8 * 1000);

  Eigen::MatrixXd target_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);

  for (std::map<std::string, double>::iterator joint_angle_it = joint_angle_pose.begin();
       joint_angle_it != joint_angle_pose.end(); joint_angle_it++)
  {
    std::string joint_name = joint_angle_it->first;
    double joint_angle_rad = joint_angle_it->second;

    std::map<std::string, int>::iterator joint_name_to_id_it = joint_name_to_id_.find(joint_name);
    if (joint_name_to_id_it != joint_name_to_id_.end())
    {
      target_pose.coeffRef(joint_name_to_id_it->second, 0) = joint_angle_rad;
    }
  }

  base_module_state_->joint_pose_ = target_pose;

  base_module_state_->mov_time_ = 5.0;
  base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;

  base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = base_module_state_->joint_pose_.coeff(id, 0);

    ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                base_module_state_->smp_time_,
                                                                base_module_state_->mov_time_);

    base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
  }

  base_module_state_->is_moving_ = true;
  base_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  ROS_INFO("[start] send trajectory");
}

bool BaseModule::isRunning()
{
  return base_module_state_->is_moving_;
}

void BaseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                         std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  /*----- write curr position -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    double joint_curr_position = dxl->dxl_state_->present_position_;
    double joint_goal_position = dxl->dxl_state_->goal_position_;

    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_curr_position;
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_goal_position;
  }

  has_goal_joints_ = true;

  /* ----- send trajectory ----- */
  if (base_module_state_->is_moving_ == true)
  {
    if (base_module_state_->cnt_ == 1)
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Init Pose");

    for (int id = 1; id <= MAX_JOINT_ID; id++)
      joint_state_->goal_joint_state_[id].position_ = base_module_state_->calc_joint_tra_(base_module_state_->cnt_, id);

    base_module_state_->cnt_++;
  }

  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    result_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_;
  }

  /*---------- initialize count number ----------*/

  if ((base_module_state_->cnt_ >= base_module_state_->all_time_steps_) && (base_module_state_->is_moving_ == true))
  {
    ROS_INFO("[end] send trajectory");

    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish Init Pose");

    base_module_state_->is_moving_ = false;
    base_module_state_->cnt_ = 0;

    // set all joints -> none
    if (ini_pose_only_ == true)
    {
      setCtrlModule("none");
      ini_pose_only_ = false;
    }
  }
}

void BaseModule::stop()
{
  return;
}

void BaseModule::onModuleEnable()
{
  ROS_INFO("Base Module is enabled");
}

void BaseModule::onModuleDisable()
{
  has_goal_joints_ = false;
}

void BaseModule::setCtrlModule(std::string module)
{
  std_msgs::String control_msg;
  control_msg.data = module_name_;

  set_ctrl_module_pub_.publish(control_msg);
}

void BaseModule::callServiceSettingModule(const std::string &module_name)
{
  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = module_name;

  if (set_module_client_.call(set_module_srv) == false)
  {
    ROS_ERROR("Failed to set module");
    return;
  }

  return ;
}

void BaseModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Base";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}
}
