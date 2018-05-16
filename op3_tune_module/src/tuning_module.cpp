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
#include "op3_tuning_module/tuning_module.h"

namespace robotis_op
{

TuningModule::TuningModule()
  : control_cycle_msec_(0),
    has_goal_joints_(false),
    ini_pose_only_(false),
    get_tuning_data_(false)
{
  enable_ = false;
  module_name_ = "tuning_module";
  control_mode_ = robotis_framework::PositionControl;

  tuning_module_state_ = new TuningModuleState();
  joint_state_ = new TuneJointState();
}

TuningModule::~TuningModule()
{
  queue_thread_.join();
}

void TuningModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&TuningModule::queueThread, this));

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

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  tune_pose_path_ = ros::package::getPath("op3_tuning_module") + "/data/tune_pose.yaml";
}

void TuningModule::moveToInitPose()
{
  // set tuning_module
  callServiceSettingModule(module_name_);

  // wait for changing the module to base_module and getting the goal position
  while (enable_ == false || has_goal_joints_ == false)
    usleep(8 * 1000);

  // get target pose from yaml file
  parseInitPoseData(tune_pose_path_);

  // generate a trajectory
  tra_gene_tread_ = boost::thread(boost::bind(&TuningModule::targetPoseTrajGenerateProc, this));
}

void TuningModule::moveToTunePose(const std::string &pose_name)
{
  // get target pose from yaml file
  parseTunePoseData(tune_pose_path_, pose_name);

  // generate a trajectory

}

bool TuningModule::parseInitPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return false;
  }

  YAML::Node init_pose_node;
  init_pose_node = doc["init_pose"];

  if(init_pose_node == NULL)
    return false;

  // parse movement time
  double move_time;
  move_time = init_pose_node["move_time"].as<double>();

  tuning_module_state_->mov_time_ = move_time;

  // no via pose

  // parse target pose
  YAML::Node tar_pose_node = init_pose_node["tar_pose"];
  for (YAML::iterator yaml_it = tar_pose_node.begin(); yaml_it != tar_pose_node.end(); ++yaml_it)
  {
    std::string joint_name;
    double value;

    joint_name = yaml_it->first.as<std::string>();
    value = yaml_it->second.as<double>();
    int id = joint_name_to_id_[joint_name];

    tuning_module_state_->joint_ini_pose_.coeffRef(id, 0) = value * DEGREE2RADIAN;
  }

  tuning_module_state_->all_time_steps_ = int(tuning_module_state_->mov_time_ / tuning_module_state_->smp_time_) + 1;
  tuning_module_state_->calc_joint_tra_.resize(tuning_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  return true;
}

bool TuningModule::parseTunePoseData(const std::string &path, const std::string &pose_name)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return false;
  }

  // get tune pose name
  YAML::Node tune_pose_node;
  tune_pose_node = doc[pose_name];

  if(tune_pose_node == NULL)
    return false;

  // check via pose
  std::vector<double> move_time;
  std::vector<std::string> target_pose_name;
  move_time = tune_pose_node["move_time"].as< std::vector<double> >();
  target_pose_name = tune_pose_node["target_pose"].as< std::vector<std::string> >();

  if(move_time.size() != target_pose_name.size() || move_time.size() == 0)
    return false;

  int via_num = move_time.size() - 1;

//  // parse movement time
//  double mov_time;
//  mov_time = doc["mov_time"].as<double>();

//  tuning_module_state_->mov_time_ = mov_time;

//  // parse via-point number
//  int via_num;
//  via_num = doc["via_num"].as<int>();

//  tuning_module_state_->via_num_ = via_num;

//  // parse via-point time
//  std::vector<double> via_time;
//  via_time = doc["via_time"].as<std::vector<double> >();

//  tuning_module_state_->via_time_.resize(via_num, 1);
//  for (int num = 0; num < via_num; num++)
//    tuning_module_state_->via_time_.coeffRef(num, 0) = via_time[num];

//  // parse via-point pose
//  tuning_module_state_->joint_via_pose_.resize(via_num, MAX_JOINT_ID + 1);
//  tuning_module_state_->joint_via_dpose_.resize(via_num, MAX_JOINT_ID + 1);
//  tuning_module_state_->joint_via_ddpose_.resize(via_num, MAX_JOINT_ID + 1);

//  tuning_module_state_->joint_via_pose_.fill(0.0);
//  tuning_module_state_->joint_via_dpose_.fill(0.0);
//  tuning_module_state_->joint_via_ddpose_.fill(0.0);

//  YAML::Node via_pose_node = doc["via_pose"];
//  for (YAML::iterator yaml_it = via_pose_node.begin(); yaml_it != via_pose_node.end(); ++yaml_it)
//  {
//    int id;
//    std::vector<double> value;

//    id = yaml_it->first.as<int>();
//    value = yaml_it->second.as<std::vector<double> >();

//    for (int num = 0; num < via_num; num++)
//      tuning_module_state_->joint_via_pose_.coeffRef(num, id) = value[num] * DEGREE2RADIAN;
//  }

//  // parse target pose
//  YAML::Node tar_pose_node = doc["tar_pose"];
//  for (YAML::iterator yaml_it = tar_pose_node.begin(); yaml_it != tar_pose_node.end(); ++yaml_it)
//  {
//    int id;
//    double value;

//    id = yaml_it->first.as<int>();
//    value = yaml_it->second.as<double>();

//    tuning_module_state_->joint_ini_pose_.coeffRef(id, 0) = value * DEGREE2RADIAN;
//  }

//  tuning_module_state_->all_time_steps_ = int(tuning_module_state_->mov_time_ / tuning_module_state_->smp_time_) + 1;
//  tuning_module_state_->calc_joint_tra_.resize(tuning_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  return true;
}

void TuningModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscribe topics */
  ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/robotis/tuning_module/tuning_pose", 5, &TuningModule::tunePoseMsgCallback,
                                                        this);

  joint_offset_data_sub_ = ros_node.subscribe("/robotis/tuning_module/joint_offset_data", 10,
                                              &TuningModule::jointOffsetDataCallback, this);
  joint_gain_data_sub_ = ros_node.subscribe("/robotis/tuning_module/joint_gain_data", 10,
                                            &TuningModule::jointGainDataCallback, this);
  joint_torque_enable_sub_ = ros_node.subscribe("/robotis/tuning_module/torque_enable", 10,
                                                &TuningModule::jointTorqueOnOffCallback, this);
  command_sub_ = ros_node.subscribe("/robotis/tuning_module/command", 5, &TuningModule::commandCallback, this);
  offset_data_server_ = ros_node.advertiseService("robotis/tuning_module/get_present_joint_offset_data",
                                                  &TuningModule::getPresentJointOffsetDataServiceCallback, this);

  set_module_client_ = ros_node.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

void TuningModule::tunePoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if(tuning_module_state_->is_generating_ == true)
  {
    ROS_ERROR("Previous pose is generating now.");
    return;
  }

  if (tuning_module_state_->is_moving_ == true)
  {
    ROS_INFO("previous task is alive");
    return;
  }

  if (msg->data == "ini_pose")
  {
    moveToInitPose();
  }
  else if (msg->data == "tune_pose_01")
  {
    moveToTunePose(msg->data);
  }
  else if (msg->data == "tune_pose_02")
  {
    moveToTunePose(msg->data);
  }
  else if (msg->data == "tune_pose_03")
  {
    moveToTunePose(msg->data);
  }
  else if (msg->data == "tune_pose_04")
  {
    moveToTunePose(msg->data);
  }

  return;
}

void TuningModule::targetPoseTrajGenerateProc()
{
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = tuning_module_state_->joint_ini_pose_.coeff(id, 0);

    Eigen::MatrixXd tra;

    if (tuning_module_state_->via_num_ == 0)
    {
      tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                  tuning_module_state_->smp_time_, tuning_module_state_->mov_time_);
    }
    else
    {
      Eigen::MatrixXd via_value = tuning_module_state_->joint_via_pose_.col(id);
      Eigen::MatrixXd d_via_value = tuning_module_state_->joint_via_dpose_.col(id);
      Eigen::MatrixXd dd_via_value = tuning_module_state_->joint_via_ddpose_.col(id);

      tra = robotis_framework::calcMinimumJerkTraWithViaPoints(tuning_module_state_->via_num_, ini_value, 0.0, 0.0,
                                                               via_value, d_via_value, dd_via_value, tar_value, 0.0,
                                                               0.0, tuning_module_state_->smp_time_,
                                                               tuning_module_state_->via_time_,
                                                               tuning_module_state_->mov_time_);
    }

    tuning_module_state_->calc_joint_tra_.block(0, id, tuning_module_state_->all_time_steps_, 1) = tra;
  }

  tuning_module_state_->is_moving_ = true;
  tuning_module_state_->cnt_ = 0;
  ROS_INFO("[start] send trajectory");
}

void TuningModule::poseGenerateProc(Eigen::MatrixXd joint_angle_pose)
{
  callServiceSettingModule(module_name_);

  while (enable_ == false || has_goal_joints_ == false)
    usleep(8 * 1000);

  tuning_module_state_->mov_time_ = 5.0;
  tuning_module_state_->all_time_steps_ = int(tuning_module_state_->mov_time_ / tuning_module_state_->smp_time_) + 1;

  tuning_module_state_->calc_joint_tra_.resize(tuning_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  tuning_module_state_->joint_pose_ = joint_angle_pose;

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = tuning_module_state_->joint_pose_.coeff(id, 0);

    ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                tuning_module_state_->smp_time_,
                                                                tuning_module_state_->mov_time_);

    tuning_module_state_->calc_joint_tra_.block(0, id, tuning_module_state_->all_time_steps_, 1) = tra;
  }

  tuning_module_state_->is_moving_ = true;
  tuning_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  ROS_INFO("[start] send trajectory");
}

void TuningModule::poseGenerateProc(std::map<std::string, double>& joint_angle_pose)
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

  tuning_module_state_->joint_pose_ = target_pose;

  tuning_module_state_->mov_time_ = 5.0;
  tuning_module_state_->all_time_steps_ = int(tuning_module_state_->mov_time_ / tuning_module_state_->smp_time_) + 1;

  tuning_module_state_->calc_joint_tra_.resize(tuning_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = tuning_module_state_->joint_pose_.coeff(id, 0);

    ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                tuning_module_state_->smp_time_,
                                                                tuning_module_state_->mov_time_);

    tuning_module_state_->calc_joint_tra_.block(0, id, tuning_module_state_->all_time_steps_, 1) = tra;
  }

  tuning_module_state_->is_moving_ = true;
  tuning_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  ROS_INFO("[start] send trajectory");
}

bool TuningModule::isRunning()
{
  return tuning_module_state_->is_moving_;
}

void TuningModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
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
    int p_gain = dxl->dxl_state_->position_p_gain_;
    int i_gain = dxl->dxl_state_->position_i_gain_;
    int d_gain = dxl->dxl_state_->position_d_gain_;

    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_curr_position;
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_goal_position;

    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].p_gain_ = p_gain;
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].i_gain_ = i_gain;
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].d_gain_ = d_gain;
  }

  has_goal_joints_ = true;

  /* ----- send trajectory ----- */
  if (tuning_module_state_->is_moving_ == true)
  {
    if (tuning_module_state_->cnt_ == 1)
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Init Pose");

    for (int id = 1; id <= MAX_JOINT_ID; id++)
      joint_state_->goal_joint_state_[id].position_ = tuning_module_state_->calc_joint_tra_(tuning_module_state_->cnt_, id);

    tuning_module_state_->cnt_++;
  }
  else if(get_tuning_data_ == true)
  {
    // update tuning data
    // check whether lock is possible
    if(data_mutex_.try_lock() == true)
    {
      // get joint name
      std::string joint_name;
      tuning_data_.joint_name_.getValue(joint_name);

      std::map<std::string, int>::iterator joint_name_to_id_it = joint_name_to_id_.find(joint_name);
      if (joint_name_to_id_it != joint_name_to_id_.end())
      {
        // get tuning data
        int joint_id = joint_name_to_id_it->second;
        tuning_data_.position_.getValue(joint_state_->goal_joint_state_[joint_id].position_);

        tuning_data_.p_gain_.getValue(joint_state_->goal_joint_state_[joint_id].p_gain_);
        tuning_data_.i_gain_.getValue(joint_state_->goal_joint_state_[joint_id].i_gain_);
        tuning_data_.d_gain_.getValue(joint_state_->goal_joint_state_[joint_id].d_gain_);
      }

      // turn off the flag
      get_tuning_data_ = false;
      data_mutex_.unlock();
    }
  }

  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    // set goal position
    result_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_;

    // set pid gain
    result_[joint_name]->position_p_gain_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].p_gain_;
    result_[joint_name]->position_i_gain_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].i_gain_;
    result_[joint_name]->position_d_gain_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].d_gain_;
  }

  /*---------- initialize count number ----------*/

  if ((tuning_module_state_->cnt_ >= tuning_module_state_->all_time_steps_) && (tuning_module_state_->is_moving_ == true))
  {
    ROS_INFO("[end] send trajectory");

    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish Init Pose");

    tuning_module_state_->is_moving_ = false;
    tuning_module_state_->cnt_ = 0;

    // set all joints -> none
    if (ini_pose_only_ == true)
    {
      setCtrlModule("none");
      ini_pose_only_ = false;
    }
  }
}

void TuningModule::stop()
{
  return;
}

void TuningModule::onModuleEnable()
{
  ROS_INFO("Tuning module is enabled");
}

void TuningModule::onModuleDisable()
{
  tuning_data_.clearData();
  has_goal_joints_ = false;
}

void TuningModule::setCtrlModule(std::string module)
{
  std_msgs::String control_msg;
  control_msg.data = module_name_;

  set_ctrl_module_pub_.publish(control_msg);
}

void TuningModule::callServiceSettingModule(const std::string &module_name)
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

void TuningModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Tune";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

// command for tuner
void TuningModule::commandCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "save")
  {
    //    YAML::Emitter yaml_out;
    //    std::map<std::string, double> offset;
    //    std::map<std::string, double> init_pose;
    //    for (std::map<std::string, JointOffsetData*>::iterator map_it = robot_offset_data_.begin();
    //         map_it != robot_offset_data_.end(); map_it++)
    //    {
    //      std::string joint_name = map_it->first;
    //      JointOffsetData* joint_data = map_it->second;

    //      offset[joint_name] = joint_data->joint_offset_rad_;  // edit one of the nodes
    //      init_pose[joint_name] = joint_data->joint_init_pos_rad_;  // edit one of the nodes
    //    }

    //    yaml_out << YAML::BeginMap;
    //    yaml_out << YAML::Key << "offset" << YAML::Value << offset;
    //    yaml_out << YAML::Key << "init_pose_for_tuning_module" << YAML::Value << init_pose;
    //    yaml_out << YAML::EndMap;
    //    std::ofstream fout(offset_file_.c_str());
    //    fout << yaml_out.c_str();  // dump it back into the file
  }
  else
  {
    ROS_INFO_STREAM("Invalid Command : " << msg->data);
  }
}

void TuningModule::jointOffsetDataCallback(const op3_tuning_module_msgs::JointOffsetData::ConstPtr &msg)
{
  if (this->enable_ == false)
  {
    ROS_ERROR("Tuning module is not enable");
    return;
  }

  if (tuning_module_state_->is_moving_ == true)
  {
    ROS_ERROR("Robot is moving, joint offset data will not be applied.");
    return;
  }

  //goal position
  ROS_INFO_STREAM(
        msg->joint_name << " " << msg->goal_value << " " << msg->offset_value << " " << msg->p_gain <<" " << msg->i_gain <<" " << msg->d_gain);

  std::map<std::string, JointOffsetData*>::iterator map_it;
  map_it = robot_offset_data_.find(msg->joint_name);
  if (map_it == robot_offset_data_.end())
  {
    ROS_ERROR("Invalid Joint Name");
    return;
  }

  if (robot_torque_enable_data_[msg->joint_name] == false)
  {
    ROS_ERROR_STREAM(msg->joint_name << "is turned off the torque");
    return;
  }

  data_mutex_.lock();

  tuning_data_.joint_name_.setValue(msg->joint_name);
  tuning_data_.position_.setValue(msg->offset_value + msg->goal_value);


  //  double goal_pose_rad = msg->offset_value + msg->goal_value;
  //  uint32_t goal_pose_value = controller_->robot_->dxls_[msg->joint_name]->convertRadian2Value(goal_pose_rad);
  //  uint8_t dxl_error = 0;
  //  uint32_t comm_result = COMM_SUCCESS;
  //  comm_result = controller_->writeCtrlItem(msg->joint_name,
  //                                           controller_->robot_->dxls_[msg->joint_name]->goal_position_item_->item_name_,
  //      goal_pose_value, &dxl_error);
  //  if (comm_result != COMM_SUCCESS)
  //  {
  //    ROS_ERROR("Failed to write goal position");
  //    return;
  //  }
  //  else
  //  {
  //    robot_offset_data_[msg->joint_name]->joint_init_pos_rad_ = msg->goal_value;
  //    robot_offset_data_[msg->joint_name]->joint_offset_rad_ = msg->offset_value;
  //  }

  //  if (dxl_error != 0)
  //  {
  //    ROS_ERROR_STREAM("goal_pos_set : " << msg->joint_name << "  has error "<< (int)dxl_error);
  //  }

  //  robot_offset_data_[msg->joint_name]->p_gain_ = msg->p_gain;
  //  robot_offset_data_[msg->joint_name]->i_gain_ = msg->i_gain;
  //  robot_offset_data_[msg->joint_name]->d_gain_ = msg->d_gain;

  // flag on to update joint data
  get_tuning_data_ = true;
  data_mutex_.unlock();
}

void TuningModule::jointGainDataCallback(const op3_tuning_module_msgs::JointOffsetData::ConstPtr &msg)
{
  if (this->enable_ == false)
  {
    ROS_ERROR("Tuning module is not enable");
    return;
  }

  if (tuning_module_state_->is_moving_ == true)
  {
    ROS_ERROR("Robot is moving, joint gain data will not be applied.");
    return;
  }

  std::map<std::string, JointOffsetData*>::iterator map_it;
  map_it = robot_offset_data_.find(msg->joint_name);
  if (map_it == robot_offset_data_.end())
  {
    ROS_ERROR("Invalid Joint Name");
    return;
  }

  if (robot_torque_enable_data_[msg->joint_name] == false)
  {
    ROS_ERROR_STREAM(msg->joint_name << "is turned off the torque");
    return;
  }

  data_mutex_.lock();

  tuning_data_.joint_name_.setValue(msg->joint_name);
  tuning_data_.p_gain_.setValue(msg->p_gain);
  tuning_data_.i_gain_.setValue(msg->i_gain);
  tuning_data_.d_gain_.setValue(msg->d_gain);

  // flag on to update joint data
  get_tuning_data_ = true;
  data_mutex_.unlock();
}

void TuningModule::jointTorqueOnOffCallback(const op3_tuning_module_msgs::JointTorqueOnOffArray::ConstPtr& msg)
{
  for (unsigned int i = 0; i < msg->torque_enable_data.size(); i++)
  {
    std::string joint_name = msg->torque_enable_data[i].joint_name;
    bool torque_enable = msg->torque_enable_data[i].torque_enable;
    ROS_INFO_STREAM(i <<" " << joint_name << torque_enable);

    //    std::map<std::string, JointOffsetData*>::iterator map_it;
    //    map_it = robot_offset_data_.find(joint_name);
    //    if (map_it == robot_offset_data_.end())
    //    {
    //      ROS_ERROR("Invalid Joint Name");
    //      continue;
    //    }
    //    else
    //    {
    //      int32_t comm_result = COMM_SUCCESS;
    //      uint8_t dxl_error = 0;
    //      uint8_t torque_enable_value = 0;

    //      if (torque_enable)
    //        torque_enable_value = 1;
    //      else
    //        torque_enable_value = 0;

    //      comm_result = controller_->writeCtrlItem(joint_name,
    //                                               controller_->robot_->dxls_[joint_name]->torque_enable_item_->item_name_,
    //                                               torque_enable_value, &dxl_error);
    //      if (comm_result != COMM_SUCCESS)
    //      {
    //        ROS_ERROR("Failed to write goal position");
    //      }
    //      else
    //      {
    //        robot_torque_enable_data_[joint_name] = torque_enable;
    //      }

    //      if (dxl_error != 0)
    //      {
    //        ROS_ERROR_STREAM("goal_pos_set : " << joint_name << "  has error "<< (int)dxl_error);
    //      }
    //    }
  }

}

bool TuningModule::getPresentJointOffsetDataServiceCallback(
    op3_tuning_module_msgs::GetPresentJointOffsetData::Request &req,
    op3_tuning_module_msgs::GetPresentJointOffsetData::Response &res)
{

  ROS_INFO("GetPresentJointOffsetDataService Called");

  //  for (std::map<std::string, JointOffsetData*>::iterator map_it = robot_offset_data_.begin();
  //       map_it != robot_offset_data_.end(); map_it++)
  //  {
  //    std::string joint_name = map_it->first;
  //    JointOffsetData* joint_data = map_it->second;

  //    op3_tuning_module_msgs::JointOffsetPositionData joint_offset_pos;

  //    int32_t present_pos_value = 0;
  //    uint8_t dxl_error = 0;
  //    int comm_result = COMM_SUCCESS;

  //    if (controller_->robot_->dxls_[joint_name]->present_position_item_ == NULL)
  //      continue;

  //    comm_result = controller_->readCtrlItem(joint_name,
  //                                            controller_->robot_->dxls_[joint_name]->present_position_item_->item_name_,
  //                                            (uint32_t*) &present_pos_value, &dxl_error);
  //    if (comm_result != COMM_SUCCESS)
  //    {
  //      ROS_ERROR("Failed to read present pos");
  //      return false;
  //    }
  //    else
  //    {
  //      if (dxl_error != 0)
  //      {
  //        ROS_ERROR_STREAM(joint_name << "  has error "<< (int)dxl_error);
  //      }

  //      joint_offset_pos.joint_name = joint_name;
  //      joint_offset_pos.goal_value = joint_data->joint_init_pos_rad_;
  //      joint_offset_pos.offset_value = joint_data->joint_offset_rad_;
  //      joint_offset_pos.present_value = controller_->robot_->dxls_[joint_name]->convertValue2Radian(present_pos_value);
  //      joint_offset_pos.p_gain = joint_data->p_gain_;
  //      joint_offset_pos.i_gain = joint_data->i_gain_;
  //      joint_offset_pos.d_gain = joint_data->d_gain_;

  //      res.present_data_array.push_back(joint_offset_pos);
  //    }
  //  }
  return true;
}

}
