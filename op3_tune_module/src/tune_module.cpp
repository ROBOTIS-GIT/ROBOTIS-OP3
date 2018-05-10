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
#include "op3_tune_module/tune_module.h"

namespace robotis_op
{

TuneModule::TuneModule()
  : control_cycle_msec_(0),
    has_goal_joints_(false),
    ini_pose_only_(false),
    get_tune_data_(false)
{
  enable_ = false;
  module_name_ = "tune_module";
  control_mode_ = robotis_framework::PositionControl;

  tune_module_state_ = new TuneModuleState();
  joint_state_ = new TuneJointState();
}

TuneModule::~TuneModule()
{
  queue_thread_.join();
}

void TuneModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&TuneModule::queueThread, this));

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
}

void TuneModule::parseInitPoseData(const std::string &path)
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

  tune_module_state_->mov_time_ = mov_time;

  // parse via-point number
  int via_num;
  via_num = doc["via_num"].as<int>();

  tune_module_state_->via_num_ = via_num;

  // parse via-point time
  std::vector<double> via_time;
  via_time = doc["via_time"].as<std::vector<double> >();

  tune_module_state_->via_time_.resize(via_num, 1);
  for (int num = 0; num < via_num; num++)
    tune_module_state_->via_time_.coeffRef(num, 0) = via_time[num];

  // parse via-point pose
  tune_module_state_->joint_via_pose_.resize(via_num, MAX_JOINT_ID + 1);
  tune_module_state_->joint_via_dpose_.resize(via_num, MAX_JOINT_ID + 1);
  tune_module_state_->joint_via_ddpose_.resize(via_num, MAX_JOINT_ID + 1);

  tune_module_state_->joint_via_pose_.fill(0.0);
  tune_module_state_->joint_via_dpose_.fill(0.0);
  tune_module_state_->joint_via_ddpose_.fill(0.0);

  YAML::Node via_pose_node = doc["via_pose"];
  for (YAML::iterator yaml_it = via_pose_node.begin(); yaml_it != via_pose_node.end(); ++yaml_it)
  {
    int id;
    std::vector<double> value;

    id = yaml_it->first.as<int>();
    value = yaml_it->second.as<std::vector<double> >();

    for (int num = 0; num < via_num; num++)
      tune_module_state_->joint_via_pose_.coeffRef(num, id) = value[num] * DEGREE2RADIAN;
  }

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator yaml_it = tar_pose_node.begin(); yaml_it != tar_pose_node.end(); ++yaml_it)
  {
    int id;
    double value;

    id = yaml_it->first.as<int>();
    value = yaml_it->second.as<double>();

    tune_module_state_->joint_ini_pose_.coeffRef(id, 0) = value * DEGREE2RADIAN;
  }

  tune_module_state_->all_time_steps_ = int(tune_module_state_->mov_time_ / tune_module_state_->smp_time_) + 1;
  tune_module_state_->calc_joint_tra_.resize(tune_module_state_->all_time_steps_, MAX_JOINT_ID + 1);
}

void TuneModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscribe topics */
  ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/robotis/tune/ini_pose", 5, &TuneModule::initPoseMsgCallback,
                                                        this);

  joint_offset_data_sub_ = ros_node.subscribe("/robotis/offset_tuner/joint_offset_data", 10,
                                              &TuneModule::jointOffsetDataCallback, this);
  joint_torque_enable_sub_ = ros_node.subscribe("/robotis/offset_tuner/torque_enable", 10,
                                                &TuneModule::jointTorqueOnOffCallback, this);
  command_sub_ = ros_node.subscribe("/robotis/offset_tuner/command", 5, &TuneModule::commandCallback, this);
  offset_data_server_ = ros_node.advertiseService("robotis/offset_tuner/get_present_joint_offset_data",
                                                  &TuneModule::getPresentJointOffsetDataServiceCallback, this);

  set_module_client_ = ros_node.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

void TuneModule::initPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (tune_module_state_->is_moving_ == false)
  {
    if (msg->data == "ini_pose")
    {
      // set module of all joints -> this module
      callServiceSettingModule(module_name_);

      // wait for changing the module to tune_module and getting the goal position
      while (enable_ == false || has_goal_joints_ == false)
        usleep(8 * 1000);

      // parse initial pose
      std::string init_pose_path = ros::package::getPath("op3_tune_module") + "/data/ini_pose.yaml";
      parseInitPoseData(init_pose_path);

      // generate trajectory
      tra_gene_tread_ = boost::thread(boost::bind(&TuneModule::initPoseTrajGenerateProc, this));
    }
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void TuneModule::initPoseTrajGenerateProc()
{
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = tune_module_state_->joint_ini_pose_.coeff(id, 0);

    Eigen::MatrixXd tra;

    if (tune_module_state_->via_num_ == 0)
    {
      tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                  tune_module_state_->smp_time_, tune_module_state_->mov_time_);
    }
    else
    {
      Eigen::MatrixXd via_value = tune_module_state_->joint_via_pose_.col(id);
      Eigen::MatrixXd d_via_value = tune_module_state_->joint_via_dpose_.col(id);
      Eigen::MatrixXd dd_via_value = tune_module_state_->joint_via_ddpose_.col(id);

      tra = robotis_framework::calcMinimumJerkTraWithViaPoints(tune_module_state_->via_num_, ini_value, 0.0, 0.0,
                                                               via_value, d_via_value, dd_via_value, tar_value, 0.0,
                                                               0.0, tune_module_state_->smp_time_,
                                                               tune_module_state_->via_time_,
                                                               tune_module_state_->mov_time_);
    }

    tune_module_state_->calc_joint_tra_.block(0, id, tune_module_state_->all_time_steps_, 1) = tra;
  }

  tune_module_state_->is_moving_ = true;
  tune_module_state_->cnt_ = 0;
  ROS_INFO("[start] send trajectory");
}

void TuneModule::poseGenerateProc(Eigen::MatrixXd joint_angle_pose)
{
  callServiceSettingModule(module_name_);

  while (enable_ == false || has_goal_joints_ == false)
    usleep(8 * 1000);

  tune_module_state_->mov_time_ = 5.0;
  tune_module_state_->all_time_steps_ = int(tune_module_state_->mov_time_ / tune_module_state_->smp_time_) + 1;

  tune_module_state_->calc_joint_tra_.resize(tune_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  tune_module_state_->joint_pose_ = joint_angle_pose;

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = tune_module_state_->joint_pose_.coeff(id, 0);

    ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                tune_module_state_->smp_time_,
                                                                tune_module_state_->mov_time_);

    tune_module_state_->calc_joint_tra_.block(0, id, tune_module_state_->all_time_steps_, 1) = tra;
  }

  tune_module_state_->is_moving_ = true;
  tune_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  ROS_INFO("[start] send trajectory");
}

void TuneModule::poseGenerateProc(std::map<std::string, double>& joint_angle_pose)
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

  tune_module_state_->joint_pose_ = target_pose;

  tune_module_state_->mov_time_ = 5.0;
  tune_module_state_->all_time_steps_ = int(tune_module_state_->mov_time_ / tune_module_state_->smp_time_) + 1;

  tune_module_state_->calc_joint_tra_.resize(tune_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = tune_module_state_->joint_pose_.coeff(id, 0);

    ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                tune_module_state_->smp_time_,
                                                                tune_module_state_->mov_time_);

    tune_module_state_->calc_joint_tra_.block(0, id, tune_module_state_->all_time_steps_, 1) = tra;
  }

  tune_module_state_->is_moving_ = true;
  tune_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  ROS_INFO("[start] send trajectory");
}

bool TuneModule::isRunning()
{
  return tune_module_state_->is_moving_;
}

void TuneModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
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
  if (tune_module_state_->is_moving_ == true)
  {
    if (tune_module_state_->cnt_ == 1)
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Init Pose");

    for (int id = 1; id <= MAX_JOINT_ID; id++)
      joint_state_->goal_joint_state_[id].position_ = tune_module_state_->calc_joint_tra_(tune_module_state_->cnt_, id);

    tune_module_state_->cnt_++;
  }
  else if(get_tune_data_ == true)
  {
    // update tune data
    // ...

    // turn off the flag
    get_tune_data_ = false;
  }

  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    result_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_;

    // set pid gain
    result_[joint_name]->position_p_gain_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].p_gain_;
    result_[joint_name]->position_i_gain_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].i_gain_;
    result_[joint_name]->position_d_gain_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].d_gain_;
  }

  /*---------- initialize count number ----------*/

  if ((tune_module_state_->cnt_ >= tune_module_state_->all_time_steps_) && (tune_module_state_->is_moving_ == true))
  {
    ROS_INFO("[end] send trajectory");

    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish Init Pose");

    tune_module_state_->is_moving_ = false;
    tune_module_state_->cnt_ = 0;

    // set all joints -> none
    if (ini_pose_only_ == true)
    {
      setCtrlModule("none");
      ini_pose_only_ = false;
    }
  }
}

void TuneModule::stop()
{
  return;
}

void TuneModule::onModuleEnable()
{
  ROS_INFO("Tune Module is enabled");
}

void TuneModule::onModuleDisable()
{
  has_goal_joints_ = false;
}

void TuneModule::setCtrlModule(std::string module)
{
  std_msgs::String control_msg;
  control_msg.data = module_name_;

  set_ctrl_module_pub_.publish(control_msg);
}

void TuneModule::callServiceSettingModule(const std::string &module_name)
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

void TuneModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Tune";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

// command for tuner
void TuneModule::commandCallback(const std_msgs::String::ConstPtr& msg)
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
//    yaml_out << YAML::Key << "init_pose_for_offset_tuner" << YAML::Value << init_pose;
//    yaml_out << YAML::EndMap;
//    std::ofstream fout(offset_file_.c_str());
//    fout << yaml_out.c_str();  // dump it back into the file
  }
  else if (msg->data == "ini_pose")
  {
    // moveToInitPose();
  }
  else
  {
    ROS_INFO_STREAM("Invalid Command : " << msg->data);
  }
}

void TuneModule::jointOffsetDataCallback(const op3_offset_tuner_msgs::JointOffsetData::ConstPtr &msg)
{
  if (this->enable_ == false)
  {
    ROS_ERROR("Tune module is not enable");
    return;
  }

  if (tune_module_state_->is_moving_ == true)
  {
    ROS_ERROR("Robot is moving, joint data will not be applied.");
    return;
  }

  //goal position
  ROS_INFO_STREAM(
        msg->joint_name << " " << msg->goal_value << " " << msg->offset_value << " " << msg->p_gain <<" " << msg->i_gain <<" " << msg->d_gain);

//  std::map<std::string, JointOffsetData*>::iterator map_it;
//  map_it = robot_offset_data_.find(msg->joint_name);
//  if (map_it == robot_offset_data_.end())
//  {
//    ROS_ERROR("Invalid Joint Name");
//    return;
//  }

//  if (robot_torque_enable_data_[msg->joint_name] == false)
//  {
//    ROS_ERROR_STREAM(msg->joint_name << "is turned off the torque");
//    return;
//  }

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
  get_tune_data_ = true;
}

void TuneModule::jointTorqueOnOffCallback(const op3_offset_tuner_msgs::JointTorqueOnOffArray::ConstPtr& msg)
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

bool TuneModule::getPresentJointOffsetDataServiceCallback(
    op3_offset_tuner_msgs::GetPresentJointOffsetData::Request &req,
    op3_offset_tuner_msgs::GetPresentJointOffsetData::Response &res)
{

  ROS_INFO("GetPresentJointOffsetDataService Called");

//  for (std::map<std::string, JointOffsetData*>::iterator map_it = robot_offset_data_.begin();
//       map_it != robot_offset_data_.end(); map_it++)
//  {
//    std::string joint_name = map_it->first;
//    JointOffsetData* joint_data = map_it->second;

//    op3_offset_tuner_msgs::JointOffsetPositionData joint_offset_pos;

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
