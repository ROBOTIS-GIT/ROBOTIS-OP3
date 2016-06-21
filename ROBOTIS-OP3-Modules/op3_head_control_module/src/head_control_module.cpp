/*
 * HeadControlModule.cpp
 *
 *  Created on: 2016. 1. 27.
 *      Author: kayman
 */

#include <stdio.h>
#include "op3_head_control_module/head_control_module.h"

using namespace ROBOTIS;

HeadControlModule::HeadControlModule()
: control_cycle_msec_(0),
  stop_process_(false),
  is_moving_(false),
  is_direct_control_(true),
  tra_count_(0),
  tra_size_(0),
  moving_time_(3.0),
  scan_state_(NoScan),
  DEBUG(false)
{
  enable = false;
  module_name = "head_control_module";
  control_mode = POSITION_CONTROL;

  result["head_pan"] = new DynamixelState();
  result["head_tilt"] = new DynamixelState();

  using_joint_name_["head_pan"] = 0;
  using_joint_name_["head_tilt"] = 1;

  max_angle_[using_joint_name_["head_pan"]] = 90 * deg2rad;
  min_angle_[using_joint_name_["head_pan"]] = -90 * deg2rad;
  max_angle_[using_joint_name_["head_tilt"]] = 10 * deg2rad;
  min_angle_[using_joint_name_["head_tilt"]] = -70 * deg2rad;

  target_position_ = Eigen::MatrixXd::Zero(1, result.size());
  current_position_ = Eigen::MatrixXd::Zero(1, result.size());
  goal_position_ = Eigen::MatrixXd::Zero(1, result.size());
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result.size());
}

HeadControlModule::~HeadControlModule()
{
  queue_thread_.join();
}

void HeadControlModule::Initialize(const int control_cycle_msec, Robot *robot)
{
  queue_thread_ = boost::thread(
      boost::bind(&HeadControlModule::QueueThread, this));

  control_cycle_msec_ = control_cycle_msec;

  ros::NodeHandle _ros_node;

  /* publish topics */
  status_msg_pub_ = _ros_node.advertise<robotis_controller_msgs::StatusMsg>(
      "/robotis/status", 0);
}

void HeadControlModule::QueueThread()
{
  ros::NodeHandle _ros_node;
  ros::CallbackQueue _callback_queue;

  _ros_node.setCallbackQueue(&_callback_queue);

  /* subscribe topics */
  ros::Subscriber set_head_joint_sub = _ros_node.subscribe("/robotis/head_control/set_joint_states", 1, &HeadControlModule::SetHeadJointCallback, this);
  ros::Subscriber set_head_joint_offset_sub = _ros_node.subscribe("/robotis/head_control/set_joint_states_offset", 1, &HeadControlModule::SetHeadJointOffsetCallback, this);
  ros::Subscriber set_head_scan_sub = _ros_node.subscribe("/robotis/head_control/scan_command", 1, &HeadControlModule::setHeadScanCallback, this);

  while (_ros_node.ok())
  {
    _callback_queue.callAvailable();

    usleep(100);
  }
}

void HeadControlModule::SetHeadJointCallback(
    const sensor_msgs::JointState::ConstPtr &msg)
{
  setHeadJoint(msg, false);
}

void HeadControlModule::SetHeadJointOffsetCallback(
    const sensor_msgs::JointState::ConstPtr &msg)
{
  setHeadJoint(msg, true);
}

void HeadControlModule::setHeadJoint(
    const sensor_msgs::JointState::ConstPtr &msg, bool is_offset)
{
  if (enable == false)
  {
    ROS_INFO("Head module is not enable.");
    PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR,
                     "Not Enable");
    return;
  }

  //  if (is_moving_ == true && is_direct_control_ == false)
  //  {
  //    ROS_INFO("Head is moving now.");
  //    PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR,
  //                     "Head is busy.");
  //    return;
  //  }

  // moving time
  moving_time_ = 1.0;               // default : 1 sec

  // set target joint angle
  target_position_ = goal_position_;        // default

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    std::string _joint_name = msg->name[ix];
    std::map<std::string, int>::iterator _iter = using_joint_name_.find(_joint_name);

    if (_iter != using_joint_name_.end())
    {
      double target_position = 0.0;
      int joint_index = _iter->second;

      // set target position
      if (is_offset == true)
        target_position = goal_position_.coeff(0, joint_index) + msg->position[ix];
      else
        target_position = msg->position[ix];

      // check angle limit
      checkAngleLimit(joint_index, target_position);

      // apply target position
      target_position_.coeffRef(0, joint_index) = target_position;

      // set time
      double angle_unit = is_offset ? 40 * M_PI / 180 : 25 * M_PI / 180;
      int _calc_moving_time = fabs(goal_position_.coeff(0, joint_index) - target_position_.coeff(0, joint_index)) / angle_unit;
      if (_calc_moving_time > moving_time_)
        moving_time_ = _calc_moving_time;

      if (DEBUG)
        std::cout << "joint : " << _joint_name << ", Index : " << joint_index
        << ", Angle : " << msg->position[ix] << ", Time : "
        << moving_time_ << std::endl;
    }
  }

  // set init joint vel, accel
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result.size());

  if (is_moving_ == true) // && is_direct_control_ == true)
  {
    goal_velocity_ = calc_joint_vel_tra_.block(tra_count_, 0, 1, result.size());
    goal_acceleration_ = calc_joint_accel_tra_.block(tra_count_, 0, 1,
                                                     result.size());
  }

  // set mode
  is_direct_control_ = true;
  scan_state_ = NoScan;

  // generate trajectory
  tra_gene_thread_ = new boost::thread(
      boost::bind(&HeadControlModule::JointTraGeneThread, this));
  delete tra_gene_thread_;
}

void HeadControlModule::setHeadScanCallback(const std_msgs::String::ConstPtr &msg)
{
  if(enable == false)
  {
    ROS_ERROR("Head control module is not enabled, scan command is canceled.");
    return;
  }
  else
    ROS_INFO("Scan command is accepted. [%d]", scan_state_);

  if(msg->data == "scan" && scan_state_ == NoScan)
  {
    scan_state_ = BottomToTop;
    is_direct_control_ = false;

    generateScanTra(scan_state_);
  }
  else if(msg->data == "stop")
  {
    scan_state_ = NoScan;
  }
}

bool HeadControlModule::checkAngleLimit(const int joint_index, double &goal_position)
{
  std::map<int, double>::iterator _iter = min_angle_.find(joint_index);
  if(_iter == min_angle_.end())
    return false;
  double min_angle = _iter->second;

  _iter = max_angle_.find(joint_index);
  if(_iter == max_angle_.end())
    return false;
  double max_angle = _iter->second;

  if(goal_position < min_angle) goal_position = min_angle;
  if(goal_position > max_angle) goal_position = max_angle;

  return true;
}

void HeadControlModule::Process(std::map<std::string, Dynamixel *> dxls,
                                std::map<std::string, double> sensors)
{
  if (enable == false)
    return;

  tra_lock_.lock();

  // get joint data
  for (std::map<std::string, DynamixelState *>::iterator state_iter = result
      .begin(); state_iter != result.end(); state_iter++)
  {
    std::string _joint_name = state_iter->first;
    int _index = using_joint_name_[_joint_name];

    Dynamixel *_dxl = NULL;
    std::map<std::string, Dynamixel*>::iterator _dxl_it = dxls.find(_joint_name);
    if (_dxl_it != dxls.end())
      _dxl = _dxl_it->second;
    else
      continue;

    current_position_.coeffRef(0, _index) = _dxl->dxl_state->present_position;
    goal_position_.coeffRef(0, _index) = _dxl->dxl_state->goal_position;
  }

  // check to stop
  if (stop_process_ == true)
  {
    StopMoving();
  }
  else
  {
    // process
    if (tra_size_ != 0)
    {
      // start of steps
      if (tra_count_ == 0)
      {
        StartMoving();
      }

      // end of steps
      if (tra_count_ >= tra_size_)
      {
        FinishMoving();
      }
      else
      {
        // update goal position
        goal_position_ = calc_joint_tra_.block(tra_count_, 0, 1, result.size());
        tra_count_ += 1;
      }
    }
  }
  tra_lock_.unlock();

  // set joint data
  for (std::map<std::string, DynamixelState *>::iterator state_iter = result
      .begin(); state_iter != result.end(); state_iter++)
  {
    std::string _joint_name = state_iter->first;
    int _index = using_joint_name_[_joint_name];

    result[_joint_name]->goal_position = goal_position_.coeff(0, _index);
  }
}

void HeadControlModule::Stop()
{
  tra_lock_.lock();

  if (is_moving_ == true)
    stop_process_ = true;

  tra_lock_.unlock();

  return;
}

bool HeadControlModule::IsRunning()
{
  return is_moving_;
}

void HeadControlModule::OnModuleEnable()
{
  scan_state_ = NoScan;
}

void HeadControlModule::OnModuleDisable()
{

}

void HeadControlModule::StartMoving()
{
  is_moving_ = true;


  // start procedure
}

void HeadControlModule::FinishMoving()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_direct_control_ = true;
  is_moving_ = false;


  // log
  PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO,
                   "Head movement is finished.");

  if (DEBUG)
    std::cout << "Trajectory End" << std::endl;

  // head scan
  switch(scan_state_)
  {
    case BottomToTop:
      scan_state_ = RightToLeft;
      break;

    case RightToLeft:
      scan_state_ = TopToBottom;
      break;

    case TopToBottom:
      scan_state_ = LeftToRight;
      break;

    case LeftToRight:
      scan_state_ = BottomToTop;
      break;

      // NoScan
    default:
      return;
  }

  generateScanTra(scan_state_);
}

void HeadControlModule::StopMoving()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_moving_ = false;
  is_direct_control_ = true;
  stop_process_ = false;
  scan_state_ = NoScan;

  // log
  PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN,
                   "Stop Module.");
}

void HeadControlModule::generateScanTra(const int head_direction)
{
  switch(head_direction)
  {
    case BottomToTop:
    {
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = max_angle_[using_joint_name_["head_pan"]] * 0.5;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.2;
      break;
    }

    case RightToLeft:
    {
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = min_angle_[using_joint_name_["head_pan"]] * 0.5;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.2;
      break;
    }

    case TopToBottom:
    {
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = min_angle_[using_joint_name_["head_pan"]] * 0.5;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.8;
      break;
    }

    case LeftToRight:
    {
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = max_angle_[using_joint_name_["head_pan"]] * 0.5;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.8;
      break;
    }

    default:
      return;
  }

  // set moving time
  moving_time_ = 1.0;               // default : 1 sec

  for (std::map<std::string, DynamixelState *>::iterator state_iter = result
      .begin(); state_iter != result.end(); state_iter++)
  {
    std::string _joint_name = state_iter->first;
    int _index = using_joint_name_[_joint_name];

    // set time
    int _calc_moving_time = fabs(goal_position_.coeff(0, _index) - target_position_.coeff(0, _index)) / 0.45;
    if (_calc_moving_time > moving_time_)
      moving_time_ = _calc_moving_time;
  }

  // set init joint vel, accel
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result.size());

  if (is_moving_ == true) // && is_direct_control_ == true)
  {
    goal_velocity_ = calc_joint_vel_tra_.block(tra_count_, 0, 1, result.size());
    goal_acceleration_ = calc_joint_accel_tra_.block(tra_count_, 0, 1, result.size());
  }

  // generate trajectory
  tra_gene_thread_ = new boost::thread(
      boost::bind(&HeadControlModule::JointTraGeneThread, this));
  delete tra_gene_thread_;
}

/*
 simple minimum jerk trajectory

 pos_start : position at initial state
 vel_start : velocity at initial state
 accel_start : acceleration at initial state

 pos_end : position at final state
 vel_end : velocity at final state
 accel_end : acceleration at final state

 smp_time : sampling time

 mov_time : movement time
 */
Eigen::MatrixXd HeadControlModule::MinimumJerkTraPVA(double pos_start,
                                                     double vel_start,
                                                     double accel_start,
                                                     double pos_end,
                                                     double vel_end,
                                                     double accel_end,
                                                     double smp_time,
                                                     double mov_time)
{
  Eigen::MatrixXd ___C(3, 3);
  Eigen::MatrixXd __C(3, 1);

  ___C << powDI(mov_time, 3), powDI(mov_time, 4), powDI(mov_time, 5), 3
      * powDI(mov_time, 2), 4 * powDI(mov_time, 3), 5 * powDI(mov_time, 4), 6
      * mov_time, 12 * powDI(mov_time, 2), 20 * powDI(mov_time, 3);

  __C << pos_end - pos_start - vel_start * mov_time
      - accel_start * pow(mov_time, 2) / 2, vel_end - vel_start
      - accel_start * mov_time, accel_end - accel_start;

  Eigen::MatrixXd _A = ___C.inverse() * __C;

  double _time_steps;

  _time_steps = mov_time / smp_time;
  int __time_steps = round(_time_steps + 1);

  Eigen::MatrixXd _time = Eigen::MatrixXd::Zero(__time_steps, 1);
  Eigen::MatrixXd _tra = Eigen::MatrixXd::Zero(__time_steps, 3);

  for (int step = 0; step < __time_steps; step++)
    _time.coeffRef(step, 0) = step * smp_time;

  for (int step = 0; step < __time_steps; step++)
  {
    // position
    _tra.coeffRef(step, 0) = pos_start + vel_start * _time.coeff(step, 0)
                            + 0.5 * accel_start * powDI(_time.coeff(step, 0), 2)
    + _A.coeff(0, 0) * powDI(_time.coeff(step, 0), 3)
    + _A.coeff(1, 0) * powDI(_time.coeff(step, 0), 4)
    + _A.coeff(2, 0) * powDI(_time.coeff(step, 0), 5);
    // velocity
    _tra.coeffRef(step, 1) = vel_start + accel_start * _time.coeff(step, 0)
                            + 3 * _A.coeff(0, 0) * powDI(_time.coeff(step, 0), 2)
                            + 4 * _A.coeff(1, 0) * powDI(_time.coeff(step, 0), 3)
                            + 5 * _A.coeff(2, 0) * powDI(_time.coeff(step, 0), 4);
    // accel
    _tra.coeffRef(step, 2) = accel_start
        + 6 * _A.coeff(0, 0) * _time.coeff(step, 0)
        + 12 * _A.coeff(1, 0) * powDI(_time.coeff(step, 0), 2)
        + 20 * _A.coeff(2, 0) * powDI(_time.coeff(step, 0), 3);
  }

  return _tra;
}

void HeadControlModule::JointTraGeneThread()
{
  tra_lock_.lock();

  double _smp_time = control_cycle_msec_ * 0.001;		// ms -> s
  int _all_time_steps = int(moving_time_ / _smp_time) + 1;

  calc_joint_tra_.resize(_all_time_steps, result.size());
  calc_joint_vel_tra_.resize(_all_time_steps, result.size());
  calc_joint_accel_tra_.resize(_all_time_steps, result.size());

  for (std::map<std::string, DynamixelState *>::iterator state_iter = result
      .begin(); state_iter != result.end(); state_iter++)
  {
    std::string _joint_name = state_iter->first;
    int _index = using_joint_name_[_joint_name];

    double _ini_value = goal_position_.coeff(0, _index);
    double _ini_vel = goal_velocity_.coeff(0, _index);
    double _ini_accel = goal_acceleration_.coeff(0, _index);

    double _tar_value = target_position_.coeff(0, _index);

    Eigen::MatrixXd tra = MinimumJerkTraPVA(_ini_value, _ini_vel, _ini_accel,
                                            _tar_value, 0.0, 0.0, _smp_time,
                                            moving_time_);

    calc_joint_tra_.block(0, _index, _all_time_steps, 1) = tra.block(
        0, 0, _all_time_steps, 1);
    calc_joint_vel_tra_.block(0, _index, _all_time_steps, 1) = tra.block(
        0, 1, _all_time_steps, 1);
    calc_joint_accel_tra_.block(0, _index, _all_time_steps, 1) = tra.block(
        0, 2, _all_time_steps, 1);
  }

  tra_size_ = calc_joint_tra_.rows();
  tra_count_ = 0;

  if (DEBUG)
    ROS_INFO("[ready] make trajectory : %d, %d", tra_size_, tra_count_);

  // init value
  // moving_time_ = 0;

  tra_lock_.unlock();
}

void HeadControlModule::PublishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg _status;
  _status.header.stamp = ros::Time::now();
  _status.type = type;
  _status.module_name = "Head Control";
  _status.status_msg = msg;

  status_msg_pub_.publish(_status);
}
