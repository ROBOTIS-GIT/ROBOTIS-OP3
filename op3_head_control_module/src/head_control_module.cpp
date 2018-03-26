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

/* Author: Kayman */

#include <stdio.h>
#include "op3_head_control_module/head_control_module.h"

namespace robotis_op
{

HeadControlModule::HeadControlModule()
  : control_cycle_msec_(0),
    stop_process_(false),
    is_moving_(false),
    is_direct_control_(true),
    tra_count_(0),
    tra_size_(0),
    moving_time_(3.0),
    scan_state_(NoScan),
    has_goal_position_(false),
    angle_unit_(35),
    DEBUG(false)
{
  enable_ = false;
  module_name_ = "head_control_module";
  control_mode_ = robotis_framework::PositionControl;

  result_["head_pan"] = new robotis_framework::DynamixelState();
  result_["head_tilt"] = new robotis_framework::DynamixelState();

  using_joint_name_["head_pan"] = 0;
  using_joint_name_["head_tilt"] = 1;

  max_angle_[using_joint_name_["head_pan"]] = 85 * DEGREE2RADIAN;
  min_angle_[using_joint_name_["head_pan"]] = -85 * DEGREE2RADIAN;
  max_angle_[using_joint_name_["head_tilt"]] = 30 * DEGREE2RADIAN;
  min_angle_[using_joint_name_["head_tilt"]] = -75 * DEGREE2RADIAN;

  target_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  current_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  last_msg_time_ = ros::Time::now();
}

HeadControlModule::~HeadControlModule()
{
  queue_thread_.join();
}

void HeadControlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  ros::NodeHandle param_nh("~");
  angle_unit_ = param_nh.param("angle_unit", 35.0);

  ROS_WARN_STREAM("Head control - angle unit : " << angle_unit_);

  queue_thread_ = boost::thread(boost::bind(&HeadControlModule::queueThread, this));

  control_cycle_msec_ = control_cycle_msec;

  ros::NodeHandle ros_node;

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 0);

}

void HeadControlModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscribe topics */
  ros::Subscriber set_head_joint_sub = ros_node.subscribe("/robotis/head_control/set_joint_states", 1,
                                                          &HeadControlModule::setHeadJointCallback, this);
  ros::Subscriber set_head_joint_offset_sub = ros_node.subscribe("/robotis/head_control/set_joint_states_offset", 1,
                                                                 &HeadControlModule::setHeadJointOffsetCallback, this);
  ros::Subscriber set_head_scan_sub = ros_node.subscribe("/robotis/head_control/scan_command", 1,
                                                         &HeadControlModule::setHeadScanCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void HeadControlModule::setHeadJointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  setHeadJoint(msg, false);
}

void HeadControlModule::setHeadJointOffsetCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  setHeadJoint(msg, true);
}

void HeadControlModule::setHeadJoint(const sensor_msgs::JointState::ConstPtr &msg, bool is_offset)
{
  if (enable_ == false)
  {
    ROS_INFO_THROTTLE(1, "Head module is not enable.");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Not Enable");
    return;
  }

  while(has_goal_position_ == false)
  {
    std::cout << "wait for receiving current position" << std::endl;
    usleep(80 * 1000);
}
  // moving time
  moving_time_ = is_offset ? 0.1 : 1.0;               // default : 1 sec

  // set target joint angle
  target_position_ = goal_position_;        // default

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    std::string joint_name = msg->name[ix];
    std::map<std::string, int>::iterator joint_it = using_joint_name_.find(joint_name);

    if (joint_it != using_joint_name_.end())
    {
      double target_position = 0.0;
      int joint_index = joint_it->second;

      // set target position
      if (is_offset == true)
        target_position = goal_position_.coeff(0, joint_index) + msg->position[ix];
      else
        target_position = msg->position[ix];

      // check angle limit
      bool is_checked = checkAngleLimit(joint_index, target_position);
      if(is_checked == false)
      {
        ROS_ERROR_STREAM("Failed to find limit angle \n    id : " << joint_index << ", value : " << (target_position_ * 180 / M_PI));
      }

      // apply target position
      target_position_.coeffRef(0, joint_index) = target_position;

      // set time
      //double angle_unit = 35 * M_PI / 180;
      double angle_unit = is_offset ? (angle_unit_ * M_PI / 180 * 1.5) : (angle_unit_ * M_PI / 180);
      double calc_moving_time = fabs(goal_position_.coeff(0, joint_index) - target_position_.coeff(0, joint_index))
          / angle_unit;
      if (calc_moving_time > moving_time_)
        moving_time_ = calc_moving_time;

      if (DEBUG)
        std::cout << " - joint : " << joint_name << ", Index : " << joint_index << "\n     Target Angle : " << target_position_.coeffRef(0, joint_index) << ", Curr Goal : " << goal_position_.coeff(0, joint_index)
                  << ", Time : " << moving_time_ << ", msg : " << msg->position[ix] << std::endl;
    }
  }

  // set mode
  is_direct_control_ = true;
  scan_state_ = NoScan;

  // generate trajectory
  tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneThread, this));
  delete tra_gene_thread_;
}

void HeadControlModule::setHeadScanCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
  {
    ROS_ERROR_THROTTLE(1, "Head control module is not enabled, scan command is canceled.");
    return;
  }
  else
    ROS_INFO_THROTTLE(1, "Scan command is accepted. [%d]", scan_state_);

  if (msg->data == "scan" && scan_state_ == NoScan)
  {
    std::srand(std::time(NULL)); // use current time as seed for random generator
    scan_state_ = std::rand() % 4 + 1;

    is_direct_control_ = false;

    generateScanTra(scan_state_);
  }
  else if (msg->data == "stop")
  {
    scan_state_ = NoScan;
  }
}

bool HeadControlModule::checkAngleLimit(const int joint_index, double &goal_position)
{
  std::map<int, double>::iterator angle_it = min_angle_.find(joint_index);
  if (angle_it == min_angle_.end())
    return false;
  double min_angle = angle_it->second;

  angle_it = max_angle_.find(joint_index);
  if (angle_it == max_angle_.end())
    return false;
  double max_angle = angle_it->second;

  if (goal_position < min_angle)
    goal_position = min_angle;
  if (goal_position > max_angle)
    goal_position = max_angle;

  return true;
}

void HeadControlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  tra_lock_.lock();

  // get joint data
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];

    robotis_framework::Dynamixel *_dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      _dxl = dxl_it->second;
    else
      continue;

    current_position_.coeffRef(0, index) = _dxl->dxl_state_->present_position_;
    goal_position_.coeffRef(0, index) = _dxl->dxl_state_->goal_position_;
  }

  has_goal_position_= true;

  // check to stop
  if (stop_process_ == true)
  {
    stopMoving();
  }
  else
  {
    // process
    if (tra_size_ != 0)
    {
      // start of steps
      if (tra_count_ == 0)
      {
        startMoving();
      }

      // end of steps
      if (tra_count_ >= tra_size_)
      {
        finishMoving();
      }
      else
      {
        // update goal position
        goal_position_ = calc_joint_tra_.block(tra_count_, 0, 1, result_.size());
        goal_velocity_ = calc_joint_vel_tra_.block(tra_count_, 0, 1, result_.size());
        goal_acceleration_ = calc_joint_accel_tra_.block(tra_count_, 0, 1, result_.size());

        tra_count_ += 1;
      }
    }
  }
  tra_lock_.unlock();

  // set joint data
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];
    double goal_position = goal_position_.coeff(0, index);
    checkAngleLimit(index, goal_position);

    result_[joint_name]->goal_position_ = goal_position;
  }
}

void HeadControlModule::stop()
{
  tra_lock_.lock();

  if (is_moving_ == true)
    stop_process_ = true;

  tra_lock_.unlock();

  return;
}

bool HeadControlModule::isRunning()
{
  return is_moving_;
}

void HeadControlModule::onModuleEnable()
{
  scan_state_ = NoScan;
}

void HeadControlModule::onModuleDisable()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_direct_control_ = true;
  is_moving_ = false;
  has_goal_position_ = false;

  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  scan_state_ = NoScan;

  std::cout << "head_control_module : disable";
}

void HeadControlModule::startMoving()
{
  is_moving_ = true;

  // start procedure
}

void HeadControlModule::finishMoving()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_direct_control_ = true;
  is_moving_ = false;

  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  // log
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Head movement is finished.");

  if (DEBUG)
    std::cout << "Trajectory End" << std::endl;

  // head scan
  switch (scan_state_)
  {
  case TopLeft:
    scan_state_ = BottomRight;
    break;

  case BottomRight:
    scan_state_ = BottomLeft;
    break;

  case BottomLeft:
    scan_state_ = TopRight;
    break;

  case TopRight:
    scan_state_ = TopLeft;
    break;

    // NoScan
  default:
    return;
  }

  generateScanTra(scan_state_);
}

void HeadControlModule::stopMoving()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_moving_ = false;
  is_direct_control_ = true;
  stop_process_ = false;
  scan_state_ = NoScan;

  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  // log
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Stop Module.");
}

void HeadControlModule::generateScanTra(const int head_direction)
{
  switch (head_direction)
  {
  case TopLeft:
  {
    target_position_.coeffRef(0, using_joint_name_["head_pan"]) =  min_angle_[using_joint_name_["head_pan"]] * 0.6;
    target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.25;
    break;
  }

  case TopRight:
  {
    target_position_.coeffRef(0, using_joint_name_["head_pan"]) =  max_angle_[using_joint_name_["head_pan"]] * 0.6;
    target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.25;
    break;
  }

  case BottomLeft:
  {
    target_position_.coeffRef(0, using_joint_name_["head_pan"]) = min_angle_[using_joint_name_["head_pan"]] * 0.45;
    target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.8;
    break;
  }

  case BottomRight:
  {
    target_position_.coeffRef(0, using_joint_name_["head_pan"]) = max_angle_[using_joint_name_["head_pan"]] * 0.45;
    target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.8;
    break;
  }

  default:
    return;
  }

  // set moving time
  moving_time_ = 0.5;               // default : 0.5 sec

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];

    // set time
    double calc_moving_time = fabs(goal_position_.coeff(0, index) - target_position_.coeff(0, index)) / (60.0 * M_PI / 180.0);
    if (calc_moving_time > moving_time_)
      moving_time_ = calc_moving_time;
  }

  // generate trajectory
  tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneThread, this));
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
Eigen::MatrixXd HeadControlModule::calcMinimumJerkTraPVA(double pos_start, double vel_start, double accel_start,
                                                         double pos_end, double vel_end, double accel_end,
                                                         double smp_time, double mov_time)
{
  Eigen::MatrixXd poly_matrix(3, 3);
  Eigen::MatrixXd poly_vector(3, 1);

  poly_matrix << robotis_framework::powDI(mov_time, 3), robotis_framework::powDI(mov_time, 4), robotis_framework::powDI(
        mov_time, 5), 3 * robotis_framework::powDI(mov_time, 2), 4 * robotis_framework::powDI(mov_time, 3), 5
      * robotis_framework::powDI(mov_time, 4), 6 * mov_time, 12 * robotis_framework::powDI(mov_time, 2), 20
      * robotis_framework::powDI(mov_time, 3);

  poly_vector << pos_end - pos_start - vel_start * mov_time - accel_start * pow(mov_time, 2) / 2, vel_end - vel_start
      - accel_start * mov_time, accel_end - accel_start;

  Eigen::MatrixXd poly_coeff = poly_matrix.inverse() * poly_vector;

  int all_time_steps = round(mov_time / smp_time + 1);

  Eigen::MatrixXd time = Eigen::MatrixXd::Zero(all_time_steps, 1);
  Eigen::MatrixXd minimum_jer_tra = Eigen::MatrixXd::Zero(all_time_steps, 3);

  for (int step = 0; step < all_time_steps; step++)
    time.coeffRef(step, 0) = step * smp_time;

  for (int step = 0; step < all_time_steps; step++)
  {
    // position
    minimum_jer_tra.coeffRef(step, 0) = pos_start + vel_start * time.coeff(step, 0)
        + 0.5 * accel_start * robotis_framework::powDI(time.coeff(step, 0), 2)
        + poly_coeff.coeff(0, 0) * robotis_framework::powDI(time.coeff(step, 0), 3)
        + poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 4)
        + poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 5);
    // velocity
    minimum_jer_tra.coeffRef(step, 1) = vel_start + accel_start * time.coeff(step, 0)
        + 3 * poly_coeff.coeff(0, 0) * robotis_framework::powDI(time.coeff(step, 0), 2)
        + 4 * poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 3)
        + 5 * poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 4);
    // accel
    minimum_jer_tra.coeffRef(step, 2) = accel_start + 6 * poly_coeff.coeff(0, 0) * time.coeff(step, 0)
        + 12 * poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 2)
        + 20 * poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 3);
  }

  return minimum_jer_tra;
}

void HeadControlModule::jointTraGeneThread()
{
  tra_lock_.lock();

  double smp_time = control_cycle_msec_ * 0.001;		// ms -> s
  int all_time_steps = int(moving_time_ / smp_time) + 1;

  // for debug
  try
  {
    calc_joint_tra_.resize(all_time_steps, result_.size());
    calc_joint_vel_tra_.resize(all_time_steps, result_.size());
    calc_joint_accel_tra_.resize(all_time_steps, result_.size());
  }
  catch(std::exception &e)
  {
    std::cout << "All step tile : " << all_time_steps << std::endl;
    std::cout << e.what() << std::endl;
    throw;
  }

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];

    double ini_value = goal_position_.coeff(0, index);
    double ini_vel = goal_velocity_.coeff(0, index);
    double ini_accel = goal_acceleration_.coeff(0, index);

    double tar_value = target_position_.coeff(0, index);

    Eigen::MatrixXd tra = calcMinimumJerkTraPVA(ini_value, ini_vel, ini_accel, tar_value, 0.0, 0.0, smp_time,
                                                moving_time_);

    calc_joint_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 0, all_time_steps, 1);
    calc_joint_vel_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 1, all_time_steps, 1);
    calc_joint_accel_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 2, all_time_steps, 1);
  }

  tra_size_ = calc_joint_tra_.rows();
  tra_count_ = 0;

  if (DEBUG)
    ROS_INFO("[ready] make trajectory : %d, %d", tra_size_, tra_count_);

  tra_lock_.unlock();
}

void HeadControlModule::publishStatusMsg(unsigned int type, std::string msg)
{
  ros::Time now = ros::Time::now();

  if(msg.compare(last_msg_) == 0)
  {
    ros::Duration dur = now - last_msg_time_;
    if(dur.sec < 1)
      return;
  }

  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = now;
  status_msg.type = type;
  status_msg.module_name = "Head Control";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);

  last_msg_ = msg;
  last_msg_time_ = now;
}
}
