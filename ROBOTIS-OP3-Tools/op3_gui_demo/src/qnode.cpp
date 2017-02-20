/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Kayman Jung */

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "../include/op3_gui_demo/qnode.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace robotis_op
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

QNodeOP3::QNodeOP3(int argc, char** argv)
    : init_argc_(argc),
      init_argv_(argv)
{
  // code to DEBUG
  debug_ = false;

  if (argc >= 2)
  {
    std::string arg_code(argv[1]);
    if (arg_code == "debug")
      debug_ = true;
    else
      debug_ = false;
  }
}

QNodeOP3::~QNodeOP3()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNodeOP3::init()
{
  ros::init(init_argc_, init_argv_, "op3_gui_demo");

  if (!ros::master::check())
  {
    return false;
  }

  ros::start();  // explicitly needed since our nodehandle is going out of scope.

  ros::NodeHandle ros_node;

  // Add your ros communications here.
  module_control_pub_ = ros_node.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules",
                                                                                     0);
  module_control_preset_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  init_pose_pub_ = ros_node.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  init_gyro_pub_ = ros_node.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  set_head_joint_angle_pub_ = ros_node.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states", 0);

  status_msg_sub_ = ros_node.subscribe("/robotis/status", 10, &QNodeOP3::statusMsgCallback, this);
  current_module_control_sub_ = ros_node.subscribe("/robotis/present_joint_ctrl_modules", 10,
                                                   &QNodeOP3::refreshCurrentJointControlCallback, this);
  current_joint_states_sub_ = ros_node.subscribe("/robotis/present_joint_states", 10,
                                                 &QNodeOP3::updateHeadJointStatesCallback, this);

  get_module_control_client_ = ros_node.serviceClient<robotis_controller_msgs::GetJointModule>(
      "/robotis/get_present_joint_ctrl_modules");

  // Walking
  set_walking_command_pub = ros_node.advertise<std_msgs::String>("/robotis/walking/command", 0);
  set_walking_param_pub = ros_node.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 0);
  get_walking_param_client_ = ros_node.serviceClient<op3_walking_module_msgs::GetWalkingParam>(
      "/robotis/walking/get_params");

  // Action
  motion_index_pub_ = ros_node.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);

  // Demo
  demo_command_pub_ = ros_node.advertise<std_msgs::String>("/ball_tracker/command", 0);

  // Config
  std::string default_path = ros::package::getPath("op3_gui_demo") + "/config/demo_config.yaml";
  std::string config_path = ros_node.param<std::string>("demo_config", default_path);
  parseJointNameFromYaml(config_path);

  std::string default_motion_path = ros::package::getPath("op3_gui_demo") + "/config/motion.yaml";
  parseMotionMapFromYaml(default_motion_path);

  // start time
  start_time_ = ros::Time::now();

  // start qthread
  start();

  return true;
}

void QNodeOP3::run()
{
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNodeOP3::parseJointNameFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load id_joint table yaml.");
    return;
  }

  // parse id_joint table
  YAML::Node id_sub_node = doc["id_joint"];
  for (YAML::iterator _it = id_sub_node.begin(); _it != id_sub_node.end(); ++_it)
  {
    int joint_id;
    std::string joint_name;

    joint_id = _it->first.as<int>();
    joint_name = _it->second.as<std::string>();

    id_joint_table_[joint_id] = joint_name;
    joint_id_table_[joint_name] = joint_id;

    if (debug_)
      std::cout << "ID : " << joint_id << " - " << joint_name << std::endl;
  }

  // parse module
  std::vector<std::string> modules = doc["module_list"].as<std::vector<std::string> >();

  int module_index = 0;
  for (std::vector<std::string>::iterator modules_it = modules.begin(); modules_it != modules.end(); ++modules_it)
  {
    std::string module_name = *modules_it;

    index_mode_table_[module_index] = module_name;
    mode_index_table_[module_name] = module_index++;

    using_mode_table_[module_name] = false;
  }

  // parse module_joint preset
  YAML::Node sub_node = doc["module_button"];
  for (YAML::iterator yaml_it = sub_node.begin(); yaml_it != sub_node.end(); ++yaml_it)
  {
    int key_index;
    std::string module_name;

    key_index = yaml_it->first.as<int>();
    module_name = yaml_it->second.as<std::string>();

    module_table_[key_index] = module_name;
    if (debug_)
      std::cout << "Preset : " << module_name << std::endl;
  }
}

void QNodeOP3::parseMotionMapFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load motion yaml.");
    return;
  }

  // parse motion_table
  YAML::Node motion_sub_node = doc["motion"];
  for (YAML::iterator yaml_it = motion_sub_node.begin(); yaml_it != motion_sub_node.end(); ++yaml_it)
  {
    int motion_index;
    std::string motion_name;

    motion_index = yaml_it->first.as<int>();
    motion_name = yaml_it->second.as<std::string>();

    motion_table_[motion_index] = motion_name;

    // if(DEBUG) std::cout << "Motion Index : " << _motion_index << " - " << _motion_name << std::endl;
  }

  // parse shortcut_table
  YAML::Node _shoutcut_sub_node = doc["motion_shortcut"];
  for (YAML::iterator _it = _shoutcut_sub_node.begin(); _it != _shoutcut_sub_node.end(); ++_it)
  {
    int shortcut_prefix = 0x30;
    int motion_index;
    int shortcut_index;

    motion_index = _it->first.as<int>();
    shortcut_index = _it->second.as<int>();

    if (shortcut_index < 0 || shortcut_index > 9)
      continue;

    motion_shortcut_table_[motion_index] = shortcut_index + shortcut_prefix;

    // if(DEBUG) std::cout << "Motion Index : " << _motion_index << " - " << _motion_name << std::endl;
  }
}

// joint id -> joint name
bool QNodeOP3::getJointNameFromID(const int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator map_it;

  map_it = id_joint_table_.find(id);
  if (map_it == id_joint_table_.end())
    return false;

  joint_name = map_it->second;
  return true;
}

// joint name -> joint id
bool QNodeOP3::getIDFromJointName(const std::string &joint_name, int &id)
{
  std::map<std::string, int>::iterator map_it;

  map_it = joint_id_table_.find(joint_name);
  if (map_it == joint_id_table_.end())
    return false;

  id = map_it->second;
  return true;
}

// map index -> joint id & joint name
bool QNodeOP3::getIDJointNameFromIndex(const int &index, int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator map_it;
  int count = 0;
  for (map_it = id_joint_table_.begin(); map_it != id_joint_table_.end(); ++map_it, count++)
  {
    if (index == count)
    {
      id = map_it->first;
      joint_name = map_it->second;
      return true;
    }
  }
  return false;
}

// mode(module) index -> mode(module) name
std::string QNodeOP3::getModeName(const int &index)
{
  std::string mode = "";
  std::map<int, std::string>::iterator map_it = index_mode_table_.find(index);

  if (map_it != index_mode_table_.end())
    mode = map_it->second;

  return mode;
}

// mode(module) name -> mode(module) index
int QNodeOP3::getModeIndex(const std::string &mode_name)
{
  int mode_index = -1;
  std::map<std::string, int>::iterator map_it = mode_index_table_.find(mode_name);

  if (map_it != mode_index_table_.end())
    mode_index = map_it->second;

  return mode_index;
}

// number of mode(module)s
int QNodeOP3::getModeSize()
{
  return index_mode_table_.size();
}

// number of joints
int QNodeOP3::getJointSize()
{
  return id_joint_table_.size();
}

void QNodeOP3::clearUsingModule()
{
  for (std::map<std::string, bool>::iterator map_it = using_mode_table_.begin(); map_it != using_mode_table_.end();
      ++map_it)
    map_it->second = false;
}

bool QNodeOP3::isUsingModule(std::string module_name)
{
  std::map<std::string, bool>::iterator map_it = using_mode_table_.find(module_name);

  if (map_it == using_mode_table_.end())
    return false;

  return map_it->second;
}

// move ini pose : wholedody module
void QNodeOP3::moveInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub_.publish(init_msg);

  log(Info, "Go to robot initial pose.");
}

// set mode(module) to each joint
void QNodeOP3::setJointControlMode(const robotis_controller_msgs::JointCtrlModule &msg)
{
  module_control_pub_.publish(msg);
}

void QNodeOP3::setControlMode(const std::string &mode)
{
  std_msgs::String set_module_msg;
  set_module_msg.data = mode;

  module_control_preset_pub_.publish(set_module_msg);

  std::stringstream _ss;
  _ss << "Set Mode : " << mode;
  log(Info, _ss.str());
}

// get current mode(module) of joints
void QNodeOP3::getJointControlMode()
{
  robotis_controller_msgs::GetJointModule get_joint;
  std::map<std::string, int> service_map;

  // _get_joint.request
  std::map<int, std::string>::iterator map_it;
  int index = 0;
  for (map_it = id_joint_table_.begin(); map_it != id_joint_table_.end(); ++map_it, index++)
  {
    get_joint.request.joint_name.push_back(map_it->second);
    service_map[map_it->second] = index;
  }

  if (get_module_control_client_.call(get_joint))
  {
    // _get_joint.response
    std::vector<int> modules;
    modules.resize(getJointSize());

    // clear current using modules
    clearUsingModule();

    for (int ix = 0; ix < get_joint.response.joint_name.size(); ix++)
    {
      std::string joint_name = get_joint.response.joint_name[ix];
      std::string module_name = get_joint.response.module_name[ix];

      std::map<std::string, int>::iterator service_it = service_map.find(joint_name);
      if (service_it == service_map.end())
        continue;

      index = service_it->second;

      service_it = mode_index_table_.find(module_name);
      if (service_it == mode_index_table_.end())
        continue;

      // ROS_INFO_STREAM("joint[" << ix << "] : " << _service_iter->second);
      modules.at(index) = service_it->second;

      std::map<std::string, bool>::iterator module_it = using_mode_table_.find(module_name);
      if (module_it != using_mode_table_.end())
        module_it->second = true;
    }

    // update ui
    Q_EMIT updateCurrentJointControlMode(modules);
    log(Info, "Get current Mode");
  }
  else
    log(Error, "Fail to get current joint control module.");
}

void QNodeOP3::refreshCurrentJointControlCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg)
{
  ROS_INFO("set current joint module");
  //int _index = 0;

  std::vector<int> modules;
  modules.resize(getJointSize());

  std::map<std::string, int> joint_module_map;

  // clear current using modules
  clearUsingModule();

  for (int ix = 0; ix < msg->joint_name.size(); ix++)
  {
    std::string joint_name = msg->joint_name[ix];
    std::string module_name = msg->module_name[ix];

    joint_module_map[joint_name] = getModeIndex(module_name);

    std::map<std::string, bool>::iterator module_it = using_mode_table_.find(module_name);
    if (module_it != using_mode_table_.end())
      module_it->second = true;
  }

  for (int ix = 0; ix < getJointSize(); ix++)
  {
    int id = 0;
    std::string joint_name = "";

    if (getIDJointNameFromIndex(ix, id, joint_name) == false)
      continue;

    std::map<std::string, int>::iterator module_it = joint_module_map.find(joint_name);
    if (module_it == joint_module_map.end())
      continue;

    // ROS_INFO_STREAM("joint[" << ix << "] : " << _module_iter->second);
    modules.at(ix) = module_it->second;
  }

  // update ui
  Q_EMIT updateCurrentJointControlMode(modules);

  log(Info, "Applied Mode", "Manager");
}

void QNodeOP3::updateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  double head_pan, head_tilt;
  int num_get = 0;

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    if (msg->name[ix] == "head_pan")
    {
      head_pan = -msg->position[ix];
      num_get += 1;
    }
    else if (msg->name[ix] == "head_tilt")
    {
      head_tilt = msg->position[ix];
      num_get += 1;
    }

    if (num_get == 2)
      break;
  }

  if (num_get > 0)
    Q_EMIT updateHeadAngles(head_pan, head_tilt);
}

void QNodeOP3::setHeadJoint(double pan, double tilt)
{
  sensor_msgs::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");

  head_angle_msg.position.push_back(-pan);
  head_angle_msg.position.push_back(tilt);

  set_head_joint_angle_pub_.publish(head_angle_msg);
}

// Walking
void QNodeOP3::setWalkingCommand(const std::string &command)
{
  std_msgs::String _commnd_msg;
  _commnd_msg.data = command;
  set_walking_command_pub.publish(_commnd_msg);

  std::stringstream ss_log;
  ss_log << "Set Walking Command : " << _commnd_msg.data << std::endl;

  log(Info, ss_log.str());
}

void QNodeOP3::refreshWalkingParam()
{
  op3_walking_module_msgs::GetWalkingParam walking_param_msg;

  if (get_walking_param_client_.call(walking_param_msg))
  {
    walking_param_ = walking_param_msg.response.parameters;

    // update ui
    Q_EMIT updateWalkingParameters(walking_param_);
    log(Info, "Get walking parameters");
  }
  else
    log(Error, "Fail to get walking parameters.");
}

void QNodeOP3::saveWalkingParam()
{
  std_msgs::String command_msg;
  command_msg.data = "save";
  set_walking_command_pub.publish(command_msg);

  log(Info, "Save Walking parameters.");
}

void QNodeOP3::applyWalkingParam(const op3_walking_module_msgs::WalkingParam &walking_param)
{
  walking_param_ = walking_param;

  set_walking_param_pub.publish(walking_param_);
  log(Info, "Apply Walking parameters.");
}

void QNodeOP3::initGyro()
{
  robotis_controller_msgs::SyncWriteItem init_gyro_msg;
  init_gyro_msg.item_name = "imu_control";
  init_gyro_msg.joint_name.push_back("open-cr");
  init_gyro_msg.value.push_back(0x08);

  init_gyro_pub_.publish(init_gyro_msg);

  log(Info, "Initialize Gyro");
}

//void QNodeOP3::setWalkingBalance(bool on_command)
//{
//    std_msgs::Bool _msg;
//    _msg.data = on_command;

//    set_walking_balance_pub.publish(_msg);

//    std::stringstream _ss;
//    _ss << "Set Walking Balance : " << (on_command ? "True" : "False");

//    log(Info, _ss.str());
//}

// Motion
void QNodeOP3::playMotion(int motion_index)
{
  if (motion_table_.find(motion_index) == motion_table_.end())
  {
    log(Error, "Motion index is not valid.");
    return;
  }

  std::stringstream log_ss;
  switch (motion_index)
  {
    case -2:
      log_ss << "Break Motion";
      break;

    case -1:
      log_ss << "STOP Motion";
      break;

    default:
      std::string _motion_name = motion_table_[motion_index];
      log_ss << "Play Motion : [" << motion_index << "] " << _motion_name;
  }

  // publish motion index
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_.publish(motion_msg);

  log(Info, log_ss.str());
}

// Demo
void QNodeOP3::setDemoCommand(const std::string &command)
{
  std_msgs::String demo_msg;
  demo_msg.data = command;

  demo_command_pub_.publish(demo_msg);

  std::stringstream log_ss;
  log_ss << "Demo command : " << command;
  log(Info, log_ss.str());
}

void QNodeOP3::setActionModuleBody()
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  std::string module_name = "action_module";

  for (int ix = 1; ix <= 18; ix++)
  {
    std::string joint_name;

    if (getJointNameFromID(ix, joint_name) == false)
      continue;

    control_msg.joint_name.push_back(joint_name);
    control_msg.module_name.push_back(module_name);
  }

  // no control
  if (control_msg.joint_name.size() == 0)
    return;

  setJointControlMode(control_msg);
}

void QNodeOP3::setModuleToDemo()
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  std::string body_module = "walking_module";
  std::string head_module = "head_control_module";

  for (int ix = 1; ix <= 20; ix++)
  {
    std::string joint_name;

    if (getJointNameFromID(ix, joint_name) == false)
      continue;

    control_msg.joint_name.push_back(joint_name);
    if (ix <= 18)
      control_msg.module_name.push_back(body_module);
    else
      control_msg.module_name.push_back(head_module);

  }

  // no control
  if (control_msg.joint_name.size() == 0)
    return;

  setJointControlMode(control_msg);
}

// LOG
void QNodeOP3::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  log((LogLevel) msg->type, msg->status_msg, msg->module_name);
}

void QNodeOP3::log(const LogLevel &level, const std::string &msg, std::string sender)
{
  logging_model_.insertRows(logging_model_.rowCount(), 1);
  std::stringstream logging_model_msg;

  ros::Duration duration_time = ros::Time::now() - start_time_;
  int current_time = duration_time.sec;
  int min_time = 0, sec_time = 0;
  min_time = (int) (current_time / 60);
  sec_time = (int) (current_time % 60);

  std::stringstream min_str, sec_str;
  if (min_time < 10)
    min_str << "0";
  if (sec_time < 10)
    sec_str << "0";
  min_str << min_time;
  sec_str << sec_time;

  std::stringstream sender_ss;
  sender_ss << "[" << sender << "] ";

  switch (level)
  {
    case (Debug):
    {
      ROS_DEBUG_STREAM(msg);
      logging_model_msg << "[DEBUG] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
    case (Info):
    {
      ROS_INFO_STREAM(msg);
      logging_model_msg << "[INFO] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
    case (Warn):
    {
      ROS_WARN_STREAM(msg);
      logging_model_msg << "[WARN] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
    case (Error):
    {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "<ERROR> [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
    case (Fatal):
    {
      ROS_FATAL_STREAM(msg);
      logging_model_msg << "[FATAL] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount() - 1), new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

void QNodeOP3::clearLog()
{
  if (logging_model_.rowCount() == 0)
    return;

  logging_model_.removeRows(0, logging_model_.rowCount());
}

}  // namespace robotis_op
