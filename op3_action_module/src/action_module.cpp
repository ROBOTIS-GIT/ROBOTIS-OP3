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

/* Authors: Kayman, Jay Song */

#include <stdio.h>
#include <sstream>
#include "op3_action_module/action_module.h"

namespace robotis_op
{

std::string ActionModule::convertIntToString(int n)
{
  std::ostringstream ostr;
  ostr << n;
  return ostr.str();
}

ActionModule::ActionModule()
    : control_cycle_msec_(8),
      PRE_SECTION(0),
      MAIN_SECTION(1),
      POST_SECTION(2),
      PAUSE_SECTION(3),
      ZERO_FINISH(0),
      NONE_ZERO_FINISH(1),
      DEBUG_PRINT(false)
{
  /////////////// Const Variable
  /**************************************
   * Section             /----\
       *                    /|    |\
       *        /+---------/ |    | \
       *       / |        |  |    |  \
       * -----/  |        |  |    |   \----
   *      PRE  MAIN   PRE MAIN POST PAUSE
   ***************************************/

  enable_ = false;
  module_name_ = "action_module";  // set unique module name
  control_mode_ = robotis_framework::PositionControl;

  //////////////////////////////////
  action_file_ = 0;
  playing_ = false;
  first_driving_start_ = false;
  playing_finished_ = true;
  page_step_count_ = 0;
  play_page_idx_ = 0;
  stop_playing_ = true;

  action_module_enabled_ = false;
  previous_running_ = false;
  present_running_ = false;
}

ActionModule::~ActionModule()
{
  queue_thread_.join();

  ////////////////////////////////////////
  if (action_file_ != 0)
    fclose(action_file_);
}

void ActionModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&ActionModule::queueThread, this));

  // init result, joint_id_table
  for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
      it != robot->dxls_.end(); it++)
  {
    std::string joint_name = it->first;
    robotis_framework::Dynamixel* dxl_info = it->second;

    joint_name_to_id_[joint_name] = dxl_info->id_;
    joint_id_to_name_[dxl_info->id_] = joint_name;
    action_result_[joint_name] = new robotis_framework::DynamixelState();
    action_result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
    action_joints_enable_[joint_name] = false;
  }

  ros::NodeHandle ros_node;

  std::string path = ros::package::getPath("op3_action_module") + "/data/motion_4095.bin";
  std::string action_file_path = ros_node.param<std::string>("action_file_path", path);

  loadFile(action_file_path);

  playing_ = false;
}

void ActionModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publisher */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 0);
  done_msg_pub_ = ros_node.advertise<std_msgs::String>("/robotis/movement_done", 1);

  /* subscriber */
  ros::Subscriber action_page_sub = ros_node.subscribe("/robotis/action/page_num", 0, &ActionModule::pageNumberCallback,
                                                       this);
  ros::Subscriber start_action_sub = ros_node.subscribe("/robotis/action/start_action", 0,
                                                        &ActionModule::startActionCallback, this);

  /* ROS Service Callback Functions */
  ros::ServiceServer is_running_server = ros_node.advertiseService("/robotis/action/is_running",
                                                                   &ActionModule::isRunningServiceCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

bool ActionModule::isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request &req,
                                            op3_action_module_msgs::IsRunning::Response &res)
{
  res.is_running = isRunning();
  return true;
}

void ActionModule::pageNumberCallback(const std_msgs::Int32::ConstPtr& msg)
{
  if (enable_ == false)
  {
    std::string status_msg = "Action Module is not enabled";
    ROS_INFO_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return;
  }

  if (msg->data == -1)
  {
    stop();
  }
  else if (msg->data == -2)
  {
    brake();
  }
  else
  {
    for (std::map<std::string, bool>::iterator joints_enable_it = action_joints_enable_.begin();
        joints_enable_it != action_joints_enable_.end(); joints_enable_it++)
      joints_enable_it->second = true;

    if (start(msg->data) == true)
    {
      std::string status_msg = "Succeed to start page " + convertIntToString(msg->data);
      ROS_INFO_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
    else
    {
      std::string status_msg = "Failed to start page " + convertIntToString(msg->data);
      ROS_ERROR_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      publishDoneMsg("action_failed");
    }
  }
}

void ActionModule::startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr& msg)
{
  if (enable_ == false)
  {
    std::string status_msg = "Action Module is not enabled";
    ROS_INFO_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return;
  }

  if (msg->page_num == -1)
  {
    stop();
  }
  else if (msg->page_num == -2)
  {
    brake();
  }
  else
  {
    for (std::map<std::string, bool>::iterator joints_enable_it = action_joints_enable_.begin();
        joints_enable_it != action_joints_enable_.end(); joints_enable_it++)
      joints_enable_it->second = false;

    int joint_name_array_size = msg->joint_name_array.size();
    std::map<std::string, bool>::iterator joints_enable_it = action_joints_enable_.begin();
    for (int joint_idx = 0; joint_idx < joint_name_array_size; joint_idx++)
    {
      joints_enable_it = action_joints_enable_.find(msg->joint_name_array[joint_idx]);
      if (joints_enable_it == action_joints_enable_.end())
      {
        std::string status_msg = "Invalid Joint Name : " + msg->joint_name_array[joint_idx];
        ROS_INFO_STREAM(status_msg);
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
        publishDoneMsg("action_failed");
        return;
      }
      else
      {
        joints_enable_it->second = true;
      }
    }

    if (start(msg->page_num) == true)
    {
      std::string status_msg = "Succeed to start page " + convertIntToString(msg->page_num);
      ROS_INFO_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
    else
    {
      std::string status_msg = "Failed to start page " + convertIntToString(msg->page_num);
      ROS_ERROR_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      publishDoneMsg("action_failed");
    }
  }
}

void ActionModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                           std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  if (action_module_enabled_ == true)
  {
    for (std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it = dxls.begin(); dxls_it != dxls.end();
        dxls_it++)
    {
      std::string joint_name = dxls_it->first;

      std::map<std::string, robotis_framework::DynamixelState*>::iterator result_it = result_.find(joint_name);
      if (result_it == result_.end())
        continue;
      else
      {
        result_it->second->goal_position_ = dxls_it->second->dxl_state_->goal_position_;
        action_result_[joint_name]->goal_position_ = dxls_it->second->dxl_state_->goal_position_;
      }
    }
    action_module_enabled_ = false;
  }

  actionPlayProcess(dxls);

  for (std::map<std::string, bool>::iterator action_enable_it = action_joints_enable_.begin();
      action_enable_it != action_joints_enable_.end(); action_enable_it++)
  {
    if (action_enable_it->second == true)
      result_[action_enable_it->first]->goal_position_ = action_result_[action_enable_it->first]->goal_position_;
  }

  previous_running_ = present_running_;
  present_running_ = isRunning();

  if (present_running_ != previous_running_)
  {
    if (present_running_ == true)
    {
      std::string status_msg = "Action_Start";
      //ROS_INFO_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
    else
    {
      for (std::map<std::string, robotis_framework::DynamixelState*>::iterator action_result_it =
          action_result_.begin(); action_result_it != action_result_.end(); action_result_it++)
        action_result_it->second->goal_position_ = result_[action_result_it->first]->goal_position_;

      std::string status_msg = "Action_Finish";
      //ROS_INFO_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
      publishDoneMsg("action");
    }
  }
}

void ActionModule::onModuleEnable()
{
  action_module_enabled_ = true;
}

void ActionModule::onModuleDisable()
{
  action_module_enabled_ = false;
  brake();
}

void ActionModule::stop()
{
  stop_playing_ = true;
}

bool ActionModule::isRunning()
{
  return playing_;
}

int ActionModule::convertRadTow4095(double rad)
{
  return (int) ((rad + M_PI) * 2048.0 / M_PI);
}

double ActionModule::convertw4095ToRad(int w4095)
{
  return (w4095 - 2048) * M_PI / 2048.0;
}

bool ActionModule::verifyChecksum(action_file_define::Page* page)
{
  unsigned char checksum = 0x00;
  unsigned char* pt = (unsigned char*) page;

  for (unsigned int i = 0; i < sizeof(action_file_define::Page); i++)
  {
    checksum += *pt;
    pt++;
  }

  if (checksum != 0xff)
    return false;

  return true;
}

void ActionModule::setChecksum(action_file_define::Page* page)
{
  unsigned char checksum = 0x00;
  unsigned char* pt = (unsigned char*) page;

  page->header.checksum = 0x00;

  for (unsigned int i = 0; i < sizeof(action_file_define::Page); i++)
  {
    checksum += *pt;
    pt++;
  }

  page->header.checksum = (unsigned char) (0xff - checksum);
}

bool ActionModule::loadFile(std::string file_name)
{
  FILE* action = fopen(file_name.c_str(), "r+b");
  if (action == 0)
  {
    std::string status_msg = "Can not open Action file!";
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return false;
  }

  fseek(action, 0, SEEK_END);
  if (ftell(action) != (long) (sizeof(action_file_define::Page) * action_file_define::MAXNUM_PAGE))
  {
    std::string status_msg = "It's not an Action file!";
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    fclose(action);
    return false;
  }

  if (action_file_ != 0)
    fclose(action_file_);

  action_file_ = action;
  return true;
}

bool ActionModule::createFile(std::string file_name)
{
  FILE* action = fopen(file_name.c_str(), "ab");
  if (action == 0)
  {
    std::string status_msg = "Can not create Action file!";
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return false;
  }

  action_file_define::Page page;
  resetPage(&page);

  for (int i = 0; i < action_file_define::MAXNUM_PAGE; i++)
    fwrite((const void *) &page, 1, sizeof(action_file_define::Page), action);

  if (action_file_ != 0)
    fclose(action_file_);

  action_file_ = action;

  return true;
}

bool ActionModule::start(int page_number)
{
  if (page_number < 1 || page_number >= action_file_define::MAXNUM_PAGE)
  {

    std::string status_msg = "Can not play page.(" + convertIntToString(page_number) + " is invalid index)";
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return false;
  }

  action_file_define::Page page;
  if (loadPage(page_number, &page) == false)
    return false;

  return start(page_number, &page);
}

bool ActionModule::start(std::string page_name)
{
  int index;
  action_file_define::Page page;

  for (index = 1; index < action_file_define::MAXNUM_PAGE; index++)
  {
    if (loadPage(index, &page) == false)
      return false;

    if (strcmp(page_name.c_str(), (char*) page.header.name) == 0)
      break;
  }

  if (index == action_file_define::MAXNUM_PAGE)
  {
    std::string str_name_page = page_name;
    std::string status_msg = "Can not play page.(" + str_name_page + " is invalid name)\n";
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return false;
  }
  else
    return start(index, &page);
}

bool ActionModule::start(int page_number, action_file_define::Page* page)
{
  if (enable_ == false)
  {
    std::string status_msg = "Action Module is disabled";
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return false;
  }

  if (playing_ == true)
  {
    std::string status_msg = "Can not play page " + convertIntToString(page_number) + ".(Now playing)";
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return false;
  }

  play_page_ = *page;

  if (play_page_.header.repeat == 0 || play_page_.header.stepnum == 0)
  {
    std::string status_msg = "Page " + convertIntToString(page_number) + " has no action\n";
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return false;
  }

  play_page_idx_ = page_number;
  first_driving_start_ = true;
  playing_ = true;

  return true;
}

void ActionModule::brake()
{
  playing_ = false;
}

bool ActionModule::isRunning(int* playing_page_num, int* playing_step_num)
{
  if (playing_page_num != 0)
    *playing_page_num = play_page_idx_;

  if (playing_step_num != 0)
    *playing_step_num = page_step_count_ - 1;

  return isRunning();
}

bool ActionModule::loadPage(int page_number, action_file_define::Page* page)
{
  if (page_number < 0 || page_number >= action_file_define::MAXNUM_PAGE)
    return false;

  long position = (long) (sizeof(action_file_define::Page) * page_number);

  if (fseek(action_file_, position, SEEK_SET) != 0)
    return false;

  if (fread(page, 1, sizeof(action_file_define::Page), action_file_) != sizeof(action_file_define::Page))
    return false;

  if (verifyChecksum(page) == false)
    resetPage(page);

  return true;
}

bool ActionModule::savePage(int page_number, action_file_define::Page* page)
{
  long position = (long) (sizeof(action_file_define::Page) * page_number);

  if (verifyChecksum(page) == false)
    setChecksum(page);

  if (fseek(action_file_, position, SEEK_SET) != 0)
    return false;

  if (fwrite(page, 1, sizeof(action_file_define::Page), action_file_) != sizeof(action_file_define::Page))
    return false;

  return true;
}

void ActionModule::resetPage(action_file_define::Page* page)
{
  unsigned char *pt = (unsigned char*) page;

  for (unsigned int i = 0; i < sizeof(action_file_define::Page); i++)
  {
    *pt = 0x00;
    pt++;
  }

  page->header.schedule = action_file_define::TIME_BASE_SCHEDULE;  // default time base
  page->header.repeat = 1;
  page->header.speed = 32;
  page->header.accel = 32;

  for (int i = 0; i < action_file_define::MAXNUM_JOINTS; i++)
    page->header.pgain[i] = 0x55;

  for (int i = 0; i < action_file_define::MAXNUM_STEP; i++)
  {
    for (int j = 0; j < action_file_define::MAXNUM_JOINTS; j++)
      page->step[i].position[j] = action_file_define::INVALID_BIT_MASK;

    page->step[i].pause = 0;
    page->step[i].time = 0;
  }

  setChecksum(page);
}

void ActionModule::enableAllJoints()
{
  for (std::map<std::string, bool>::iterator it = action_joints_enable_.begin(); it != action_joints_enable_.end();
      it++)
  {
    it->second = true;
  }
}

void ActionModule::actionPlayProcess(std::map<std::string, robotis_framework::Dynamixel *> dxls)
{
  //////////////////// local Variable
  uint8_t id;
  uint32_t total_time_256t;
  uint32_t pre_section_time_256t;
  uint32_t main_time_256t;
  int32_t start_speed1024_pre_time_256t;
  int32_t moving_angle_speed1024_scale_256t_2t;
  int32_t divider1, divider2;

  int16_t max_angle;
  int16_t max_speed;
  int16_t tmp;
  int16_t prev_target_angle;  // Start position
  int16_t curr_target_angle;  // Target position
  int16_t next_target_angle;  // Next target position
  uint8_t direction_changed;
  int16_t speed_n;

  ///////////////// Static Variable
  static uint16_t start_angle[action_file_define::MAXNUM_JOINTS];    // Start point of interpolation
  static uint16_t target_angle[action_file_define::MAXNUM_JOINTS];   // Target point of interpolation
  static int16_t moving_angle[action_file_define::MAXNUM_JOINTS];   // Total Moving Angle
  static int16_t main_angle[action_file_define::MAXNUM_JOINTS];     // Moving angle at Constant Velocity Section
  static int16_t accel_angle[action_file_define::MAXNUM_JOINTS];    // Moving angle at Acceleration Section
  static int16_t main_speed[action_file_define::MAXNUM_JOINTS];     // Target constant velocity
  static int16_t last_out_speed[action_file_define::MAXNUM_JOINTS];  // Velocity of Previous State
  static int16_t goal_speed[action_file_define::MAXNUM_JOINTS];     // Target velocity
  static uint8_t finish_type[action_file_define::MAXNUM_JOINTS];    // Desired State at Target angle

  static uint16_t unit_time_count;
  static uint16_t unit_time_num;
  static uint16_t pause_time;
  static uint16_t unit_time_total_num;
  static uint16_t accel_step;
  static uint8_t section;
  static uint8_t play_repeat_count;
  static uint16_t next_play_page;

  /////////////// Const Variable
  /**************************************
   * Section             /----\
    *                    /|    |\
    *        /+---------/ |    | \
    *       / |        |  |    |  \
    * -----/  |        |  |    |   \----
   *      PRE  MAIN   PRE MAIN POST PAUSE
   ***************************************/

  if (playing_ == false)
  {
    for (std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it = dxls.begin(); dxls_it != dxls.end();
        dxls_it++)
    {
      std::string joint_name = dxls_it->first;

      std::map<std::string, robotis_framework::DynamixelState*>::iterator result_it = action_result_.find(joint_name);
      if (result_it == result_.end())
        continue;
      else
      {
        result_it->second->goal_position_ = dxls_it->second->dxl_state_->goal_position_;
      }
    }
    return;
  }

  if (first_driving_start_ == true)  // First start
  {
    first_driving_start_ = false;  //First Process end
    playing_finished_ = false;
    stop_playing_ = false;
    unit_time_count = 0;
    unit_time_num = 0;
    pause_time = 0;
    section = PAUSE_SECTION;
    page_step_count_ = 0;
    play_repeat_count = play_page_.header.repeat;
    next_play_page = 0;

    for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
    {
      id = joint_index;
      std::string joint_name = "";

      std::map<int, std::string>::iterator id_to_name_it = joint_id_to_name_.find(id);
      if (id_to_name_it == joint_id_to_name_.end())
        continue;
      else
        joint_name = id_to_name_it->second;

      std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it = dxls.find(joint_name);
      if (dxls_it == dxls.end())
        continue;
      else
      {
        double goal_joint_angle_rad = dxls_it->second->dxl_state_->goal_position_;
        target_angle[id] = convertRadTow4095(goal_joint_angle_rad);
        last_out_speed[id] = 0;
        moving_angle[id] = 0;
        goal_speed[id] = 0;
      }
    }
  }

  if (unit_time_count < unit_time_num)  // Ongoing
  {
    unit_time_count++;
    if (section == PAUSE_SECTION)
    {
    }
    else
    {
      for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
      {
        id = joint_index;
        std::string joint_name = "";

        std::map<int, std::string>::iterator id_to_name_it = joint_id_to_name_.find(id);
        if (id_to_name_it == joint_id_to_name_.end())
          continue;
        else
          joint_name = id_to_name_it->second;

        std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it = dxls.find(joint_name);
        if (dxls_it == dxls.end())
        {
          continue;
        }
        else
        {
          if (moving_angle[id] == 0)
          {
            action_result_[joint_name]->goal_position_ = convertw4095ToRad(start_angle[id]);
          }
          else
          {
            if (section == PRE_SECTION)
            {
              speed_n = (short) (((long) (main_speed[id] - last_out_speed[id]) * unit_time_count) / unit_time_num);
              goal_speed[id] = last_out_speed[id] + speed_n;
              accel_angle[id] = (short) ((((long) (last_out_speed[id] + (speed_n >> 1)) * unit_time_count * 144) / 15)
                  >> 9);

              action_result_[joint_name]->goal_position_ = convertw4095ToRad(start_angle[id] + accel_angle[id]);
            }
            else if (section == MAIN_SECTION)
            {
              action_result_[joint_name]->goal_position_ = convertw4095ToRad(
                  start_angle[id] + (short int) (((long) (main_angle[id]) * unit_time_count) / unit_time_num));

              goal_speed[id] = main_speed[id];
            }
            else  // POST_SECTION
            {
              if (unit_time_count == (unit_time_num - 1))
              {
                // use target angle in order to reduce the last step error
                action_result_[joint_name]->goal_position_ = convertw4095ToRad(target_angle[id]);
              }
              else
              {
                if (finish_type[id] == ZERO_FINISH)
                {
                  speed_n = (short int) (((long) (0 - last_out_speed[id]) * unit_time_count) / unit_time_num);
                  goal_speed[id] = last_out_speed[id] + speed_n;

                  action_result_[joint_name]->goal_position_ =
                      convertw4095ToRad(
                          start_angle[id]
                              + (short) ((((long) (last_out_speed[id] + (speed_n >> 1)) * unit_time_count * 144) / 15)
                                  >> 9));

                }
                else  // NONE_ZERO_FINISH
                {
                  // Same as MAIN Section
                  // because some servos need to be rotate, others do not.
                  action_result_[joint_name]->goal_position_ = convertw4095ToRad(
                      start_angle[id] + (short int) (((long) (main_angle[id]) * unit_time_count) / unit_time_num));

                  goal_speed[id] = main_speed[id];
                }
              }
            }
          }
        }
      }
    }
  }
  else if (unit_time_count >= unit_time_num)  // If current section is completed
  {
    unit_time_count = 0;

    for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
    {
      id = joint_index;
      std::string joint_name = "";
      std::map<int, std::string>::iterator id_to_name_it = joint_id_to_name_.find(id);
      if (id_to_name_it == joint_id_to_name_.end())
        continue;
      else
        joint_name = id_to_name_it->second;

      std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it = dxls.find(joint_name);
      if (dxls_it == dxls.end())
        continue;
      else
      {
        double _goal_joint_angle_rad = dxls_it->second->dxl_state_->goal_position_;
        start_angle[id] = convertRadTow4095(_goal_joint_angle_rad);
        last_out_speed[id] = goal_speed[id];
      }
    }

    // Update section ( PRE -> MAIN -> POST -> (PAUSE or PRE) ... )
    if (section == PRE_SECTION)
    {
      // Prepare for MAIN Section
      section = MAIN_SECTION;
      unit_time_num = unit_time_total_num - (accel_step << 1);

      for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
      {
        id = joint_index;

        if (finish_type[id] == NONE_ZERO_FINISH)
        {
          if ((unit_time_total_num - accel_step) == 0)  // if there is not any constant velocity section
            main_angle[id] = 0;
          else
            main_angle[id] = (short) ((((long) (moving_angle[id] - accel_angle[id])) * unit_time_num)
                / (unit_time_total_num - accel_step));
        }
        else
          // ZERO_FINISH
          main_angle[id] = moving_angle[id] - accel_angle[id]
              - (short int) ((((long) main_speed[id] * accel_step * 12) / 5) >> 8);
      }
    }
    else if (section == MAIN_SECTION)
    {
      //preparations for POST Section
      section = POST_SECTION;
      unit_time_num = accel_step;

      for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
      {
        id = joint_index;
        main_angle[id] = moving_angle[id] - main_angle[id] - accel_angle[id];
      }
    }
    else if (section == POST_SECTION)
    {
      //it will be decided by Pause time exist or not
      if (pause_time)
      {
        section = PAUSE_SECTION;
        unit_time_num = pause_time;
      }
      else
      {
        section = PRE_SECTION;
      }
    }
    else if (section == PAUSE_SECTION)
    {
      //preparations for PRE Section
      section = PRE_SECTION;

      for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
      {
        id = joint_index;
        last_out_speed[id] = 0;
      }
    }

    // Ready for all in PRE Section
    if (section == PRE_SECTION)
    {
      if (playing_finished_ == true)  // If motion is finished
      {
        playing_ = false;
        return;
      }

      page_step_count_++;

      if (page_step_count_ > play_page_.header.stepnum)  // If motion playing of present page is finished
      {
        // copy next page
        play_page_ = next_play_page_;
        if (play_page_idx_ != next_play_page)
          play_repeat_count = play_page_.header.repeat;
        page_step_count_ = 1;
        play_page_idx_ = next_play_page;
      }

      if (page_step_count_ == play_page_.header.stepnum)  // If this is last step
      {
        // load next page
        if (stop_playing_ == true)  // STOP command
        {
          next_play_page = play_page_.header.exit;  // Go to Exit page
        }
        else
        {
          play_repeat_count--;
          if (play_repeat_count > 0)  // if repeat count is remained
            next_play_page = play_page_idx_;  // Set next page to present page
          else
            // Complete repeat
            next_play_page = play_page_.header.next;  // set next page
        }

        if (next_play_page == 0)  // If there is no NEXT page, the motion playing will be finished after current step.
          playing_finished_ = true;
        else
        {
          // load next page
          if (play_page_idx_ != next_play_page)
            loadPage(next_play_page, &next_play_page_);
          else
            next_play_page_ = play_page_;

          // If there is no playing information, the motion playing will be finished after current step.
          if (next_play_page_.header.repeat == 0 || next_play_page_.header.stepnum == 0)
            playing_finished_ = true;
        }
      }

      //////// Calc Step Parameter
      pause_time = (((unsigned short) play_page_.step[page_step_count_ - 1].pause) << 5) / play_page_.header.speed;
      max_speed = ((unsigned short) play_page_.step[page_step_count_ - 1].time
          * (unsigned short) play_page_.header.speed) >> 5;
      if (max_speed == 0)
        max_speed = 1;
      max_angle = 0;

      ////////// Calculate parameter of Joint
      for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
      {
        id = joint_index;
        //Calculate the trajectory using previous, present and future
        accel_angle[id] = 0;

        // Find current target angle
        if (play_page_.step[page_step_count_ - 1].position[id] & action_file_define::INVALID_BIT_MASK)
          curr_target_angle = target_angle[id];
        else
          curr_target_angle = play_page_.step[page_step_count_ - 1].position[id];

        // Update start, prev_target, curr_target
        start_angle[id] = target_angle[id];
        prev_target_angle = target_angle[id];
        target_angle[id] = curr_target_angle;

        // Find Moving offset
        moving_angle[id] = (int) (target_angle[id] - start_angle[id]);

        // Find Next target angle
        if (page_step_count_ == play_page_.header.stepnum)  // If current step is last step
        {
          if (playing_finished_ == true)  // If it will be finished
            next_target_angle = curr_target_angle;
          else
          {
            if (next_play_page_.step[0].position[id] & action_file_define::INVALID_BIT_MASK)
              next_target_angle = curr_target_angle;
            else
              next_target_angle = next_play_page_.step[0].position[id];
          }
        }
        else
        {
          if (play_page_.step[page_step_count_].position[id] & action_file_define::INVALID_BIT_MASK)
            next_target_angle = curr_target_angle;
          else
            next_target_angle = play_page_.step[page_step_count_].position[id];
        }

        // Find direction change
        if (((prev_target_angle < curr_target_angle) && (curr_target_angle < next_target_angle))
            || ((prev_target_angle > curr_target_angle) && (curr_target_angle > next_target_angle)))
        {
          // same direction
          direction_changed = 0;
        }
        else
        {
          direction_changed = 1;
        }

        // Find finish type
        if (direction_changed || pause_time || playing_finished_ == true)
          finish_type[id] = ZERO_FINISH;
        else
          finish_type[id] = NONE_ZERO_FINISH;

        if (play_page_.header.schedule == action_file_define::SPEED_BASE_SCHEDULE)
        {
          //MaxAngle1024 update
          if (moving_angle[id] < 0)
            tmp = -moving_angle[id];
          else
            tmp = moving_angle[id];

          if (tmp > max_angle)
            max_angle = tmp;
        }

      }

      // calculation the time. And, the calculated time will be divided by 7.8msec(<<7)- calculate there are how many 7.8msec
      // after unit conversion, calculate angle/velocity, and the following code computes how many units of 7.8s occurs within the specified time
      // unit conversion ---  angle :1024->300deg,  velocity: 256 ->720
      // wUnitTimeNum = ((wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) /7.8msec;
      //             = ((128*wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) ;    (/7.8msec == *128)
      //             = (wMaxAngle1024*40) /(wMaxSpeed256 *3);
      if (play_page_.header.schedule == action_file_define::TIME_BASE_SCHEDULE)
        unit_time_total_num = max_speed;  //TIME BASE 051025
      else
        unit_time_total_num = (max_angle * 40) / (max_speed * 3);

      accel_step = play_page_.header.accel;
      if (unit_time_total_num <= (accel_step << 1))
      {
        if (unit_time_total_num == 0)
        {
          accel_step = 0;
        }
        else
        {
          accel_step = (unit_time_total_num - 1) >> 1;
          if (accel_step == 0)
            unit_time_total_num = 0;  // Acceleration and constant velocity steps have to be more than one in order to move
        }
      }

      total_time_256t = ((unsigned long) unit_time_total_num) << 1;  // /128 * 256
      pre_section_time_256t = ((unsigned long) accel_step) << 1;  // /128 * 256
      main_time_256t = total_time_256t - pre_section_time_256t;
      divider1 = pre_section_time_256t + (main_time_256t << 1);
      divider2 = (main_time_256t << 1);

      if (divider1 == 0)
        divider1 = 1;

      if (divider2 == 0)
        divider2 = 1;

      for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
      {
        id = joint_index;
        start_speed1024_pre_time_256t = (long) last_out_speed[id] * pre_section_time_256t;  //  *300/1024 * 1024/720 * 256 * 2
        moving_angle_speed1024_scale_256t_2t = (((long) moving_angle[id]) * 2560L) / 12;

        if (finish_type[id] == ZERO_FINISH)
          main_speed[id] = (short int) ((moving_angle_speed1024_scale_256t_2t - start_speed1024_pre_time_256t)
              / divider2);
        else
          main_speed[id] = (short int) ((moving_angle_speed1024_scale_256t_2t - start_speed1024_pre_time_256t)
              / divider1);

        if (main_speed[id] > 1023)
          main_speed[id] = 1023;

        if (main_speed[id] < -1023)
          main_speed[id] = -1023;
      }
      unit_time_num = accel_step;  //PreSection
    }
  }
}

void ActionModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type = type;
  status.module_name = "Action";
  status.status_msg = msg;

  status_msg_pub_.publish(status);
}

void ActionModule::publishDoneMsg(std::string msg)
{
  std_msgs::String done_msg;
  done_msg.data = msg;

  done_msg_pub_.publish(done_msg);
}
}
