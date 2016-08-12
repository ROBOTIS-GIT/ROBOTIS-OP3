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

/* Author: Kayman Jung, Jay Song */

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
      NONE_ZERO_FINISH(1)
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
  playing_finished = true;
  page_step_count_ = 0;
  play_page_idx_ = 0;
  stop_playing_ = true;

  previous_enable_ = false;
  present_enable_ = false;
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
    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
  }

  ros::NodeHandle node_handle;

  std::string default_path = ros::package::getPath("op3_action_module") + "/data/motion_4095.bin";
  std::string action_file_path;
  node_handle.param<std::string>("action_file_path", action_file_path, default_path);

  loadFile((char*) action_file_path.c_str());

  playing_ = false;
}

void ActionModule::queueThread()
{
  ros::NodeHandle node_handle;
  ros::CallbackQueue callback_queue;

  node_handle.setCallbackQueue(&callback_queue);

  /* subscriber */
  action_page_sub_ = node_handle.subscribe("/robotis/action/page_num", 1, &ActionModule::pageNumberCallback, this);

  /* publisher */
  status_msg_pub_ = node_handle.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 0);

  ros::ServiceServer is_running_server = node_handle.advertiseService("/robotis/action/is_running",
                                                                      &ActionModule::isRunningServiceCallback, this);

  while (node_handle.ok())
  {
    callback_queue.callAvailable();

    usleep(100);
  }
}

bool ActionModule::isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request &req,
                                            op3_action_module_msgs::IsRunning::Response &res)
{
  res.is_running = isRunning();
  return true;
}

void ActionModule::pageNumberCallback(const std_msgs::Int32::ConstPtr& msg)
{
  if (msg->data == -1)
  {
    stop();
  }
  else if (msg->data == -2)
  {
    brakeAction();
  }
  else
  {
    if (isRunning() == true)
    {
      ROS_ERROR_STREAM("Previous Motion is not finished.");
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Previous Motion is not finished.");

    }

    if (startAction(msg->data) == true)
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
    }
  }
}

void ActionModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                           std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  if (present_enable_ == true)
  {
    for (std::map<std::string, robotis_framework::Dynamixel *>::iterator _it = dxls.begin(); _it != dxls.end(); _it++)
    {
      std::string joint_name = _it->first;

      if (result_.find(joint_name) == result_.end())
        continue;
      else
      {
        result_[joint_name]->goal_position_ = _it->second->dxl_state_->goal_position_;
      }
    }
  }

  previous_running_ = present_running_;
  present_running_ = isRunning();

  if (present_running_ != previous_running_)
  {
    if (present_running_ == true)
    {
      std::string status_msg = "Action_Start";
      ROS_INFO_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
    else
    {
      std::string status_msg = "Action_Finish";
      ROS_INFO_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
  }

  actionPlayProcess(dxls);
}

void ActionModule::stop()
{
  stop_playing_ = true;
}

bool ActionModule::isRunning()
{
  return playing_;
}

int ActionModule::radTow4095(double rad)
{
  return (int) ((rad + M_PI) * 2048.0 / M_PI);
}

double ActionModule::w4095ToRad(int w4095)
{
  return (w4095 - 2048) * M_PI / 2048.0;
}

bool ActionModule::verifyChecksum(action_file_define::PAGE* page)
{
  unsigned char checksum = 0x00;
  unsigned char* pt = (unsigned char*) page;

  for (unsigned int i = 0; i < sizeof(action_file_define::PAGE); i++)
  {
    checksum += *pt;
    pt++;
  }

  if (checksum != 0xff)
    return false;

  return true;
}

void ActionModule::setChecksum(action_file_define::PAGE* page)
{
  unsigned char checksum = 0x00;
  unsigned char* pt = (unsigned char*) page;

  page->header.checksum = 0x00;

  for (unsigned int i = 0; i < sizeof(action_file_define::PAGE); i++)
  {
    checksum += *pt;
    pt++;
  }

  page->header.checksum = (unsigned char) (0xff - checksum);
}

bool ActionModule::loadFile(char* file_name)
{
  FILE* action_file = fopen(file_name, "r+b");
  if (action_file == 0)
  {
    std::string status_msg = "Can not open Action file!";
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return false;
  }

  fseek(action_file, 0, SEEK_END);
  if (ftell(action_file) != (long) (sizeof(action_file_define::PAGE) * action_file_define::MAXNUM_PAGE))
  {
    std::string status_msg = "It's not an Action file!";
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    fclose(action_file);
    return false;
  }

  if (action_file_ != 0)
    fclose(action_file_);

  action_file_ = action_file;
  return true;
}

bool ActionModule::createFile(char* file_name)
{
  FILE* action_file = fopen(file_name, "ab");
  if (action_file == 0)
  {
    std::string status_msg = "Can not create Action file!";
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return false;
  }

  action_file_define::PAGE page;
  resetPage(&page);

  for (int i = 0; i < action_file_define::MAXNUM_PAGE; i++)
    fwrite((const void *) &page, 1, sizeof(action_file_define::PAGE), action_file);

  if (action_file_ != 0)
    fclose(action_file_);

  action_file_ = action_file;

  return true;
}

bool ActionModule::startAction(int page_number)
{
  if (page_number < 1 || page_number >= action_file_define::MAXNUM_PAGE)
  {

    std::string status_msg = "Can not play page.(" + convertIntToString(page_number) + " is invalid index)";
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return false;
  }

  action_file_define::PAGE page;
  if (loadPage(page_number, &page) == false)
    return false;

  return startAction(page_number, &page);
}

bool ActionModule::startAction(char* page_name)
{
  int index;
  action_file_define::PAGE page;

  for (index = 1; index < action_file_define::MAXNUM_PAGE; index++)
  {
    if (loadPage(index, &page) == false)
      return false;

    if (strcmp(page_name, (char*) page.header.name) == 0)
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
    return startAction(index, &page);
}

bool ActionModule::startAction(int page_number, action_file_define::PAGE *page)
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

void ActionModule::brakeAction()
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

bool ActionModule::loadPage(int page_number, action_file_define::PAGE* page)
{
  long position = (long) (sizeof(action_file_define::PAGE) * page_number);

  if (fseek(action_file_, position, SEEK_SET) != 0)
    return false;

  if (fread(page, 1, sizeof(action_file_define::PAGE), action_file_) != sizeof(action_file_define::PAGE))
    return false;

  if (verifyChecksum(page) == false)
    resetPage(page);

  return true;
}

bool ActionModule::savePage(int page_number, action_file_define::PAGE* page)
{
  long position = (long) (sizeof(action_file_define::PAGE) * page_number);

  if (verifyChecksum(page) == false)
    setChecksum(page);

  if (fseek(action_file_, position, SEEK_SET) != 0)
    return false;

  if (fwrite(page, 1, sizeof(action_file_define::PAGE), action_file_) != sizeof(action_file_define::PAGE))
    return false;

  return true;
}

void ActionModule::resetPage(action_file_define::PAGE *page)
{
  unsigned char *pt = (unsigned char*) page;

  for (unsigned int i = 0; i < sizeof(action_file_define::PAGE); i++)
  {
    *pt = 0x00;
    pt++;
  }

  page->header.schedule = action_file_define::TIME_BASE_SCHEDULE;  // default time base
  page->header.repeat = 1;
  page->header.speed = 32;
  page->header.accel = 32;

  for (int i = 0; i < 38; i++)
    page->header.pgain[i] = 0x55;

  for (int i = 0; i < action_file_define::MAXNUM_STEP; i++)
  {
    for (int j = 0; j < 38; j++)
      page->step[i].position[j] = action_file_define::INVALID_BIT_MASK;

    page->step[i].pause = 0;
    page->step[i].time = 0;
  }

  setChecksum(page);
}

void ActionModule::actionPlayProcess(std::map<std::string, robotis_framework::Dynamixel *> dxls)
{
  //////////////////// Local variable
  unsigned char id;
  unsigned long total_time_256t;
  unsigned long pre_section_time_256t;
  unsigned long main_time_256t;
  long start_speed1024_pre_time_256t;
  long moving_angle_speed1024_scale_time_256t_2t;
  long divider1, divider2;
  //unsigned short
  int max_angle1024;
  int max_speed;
  int tmp;
  int prev_target_angle;  // Start position
  int curr_target_angle;  // Target position
  int next_target_angle;  // Next target position
  unsigned char direction_changed;

  ///////////////// Static variable
  static unsigned short start_angle1024[action_file_define::MAXNUM_JOINTS];  // Starting point of interpolation
  static unsigned short target_angle1024[action_file_define::MAXNUM_JOINTS];  // Target point of interpolation
  static short int moving_angle1024[action_file_define::MAXNUM_JOINTS];  // Total moving angle
  static short int main_angle1024[action_file_define::MAXNUM_JOINTS];  // Moving angle of constant velocity
  static short int accel_angle1024[action_file_define::MAXNUM_JOINTS];  // Moving anble of acceleration
  static short int main_speed1024[action_file_define::MAXNUM_JOINTS];  // Target constant velocity
  static short int last_out_speed1024[action_file_define::MAXNUM_JOINTS];  // Velocity of last state
  static short int goal_speed1024[action_file_define::MAXNUM_JOINTS];  // Target velocity
  static unsigned char finish_type[action_file_define::MAXNUM_JOINTS];  // Condition of target angle

  short int speed_n;
  static unsigned short unit_time_count;
  static unsigned short unit_time_num;
  static unsigned short pause_time;
  static unsigned short unit_time_total_num;
  static unsigned short accel_step;
  static unsigned char section;
  static unsigned char play_repeat_count;
  static unsigned short next_play_page;

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
    return;

  if (first_driving_start_ == true)  // First start
  {
    first_driving_start_ = false;  //First Process end
    playing_finished = false;
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

      if (joint_id_to_name_.find(id) == joint_id_to_name_.end())
        continue;
      else
        joint_name = joint_id_to_name_[id];

      if (dxls.find(joint_name) == dxls.end())
        continue;
      else
      {
        double _goal_joint_angle_rad = dxls[joint_id_to_name_[id]]->dxl_state_->goal_position_;
        target_angle1024[id] = radTow4095(_goal_joint_angle_rad);
        last_out_speed1024[id] = 0;
        moving_angle1024[id] = 0;
        goal_speed1024[id] = 0;
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

        if (joint_id_to_name_.find(id) == joint_id_to_name_.end())
          continue;
        else
          joint_name = joint_id_to_name_[id];

        if (dxls.find(joint_name) == dxls.end())
        {
          continue;
        }
        else
        {
          if (moving_angle1024[id] == 0)
          {
            result_[joint_name]->goal_position_ = w4095ToRad(start_angle1024[id]);
          }
          else
          {
            if (section == PRE_SECTION)
            {
              speed_n = (short) (((long) (main_speed1024[id] - last_out_speed1024[id]) * unit_time_count)
                  / unit_time_num);
              goal_speed1024[id] = last_out_speed1024[id] + speed_n;
              accel_angle1024[id] = (short) ((((long) (last_out_speed1024[id] + (speed_n >> 1)) * unit_time_count * 144)
                  / 15) >> 9);

              result_[joint_name]->goal_position_ = w4095ToRad(start_angle1024[id] + accel_angle1024[id]);
            }
            else if (section == MAIN_SECTION)
            {
              result_[joint_name]->goal_position_ = w4095ToRad(
                  start_angle1024[id] + (short int) (((long) (main_angle1024[id]) * unit_time_count) / unit_time_num));

              goal_speed1024[id] = main_speed1024[id];
            }
            else  // POST_SECTION
            {
              if (unit_time_count == (unit_time_num - 1))
              {
                // use target angle in order to reduce the last step error
                result_[joint_name]->goal_position_ = w4095ToRad(target_angle1024[id]);
              }
              else
              {
                if (finish_type[id] == ZERO_FINISH)
                {
                  speed_n = (short int) (((long) (0 - last_out_speed1024[id]) * unit_time_count) / unit_time_num);
                  goal_speed1024[id] = last_out_speed1024[id] + speed_n;

                  result_[joint_name]->goal_position_ = w4095ToRad(
                      start_angle1024[id]
                          + (short) ((((long) (last_out_speed1024[id] + (speed_n >> 1)) * unit_time_count * 144) / 15)
                              >> 9));

                }
                else  // NONE_ZERO_FINISH
                {
                  // same as MAIN Section
                  // some servors are moving, others aren't in this step
                  result_[joint_name]->goal_position_ = w4095ToRad(
                      start_angle1024[id]
                          + (short int) (((long) (main_angle1024[id]) * unit_time_count) / unit_time_num));

                  goal_speed1024[id] = main_speed1024[id];
                }
              }
            }
          }

          // gains are excepted
          // result[_joint_name]->position_p_gain = ( 256 >> (play_page_.header.pgain[bID] >> 4) ) << 2 ;
        }
      }
    }
  }
  else if (unit_time_count >= unit_time_num)  // Current section is completed
  {
    unit_time_count = 0;

    for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
    {
      id = joint_index;
      std::string joint_name = "";

      if (joint_id_to_name_.find(id) == joint_id_to_name_.end())
        continue;
      else
        joint_name = joint_id_to_name_[id];

      if (dxls.find(joint_name) == dxls.end())
        continue;
      else
      {
        double goal_joint_angle_rad = dxls[joint_id_to_name_[id]]->dxl_state_->goal_position_;
        start_angle1024[id] = radTow4095(goal_joint_angle_rad);
        last_out_speed1024[id] = goal_speed1024[id];
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
          if ((unit_time_total_num - accel_step) == 0)  // No point of constant velocity
            main_angle1024[id] = 0;
          else
            main_angle1024[id] = (short) ((((long) (moving_angle1024[id] - accel_angle1024[id])) * unit_time_num)
                / (unit_time_total_num - accel_step));
        }
        else
          // ZERO_FINISH
          main_angle1024[id] = moving_angle1024[id] - accel_angle1024[id]
              - (short int) ((((long) main_speed1024[id] * accel_step * 12) / 5) >> 8);
      }
    }
    else if (section == MAIN_SECTION)
    {
      // Prepare for POST Section
      section = POST_SECTION;
      unit_time_num = accel_step;

      for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
      {
        id = joint_index;
        main_angle1024[id] = moving_angle1024[id] - main_angle1024[id] - accel_angle1024[id];
      }
    }
    else if (section == POST_SECTION)
    {
      // Pause time
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
      // Prepare for PRE Section
      section = PRE_SECTION;

      for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
      {
        id = joint_index;
        last_out_speed1024[id] = 0;
      }
    }

    // Ready for all in PRE Section
    if (section == PRE_SECTION)
    {
      if (playing_finished == true)  // if motion is finished
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

        if (next_play_page == 0)  // If next page don't exist
          playing_finished = true;
        else
        {
          // load next page
          if (play_page_idx_ != next_play_page)
            loadPage(next_play_page, &next_play_page_);
          else
            next_play_page_ = play_page_;

          // If next page doesn't have information for playing action, Process will be finished.
          if (next_play_page_.header.repeat == 0 || next_play_page_.header.stepnum == 0)
            playing_finished = true;
        }
      }

      //////// Calculate Step parameter
      pause_time = (((unsigned short) play_page_.step[page_step_count_ - 1].pause) << 5) / play_page_.header.speed;
      max_speed = ((unsigned short) play_page_.step[page_step_count_ - 1].time
          * (unsigned short) play_page_.header.speed) >> 5;
      if (max_speed == 0)
        max_speed = 1;
      max_angle1024 = 0;

      ////////// Calculate parameter of Joint
      for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
      {
        id = joint_index;
        // calculate the trajectory on the basis of previous, present and future
        accel_angle1024[id] = 0;

        // Find current target angle
        if (play_page_.step[page_step_count_ - 1].position[id] & action_file_define::INVALID_BIT_MASK)
          curr_target_angle = target_angle1024[id];
        else
          curr_target_angle = play_page_.step[page_step_count_ - 1].position[id];

        // Update start, prev_target, curr_target
        start_angle1024[id] = target_angle1024[id];
        prev_target_angle = target_angle1024[id];
        target_angle1024[id] = curr_target_angle;

        // Find Moving offset
        moving_angle1024[id] = (int) (target_angle1024[id] - start_angle1024[id]);

        // Find Next target angle
        if (page_step_count_ == play_page_.header.stepnum)  // If current step is the last one
        {
          if (playing_finished == true)  // Finished
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
        if (direction_changed || pause_time || playing_finished == true)
        {
          finish_type[id] = ZERO_FINISH;
        }
        else
        {
          finish_type[id] = NONE_ZERO_FINISH;
        }

        if (play_page_.header.schedule == action_file_define::SPEED_BASE_SCHEDULE)
        {
          //MaxAngle1024 update
          if (moving_angle1024[id] < 0)
            tmp = -moving_angle1024[id];
          else
            tmp = moving_angle1024[id];

          if (tmp > max_angle1024)
            max_angle1024 = tmp;
        }

      }

      // Unit count of total moving time (one unit time : 7.8ms)
      // Transformation --- Angle : 1024 unit -> 300 unit, Velocity : 256 unit -> 720 unit
      // wUnitTimeNum = ((wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) /7.8msec;
      //             = ((128*wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) ;    (/7.8msec == *128)
      //             = (wMaxAngle1024*40) /(wMaxSpeed256 *3);
      if (play_page_.header.schedule == action_file_define::TIME_BASE_SCHEDULE)
        unit_time_total_num = max_speed;  //TIME BASE 051025
      else
        unit_time_total_num = (max_angle1024 * 40) / (max_speed * 3);

      accel_step = play_page_.header.accel;
      if (unit_time_total_num <= (accel_step << 1))
      {
        if (unit_time_total_num == 0)
          accel_step = 0;
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
        start_speed1024_pre_time_256t = (long) last_out_speed1024[id] * pre_section_time_256t;  //  *300/1024 * 1024/720 * 256 * 2
        moving_angle_speed1024_scale_time_256t_2t = (((long) moving_angle1024[id]) * 2560L) / 12;

        if (finish_type[id] == ZERO_FINISH)
          main_speed1024[id] = (short int) ((moving_angle_speed1024_scale_time_256t_2t - start_speed1024_pre_time_256t)
              / divider2);
        else
          main_speed1024[id] = (short int) ((moving_angle_speed1024_scale_time_256t_2t - start_speed1024_pre_time_256t)
              / divider1);

        if (main_speed1024[id] > 1023)
          main_speed1024[id] = 1023;

        if (main_speed1024[id] < -1023)
          main_speed1024[id] = -1023;

      }
      unit_time_num = accel_step;  //PreSection
    }
  }
}

void ActionModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Action";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

void ActionModule::onModuleEnable()
{
  present_enable_ = true;
}

void ActionModule::onModuleDisable()
{
  present_enable_ = false;
}
}
