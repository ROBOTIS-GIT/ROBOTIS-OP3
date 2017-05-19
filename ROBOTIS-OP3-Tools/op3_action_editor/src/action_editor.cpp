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

/*
 * action_editor.cpp
 *
 *  Created on: 2016. 12. 16.
 *      Author: JaySong
 */

#include "op3_action_editor/action_editor.h"

using namespace robotis_op;

ActionEditor::ActionEditor()
{
  ctrl_ = 0;
  robot_ = 0;

  cmd_col_ = 2;
  stp7_col_ = 19 + 2;
  stp0_col_ = 25 + 2;
  stp1_col_ = 30 + 2;
  stp2_col_ = 35 + 2;
  stp3_col_ = 40 + 2;
  stp4_col_ = 45 + 2;
  stp5_col_ = 50 + 2;
  stp6_col_ = 55 + 2;
  cwslope_col_ = 60 + 2;
  ccwslope_col_ = 61 + 2;
  name_col_ = 63 + 2;
  addr_col_ = 72 + 2;
  pagenum_col_ = 75 + 2;
  pageparam_col_ = 76 + 2;

  name_row_ = 0;
  page_num_row_ = 1;
  addr_row_ = 2;
  play_count_row_ = 3;
  step_num_row_ = 4;
  play_time_row_ = 5;
  accel_row_ = 6;
  next_row_ = 7;
  exit_row_ = 8;

  first_joint_row_ = 0;
  begin_command_mode_ = false;
  edited_ = false;
  page_idx_ = 1;
  curr_col_ = stp7_col_;
  curr_row_ = first_joint_row_;

  old_col_ = curr_col_;
  old_row_ = curr_row_;

  max_joint_name_length_ = 12;

  //The below variables will be initialized in initialize function.
  num_of_dxls_ = 1;

  first_joint_row_ = 0;
  last_joint_row_ = num_of_dxls_ - 1;

  pause_row_ = last_joint_row_ + 1;
  time_row_ = pause_row_ + 1;
  cmd_row_ = time_row_ + 2;

  screen_col_ = 80;
  screen_row_ = cmd_row_ + 1;

  profile_velocity_addr_ = 112;
  cache_value_ = -1;
}

ActionEditor::~ActionEditor()
{

}

int ActionEditor::_getch()
{
  struct termios oldt, newt;
  int ch;
  tcgetattr( STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int ActionEditor::kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

void ActionEditor::setSTDin()
{
  tcgetattr(0, &oldterm);
  new_term = oldterm;
  new_term.c_lflag &= ~(ICANON | ECHO | ISIG);  // �ǹ̴� struct termios�� ã���� ��.
  new_term.c_cc[VMIN] = 1;
  new_term.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &new_term);
}

void ActionEditor::resetSTDin()
{
  tcsetattr(0, TCSANOW, &oldterm);
}

void ActionEditor::goToCursor(int col, int row)
{
  char *cursor;
  char *esc_sequence;
  cursor = tigetstr("cup");
  esc_sequence = tparm(cursor, row, col);
  putp(esc_sequence);

  curr_col_ = col;
  curr_row_ = row;
}

void ActionEditor::moveUpCursor()
{
  if (curr_col_ >= stp7_col_ && curr_col_ <= ccwslope_col_)
  {
    if (curr_row_ > first_joint_row_)
    {
      // If last_joint_row_ is less than 9 and the difference from pause_row_ is not 1,
      // it must be specified separately if it is pause_row_.
      if (curr_row_ == pause_row_)
        goToCursor(curr_col_, last_joint_row_);
      else
        goToCursor(curr_col_, curr_row_ - 1);
    }
  }
  else
  {
    if (curr_row_ > play_count_row_)
      goToCursor(curr_col_, curr_row_ - 1);
  }
}

void ActionEditor::moveDownCursor()
{
  if (curr_col_ >= stp7_col_ && curr_col_ <= stp6_col_)
  {
    if (curr_row_ < time_row_)
    {
      // If last_joint_row_ is less than 9 and the difference from pause_row_ is not 1,
      // it must be specified separately if it is last_joint_row_.
      if (curr_row_ == last_joint_row_)
        goToCursor(curr_col_, pause_row_);
      else
        goToCursor(curr_col_, curr_row_ + 1);
    }
  }
  else if (curr_col_ <= ccwslope_col_)
  {
    if (curr_row_ < last_joint_row_)
      goToCursor(curr_col_, curr_row_ + 1);
  }
  else
  {
    if (curr_row_ < exit_row_)
      goToCursor(curr_col_, curr_row_ + 1);
  }
}

void ActionEditor::moveLeftCursor()
{
  if (curr_col_ == stp0_col_)
    goToCursor(stp7_col_, curr_row_);
  else if (curr_col_ == stp1_col_)
    goToCursor(stp0_col_, curr_row_);
  else if (curr_col_ == stp2_col_)
    goToCursor(stp1_col_, curr_row_);
  else if (curr_col_ == stp3_col_)
    goToCursor(stp2_col_, curr_row_);
  else if (curr_col_ == stp4_col_)
    goToCursor(stp3_col_, curr_row_);
  else if (curr_col_ == stp5_col_)
    goToCursor(stp4_col_, curr_row_);
  else if (curr_col_ == stp6_col_)
    goToCursor(stp5_col_, curr_row_);
  else if (curr_col_ == cwslope_col_)
    goToCursor(stp6_col_, curr_row_);
  else if (curr_col_ == ccwslope_col_)
    goToCursor(cwslope_col_, curr_row_);
  else if (curr_col_ == pageparam_col_)
    goToCursor(ccwslope_col_, curr_row_);
  else
    return;
}

void ActionEditor::moveRightCursor()
{
  if (curr_col_ == stp7_col_)
    goToCursor(stp0_col_, curr_row_);
  else if (curr_col_ == stp0_col_)
    goToCursor(stp1_col_, curr_row_);
  else if (curr_col_ == stp1_col_)
    goToCursor(stp2_col_, curr_row_);
  else if (curr_col_ == stp2_col_)
    goToCursor(stp3_col_, curr_row_);
  else if (curr_col_ == stp3_col_)
    goToCursor(stp4_col_, curr_row_);
  else if (curr_col_ == stp4_col_)
    goToCursor(stp5_col_, curr_row_);
  else if (curr_col_ == stp5_col_)
    goToCursor(stp6_col_, curr_row_);
  else if (curr_col_ == stp6_col_)
    goToCursor(cwslope_col_, curr_row_);
  else if (curr_col_ == cwslope_col_)
    goToCursor(ccwslope_col_, curr_row_);
  else if (curr_col_ == ccwslope_col_)
  {
    if (curr_row_ >= play_count_row_ && curr_row_ <= exit_row_)
      goToCursor(pageparam_col_, curr_row_);
  }
  else
    return;
}

bool ActionEditor::initializeActionEditor(std::string robot_file_path, std::string init_file_path,
                                          std::string offset_file_path)
{
  ctrl_ = robotis_framework::RobotisController::getInstance();

  //Controller Initialize with robot file info
  if (ctrl_->initialize(robot_file_path, init_file_path) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return false;
  }

  ctrl_->loadOffset(offset_file_path);
  ctrl_->addMotionModule((robotis_framework::MotionModule*) ActionModule::getInstance());
  ActionModule::getInstance()->enableAllJoints();

  robot_ = ctrl_->robot_;

  //Initialize Publisher
  ros::NodeHandle nh;
  enable_ctrl_module_pub_ = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  play_sound_pub_ = nh.advertise<std_msgs::String>("/play_sound_file", 0);

  //Initialize Member variable
  for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot_->dxls_.begin();
      it != robot_->dxls_.end(); it++)
  {
    std::string joint_name = it->first;
    robotis_framework::Dynamixel* dxl_info = it->second;

    joint_name_to_id_[joint_name] = dxl_info->id_;
    joint_id_to_name_[dxl_info->id_] = joint_name;
  }

  //Since joint_id_to_name_ is automatically sorted by id,
  //joint_id_to_row_index_ should be initialized after initialization of joint_id_to_name_.
  int row_idx = 0;
  for (std::map<int, std::string>::iterator it = joint_id_to_name_.begin(); it != joint_id_to_name_.end(); it++)
  {
    int id = it->first;
    if ((id != 0) && (id <= action_file_define::MAXNUM_JOINTS))
    {
      joint_id_to_row_index_[id] = row_idx;
      joint_row_index_to_id_[row_idx] = id;
      row_idx++;
    }
  }

  num_of_dxls_ = joint_id_to_name_.size();

  first_joint_row_ = 0;
  last_joint_row_ = num_of_dxls_ - 1;

  if (last_joint_row_ < 9)
    pause_row_ = 9 + 1;
  else
    pause_row_ = last_joint_row_ + 1;

  time_row_ = pause_row_ + 1;
  cmd_row_ = time_row_ + 2;

  screen_col_ = 80;
  screen_row_ = cmd_row_ + 1;

  /**
   * below code must be changed
   */
  for (std::map<std::string, dynamixel::PortHandler *>::iterator it = robot_->ports_.begin();
      it != robot_->ports_.end(); it++)
  {
    port_to_sync_write_go_cmd_[it->first] = new dynamixel::GroupSyncWrite(
        it->second, dynamixel::PacketHandler::getPacketHandler(2.0), profile_velocity_addr_,  // "profile_velocity"
        8);
  }

  default_editor_script_path_ = ros::package::getPath("op3_action_editor") + "/script/editor_script.yaml";
  mirror_joint_file_path_ = ros::package::getPath("op3_action_editor") + "/config/config_mirror_joint.yaml";

  // for mirroring
  upper_body_mirror_joints_rl_.clear();
  upper_body_mirror_joints_lr_.clear();
  lower_body_mirror_joints_rl_.clear();
  lower_body_mirror_joints_lr_.clear();

  loadMirrorJoint();

  return true;
}

int ActionEditor::convert4095ToPositionValue(int id, int w4095)
{
  double rad = (w4095 - 2048) * M_PI / 2048.0;
  return robot_->dxls_[joint_id_to_name_[id]]->convertRadian2Value(rad);
}

int ActionEditor::convertPositionValueTo4095(int id, int PositionValue)
{
  double rad = robot_->dxls_[joint_id_to_name_[id]]->convertValue2Radian(PositionValue);
  return (int) ((rad + M_PI) * 2048.0 / M_PI);
}

int ActionEditor::convert4095ToMirror(int id, int w4095)
{
  return 4095 - w4095;
}

bool ActionEditor::loadMp3Path(int mp3_index, std::string &path)
{
  YAML::Node doc;

  try
  {
    // load yaml
    doc = YAML::LoadFile(default_editor_script_path_.c_str());
  } catch (const std::exception& e)
  {
    return false;
  }

  // parse action_sound table
  YAML::Node sub_node = doc["action_and_sound"];
  for (YAML::iterator yaml_it = sub_node.begin(); yaml_it != sub_node.end(); ++yaml_it)
  {
    int index = yaml_it->first.as<int>();
    if (mp3_index == index)
    {
      path = yaml_it->second.as<std::string>();
      return true;
    }
  }

  return false;
}

bool ActionEditor::loadMirrorJoint()
{
  YAML::Node doc;

  try
  {
    // load yaml
    doc = YAML::LoadFile(mirror_joint_file_path_.c_str());
  } catch (const std::exception& e)
  {
    return false;
  }

  // parse action_sound table
  YAML::Node sub_node = doc["upper_body"];
  for (YAML::iterator yaml_it = sub_node.begin(); yaml_it != sub_node.end(); ++yaml_it)
  {
    int right_id = yaml_it->first.as<int>();
    int left_id = yaml_it->second.as<int>();

    upper_body_mirror_joints_rl_[right_id] = left_id;
    upper_body_mirror_joints_lr_[left_id] = right_id;
  }

  YAML::Node sub_node2 = doc["lower_body"];
  for (YAML::iterator yaml_it = sub_node2.begin(); yaml_it != sub_node2.end(); ++yaml_it)
  {
    int right_id = yaml_it->first.as<int>();
    int left_id = yaml_it->second.as<int>();

    lower_body_mirror_joints_rl_[right_id] = left_id;
    lower_body_mirror_joints_lr_[left_id] = right_id;
  }

  return true;
}

// Disp & Drawing
void ActionEditor::drawIntro()
{
  int nrows, ncolumns;
  setupterm(NULL, fileno(stdout), (int *) 0);
  nrows = tigetnum("lines");
  ncolumns = tigetnum("cols");

  system("clear");
  printf("\n");
  printf("[Action Editor for %s]\n", ROBOT_NAME);
  printf("\n");
  printf(" *Terminal screen size must be %d(col)x%d(row).\n", screen_col_, screen_row_);
  printf(" *Current terminal has %d columns and %d rows.\n", ncolumns, nrows);
  printf("\n");
  printf("\n");
  printf("Press any key to start program...\n");
  _getch();

  ActionModule::getInstance()->loadPage(page_idx_, &page_);

  readStep();

  step_.pause = 0;
  step_.time = 0;

  drawPage();
}

void ActionEditor::drawEnding()
{
  system("clear");
  printf("\n");
  printf("Terminate Action Editor");
  printf("\n");
}

void ActionEditor::drawPage()
{
  int old_col = curr_col_;
  int old_row = curr_row_;

  system("clear");
  // 80   0         1         2         3         4         5         6         7
  // 80   01234567890123456789012345678901234567890123456789012345678901234567890123456789     //24
  printf("                                                                                 \n");
  printf("                                                                 Page Number:    \n");  //1
  printf("                                                                  Address:       \n");  //2
  printf("                                                                   Play Count:   \n");  //3
  printf("                                                                    Page Step:   \n");  //4
  printf("                                                                   Page Speed:   \n");  //5
  printf("                                                                   Accel Time:   \n");  //6
  printf("                                                                 Link to Next:   \n");  //7
  printf("                                                                 Link to Exit:   \n");  //8
  printf("                                                                                 \n");  //9

  // Draw joint list
  goToCursor(0, 0);
  for (std::map<int, std::string>::iterator it = joint_id_to_name_.begin(); it != joint_id_to_name_.end(); it++)
  {
    int id = it->first;
    std::string joint_name = it->second;

    if ((id == 0) || (id > action_file_define::MAXNUM_JOINTS))
      continue;

    printf("ID:%3d", id);

    printf("(");
    for (int joint_name_idx = 0; joint_name_idx < max_joint_name_length_; joint_name_idx++)
    {
      if (joint_name_idx >= joint_name.size())
        printf(" ");
      else
        printf("%c", joint_name.at(joint_name_idx));
    }
    printf(")");

    //printf("[    ]                                                       ");
    printf("[    ]");
    goToCursor(cwslope_col_, curr_row_);
    printf("%.1d%.1d", page_.header.pgain[id] >> 4, page_.header.pgain[id] & 0x0f);

    goToCursor(0, curr_row_ + 1);
  }

  // Draw pause row
  goToCursor(0, pause_row_);
  printf("   PauseTime        [    ]                                                       \n");

  if (page_.header.schedule == action_file_define::SPEED_BASE_SCHEDULE)
    printf("   Speed            [    ]                                                       \n");
  else if (page_.header.schedule == action_file_define::TIME_BASE_SCHEDULE)
    printf("   Time(x 8msec)    [    ]                                                       \n");

  printf("                     STP7  STP0 STP1 STP2 STP3 STP4 STP5 STP6                    \n");
  printf("]                                                                              ");

  // Draw Step
  for (int i = 0; i <= action_file_define::MAXNUM_STEP; i++)
    drawStep(i);

  // Draw Page parameter
  goToCursor(pageparam_col_, play_count_row_);
  printf("%.3d", page_.header.repeat);

  goToCursor(pageparam_col_, step_num_row_);
  printf("%.3d", page_.header.stepnum);

  goToCursor(pageparam_col_, play_time_row_);
  printf("%.3d", page_.header.speed);

  goToCursor(pageparam_col_, accel_row_);
  printf("%.3d", page_.header.accel);

  goToCursor(pageparam_col_, next_row_);
  printf("%.3d", page_.header.next);

  goToCursor(pageparam_col_, exit_row_);
  printf("%.3d", page_.header.exit);

  // Draw Page information
  drawName();

  goToCursor(pagenum_col_, page_num_row_);
  printf("%.4d", page_idx_);

  goToCursor(addr_col_, addr_row_);
  printf("0x%.5X", (int) (page_idx_ * sizeof(action_file_define::Page)));

  drawStepLine(false);

  goToCursor(old_col, old_row);
}

void ActionEditor::drawStep(int index)
{
  int old_col = curr_col_;
  int old_row = curr_row_;
  action_file_define::Step *step;
  int col;

  switch (index)
  {
    case 0:
      col = stp0_col_;
      step = &page_.step[0];
      break;
    case 1:
      col = stp1_col_;
      step = &page_.step[1];
      break;
    case 2:
      col = stp2_col_;
      step = &page_.step[2];
      break;
    case 3:
      col = stp3_col_;
      step = &page_.step[3];
      break;
    case 4:
      col = stp4_col_;
      step = &page_.step[4];
      break;
    case 5:
      col = stp5_col_;
      step = &page_.step[5];
      break;
    case 6:
      col = stp6_col_;
      step = &page_.step[6];
      break;
    case 7:
      col = stp7_col_;
      step = &step_;
      break;
    default:
      return;
  }

  for (std::map<int, int>::iterator it = joint_id_to_row_index_.begin(); it != joint_id_to_row_index_.end(); it++)
  {
    int id = it->first;
    int row_idx = it->second;
    if (id <= action_file_define::MAXNUM_JOINTS)
    {
      goToCursor(col, row_idx);
      if (step->position[id] & action_file_define::INVALID_BIT_MASK)
      {
        printf("----");
      }
      else if (step->position[id] & action_file_define::TORQUE_OFF_BIT_MASK)
        printf("????");
      else
        printf("%.4d", step->position[id]);
    }
    else
      continue;
  }

  goToCursor(col, pause_row_);
  printf("%4.3d", step->pause);

  goToCursor(col, time_row_);
  printf("%4.3d", step->time);

  goToCursor(old_col, old_row);
}

void ActionEditor::drawName()
{
  int old_col = curr_col_;
  int old_row = curr_row_;

  goToCursor(name_col_, name_row_);
  printf("                ");
  goToCursor(name_col_, name_row_);

  for (int i = 0; i < action_file_define::MAXNUM_NAME; i++)
    printf("%c", (char) page_.header.name[i]);

  goToCursor(old_col, old_row);
}

void ActionEditor::drawStepLine(bool erase)
{
  int old_col = curr_col_;
  int old_row = curr_row_;
  int col;

  switch (page_.header.stepnum)
  {
    case 0:
      col = stp0_col_;
      break;

    case 1:
      col = stp1_col_;
      break;

    case 2:
      col = stp2_col_;
      break;

    case 3:
      col = stp3_col_;
      break;

    case 4:
      col = stp4_col_;
      break;

    case 5:
      col = stp5_col_;
      break;

    case 6:
      col = stp6_col_;
      break;

    case 7:
      col = cwslope_col_;
      break;

    default:
      return;
  }
  col--;

  for (int index = 0; index < joint_id_to_row_index_.size(); index++)
  {
    goToCursor(col, index);
    if (erase == true)
      printf(" ");
    else
      printf("|");
  }

  goToCursor(old_col, old_row);
}

void ActionEditor::readStep()
{
  uint8_t error = 0;
  uint32_t value = 0;
  int id;
  bool* enable = new bool[joint_id_to_row_index_.size()];

  for (int index = 0; index < joint_id_to_row_index_.size(); index++)
  {
    enable[index] = false;
  }

  for (std::map<int, int>::iterator it = joint_id_to_row_index_.begin(); it != joint_id_to_row_index_.end(); it++)
  {
    id = it->first;
    std::string joint_name = joint_id_to_name_[id];
    std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it;

    dxls_it = robot_->dxls_.find(joint_name);
    if (dxls_it != robot_->dxls_.end())
    {
      if (ctrl_->readCtrlItem(joint_name, "torque_enable", &value, &error) == COMM_SUCCESS)
      {
        if (value == 1)
        {
          if (ctrl_->readCtrlItem(joint_name, "present_position", &value, &error) == COMM_SUCCESS)
          {
            int offset = robot_->dxls_[joint_name]->convertRadian2Value(
                robot_->dxls_[joint_name]->dxl_state_->position_offset_)
                - robot_->dxls_[joint_name]->value_of_0_radian_position_;
            value = value - offset;
            step_.position[id] = convertPositionValueTo4095(id, value);
            enable[id] = true;
          }
          else
          {
            step_.position[id] = action_file_define::INVALID_BIT_MASK;
          }
        }
        else
        {
          step_.position[id] = action_file_define::INVALID_BIT_MASK;
        }
      }
      else
      {
        step_.position[id] = action_file_define::INVALID_BIT_MASK;
      }

    }
    else
    {
      step_.position[id] = action_file_define::INVALID_BIT_MASK;
    }
  }
}

void ActionEditor::clearCmd()
{
  printCmd("                              ");
}

void ActionEditor::printCmd(const char *message)
{
  int len = strlen(message);
  goToCursor(0, cmd_row_);

  printf("] %s", message);
  for (int i = 0; i < (screen_col_ - (len + 2)); i++)
    printf(" ");

  goToCursor(len + 2, cmd_row_);
}

bool ActionEditor::askSave()
{
  if (edited_ == true)
  {
    printCmd("Are you sure? (y/n)");
    if (_getch() != 'y')
    {
      clearCmd();
      return true;
    }
  }

  return false;
}

// Edit value

void ActionEditor::setValueUpDown(int offset)
{
  setValue(getValue() + offset);
}

void ActionEditor::setValue(int value)
{
  int col;
  int row;
  if (begin_command_mode_ == true)
  {
    col = old_col_;
    row = old_row_;
  }
  else
  {
    col = curr_col_;
    row = curr_row_;
  }

  goToCursor(col, row);

  if (col == stp7_col_)
  {
    if (row == pause_row_)
    {
      if (value >= 0 && value <= 255)
      {
        step_.pause = value;
        printf("%4.3d", value);
        edited_ = true;
      }
    }
    else if (row == time_row_)
    {
      if (value >= 0 && value <= 255)
      {
        step_.time = value;
        printf("%4.3d", value);
        edited_ = true;
      }
    }
    else
    {
      if (value >= 0 && value <= 4095)
      {
        int id = joint_row_index_to_id_[row];
        if (!(step_.position[id] & action_file_define::INVALID_BIT_MASK)
            && !(step_.position[id] & action_file_define::TORQUE_OFF_BIT_MASK))
        {
          uint8_t error;
          uint32_t ivalue;
          ivalue = (uint32_t) convert4095ToPositionValue(id, value);
          if (ctrl_->writeCtrlItem(joint_id_to_name_[id], "goal_position", ivalue, &error) == COMM_SUCCESS)
          {
            step_.position[id] = value;
            printf("%.4d", value);
            edited_ = true;
          }
        }
      }
    }
  }
  else if (col <= stp6_col_)
  {
    int i = 0;
    if (col == stp0_col_)
      i = 0;
    else if (col == stp1_col_)
      i = 1;
    else if (col == stp2_col_)
      i = 2;
    else if (col == stp3_col_)
      i = 3;
    else if (col == stp4_col_)
      i = 4;
    else if (col == stp5_col_)
      i = 5;
    else if (col == stp6_col_)
      i = 6;

    int id = joint_row_index_to_id_[row];

    if (row == pause_row_)
    {
      if (value >= 0 && value <= 255)
      {
        page_.step[i].pause = value;
        printf("%4.3d", value);
        edited_ = true;
      }
    }
    else if (row == time_row_)
    {
      if (value >= 0 && value <= 255)
      {
        page_.step[i].time = value;
        printf("%4.3d", value);
        edited_ = true;
      }
    }
    else
    {
      if (value >= 0 && value <= 4095)
      {
        page_.step[i].position[id] = value;
        printf("%.4d", value);
        edited_ = true;
      }
    }
  }
  else if (col == cwslope_col_)
  {
    int id = joint_row_index_to_id_[row];
    if (value >= 1 && value <= 7)
    {
      page_.header.pgain[id] = (value << 4) + (page_.header.pgain[id] & 0x0f);
      printf("%.1d", value);
      edited_ = true;
    }
  }
  else if (col == ccwslope_col_)
  {
    int id = joint_row_index_to_id_[row];
    if (value >= 1 && value <= 7)
    {
      page_.header.pgain[id] = (page_.header.pgain[id] & 0xf0) + (value & 0x0f);
      printf("%.1d", value);
      edited_ = true;
    }
  }
  else if (col == pageparam_col_)
  {
    if (row == play_count_row_)
    {
      if (value >= 0 && value <= 255)
      {
        page_.header.repeat = value;
        printf("%.3d", value);
        edited_ = true;
      }
    }
    else if (row == step_num_row_)
    {
      if (value >= 0 && value <= action_file_define::MAXNUM_STEP)
      {
        if (page_.header.stepnum != value)
        {
          drawStepLine(true);
          page_.header.stepnum = value;
          drawStepLine(false);
          printf("%.3d", value);
          edited_ = true;
        }
      }
    }
    else if (row == play_time_row_)
    {
      if (value >= 0 && value <= 255)
      {
        page_.header.speed = value;
        printf("%.3d", value);
        edited_ = true;
      }
    }
    else if (row == accel_row_)
    {
      if (value >= 0 && value <= 255)
      {
        page_.header.accel = value;
        printf("%.3d", value);
        edited_ = true;
      }
    }
    else if (row == next_row_)
    {
      if (value >= 0 && value <= 255)
      {
        page_.header.next = value;
        printf("%.3d", value);
        edited_ = true;
      }
    }
    else if (row == exit_row_)
    {
      if (value >= 0 && value <= 255)
      {
        page_.header.exit = value;
        printf("%.3d", value);
        edited_ = true;
      }
    }
  }

  goToCursor(col, row);
}

int ActionEditor::getValue()
{
  int col;
  int row;
  if (begin_command_mode_ == true)
  {
    col = old_col_;
    row = old_row_;
  }
  else
  {
    col = curr_col_;
    row = curr_row_;
  }

  if (col == stp7_col_)
  {
    if (row == pause_row_)
      return step_.pause;
    else if (row == time_row_)
      return step_.time;
    else
      return step_.position[row + 1];
  }
  else if (col <= stp6_col_)
  {
    int i = 0;
    if (col == stp0_col_)
      i = 0;
    else if (col == stp1_col_)
      i = 1;
    else if (col == stp2_col_)
      i = 2;
    else if (col == stp3_col_)
      i = 3;
    else if (col == stp4_col_)
      i = 4;
    else if (col == stp5_col_)
      i = 5;
    else if (col == stp6_col_)
      i = 6;

    int id = joint_row_index_to_id_[row];

    if (row == pause_row_)
      return page_.step[i].pause;
    else if (row == time_row_)
      return page_.step[i].time;
    else
      return page_.step[i].position[id];
  }
  else if (col == cwslope_col_)
    return (page_.header.pgain[joint_row_index_to_id_[row]] >> 4);
  else if (col == ccwslope_col_)
    return (page_.header.pgain[joint_row_index_to_id_[row]] & 0x0f);
  else if (col == pageparam_col_)
  {
    if (row == play_count_row_)
      return page_.header.repeat;
    else if (row == step_num_row_)
      return page_.header.stepnum;
    else if (row == play_time_row_)
      return page_.header.speed;
    else if (row == accel_row_)
      return page_.header.accel;
    else if (row == next_row_)
      return page_.header.next;
    else if (row == exit_row_)
      return page_.header.exit;
  }

  return -1;
}

void ActionEditor::toggleTorque()
{
  if (curr_col_ != stp7_col_ || curr_row_ > last_joint_row_)
    return;

  int id = joint_row_index_to_id_[curr_row_];

  std::string joint_name = joint_id_to_name_[id];

  if (step_.position[id] & action_file_define::TORQUE_OFF_BIT_MASK)
  {
    if (ctrl_->writeCtrlItem(joint_name, "torque_enable", 1, 0) != COMM_SUCCESS)
      return;

    uint32_t value;
    if (ctrl_->readCtrlItem(joint_name, "present_position", &value, 0) != COMM_SUCCESS)
      return;

    int offset = robot_->dxls_[joint_name]->convertRadian2Value(robot_->dxls_[joint_name]->dxl_state_->position_offset_)
        - robot_->dxls_[joint_name]->value_of_0_radian_position_;
    value = value - offset;

    step_.position[id] = convertPositionValueTo4095(id, value);
    printf("%.4d", value);
  }
  else
  {
    if (ctrl_->writeCtrlItem(joint_name, "torque_enable", 0, 0) != COMM_SUCCESS)
      return;

    step_.position[id] = action_file_define::TORQUE_OFF_BIT_MASK;
    printf("????");
  }

  goToCursor(curr_col_, curr_row_);
}

void ActionEditor::storeValueToCache()
{
  cache_value_ = getValue();
}

void ActionEditor::setValueFromCache()
{
  // cache is empty.
  if (cache_value_ == -1)
  {
    int cursor_col = curr_col_;
    int cursor_row = curr_row_;

    printCmd("Cache is empty.");
    goToCursor(cursor_col, cursor_row);

    return;
  }

  // set value
  setValue(cache_value_);
}

void ActionEditor::clearCache()
{
  cache_value_ = -1;
}

// Command process
void ActionEditor::beginCommandMode()
{
  old_col_ = curr_col_;
  old_row_ = curr_row_;
  clearCmd();
  goToCursor(cmd_col_, cmd_row_);
  begin_command_mode_ = true;
}

void ActionEditor::endCommandMode()
{
  goToCursor(old_col_, old_row_);
  begin_command_mode_ = false;
}

void ActionEditor::helpCmd()
{
  system("clear");
  printf(" exit               Exits the program.\n");
  printf(" re                 Refreshes the screen.\n");
  printf(" b                  Move to previous page.\n");
  printf(" n                  Move to next page.\n");
  printf(" page [index]       Move to page [index].\n");
  printf(" list               View list of pages.\n");
  printf(" new                Clears data of current page and initializes page.\n");
  printf(" copy [index]       Copy data from page [index].\n");
  printf(" set [value]        Sets value on cursor [value].\n");
  printf(" save               Saves changes.\n");
  printf(" play               Motion playback of current page.\n");
  printf(" g [index]          Motion playback of STP[index].\n");
  printf(" name               Name for current page or changes the name of current page.\n");
  printf(" time               Change time base playing.\n");
  printf(" speed              Change speed base playing.\n");
  printf(" w [index]          Overwrites data from STP[index] with STP7.\n");
  printf(" i                  Inserts data from STP7 to STP0. \n"
         "                    Moves data from STP[x] to STP[x+1].\n");
  printf(" i [index]          Inserts data from STP7 to STP[index]. \n"
         "                    Moves data from STP[index] to STP[index+1].\n");
  printf(" m [index] [index2] Moves data from [index] to [index2] step.\n");
  printf(" d [index]          Deletes data from STP[index]. \n"
         "                    Pushes data from STP[index] to STP[index-1].\n");
  printf(" on/off             Turn On/Off torque from ALL actuators.\n");
  printf(" on/off [index1] [index2] ...  \n"
         "                    turns On/Off torque from ID[index1] ID[index2]...\n");
  printf("\n");
  printf("       Copyright ROBOTIS CO.,LTD.\n");
  printf("\n");
  printf(" Press any key to continue...");
  _getch();

  drawPage();
}

void ActionEditor::nextCmd()
{
  pageCmd(page_idx_ + 1);
}

void ActionEditor::prevCmd()
{
  pageCmd(page_idx_ - 1);
}

void ActionEditor::pageCmd(int index)
{
  if (askSave() == true)
    return;

  if (index > 0 && index < action_file_define::MAXNUM_PAGE)
  {
    page_idx_ = index;
    ActionModule::getInstance()->loadPage(page_idx_, &page_);

    curr_col_ = stp7_col_;
    curr_row_ = first_joint_row_;
    drawPage();
  }
  else
    printCmd("Invalid page index");

  edited_ = false;
}

void ActionEditor::timeCmd()
{
  page_.header.schedule = action_file_define::TIME_BASE_SCHEDULE;
  edited_ = true;
  drawPage();
}

void ActionEditor::speedCmd()
{
  page_.header.schedule = action_file_define::SPEED_BASE_SCHEDULE;
  edited_ = true;
  drawPage();
}

void ActionEditor::playCmd()
{
  playCmd(-1);
}

void ActionEditor::playCmd(int mp3_index)
{
  uint32_t value;

  for (int i = 0; i < page_.header.stepnum; i++)
  {
    for (std::map<int, int>::iterator it = joint_id_to_row_index_.begin(); it != joint_id_to_row_index_.end(); it++)
    {
      int id = it->first;
      if (page_.step[i].position[id] & action_file_define::INVALID_BIT_MASK)
      {
        printCmd("Exist invalid joint value");
        return;
      }
    }
  }

  printCmd("Playing... ('s' to stop, 'b' to brake)");

  ctrl_->startTimer();
  ros::Duration(0.03).sleep();  // waiting for timer start

  std_msgs::String msg;
  msg.data = "action_module";
  enable_ctrl_module_pub_.publish(msg);
  ros::Duration(0.03).sleep();  // waiting for enable

  if (ActionModule::getInstance()->start(page_idx_, &page_) == false)
  {
    printCmd("Failed to play this page!");
    ctrl_->stopTimer();
    return;
  }

  // play mp3
  if (mp3_index != -1)
  {
    std::string mp3_path = "";
    bool get_path_result = loadMp3Path(mp3_index, mp3_path);

    if (get_path_result == true)
    {
      std_msgs::String sound_msg;
      sound_msg.data = mp3_path;

      play_sound_pub_.publish(sound_msg);
    }
  }

  setSTDin();
  while (1)
  {
    if (ActionModule::getInstance()->isRunning() == false)
      break;

    if (kbhit())
    {
      int key = _getch();
      goToCursor(cmd_col_, cmd_row_);
      if (key == 's')
      {
        ActionModule::getInstance()->stop();
        fprintf(stderr, "\r] Stopping...                                  ");
      }
      else if (key == 'b')
      {
        ActionModule::getInstance()->brake();
        fprintf(stderr, "\r] Braking...                                   ");
      }
      else
        fprintf(stderr, "\r] Playing... ('s' to stop, 'b' to brake)");
    }

    usleep(10000);
  }
  resetSTDin();

  ctrl_->stopTimer();

  goToCursor(cmd_col_, cmd_row_);
  printCmd("Done.");

  usleep(10000);

  readStep();
  drawStep(7);
}

void ActionEditor::listCmd()
{
  int old_col = curr_col_;
  int old_row = curr_row_;
  int index = 0;

  while (1)
  {
    system("clear");
    for (int i = 0; i < 22; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        int k = (index * 88) + (j * 22 + i);  //first page number is 1
        action_file_define::Page page;
        if (ActionModule::getInstance()->loadPage(k, &page) == true)
        {
          printf(" |%.3d.", k);
          for (int n = 0; n < action_file_define::MAXNUM_NAME; n++)
          {
            if ((char) page.header.name[n] >= ' ' && (char) page.header.name[n] <= '~')
              printf("%c", (char) page.header.name[n]);
            else
              printf(" ");
          }
        }
        else
        {
          printf(" |                ");
        }
      }
      printf("\n");
    }

    printf("\nAction Page List (%d/3) - Press key n(Next), b(Prev), q(Quit)", index + 1);
    while (1)
    {
      int ch = _getch();
      if (ch == 'n')
      {
        if (index < 2)
        {
          index++;
          break;
        }
      }
      else if (ch == 'b')
      {
        if (index > 0)
        {
          index--;
          break;
        }
      }
      else if (ch == 'q')
      {
        drawPage();
        goToCursor(old_col, old_row);
        return;
      }
    }
  }
}

void ActionEditor::turnOnOffCmd(bool on, int num_param, int *list)
{
  uint32_t torque_enable = 0;
  if (on)
    torque_enable = 1;

  if (num_param == 0)
  {
    for (std::map<int, int>::iterator it = joint_id_to_row_index_.begin(); it != joint_id_to_row_index_.end(); it++)
    {
      int id = it->first;
      std::string joint_name = joint_id_to_name_[id];
      ctrl_->writeCtrlItem(joint_name, "torque_enable", torque_enable, 0);
    }
  }
  else
  {
    for (int i = 0; i < num_param; i++)
    {
      int id = list[i];
      std::map<int, int>::iterator it = joint_id_to_row_index_.find(id);
      if (it != joint_id_to_row_index_.end())
      {
        std::string joint_name = joint_id_to_name_[id];
        ctrl_->writeCtrlItem(joint_name, "torque_enable", torque_enable, 0);
      }
    }
  }

  readStep();
  drawStep(7);
}

void ActionEditor::mirrorStepCmd(int index, int mirror_type, int target_type)
{
  // check index
  if (index < 0 || index >= action_file_define::MAXNUM_STEP)
  {
    printCmd("Invalid step index");
    return;
  }

  // store previous step
  action_file_define::Step before_step = page_.step[index];

  //check target_type
  if (target_type == UpperBody || target_type == AllBody)
  {
    if (mirror_type == RightToLeft || mirror_type == SwitchEach)
    {
      for (std::map<int, int>::iterator it = upper_body_mirror_joints_rl_.begin();
          it != upper_body_mirror_joints_rl_.end(); it++)
      {
        int right_id = it->first;
        int left_id = it->second;
        int mirror_value = convert4095ToMirror(right_id, before_step.position[right_id]);

        page_.step[index].position[left_id] = mirror_value;
      }
    }

    if (mirror_type == LeftToRight || mirror_type == SwitchEach)
    {
      for (std::map<int, int>::iterator it = upper_body_mirror_joints_lr_.begin();
          it != upper_body_mirror_joints_lr_.end(); it++)
      {
        int left_id = it->first;
        int right_id = it->second;
        int mirror_value = convert4095ToMirror(left_id, before_step.position[left_id]);

        page_.step[index].position[right_id] = mirror_value;
      }
    }
  }

  if (target_type == LowerBody || target_type == AllBody)
  {
    if (mirror_type == RightToLeft || mirror_type == SwitchEach)
    {
      for (std::map<int, int>::iterator it = lower_body_mirror_joints_rl_.begin();
          it != lower_body_mirror_joints_rl_.end(); it++)
      {
        int right_id = it->first;
        int left_id = it->second;
        int mirror_value = convert4095ToMirror(right_id, before_step.position[right_id]);

        page_.step[index].position[left_id] = mirror_value;
      }
    }

    if (mirror_type == LeftToRight || mirror_type == SwitchEach)
    {
      for (std::map<int, int>::iterator it = lower_body_mirror_joints_lr_.begin();
          it != lower_body_mirror_joints_lr_.end(); it++)
      {
        int left_id = it->first;
        int right_id = it->second;
        int mirror_value = convert4095ToMirror(left_id, before_step.position[left_id]);

        page_.step[index].position[right_id] = mirror_value;
      }
    }
  }

  // draw step
  drawStep(index);
  edited_ = true;
}

void ActionEditor::writeStepCmd(int index)
{
  for (std::map<int, int>::iterator it = joint_id_to_row_index_.begin(); it != joint_id_to_row_index_.end(); it++)
  {
    int id = it->first;

    if (step_.position[id] & action_file_define::TORQUE_OFF_BIT_MASK)
      return;
  }

  if (index >= 0 && index < action_file_define::MAXNUM_STEP)
  {
    page_.step[index] = step_;
    drawStep(index);
    edited_ = true;
  }
  else
    printCmd("Invalid step index");
}

void ActionEditor::deleteStepCmd(int index)
{
  if (index >= 0 && index < action_file_define::MAXNUM_STEP)
  {
    for (int i = index; i < action_file_define::MAXNUM_STEP; i++)
    {
      if (i == (action_file_define::MAXNUM_STEP - 1))
      {
        for (int jointIndex = 0; jointIndex < action_file_define::MAXNUM_JOINTS; jointIndex++)
          page_.step[i].position[jointIndex] = action_file_define::INVALID_BIT_MASK;

        page_.step[i].pause = 0;
        page_.step[i].time = 0;
      }
      else
        page_.step[i] = page_.step[i + 1];

      drawStep(i);
    }

    if (index < page_.header.stepnum)
    {
      if (page_.header.stepnum != 0)
      {
        drawStepLine(true);
        page_.header.stepnum--;
        drawStepLine(false);
      }

      goToCursor(pageparam_col_, step_num_row_);
      printf("%.3d", page_.header.stepnum);
    }

    edited_ = true;
  }
  else
    printCmd("Invalid step index");
}

void ActionEditor::insertStepCmd(int index)
{
  for (std::map<int, int>::iterator it = joint_id_to_row_index_.begin(); it != joint_id_to_row_index_.end(); it++)
  {
    int id = it->first;
    if (step_.position[id] & action_file_define::TORQUE_OFF_BIT_MASK)
    {
      printCmd("Exist invalid joint value");
      return;
    }
  }

  if (index >= 0 && index < action_file_define::MAXNUM_STEP)
  {
    for (int i = action_file_define::MAXNUM_STEP - 1; i > index; i--)
    {
      page_.step[i] = page_.step[i - 1];
      drawStep(i);
    }

    page_.step[index] = step_;
    drawStep(index);

    if (index == 0 || index < page_.header.stepnum)
    {
      if (page_.header.stepnum != action_file_define::MAXNUM_STEP)
      {
        drawStepLine(true);
        page_.header.stepnum++;
        drawStepLine(false);
      }

      goToCursor(pageparam_col_, step_num_row_);
      printf("%.3d", page_.header.stepnum);
    }

    edited_ = true;
  }
  else
    printCmd("Invalid step index");
}

void ActionEditor::insertInterpolationStepCmd(int index, int ratio)
{
  // check index (0 ~ 6)
  if (index < 0 || (index + 1) >= action_file_define::MAXNUM_STEP)
  {
    printCmd("Invalid step index. input [0 - 6].");
    return;
  }

  if((index + 1) >= page_.header.stepnum)
  {
    printCmd("Invalid step index.");
    return;
  }

  // check ratio
  if (ratio < 0 || ratio > 10)
  {
    printCmd("Invalid ratio. input from [0 - 10].");
    return;
  }

  // move further steps to next
  action_file_define::Step step_b = page_.step[index + 1];
  action_file_define::Step step_a = page_.step[index];
  double ratio_double = ratio / 10.0;

  for (int i = action_file_define::MAXNUM_STEP - 1; i > (index + 1); i--)
  {
    page_.step[i] = page_.step[i - 1];
    drawStep(i);
  }

  // interpolation and insert
  for (std::map<int, int>::iterator it = joint_id_to_row_index_.begin(); it != joint_id_to_row_index_.end(); it++)
  {
    int id = it->first;
    step_a.position[id] = step_a.position[id] * (1 - ratio_double) + step_b.position[id] * ratio_double;
  }

  page_.step[index + 1] = step_a;
  drawStep(index + 1);

  if (index == 0 || index < page_.header.stepnum)
  {
    if (page_.header.stepnum != action_file_define::MAXNUM_STEP)
    {
      drawStepLine(true);
      page_.header.stepnum++;
      drawStepLine(false);
    }

    goToCursor(pageparam_col_, step_num_row_);
    printf("%.3d", page_.header.stepnum);
  }

  edited_ = true;
}

void ActionEditor::moveStepCmd(int src, int dst)
{
  if (src < 0 || src >= action_file_define::MAXNUM_STEP)
  {
    printCmd("Invalid step index");
    return;
  }

  if (dst < 0 || dst >= action_file_define::MAXNUM_STEP)
  {
    printCmd("Invalid step index");
    return;
  }

  if (src == dst)
    return;

  action_file_define::Step step = page_.step[src];
  if (src < dst)
  {
    for (int i = src; i < dst; i++)
    {
      page_.step[i] = page_.step[i + 1];
      drawStep(i);
    }
  }
  else
  {
    for (int i = src; i > dst; i--)
    {
      page_.step[i] = page_.step[i - 1];
      drawStep(i);
    }
  }

  page_.step[dst] = step;
  drawStep(dst);
  edited_ = true;
}

void ActionEditor::copyCmd(int index)
{
  if (index == page_idx_)
    return;

  if (ActionModule::getInstance()->loadPage(index, &page_) == true)
  {
    drawPage();
    edited_ = true;
  }
  else
    printCmd("Invalid page index");
}

void ActionEditor::newCmd()
{
  ActionModule::getInstance()->resetPage(&page_);
  drawPage();
  edited_ = true;
}

void ActionEditor::goCmd(int index)
{
  if (index < 0 || index >= action_file_define::MAXNUM_STEP)
  {
    printCmd("Invalid step index");
    return;
  }

  if (index > page_.header.stepnum)
  {
    printCmd("Are you sure? (y/n)");
    if (_getch() != 'y')
    {
      clearCmd();
      return;
    }
  }

  int id;
  int32_t goal_position, start_position, distance;
  int max_distance = 0;

  for (std::map<std::string, dynamixel::GroupSyncWrite *>::iterator it = port_to_sync_write_go_cmd_.begin();
      it != port_to_sync_write_go_cmd_.end(); it++)
  {
    it->second->clearParam();
  }

  for (std::map<int, int>::iterator it = joint_id_to_row_index_.begin(); it != joint_id_to_row_index_.end(); it++)
  {
    id = it->first;
    std::string joint_name = joint_id_to_name_[id];
    if (page_.step[index].position[id] & action_file_define::INVALID_BIT_MASK)
    {
      printCmd("Exist invalid joint value");
      return;
    }

    if (ctrl_->readCtrlItem(joint_name, "present_position", (uint32_t*) &start_position, 0) != COMM_SUCCESS)
    {
      printCmd("Failed to read position");
      return;
    }

    goal_position = convert4095ToPositionValue(id, page_.step[index].position[id]);
    if (start_position > goal_position)
      distance = start_position - goal_position;
    else
      distance = goal_position - start_position;

//    wDistance = 200;
    distance = distance * 0.05;

    if (max_distance < distance)
      max_distance = distance;

    int offset = robot_->dxls_[joint_name]->convertRadian2Value(robot_->dxls_[joint_name]->dxl_state_->position_offset_)
        - robot_->dxls_[joint_name]->value_of_0_radian_position_;
    goal_position = goal_position + offset;

    uint8_t param[8];

    param[0] = DXL_LOBYTE(DXL_LOWORD(distance));
    param[1] = DXL_HIBYTE(DXL_LOWORD(distance));
    param[2] = DXL_LOBYTE(DXL_HIWORD(distance));
    param[3] = DXL_HIBYTE(DXL_HIWORD(distance));
    param[4] = DXL_LOBYTE(DXL_LOWORD(goal_position));
    param[5] = DXL_HIBYTE(DXL_LOWORD(goal_position));
    param[6] = DXL_LOBYTE(DXL_HIWORD(goal_position));
    param[7] = DXL_HIBYTE(DXL_HIWORD(goal_position));

    for (std::map<std::string, dynamixel::GroupSyncWrite *>::iterator it = port_to_sync_write_go_cmd_.begin();
        it != port_to_sync_write_go_cmd_.end(); it++)
    {
      it->second->addParam(id, param);
    }
  }

  for (std::map<std::string, dynamixel::GroupSyncWrite *>::iterator it = port_to_sync_write_go_cmd_.begin();
      it != port_to_sync_write_go_cmd_.end(); it++)
  {
    it->second->txPacket();
  }

  sleep(max_distance / 1000 + 2);

  for (std::map<std::string, dynamixel::GroupSyncWrite *>::iterator it = port_to_sync_write_go_cmd_.begin();
      it != port_to_sync_write_go_cmd_.end(); it++)
  {
    it->second->clearParam();
  }

  for (std::map<int, int>::iterator it = joint_id_to_row_index_.begin(); it != joint_id_to_row_index_.end(); it++)
  {
    id = it->first;
    std::string joint_name = joint_id_to_name_[id];

    distance = 0;

    goal_position = page_.step[index].position[id];

    int offset = robot_->dxls_[joint_name]->convertRadian2Value(robot_->dxls_[joint_name]->dxl_state_->position_offset_)
        - robot_->dxls_[joint_name]->value_of_0_radian_position_;
    goal_position = goal_position + offset;

    uint8_t param[8];

    param[0] = DXL_LOBYTE(DXL_LOWORD(distance));
    param[1] = DXL_HIBYTE(DXL_LOWORD(distance));
    param[2] = DXL_LOBYTE(DXL_HIWORD(distance));
    param[3] = DXL_HIBYTE(DXL_HIWORD(distance));
    param[4] = DXL_LOBYTE(DXL_LOWORD(goal_position));
    param[5] = DXL_HIBYTE(DXL_LOWORD(goal_position));
    param[6] = DXL_LOBYTE(DXL_HIWORD(goal_position));
    param[7] = DXL_HIBYTE(DXL_HIWORD(goal_position));

    for (std::map<std::string, dynamixel::GroupSyncWrite *>::iterator it = port_to_sync_write_go_cmd_.begin();
        it != port_to_sync_write_go_cmd_.end(); it++)
    {
      it->second->addParam(id, param);
    }
  }

  for (std::map<std::string, dynamixel::GroupSyncWrite *>::iterator it = port_to_sync_write_go_cmd_.begin();
      it != port_to_sync_write_go_cmd_.end(); it++)
  {
    it->second->txPacket();
  }

  step_ = page_.step[index];
  drawStep(7);
  goToCursor(cmd_col_, cmd_row_);
  printf("Go Command Completed");
}

void ActionEditor::saveCmd()
{
  if (edited_ == false)
    return;

  if (ActionModule::getInstance()->savePage(page_idx_, &page_) == true)
    edited_ = false;
}

void ActionEditor::nameCmd()
{
  clearCmd();
  goToCursor(cmd_col_, cmd_row_);
  printf("name: ");
  char name[80] = { 0 };
  fgets(name, 80, stdin);
  fflush(stdin);
  for (int i = 0; i <= action_file_define::MAXNUM_NAME; i++)
    page_.header.name[i] = name[i];
  drawName();
  edited_ = true;
}
