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

/* Author: Jay Song */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include "op3_walking_tuner/cmd_process.h"

#define SYNC_WRITE_START_ADDR  112 // Profile Velocity
#define SYNC_WRITE_LENGH       8   // Profile Velocity(4) + Goal Position(4)

//using namespace ROBOTIS;

int Col = PARAM_COL;
int Row = WALKING_MODE_ROW;
int Old_Col;
int Old_Row;
bool bBeginCommandMode = false;
bool bEdited = false;
int indexPage = 1;

robotis_framework::RobotisController* ctrl;
robotis_framework::Robot* op3_robot;

ros::Publisher enable_ctrl_module_pub;
ros::Publisher walking_command_pub;
ros::Publisher set_walking_param_pub;
ros::Publisher ini_pose_pub;
ros::ServiceClient get_walking_param_client;

std::map<std::string, dynamixel::GroupSyncWrite *> port_to_sync_write_go_cmd;

op3_walking_module_msgs::WalkingParam walking_param_msg;
op3_walking_module_msgs::GetWalkingParam get_walking_param_srv;

int _getch()
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

int kbhit(void)
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

struct termios oldterm, new_term;

void set_stdin(void)
{
  tcgetattr(0, &oldterm);
  new_term = oldterm;
  new_term.c_lflag &= ~(ICANON | ECHO | ISIG);
  new_term.c_cc[VMIN] = 1;
  new_term.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &new_term);
}

void reset_stdin(void)
{
  tcsetattr(0, TCSANOW, &oldterm);
}

bool AskSave()
{
  if (bEdited == true)
  {
    PrintCmd("Are you sure? (y/n)");
    if (_getch() != 'y')
    {
      ClearCmd();
      return true;
    }
  }
  return false;
}

void GoToCursor(int col, int row)
{
  char *cursor;
  char *esc_sequence;
  cursor = tigetstr("cup");
  esc_sequence = tparm(cursor, row, col);
  putp(esc_sequence);

  Col = col;
  Row = row;
}

void MoveUpCursor()
{
  if (Col == PARAM_COL)
  {
    if (Row > 0)
      GoToCursor(Col, Row - 1);
  }
}

void MoveDownCursor()
{
  if (Col == PARAM_COL)
  {
    if (Row < CMD_ROW - 1)
      GoToCursor(Col, Row + 1);
  }
}

bool InitializeWalkingTuner(std::string robot_file_path, std::string init_file_path, std::string offset_file_path)
{
  ctrl = robotis_framework::RobotisController::getInstance();

  dynamixel::PortHandler *_port_h = (dynamixel::PortHandler *) dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
  bool _set_port = _port_h->setBaudRate(1000000);
  if (_set_port == false)
    ROS_ERROR("Error Set port");
  dynamixel::PacketHandler *_packet_h = dynamixel::PacketHandler::getPacketHandler(1.0);

  int _return = _packet_h->write1ByteTxRx(_port_h, 200, 24, 1);
  ROS_INFO("Torque on DXLs! [%d]", _return);
  _packet_h->printTxRxResult(_return);

  _port_h->closePort();

  //Controller Initialize with robot file info
  if (ctrl->initialize(robot_file_path, init_file_path) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return false;
  }

  op3_robot = ctrl->robot_;

  ctrl->loadOffset(offset_file_path);

  //Add Sensor Module
  ctrl->addSensorModule((robotis_framework::SensorModule*) robotis_op::CM740Module::getInstance());

  //Add Motion Module
  ctrl->addMotionModule((robotis_framework::MotionModule*) robotis_op::BaseModule::getInstance());
  ctrl->addMotionModule((robotis_framework::MotionModule*) robotis_op::WalkingModule::getInstance());

  //Service and Topic registration(?)
  ros::NodeHandle _nh;
  ini_pose_pub = _nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  enable_ctrl_module_pub = _nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  walking_command_pub = _nh.advertise<std_msgs::String>("/robotis/walking/command", 0);
  set_walking_param_pub = _nh.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 0);
  get_walking_param_client = _nh.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/robotis/walking/get_params",
                                                                                         0);

  for (std::map<std::string, dynamixel::PortHandler *>::iterator _it = op3_robot->ports_.begin();
      _it != op3_robot->ports_.end(); _it++)
  {
    std::string _port_defalut_joint_name = op3_robot->port_default_device_[_it->first];

    //if there is actuator at this port, then add to port_to_sync_write.
    if (op3_robot->dxls_[_port_defalut_joint_name]->goal_position_item_ == NULL)
      continue;
    else
    {
      port_to_sync_write_go_cmd[_it->first] = new dynamixel::GroupSyncWrite(
          _it->second, dynamixel::PacketHandler::getPacketHandler(2.0),
          SYNC_WRITE_START_ADDR,
          SYNC_WRITE_LENGH);
    }
  }

  return true;
}

bool DrawIntro()
{
  for (std::map<std::string, dynamixel::GroupSyncWrite *>::iterator _it = port_to_sync_write_go_cmd.begin();
      _it != port_to_sync_write_go_cmd.end(); _it++)
  {
    _it->second->clearParam();
  }

  int _goal_position, _start_position, _distance, _offset;
//
//	//make syncwrite packet for going to initial fose
//	for(std::map<std::string, Dynamixel*>::iterator _it = op3_robot->dxls.begin(); _it != op3_robot->dxls.end(); _it++)
//	{
//	    std::string _joint_name = _it->first;
//	    Dynamixel*  _dxl        = _it->second;
//
//	    if(_dxl->goal_position_item == NULL)
//	        continue;
//
//	    _start_position = _dxl->ConvertRadian2Value(_dxl->dxl_state->goal_position);
//
//	    if(WalkingMotionModule::getInstance()->result_[_joint_name] == NULL)
//	        _goal_position = _dxl->ConvertRadian2Value(0.0);
//	    else
//	        _goal_position = _dxl->ConvertRadian2Value(WalkingMotionModule::getInstance()->result_[_joint_name]->goal_position);
//
//
//	    _offset = _dxl->ConvertRadian2Value(_dxl->dxl_state->position_offset) - _dxl->value_of_0_radian_position;
//
//	    _goal_position = _goal_position + _offset;
//
//	    if( _start_position > _goal_position )
//	        _distance = _start_position - _goal_position;
//	    else
//	        _distance = _goal_position - _start_position;
//
//	    _distance >>= 2;
//	    if( _distance < 8 )
//	        _distance = 8;
//
//	    UINT8_T param[8];
//        param[0] = DXL_LOBYTE(DXL_LOWORD(_distance));
//        param[1] = DXL_HIBYTE(DXL_LOWORD(_distance));
//        param[2] = DXL_LOBYTE(DXL_HIWORD(_distance));
//        param[3] = DXL_HIBYTE(DXL_HIWORD(_distance));
//        param[4] = DXL_LOBYTE(DXL_LOWORD(_goal_position));
//        param[5] = DXL_HIBYTE(DXL_LOWORD(_goal_position));
//        param[6] = DXL_LOBYTE(DXL_HIWORD(_goal_position));
//        param[7] = DXL_HIBYTE(DXL_HIWORD(_goal_position));
//
//        port_to_sync_write_go_cmd[_dxl->port_name]->AddParam(_dxl->id, param);
//	}
//
//    for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_go_cmd.begin(); _it != port_to_sync_write_go_cmd.end(); _it++)
//    {
//        _it->second->TxPacket();
//    }

  ctrl->startTimer();
  usleep(800 * 1000);

  std_msgs::String _msg;
  _msg.data = "ini_pose";
  ini_pose_pub.publish(_msg);

  usleep(8000 * 1000);

  int nrows, ncolumns;
  setupterm(NULL, fileno(stdout), (int *) 0);
  nrows = tigetnum("lines");
  ncolumns = tigetnum("cols");

  system("clear");
  printf("\n");
  printf("[Walking Tuner for DARwIn %s]\n", PROGRAM_VERSION);
  printf("\n");
  printf(" *Terminal screen size must be %d(col)x%d(row).\n", SCREEN_COL, SCREEN_ROW);
  printf(" *Current terminal has %d columns and %d rows.\n", ncolumns, nrows);
  printf("\n");
  printf("\n");
  printf("Press any key to start program...\n");
  _getch();

//	MotionManager::getInstance()->ResetGyroCalibration();

//    for(std::map<std::string, Dynamixel*>::iterator _it = op3_robot->dxls.begin(); _it != op3_robot->dxls.end(); _it++)
//    {
//        std::string _joint_name = _it->first;
//        Dynamixel*  _dxl        = _it->second;
//
//        if(_dxl->goal_position_item == NULL)
//            continue;
//
//        _start_position = _dxl->ConvertRadian2Value(_dxl->dxl_state->goal_position);
//
//        if(WalkingMotionModule::getInstance()->result_[_joint_name] == NULL)
//            _goal_position = _dxl->ConvertRadian2Value(0);
//        else
//            _goal_position = _dxl->ConvertRadian2Value(WalkingMotionModule::getInstance()->result_[_joint_name]->goal_position);
//
//        _offset = _dxl->ConvertRadian2Value(_dxl->dxl_state->position_offset) - _dxl->value_of_0_radian_position;
//
//        _goal_position = _goal_position + _offset;
//
//        if( _start_position > _goal_position )
//            _distance = _start_position - _goal_position;
//        else
//            _distance = _goal_position - _start_position;
//
//        _distance = 0;
//
//        UINT8_T param[8];
//        param[0] = DXL_LOBYTE(DXL_LOWORD(_distance));
//        param[1] = DXL_HIBYTE(DXL_LOWORD(_distance));
//        param[2] = DXL_LOBYTE(DXL_HIWORD(_distance));
//        param[3] = DXL_HIBYTE(DXL_HIWORD(_distance));
//        param[4] = DXL_LOBYTE(DXL_LOWORD(_goal_position));
//        param[5] = DXL_HIBYTE(DXL_LOWORD(_goal_position));
//        param[6] = DXL_LOBYTE(DXL_HIWORD(_goal_position));
//        param[7] = DXL_HIBYTE(DXL_HIWORD(_goal_position));
//
//        port_to_sync_write_go_cmd[_dxl->port_name]->AddParam(_dxl->id, param);
//    }
//
//    for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write_go_cmd.begin(); _it != port_to_sync_write_go_cmd.end(); _it++)
//    {
//        _it->second->TxPacket();
//    }
//    usleep(10*1000);

  _msg.data = "walking_module";
  enable_ctrl_module_pub.publish(_msg);

  ROS_WARN("Change Enable CTRL MODE!!!");

  if (get_walking_param_client.call(get_walking_param_srv) == false)
  {
    ROS_ERROR("Failed to get walking parameters");
    return false;
  }
  else
  {
    walking_param_msg = get_walking_param_srv.response.parameters;
    DrawScreen();
    return true;
  }

  return true;
}

void DrawEnding()
{
  system("clear");
  printf("\n");
  printf("Terminate Walking Tuner");
  printf("\n");
}

void DrawScreen()
{
  int old_col = Col;
  int old_row = Row;

  system("clear");
  GoToCursor(0, 0);

  // Display menu
  //      01234567890123456789012345678901234  Total:35x29
  printf("Walking Mode(on/off)            \n");  // 0
  printf("X offset(m)                     \n");  // 1
  printf("Y offset(m)                     \n");  // 2
  printf("Z offset(m)                     \n");  // 3
  printf("Roll(x) offset(degree)          \n");  // 4
  printf("Pitch(y) offset(degree)         \n");  // 5
  printf("Yaw(z) offset(degree)           \n");  // 6
  printf("Hip pitch offset(degree)        \n");  // 7
  printf("Auto balance(on/off)            \n");  // 8
  printf("Period time(sec)                \n");  // 9
  printf("DSP ratio                       \n");  // 0
  printf("Step forward/back ratio         \n");  // 1
  printf("Step forward/back(m)            \n");  // 2
  printf("Step right/left(m)              \n");  // 3
  printf("Step direction(degree)          \n");  // 4
  printf("Turning aim(on/off)             \n");  // 5
  printf("Foot height(m)                  \n");  // 6
  printf("Swing right/left(m)             \n");  // 7
  printf("Swing top/down(m)               \n");  // 8
  printf("Pelvis offset(degree)           \n");  // 9
  printf("Arm swing gain                  \n");  // 0
  printf("Balance knee gain               \n");  // 1
  printf("Balance ankle pitch gain        \n");  // 2
  printf("Balance hip roll gain           \n");  // 3
  printf("Balance ankle roll gain         \n");  // 4
  printf("P gain                          \n");  // 5
  printf("I gain                          \n");  // 6
  printf("D gain                          \n");  // 7
  ClearCmd();  // 8

  GoToCursor(PARAM_COL, WALKING_MODE_ROW);
  if (robotis_op::WalkingModule::getInstance()->isRunning() == true)
    printf("ON     ");
  else
    printf("OFF    ");

  GoToCursor(PARAM_COL, X_OFFSET_ROW);
  printf("%.3f    ", walking_param_msg.init_x_offset);

  GoToCursor(PARAM_COL, Y_OFFSET_ROW);
  printf("%.3f    ", walking_param_msg.init_y_offset);

  GoToCursor(PARAM_COL, Z_OFFSET_ROW);
  printf("%.3f    ", walking_param_msg.init_z_offset);

  GoToCursor(PARAM_COL, ROLL_OFFSET_ROW);
  printf("%.1f    ", walking_param_msg.init_roll_offset * RADIAN2DEGREE);

  GoToCursor(PARAM_COL, PITCH_OFFSET_ROW);
  printf("%.1f    ", walking_param_msg.init_pitch_offset * RADIAN2DEGREE);

  GoToCursor(PARAM_COL, YAW_OFFSET_ROW);
  printf("%.1f    ", walking_param_msg.init_yaw_offset * RADIAN2DEGREE);

  GoToCursor(PARAM_COL, HIP_PITCH_OFFSET_ROW);
  printf("%.1f    ", walking_param_msg.hip_pitch_offset * RADIAN2DEGREE);

  GoToCursor(PARAM_COL, AUTO_BALANCE_ROW);
  if (walking_param_msg.balance_enable == true)
    printf("ON     ");
  else
    printf("OFF    ");

  GoToCursor(PARAM_COL, PERIOD_TIME_ROW);
  printf("%.3f    ", walking_param_msg.period_time);

  GoToCursor(PARAM_COL, DSP_RATIO_ROW);
  printf("%.2f    ", walking_param_msg.dsp_ratio);

  GoToCursor(PARAM_COL, STEP_FORWARDBACK_RATIO_ROW);
  printf("%.2f    ", walking_param_msg.step_fb_ratio);

  GoToCursor(PARAM_COL, STEP_FORWARDBACK_ROW);
  printf("%.3f    ", walking_param_msg.x_move_amplitude);

  GoToCursor(PARAM_COL, STEP_RIGHTLEFT_ROW);
  printf("%.3f    ", walking_param_msg.y_move_amplitude);

  GoToCursor(PARAM_COL, STEP_DIRECTION_ROW);
  printf("%.1f    ", walking_param_msg.angle_move_amplitude * RADIAN2DEGREE);

  GoToCursor(PARAM_COL, TURNING_AIM_ROW);
  if (walking_param_msg.move_aim_on == true)
    printf("ON     ");
  else
    printf("OFF    ");

  GoToCursor(PARAM_COL, FOOT_HEIGHT_ROW);
  printf("%.3f    ", walking_param_msg.z_move_amplitude);

  GoToCursor(PARAM_COL, SWING_RIGHTLEFT_ROW);
  printf("%.3f    ", walking_param_msg.y_swap_amplitude);

  GoToCursor(PARAM_COL, SWING_TOPDOWN_ROW);
  printf("%.3f    ", walking_param_msg.z_swap_amplitude);

  GoToCursor(PARAM_COL, PELVIS_OFFSET_ROW);
  printf("%.1f    ", walking_param_msg.pelvis_offset * RADIAN2DEGREE);

  GoToCursor(PARAM_COL, ARM_SWING_GAIN_ROW);
  printf("%.1f    ", walking_param_msg.arm_swing_gain);

  GoToCursor(PARAM_COL, BAL_KNEE_GAIN_ROW);
  printf("%.2f    ", walking_param_msg.balance_knee_gain);

  GoToCursor(PARAM_COL, BAL_ANKLE_PITCH_GAIN_ROW);
  printf("%.2f    ", walking_param_msg.balance_ankle_pitch_gain);

  GoToCursor(PARAM_COL, BAL_HIP_ROLL_GAIN_ROW);
  printf("%.2f    ", walking_param_msg.balance_hip_roll_gain);

  GoToCursor(PARAM_COL, BAL_ANKLE_ROLL_GAIN_ROW);
  printf("%.2f    ", walking_param_msg.balance_ankle_roll_gain);

  GoToCursor(PARAM_COL, P_GAIN_ROW);
  printf("%d    ", walking_param_msg.p_gain);

  GoToCursor(PARAM_COL, I_GAIN_ROW);
  printf("%d    ", walking_param_msg.i_gain);

  GoToCursor(PARAM_COL, D_GAIN_ROW);
  printf("%d    ", walking_param_msg.d_gain);

  GoToCursor(old_col, old_row);
}

void ClearCmd()
{
  PrintCmd("");
}

void PrintCmd(std::string message)
{
  int len = strlen(message.c_str());
  GoToCursor(0, CMD_ROW);

  printf("] %s", message.c_str());
  for (int i = 0; i < (SCREEN_COL - (len + 2)); i++)
    printf(" ");

  GoToCursor(len + 2, CMD_ROW);
}

//void IncreaseValue(bool large)
//{
//    int col;
//    int row;
//    if(bBeginCommandMode == true)
//    {
//        col = Old_Col;
//        row = Old_Row;
//    }
//    else
//    {
//        col = Col;
//        row = Row;
//    }
//
//    if(col != PARAM_COL)
//        return;
//
//    GoToCursor(col, row);
//
//    std_msgs::String _walking_command_msg;
//
//    switch(row)
//    {
//    case WALKING_MODE_ROW:
//        _walking_command_msg.data = "start";
//        walking_command_pub.publish(_walking_command_msg);
//        printf("ON    ");
//        break;
//
//    case X_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.init_x_offset += 10*0.001;
//        else
//            walking_param_msg.init_x_offset += 1*0.001;
//        printf("%d    ", walking_param_msg.init_x_offset);
//        break;
//
//    case Y_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.init_y_offset += 10*0.001;
//        else
//            walking_param_msg.init_y_offset += 1*0.001;
//        printf("%d    ", walking_param_msg.init_y_offset);
//        break;
//
//    case Z_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.init_z_offset += 10*0.001;
//        else
//            walking_param_msg.init_z_offset += 1*0.001;
//        printf("%d    ", walking_param_msg.init_z_offset);
//        break;
//
//    case ROLL_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.init_roll_offset += 1.0*DEGREE2RADIAN;
//        else
//            walking_param_msg.init_roll_offset += 0.1*DEGREE2RADIAN;
//        printf("%.1f    ", walking_param_msg.init_roll_offset*RADIAN2DEGREE);
//        break;
//
//    case PITCH_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.init_pitch_offset += 1.0*DEGREE2RADIAN;
//        else
//            walking_param_msg.init_pitch_offset += 0.1*DEGREE2RADIAN;
//        printf("%.1f    ", walking_param_msg.init_pitch_offset*RADIAN2DEGREE);
//        break;
//
//    case YAW_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.init_yaw_offset += 1.0*DEGREE2RADIAN;
//        else
//            walking_param_msg.init_yaw_offset += 0.1*DEGREE2RADIAN;
//        printf("%.1f    ", walking_param_msg.init_yaw_offset*RADIAN2DEGREE);
//        break;
//
//    case HIP_PITCH_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.hip_pitch_offset += 1.0*DEGREE2RADIAN;
//        else
//            walking_param_msg.hip_pitch_offset += 0.1*DEGREE2RADIAN;
//        printf("%.1f    ", walking_param_msg.hip_pitch_offset*RADIAN2DEGREE);
//        break;
//
//    case AUTO_BALANCE_ROW:
//        walking_param_msg.balance_enable = true;
//        printf("ON    ");
//        break;
//
//    case PERIOD_TIME_ROW:
//        if(large == true)
//            walking_param_msg.period_time += 10*0.001;
//        else
//            walking_param_msg.period_time += 1*0.001;
//        printf("%d    ", walking_param_msg.period_time);
//        break;
//
//    case DSP_RATIO_ROW:
//        if(large == true)
//            walking_param_msg.dsp_ratio += 0.1;
//        else
//            walking_param_msg.dsp_ratio += 0.01;
//        printf("%.2f    ", walking_param_msg.dsp_ratio);
//        break;
//
//    case STEP_FORWARDBACK_RATIO_ROW:
//        if(large == true)
//            walking_param_msg.step_fb_ratio += 0.1;
//        else
//            walking_param_msg.step_fb_ratio += 0.01;
//        printf("%.2f    ", walking_param_msg.step_fb_ratio);
//        break;
//
//    case STEP_FORWARDBACK_ROW:
//        if(large == true)
//            walking_param_msg.x_move_amplitude += 10;
//        else
//            walking_param_msg.x_move_amplitude += 1;
//        printf("%d    ", (int)walking_param_msg.x_move_amplitude);
//        break;
//
//    case STEP_RIGHTLEFT_ROW:
//        if(large == true)
//            walking_param_msg.y_move_amplitude += 10;
//        else
//            walking_param_msg.y_move_amplitude += 1;
//        printf("%d    ", (int)walking_param_msg.y_move_amplitude);
//        break;
//
//    case STEP_DIRECTION_ROW:
//        if(large == true)
//            walking_param_msg.angle_move_amplitude += 10;
//        else
//            walking_param_msg.angle_move_amplitude += 1;
//        printf("%d    ", (int)walking_param_msg.angle_move_amplitude);
//        break;
//
//    case TURNING_AIM_ROW:
//        walking_param_msg.move_aim_on = true;
//        printf("ON   ");
//        break;
//
//    case FOOT_HEIGHT_ROW:
//        if(large == true)
//            walking_param_msg.z_move_amplitude += 10;
//        else
//            walking_param_msg.z_move_amplitude += 1;
//        printf("%d    ", (int)walking_param_msg.z_move_amplitude);
//        break;
//
//    case SWING_RIGHTLEFT_ROW:
//        if(large == true)
//            walking_param_msg.y_swap_amplitude += 1.0;
//        else
//            walking_param_msg.y_swap_amplitude += 0.1;
//        printf("%.1f    ", walking_param_msg.y_swap_amplitude);
//        break;
//
//    case SWING_TOPDOWN_ROW:
//        if(large == true)
//            walking_param_msg.z_swap_amplitude += 10;
//        else
//            walking_param_msg.z_swap_amplitude += 1;
//        printf("%d    ", (int)walking_param_msg.z_swap_amplitude);
//        break;
//
//    case PELVIS_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.pelvis_offset += 1.0;
//        else
//            walking_param_msg.pelvis_offset += 0.1;
//        printf("%.1f    ", walking_param_msg.pelvis_offset);
//        break;
//
//    case ARM_SWING_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.arm_swing_gain += 1.0;
//        else
//            walking_param_msg.arm_swing_gain += 0.1;
//        printf("%.1f    ", walking_param_msg.arm_swing_gain);
//        break;
//
//    case BAL_KNEE_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.balance_knee_gain += 0.1;
//        else
//            walking_param_msg.balance_knee_gain += 0.01;
//        printf("%.2f    ", walking_param_msg.balance_knee_gain);
//        break;
//
//    case BAL_ANKLE_PITCH_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.balance_ankle_pitch_gain += 0.1;
//        else
//            walking_param_msg.balance_ankle_pitch_gain += 0.01;
//        printf("%.2f    ", walking_param_msg.balance_ankle_pitch_gain);
//        break;
//
//    case BAL_HIP_ROLL_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.balance_hip_roll_gain += 0.1;
//        else
//            walking_param_msg.balance_hip_roll_gain += 0.01;
//        printf("%.2f    ", walking_param_msg.balance_hip_roll_gain);
//        break;
//
//    case BAL_ANKLE_ROLL_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.balance_ankle_roll_gain += 0.1;
//        else
//            walking_param_msg.balance_ankle_roll_gain += 0.01;
//        printf("%.2f    ", walking_param_msg.balance_ankle_roll_gain);
//        break;
//
//    case P_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.p_gain += 10;
//        else
//            walking_param_msg.p_gain += 1;
//        printf("%d    ", walking_param_msg.p_gain);
//        break;
//
//    case I_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.i_gain += 10;
//        else
//            walking_param_msg.i_gain += 1;
//        printf("%d    ", walking_param_msg.i_gain);
//        break;
//
//    case D_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.d_gain += 10;
//        else
//            walking_param_msg.d_gain += 1;
//        printf("%d    ", walking_param_msg.d_gain);
//        break;
//    }
//
//    set_walking_param_pub.publish(walking_param_msg);
//
//    GoToCursor(col, row);
//}
//

void UpdateValue(bool large, double dir)
{
  double _mm_to_m = 0.001;
  int col;
  int row;
  if (bBeginCommandMode == true)
  {
    col = Old_Col;
    row = Old_Row;
  }
  else
  {
    col = Col;
    row = Row;
  }

  if (col != PARAM_COL)
    return;

  GoToCursor(col, row);

  std_msgs::String _walking_command_msg;

  switch (row)
  {
    case WALKING_MODE_ROW:
      if (dir > 0)
      {
        _walking_command_msg.data = "start";
        walking_command_pub.publish(_walking_command_msg);
        printf("ON    ");
      }
      else
      {
        _walking_command_msg.data = "stop";
        walking_command_pub.publish(_walking_command_msg);
        printf("OFF");
        walking_param_msg.x_move_amplitude = 0;
        walking_param_msg.y_move_amplitude = 0;
        walking_param_msg.angle_move_amplitude = 0;
        set_walking_param_pub.publish(walking_param_msg);
        GoToCursor(PARAM_COL, STEP_FORWARDBACK_ROW);
        printf("%d    ", (int) walking_param_msg.x_move_amplitude);
        GoToCursor(PARAM_COL, STEP_RIGHTLEFT_ROW);
        printf("%d    ", (int) walking_param_msg.y_move_amplitude);
        GoToCursor(PARAM_COL, STEP_DIRECTION_ROW);
        printf("%.1f    ", walking_param_msg.angle_move_amplitude);
      }
      break;

    case X_OFFSET_ROW:
      if (large == true)
        walking_param_msg.init_x_offset += dir * 10 * _mm_to_m;
      else
        walking_param_msg.init_x_offset += dir * 1 * _mm_to_m;
      printf("%.3f    ", walking_param_msg.init_x_offset);
      break;

    case Y_OFFSET_ROW:
      if (large == true)
        walking_param_msg.init_y_offset += dir * 10 * _mm_to_m;
      else
        walking_param_msg.init_y_offset += dir * 1 * _mm_to_m;
      printf("%.3f    ", walking_param_msg.init_y_offset);
      break;

    case Z_OFFSET_ROW:
      if (large == true)
        walking_param_msg.init_z_offset += dir * 10 * _mm_to_m;
      else
        walking_param_msg.init_z_offset += dir * 1 * _mm_to_m;
      printf("%.3f    ", walking_param_msg.init_z_offset);
      break;

    case ROLL_OFFSET_ROW:
      if (large == true)
        walking_param_msg.init_roll_offset += dir * 1.0 * DEGREE2RADIAN;
      else
        walking_param_msg.init_roll_offset += dir * 0.1 * DEGREE2RADIAN;
      printf("%.1f    ", walking_param_msg.init_roll_offset * RADIAN2DEGREE);
      break;

    case PITCH_OFFSET_ROW:
      if (large == true)
        walking_param_msg.init_pitch_offset += dir * 1.0 * DEGREE2RADIAN;
      else
        walking_param_msg.init_pitch_offset += dir * 0.1 * DEGREE2RADIAN;
      printf("%.1f    ", walking_param_msg.init_pitch_offset * RADIAN2DEGREE);
      break;

    case YAW_OFFSET_ROW:
      if (large == true)
        walking_param_msg.init_yaw_offset += dir * 1.0 * DEGREE2RADIAN;
      else
        walking_param_msg.init_yaw_offset += dir * 0.1 * DEGREE2RADIAN;
      printf("%.1f    ", walking_param_msg.init_yaw_offset * RADIAN2DEGREE);
      break;

    case HIP_PITCH_OFFSET_ROW:
      if (large == true)
        walking_param_msg.hip_pitch_offset += dir * 1.0 * DEGREE2RADIAN;
      else
        walking_param_msg.hip_pitch_offset += dir * 0.1 * DEGREE2RADIAN;
      printf("%.1f    ", walking_param_msg.hip_pitch_offset * RADIAN2DEGREE);
      break;

    case AUTO_BALANCE_ROW:
      if (dir > 0)
      {
        walking_param_msg.balance_enable = true;
        printf("ON    ");
      }
      else
      {
        walking_param_msg.balance_enable = true;
        printf("OFF   ");
      }
      break;

    case PERIOD_TIME_ROW:
      if (large == true)
        walking_param_msg.period_time += dir * 10 * _mm_to_m;
      else
        walking_param_msg.period_time += dir * 1 * _mm_to_m;
      printf("%.3f    ", walking_param_msg.period_time);
      break;

    case DSP_RATIO_ROW:
      if (large == true)
        walking_param_msg.dsp_ratio += dir * 0.1;
      else
        walking_param_msg.dsp_ratio += dir * 0.01;
      printf("%.2f    ", walking_param_msg.dsp_ratio);
      break;

    case STEP_FORWARDBACK_RATIO_ROW:
      if (large == true)
        walking_param_msg.step_fb_ratio += dir * 0.1;
      else
        walking_param_msg.step_fb_ratio += dir * 0.01;
      printf("%.2f    ", walking_param_msg.step_fb_ratio);
      break;

    case STEP_FORWARDBACK_ROW:
      if (large == true)
        walking_param_msg.x_move_amplitude += dir * 10 * _mm_to_m;
      else
        walking_param_msg.x_move_amplitude += dir * 1 * _mm_to_m;
      printf("%.3f    ", walking_param_msg.x_move_amplitude);
      break;

    case STEP_RIGHTLEFT_ROW:
      if (large == true)
        walking_param_msg.y_move_amplitude += dir * 10 * _mm_to_m;
      else
        walking_param_msg.y_move_amplitude += dir * 1 * _mm_to_m;
      printf("%.3f    ", walking_param_msg.y_move_amplitude);
      break;

    case STEP_DIRECTION_ROW:
      if (large == true)
        walking_param_msg.angle_move_amplitude += dir * 10 * DEGREE2RADIAN;
      else
        walking_param_msg.angle_move_amplitude += dir * 1 * DEGREE2RADIAN;
      printf("%.1f    ", walking_param_msg.angle_move_amplitude * RADIAN2DEGREE);
      break;

    case TURNING_AIM_ROW:
      walking_param_msg.move_aim_on = true;
      printf("ON   ");
      break;

    case FOOT_HEIGHT_ROW:
      if (large == true)
        walking_param_msg.z_move_amplitude += dir * 10 * _mm_to_m;
      else
        walking_param_msg.z_move_amplitude += dir * 1 * _mm_to_m;
      printf("%.3f    ", walking_param_msg.z_move_amplitude);
      break;

    case SWING_RIGHTLEFT_ROW:
      if (large == true)
        walking_param_msg.y_swap_amplitude += dir * 1.0 * _mm_to_m;
      else
        walking_param_msg.y_swap_amplitude += dir * 0.1 * _mm_to_m;
      printf("%.3f    ", walking_param_msg.y_swap_amplitude);
      break;

    case SWING_TOPDOWN_ROW:
      if (large == true)
        walking_param_msg.z_swap_amplitude += dir * 10 * _mm_to_m;
      else
        walking_param_msg.z_swap_amplitude += dir * 1 * _mm_to_m;
      printf("%.3f    ", walking_param_msg.z_swap_amplitude);
      break;

    case PELVIS_OFFSET_ROW:
      if (large == true)
        walking_param_msg.pelvis_offset += dir * 1.0 * DEGREE2RADIAN;
      else
        walking_param_msg.pelvis_offset += dir * 0.1 * DEGREE2RADIAN;
      printf("%.1f    ", walking_param_msg.pelvis_offset * RADIAN2DEGREE);
      break;

    case ARM_SWING_GAIN_ROW:
      if (large == true)
        walking_param_msg.arm_swing_gain += dir * 1.0;
      else
        walking_param_msg.arm_swing_gain += dir * 0.1;
      printf("%.1f    ", walking_param_msg.arm_swing_gain);
      break;

    case BAL_KNEE_GAIN_ROW:
      if (large == true)
        walking_param_msg.balance_knee_gain += dir * 0.1;
      else
        walking_param_msg.balance_knee_gain += dir * 0.01;
      printf("%.2f    ", walking_param_msg.balance_knee_gain);
      break;

    case BAL_ANKLE_PITCH_GAIN_ROW:
      if (large == true)
        walking_param_msg.balance_ankle_pitch_gain += dir * 0.1;
      else
        walking_param_msg.balance_ankle_pitch_gain += dir * 0.01;
      printf("%.2f    ", walking_param_msg.balance_ankle_pitch_gain);
      break;

    case BAL_HIP_ROLL_GAIN_ROW:
      if (large == true)
        walking_param_msg.balance_hip_roll_gain += dir * 0.1;
      else
        walking_param_msg.balance_hip_roll_gain += dir * 0.01;
      printf("%.2f    ", walking_param_msg.balance_hip_roll_gain);
      break;

    case BAL_ANKLE_ROLL_GAIN_ROW:
      if (large == true)
        walking_param_msg.balance_ankle_roll_gain += dir * 0.1;
      else
        walking_param_msg.balance_ankle_roll_gain += dir * 0.01;
      printf("%.2f    ", walking_param_msg.balance_ankle_roll_gain);
      break;

    case P_GAIN_ROW:
      if (large == true)
        walking_param_msg.p_gain += dir * 10;
      else
        walking_param_msg.p_gain += dir * 1;
      printf("%d    ", walking_param_msg.p_gain);
      break;

    case I_GAIN_ROW:
      if (large == true)
        walking_param_msg.i_gain += dir * 10;
      else
        walking_param_msg.i_gain += dir * 1;
      printf("%d    ", walking_param_msg.i_gain);
      break;

    case D_GAIN_ROW:
      if (large == true)
        walking_param_msg.d_gain += dir * 10;
      else
        walking_param_msg.d_gain += dir * 1;
      printf("%d    ", walking_param_msg.d_gain);
      break;
  }

  set_walking_param_pub.publish(walking_param_msg);

  GoToCursor(col, row);
}

//void DecreaseValue(bool large)
//{
//	int col;
//	int row;
//	if(bBeginCommandMode == true)
//	{
//		col = Old_Col;
//		row = Old_Row;
//	}
//	else
//	{
//		col = Col;
//		row = Row;
//	}
//
//	if(col != PARAM_COL)
//		return;
//
//	GoToCursor(col, row);
//
//	std_msgs::String _walking_command_msg;
//
//	switch(row)
//	{
//	case WALKING_MODE_ROW:
//        _walking_command_msg.data = "stop";
//        walking_command_pub.publish(_walking_command_msg);
//		printf("OFF");
//		walking_param_msg.x_move_amplitude = 0;
//		walking_param_msg.y_move_amplitude = 0;
//		walking_param_msg.angle_move_amplitude = 0;
//		set_walking_param_pub.publish(walking_param_msg);
//		GoToCursor(PARAM_COL, STEP_FORWARDBACK_ROW);
//		printf("%d    ", (int)walking_param_msg.x_move_amplitude);
//		GoToCursor(PARAM_COL, STEP_RIGHTLEFT_ROW);
//		printf("%d    ", (int)walking_param_msg.y_move_amplitude);
//		GoToCursor(PARAM_COL, STEP_DIRECTION_ROW);
//		printf("%.1f    ", walking_param_msg.angle_move_amplitude);
//		break;
//
//	case X_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.init_x_offset -= 10;
//        else
//            walking_param_msg.init_x_offset -= 1;
//        printf("%d    ", (int)walking_param_msg.init_x_offset);
//		break;
//
//	case Y_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.init_y_offset -= 10;
//        else
//            walking_param_msg.init_y_offset -= 1;
//        printf("%d    ", (int)walking_param_msg.init_y_offset);
//		break;
//
//	case Z_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.init_z_offset -= 10;
//        else
//            walking_param_msg.init_z_offset -= 1;
//        printf("%d    ", (int)walking_param_msg.init_z_offset);
//		break;
//
//	case ROLL_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.init_roll_offset -= 1.0;
//        else
//            walking_param_msg.init_roll_offset -= 0.1;
//        printf("%.1f    ", walking_param_msg.init_roll_offset);
//		break;
//
//	case PITCH_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.init_pitch_offset -= 1.0;
//        else
//            walking_param_msg.init_pitch_offset -= 0.1;
//        printf("%.1f    ", walking_param_msg.init_pitch_offset);
//		break;
//
//	case YAW_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.init_yaw_offset -= 1.0;
//        else
//            walking_param_msg.init_yaw_offset -= 0.1;
//        printf("%.1f    ", walking_param_msg.init_yaw_offset);
//		break;
//
//	case HIP_PITCH_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.hip_pitch_offset -= 1.0;
//        else
//            walking_param_msg.hip_pitch_offset -= 0.1;
//        printf("%.1f    ", walking_param_msg.hip_pitch_offset);
//		break;
//
//	case AUTO_BALANCE_ROW:
//        walking_param_msg.balance_enable = true;
//        set_walking_param_pub.publish(walking_param_msg);
//		printf("OFF   ");
//		break;
//
//	case PERIOD_TIME_ROW:
//        if(large == true)
//            walking_param_msg.period_time -= 10;
//        else
//            walking_param_msg.period_time -= 1;
//        printf("%d    ", (int)walking_param_msg.period_time);
//		break;
//
//	case DSP_RATIO_ROW:
//        if(large == true)
//            walking_param_msg.dsp_ratio -= 0.1;
//        else
//            walking_param_msg.dsp_ratio -= 0.01;
//        printf("%.2f    ", walking_param_msg.dsp_ratio);
//		break;
//
//    case STEP_FORWARDBACK_RATIO_ROW:
//        if(large == true)
//            walking_param_msg.step_fb_ratio -= 0.1;
//        else
//            walking_param_msg.step_fb_ratio -= 0.01;
//        printf("%.2f    ", walking_param_msg.step_fb_ratio);
//        break;
//
//	case STEP_FORWARDBACK_ROW:
//        if(large == true)
//            walking_param_msg.x_move_amplitude -= 10;
//        else
//            walking_param_msg.x_move_amplitude -= 1;
//        printf("%d    ", (int)walking_param_msg.x_move_amplitude);
//		break;
//
//	case STEP_RIGHTLEFT_ROW:
//        if(large == true)
//            walking_param_msg.y_move_amplitude -= 10;
//        else
//            walking_param_msg.y_move_amplitude -= 1;
//        printf("%d    ", (int)walking_param_msg.y_move_amplitude);
//		break;
//
//	case STEP_DIRECTION_ROW:
//        if(large == true)
//            walking_param_msg.angle_move_amplitude -= 10;
//        else
//            walking_param_msg.angle_move_amplitude -= 1;
//        printf("%d    ", (int)walking_param_msg.angle_move_amplitude);
//		break;
//
//	case TURNING_AIM_ROW:
//        walking_param_msg.move_aim_on = true;
//		printf("OFF   ");
//		break;
//
//	case FOOT_HEIGHT_ROW:
//        if(large == true)
//            walking_param_msg.z_move_amplitude -= 10;
//        else
//            walking_param_msg.z_move_amplitude -= 1;
//        printf("%d    ", (int)walking_param_msg.z_move_amplitude);
//		break;
//
//	case SWING_RIGHTLEFT_ROW:
//        if(large == true)
//            walking_param_msg.y_swap_amplitude -= 1.0;
//        else
//            walking_param_msg.y_swap_amplitude -= 0.1;
//        printf("%.1f    ", walking_param_msg.y_swap_amplitude);
//		break;
//
//	case SWING_TOPDOWN_ROW:
//        if(large == true)
//            walking_param_msg.z_swap_amplitude -= 10;
//        else
//            walking_param_msg.z_swap_amplitude -= 1;
//        printf("%d    ", (int)walking_param_msg.z_swap_amplitude);
//		break;
//
//	case PELVIS_OFFSET_ROW:
//        if(large == true)
//            walking_param_msg.pelvis_offset -= 1.0;
//        else
//            walking_param_msg.pelvis_offset -= 0.1;
//        printf("%.1f    ", walking_param_msg.pelvis_offset);
//		break;
//
//	case ARM_SWING_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.arm_swing_gain -= 1.0;
//        else
//            walking_param_msg.arm_swing_gain -= 0.1;
//        printf("%.1f    ", walking_param_msg.arm_swing_gain);
//		break;
//
//	case BAL_KNEE_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.balance_knee_gain -= 0.1;
//        else
//            walking_param_msg.balance_knee_gain -= 0.01;
//        printf("%.2f    ", walking_param_msg.balance_knee_gain);
//		break;
//
//	case BAL_ANKLE_PITCH_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.balance_ankle_pitch_gain -= 0.1;
//        else
//            walking_param_msg.balance_ankle_pitch_gain -= 0.01;
//        printf("%.2f    ", walking_param_msg.balance_ankle_pitch_gain);
//		break;
//
//	case BAL_HIP_ROLL_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.balance_hip_roll_gain -= 0.1;
//        else
//            walking_param_msg.balance_hip_roll_gain -= 0.01;
//        printf("%.2f    ", walking_param_msg.balance_hip_roll_gain);
//		break;
//
//	case BAL_ANKLE_ROLL_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.balance_ankle_roll_gain -= 0.1;
//        else
//            walking_param_msg.balance_ankle_roll_gain -= 0.01;
//        printf("%.2f    ", walking_param_msg.balance_ankle_roll_gain);
//		break;
//
//	case P_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.p_gain -= 10;
//        else
//            walking_param_msg.p_gain -= 1;
//        printf("%d    ", walking_param_msg.p_gain);
//	    break;
//
//    case I_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.i_gain -= 10;
//        else
//            walking_param_msg.i_gain -= 1;
//        printf("%d    ", walking_param_msg.i_gain);
//        break;
//
//    case D_GAIN_ROW:
//        if(large == true)
//            walking_param_msg.d_gain -= 10;
//        else
//            walking_param_msg.d_gain -= 1;
//        printf("%d    ", walking_param_msg.d_gain);
//        break;
//	}
//
//	set_walking_param_pub.publish(walking_param_msg);
//
//	GoToCursor(col, row);
//}

void BeginCommandMode()
{
  Old_Col = Col;
  Old_Row = Row;
  ClearCmd();
  GoToCursor(CMD_COL, CMD_ROW);
  bBeginCommandMode = true;
}

void EndCommandMode()
{
  GoToCursor(Old_Col, Old_Row);
  bBeginCommandMode = false;
}

void HelpCmd()
{
  system("clear");
  printf("\n");
  printf(" exit: Exits the program\n");
  printf(" re: Refreshes the screen\n");
  printf(" save: Saves any changes made\n");
  printf(" mon: Monitoring sensor\n");
  printf("\n");
  printf("       Copyright ROBOTIS CO.,LTD.\n");
  printf("\n");
  printf(" Press any key to continue...");
  _getch();

  DrawScreen();
}

void SaveCmd()
{
  set_walking_param_pub.publish(walking_param_msg);
  std_msgs::String _walking_command_msg;
  usleep(30 * 1000);
  _walking_command_msg.data = "save";
  walking_command_pub.publish(_walking_command_msg);

  bEdited = false;
}

void MonitorCmd()
{
  int col;
  int row;
  int ch;
  double value;
  double GyroFB_min = 1000, GyroFB_max = -1000;
  double GyroRL_min = 1000, GyroRL_max = -1000;
  double AccelFB_min = 1000, AccelFB_max = -1000;
  double AccelRL_min = 1000, AccelRL_max = -1000;

  if (bBeginCommandMode == true)
  {
    col = Old_Col;
    row = Old_Row;
  }
  else
  {
    col = Col;
    row = Row;
  }

  system("clear");
  printf("\n");
  printf("Gyro F/B                                       \n");  // 0
  printf("Gyro R/L                                       \n");  // 1
  printf("Accel F/B                                      \n");  // 2
  printf("Accel R/L                                      \n");  // 3
  printf("ESC (quit), SPACE (reset)                      \n");

  set_stdin();
  while (1)
  {
    value = robotis_op::CM740Module::getInstance()->result_["gyro_y"];
    if (GyroFB_min > value)
      GyroFB_min = value;
    if (GyroFB_max < value)
      GyroFB_max = value;
    GoToCursor(PARAM_COL, X_OFFSET_ROW);
    printf("%1.3f (%1.3f~%1.3f)   ", value, GyroFB_min, GyroFB_max);

    value = robotis_op::CM740Module::getInstance()->result_["gyro_x"];
    if (GyroRL_min > value)
      GyroRL_min = value;
    if (GyroRL_max < value)
      GyroRL_max = value;
    GoToCursor(PARAM_COL, Y_OFFSET_ROW);
    printf("%1.3f (%1.3f~%1.3f)   ", value, GyroRL_min, GyroRL_max);

    value = robotis_op::CM740Module::getInstance()->result_["acc_y"];
    if (AccelFB_min > value)
      AccelFB_min = value;
    if (AccelFB_max < value)
      AccelFB_max = value;
    GoToCursor(PARAM_COL, Z_OFFSET_ROW);
    printf("%1.3f (%1.3f~%1.3f)   ", value, AccelFB_min, AccelFB_max);

    value = robotis_op::CM740Module::getInstance()->result_["acc_x"];
    if (AccelRL_min > value)
      AccelRL_min = value;
    if (AccelRL_max < value)
      AccelRL_max = value;
    GoToCursor(PARAM_COL, ROLL_OFFSET_ROW);
    printf("%1.3f (%1.3f~%1.3f)   ", value, AccelRL_min, AccelRL_max);

    if (kbhit())
    {
      ch = _getch();
      if (ch == 0x1b)  // ESC
        break;
      else if (ch == 0x20)  // Space
      {
        GyroFB_min = 1000;
        GyroFB_max = -1000;
        GyroRL_min = 1000;
        GyroRL_max = -1000;
        AccelFB_min = 1000;
        AccelFB_max = -1000;
        AccelRL_min = 1000;
        AccelRL_max = -1000;
      }
    }

    usleep(50000);
  }
  reset_stdin();
  GoToCursor(col, row);
  DrawScreen();
}
