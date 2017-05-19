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
 * main.cpp
 *
 *  Created on: 2016. 12. 16.
 *      Author: JaySong
 */

#include "op3_action_editor/action_editor.h"

const int BAUD_RATE = 2000000;
const double PROTOCOL_VERSION = 2.0;
const int SUB_CONTROLLER_ID = 200;
const std::string SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
const int POWER_CTRL_TABLE = 24;

void sighandler(int sig)
{
  struct termios term;
  tcgetattr( STDIN_FILENO, &term);
  term.c_lflag |= ICANON | ECHO;
  tcsetattr( STDIN_FILENO, TCSANOW, &term);

  exit(0);
}

bool turnOnDynamixelPower(const std::string &device_name, const int &baud_rate)
{
  // power on
  dynamixel::PortHandler *_port_h = (dynamixel::PortHandler *) dynamixel::PortHandler::getPortHandler(device_name.c_str());
  bool _set_port = _port_h->setBaudRate(baud_rate);
  if (_set_port == false)
  {
    ROS_ERROR("Error Set port");
    return false;
  }
  dynamixel::PacketHandler *_packet_h = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int _return = _packet_h->write1ByteTxRx(_port_h, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);

  if(_return != COMM_SUCCESS)
  {
    ROS_ERROR("Failed to turn on the Power of DXLs!");
    return false;
  }
  else
  {
    ROS_INFO("Power on DXLs!");
  }


  usleep(100 * 1000);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "THORMANG3 Action Editor");
  ros::NodeHandle nh;

  std::string offset_file   = nh.param<std::string>("offset_table", "");
  std::string robot_file    = nh.param<std::string>("robot_file_path", "");
  std::string dxl_init_file = nh.param<std::string>("init_file_path", "");
  std::string _device_name = nh.param<std::string>("device_name", SUB_CONTROLLER_DEVICE);
  int _baud_rate = nh.param<int>("baudrate", BAUD_RATE);

  signal(SIGABRT, &sighandler);
  signal(SIGTERM, &sighandler);
  signal(SIGQUIT, &sighandler);
  signal(SIGINT, &sighandler);

  int ch;

  if(turnOnDynamixelPower(_device_name, _baud_rate) == false)
    return 0;

  robotis_op::ActionEditor editor;
  if (editor.initializeActionEditor(robot_file, dxl_init_file, offset_file) == false)
  {
    ROS_ERROR("Failed to Initialize");
    return 0;
  }

  editor.drawIntro();

  while (1)
   {
     ch = editor._getch();

     if (ch == 0x1b)
     {
       ch = editor._getch();
       if (ch == 0x5b)
       {
         ch = editor._getch();
         if (ch == 0x41)      // Up arrow key
           editor.moveUpCursor();
         else if (ch == 0x42)  // Down arrow key
           editor.moveDownCursor();
         else if (ch == 0x44)  // Left arrow key
           editor.moveLeftCursor();
         else if (ch == 0x43)  // Right arrow key
           editor.moveRightCursor();
       }
     }
     else if (ch == '[')
       editor.setValueUpDown(-1);
     else if (ch == ']')
       editor.setValueUpDown(1);
     else if (ch == '{')
       editor.setValueUpDown(-10);
     else if (ch == '}')
       editor.setValueUpDown(10);
     else if (ch == ' ')
       editor.toggleTorque();
     else if (ch == ',')
       editor.storeValueToCache();
     else if (ch == '.')
       editor.setValueFromCache();
     else if (ch == '/')
       editor.clearCache();
     else if (ch >= 'A' && ch <= 'z')
     {
       char input[128] = { 0, };
       char *token;
       int input_len;
       char cmd[80];
       int num_param;
       int iparam[30];

       int idx = 0;

       editor.beginCommandMode();

       printf("%c", ch);
       input[idx++] = (char) ch;

       while (1)
       {
         ch = editor._getch();
         if (ch == 0x0A)
           break;
         else if (ch == 0x7F)
         {
           if (idx > 0)
           {
             ch = 0x08;
             printf("%c", ch);
             ch = ' ';
             printf("%c", ch);
             ch = 0x08;
             printf("%c", ch);
             input[--idx] = 0;
           }
         }
         else if ((ch >= 'A' && ch <= 'z') || ch == ' ' || (ch >= '0' && ch <= '9'))
         {
           if (idx < 127)
           {
             printf("%c", ch);
             input[idx++] = (char) ch;
           }
         }
       }

       fflush(stdin);
       input_len = strlen(input);
       if (input_len > 0)
       {
         token = strtok(input, " ");
         if (token != 0)
         {
           strcpy(cmd, token);
           token = strtok(0, " ");
           num_param = 0;
           while (token != 0)
           {
             iparam[num_param++] = atoi(token);
             token = strtok(0, " ");
           }

           if (strcmp(cmd, "exit") == 0)
           {
             if (editor.askSave() == false)
               break;
           }
           else if (strcmp(cmd, "re") == 0)
             editor.drawPage();
           else if (strcmp(cmd, "help") == 0)
             editor.helpCmd();
           else if (strcmp(cmd, "n") == 0)
             editor.nextCmd();
           else if (strcmp(cmd, "b") == 0)
             editor.prevCmd();
           else if (strcmp(cmd, "time") == 0)
             editor.timeCmd();
           else if (strcmp(cmd, "speed") == 0)
             editor.speedCmd();
           else if (strcmp(cmd, "page") == 0)
           {
             if (num_param > 0)
               editor.pageCmd(iparam[0]);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "play") == 0)
           {
             editor.playCmd();
           }
           else if (strcmp(cmd, "playboth") == 0)
           {
             if (num_param > 0)
               editor.playCmd(iparam[0]);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "set") == 0)
           {
             if (num_param > 0)
               editor.setValue(iparam[0]);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "list") == 0)
             editor.listCmd();
           else if (strcmp(cmd, "on") == 0)
             editor.turnOnOffCmd(true, num_param, iparam);
           else if (strcmp(cmd, "off") == 0)
             editor.turnOnOffCmd(false, num_param, iparam);
           else if (strcmp(cmd, "mrl") == 0)
           {
             if (num_param > 0)
               editor.mirrorStepCmd(iparam[0], robotis_op::ActionEditor::RightToLeft,
                                    robotis_op::ActionEditor::AllBody);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "murl") == 0)
           {
             if (num_param > 0)
               editor.mirrorStepCmd(iparam[0], robotis_op::ActionEditor::RightToLeft,
                                    robotis_op::ActionEditor::UpperBody);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "mlrl") == 0)
           {
             if (num_param > 0)
               editor.mirrorStepCmd(iparam[0], robotis_op::ActionEditor::RightToLeft,
                                    robotis_op::ActionEditor::LowerBody);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "mlr") == 0)
           {
             if (num_param > 0)
               editor.mirrorStepCmd(iparam[0], robotis_op::ActionEditor::LeftToRight,
                                    robotis_op::ActionEditor::AllBody);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "mulr") == 0)
           {
             if (num_param > 0)
               editor.mirrorStepCmd(iparam[0], robotis_op::ActionEditor::LeftToRight,
                                    robotis_op::ActionEditor::UpperBody);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "mllr") == 0)
           {
             if (num_param > 0)
               editor.mirrorStepCmd(iparam[0], robotis_op::ActionEditor::LeftToRight,
                                    robotis_op::ActionEditor::LowerBody);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "ms") == 0)
           {
             if (num_param > 0)
               editor.mirrorStepCmd(iparam[0], robotis_op::ActionEditor::SwitchEach,
                                    robotis_op::ActionEditor::AllBody);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "w") == 0)
           {
             if (num_param > 0)
               editor.writeStepCmd(iparam[0]);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "d") == 0)
           {
             if (num_param > 0)
               editor.deleteStepCmd(iparam[0]);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "i") == 0)
           {
             if (num_param == 0)
               editor.insertStepCmd(0);
             else
               editor.insertStepCmd(iparam[0]);
           }
           else if (strcmp(cmd, "int") == 0)
           {
             if (num_param == 2)
               editor.insertInterpolationStepCmd(iparam[0], iparam[1]);
             else
               editor.printCmd("Need 2 parameters");
           }
           else if (strcmp(cmd, "copy") == 0)
           {
             if (num_param > 0)
               editor.copyCmd(iparam[0]);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "new") == 0)
             editor.newCmd();
           else if (strcmp(cmd, "g") == 0)
           {
             if (num_param > 0)
               editor.goCmd(iparam[0]);
             else
               editor.printCmd("Need parameter");
           }
           else if (strcmp(cmd, "save") == 0)
             editor.saveCmd();
           else if (strcmp(cmd, "name") == 0)
             editor.nameCmd();
           else
             editor.printCmd("Bad command! please input 'help'");
         }
       }

       editor.endCommandMode();
     }
   }

   editor.drawEnding();

   return 0;
 }
