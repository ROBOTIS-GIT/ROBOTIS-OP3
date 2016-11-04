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
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include <signal.h>
#include <libgen.h>

#include <ros/ros.h>

#include "op3_walking_tuner/cmd_process.h"

//#define INI_FILE_PATH       "../../../Data/config.ini"
//
//using namespace Robot;
//
//LinuxCM730 linux_cm730("/dev/ttyUSB0");
//CM730 cm730(&linux_cm730);

void sighandler(int sig)
{
  struct termios term;
  tcgetattr( STDIN_FILENO, &term);
  term.c_lflag |= ICANON | ECHO;
  tcsetattr( STDIN_FILENO, TCSANOW, &term);

  exit(0);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "OP3 Walking Tuner");
  ros::NodeHandle _nh;

  std::string _offset_file = _nh.param<std::string>("offset_table", "");
  std::string _robot_file = _nh.param<std::string>("robot_file_path", "");

  std::string _init_file = _nh.param<std::string>("init_file_path", "");

  signal(SIGABRT, &sighandler);
  signal(SIGTERM, &sighandler);
  signal(SIGQUIT, &sighandler);
  signal(SIGINT, &sighandler);

  if (InitializeWalkingTuner(_robot_file, _init_file, _offset_file) == false)
  {
    ROS_ERROR("Failed to Initialize");
    return 0;
  }

  if (DrawIntro() == false)
  {
    ROS_ERROR("Failed to Get Current Walking Param");
    return 0;
  }

  while (1)
  {
    int ch = _getch();
    if (ch == 0x1b)
    {
      ch = _getch();
      if (ch == 0x5b)
      {
        ch = _getch();
        if (ch == 0x41)  // Up arrow key
          MoveUpCursor();
        else if (ch == 0x42)  // Down arrow key
          MoveDownCursor();
      }
    }
    else if (ch == '[')
      UpdateValue(false, -1.0);
    else if (ch == ']')
      UpdateValue(false, 1.0);
    else if (ch == '{')
      UpdateValue(true, -1.0);
    else if (ch == '}')
      UpdateValue(true, 1.0);
    else if (ch >= 'A' && ch <= 'z')
    {
      char input[128] = { 0, };
      char *token;
      int input_len;
      char cmd[80];
      char strParam[20][30];
      int num_param;

      int idx = 0;

      BeginCommandMode();

      printf("%c", ch);
      input[idx++] = (char) ch;

      while (1)
      {
        ch = _getch();
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
        else if (ch >= 'A' && ch <= 'z')
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
            strcpy(strParam[num_param++], token);
            token = strtok(0, " ");
          }

          if (strcmp(cmd, "exit") == 0)
          {
            if (AskSave() == false)
              break;
          }
          if (strcmp(cmd, "re") == 0)
            DrawScreen();
          else if (strcmp(cmd, "save") == 0)
          {
            SaveCmd();
          }
          else if (strcmp(cmd, "mon") == 0)
          {
            MonitorCmd();
          }
          else if (strcmp(cmd, "help") == 0)
            HelpCmd();
          else
            PrintCmd("Bad command! please input 'help'");
        }
      }

      EndCommandMode();
    }
  }

  DrawEnding();
  return 0;
}
