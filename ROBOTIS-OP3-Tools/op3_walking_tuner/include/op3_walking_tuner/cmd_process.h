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

#ifndef _DXL_MANAGER_CMD_PROCESS_H_
#define _DXL_MANAGER_CMD_PROCESS_H_

#include <string>
#include <ros/ros.h>

#include "robotis_controller/robotis_controller.h"

#include "cm_740_module/cm_740_module.h"
#include "op3_base_module/base_module.h"
#include "op3_walking_module/op3_walking_module.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"

#define PROGRAM_VERSION	"v1.00"
#define SCREEN_COL		35

// Position of Column
#define CMD_COL			2
#define PARAM_COL		27

// Position of Row
enum
{
  WALKING_MODE_ROW,
  X_OFFSET_ROW,
  Y_OFFSET_ROW,
  Z_OFFSET_ROW,
  ROLL_OFFSET_ROW,
  PITCH_OFFSET_ROW,
  YAW_OFFSET_ROW,
  HIP_PITCH_OFFSET_ROW,
  AUTO_BALANCE_ROW,
  PERIOD_TIME_ROW,
  DSP_RATIO_ROW,
  STEP_FORWARDBACK_RATIO_ROW,
  STEP_FORWARDBACK_ROW,
  STEP_RIGHTLEFT_ROW,
  STEP_DIRECTION_ROW,
  TURNING_AIM_ROW,
  FOOT_HEIGHT_ROW,
  SWING_RIGHTLEFT_ROW,
  SWING_TOPDOWN_ROW,
  PELVIS_OFFSET_ROW,
  ARM_SWING_GAIN_ROW,
  BAL_KNEE_GAIN_ROW,
  BAL_ANKLE_PITCH_GAIN_ROW,
  BAL_HIP_ROLL_GAIN_ROW,
  BAL_ANKLE_ROLL_GAIN_ROW,
  P_GAIN_ROW,
  I_GAIN_ROW,
  D_GAIN_ROW,
  CMD_ROW,
  SCREEN_ROW
};

int _getch();
bool AskSave();

bool InitializeWalkingTuner(std::string robot_file_path, std::string init_file_path, std::string offset_file_path);

// Move cursor
void GoToCursor(int col, int row);
void MoveUpCursor();
void MoveDownCursor();

// Disp & Drawing
bool DrawIntro();
void DrawEnding();
void DrawScreen();
void ClearCmd();
void PrintCmd(std::string message);

// Edit value
void UpdateValue(bool large, double dir);
//void IncreaseValue(bool large);
//void DecreaseValue(bool large);

// Command process
void BeginCommandMode();
void EndCommandMode();
void HelpCmd();
void SaveCmd();
void MonitorCmd();

#endif
