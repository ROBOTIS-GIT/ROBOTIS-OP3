#ifndef _DXL_MANAGER_CMD_PROCESS_H_
#define _DXL_MANAGER_CMD_PROCESS_H_

#include <string>
#include <ros/ros.h>

#include "robotis_controller/RobotisController.h"

#include "cm_740_module/cm_740_module.h"
#include "op3_base_module/BaseModule.h"
#include "op3_walking_module/op3_walking_module.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"


#define PROGRAM_VERSION	"v1.00"
#define SCREEN_COL		35

// Position of Column
#define CMD_COL			2
#define PARAM_COL		27

// Position of Row
enum {
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
