/*
 * Link.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: sch
 */

#include "op2_base_module/RobotisState.h"

namespace ROBOTIS_BASE
{

RobotisState::RobotisState()
{
    is_moving = false;

    cnt = 0;

    mov_time = 1.0;
    smp_time = 0.008;
    all_time_steps = int( mov_time / smp_time ) + 1;

    calc_joint_tra = Eigen::MatrixXd::Zero( all_time_steps , MAX_JOINT_ID + 1 );

    joint_ini_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1 , 1 );
    joint_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1 , 1 );

    via_num = 1;

    joint_via_pose = Eigen::MatrixXd::Zero( via_num , MAX_JOINT_ID + 1 );
    joint_via_dpose = Eigen::MatrixXd::Zero( via_num , MAX_JOINT_ID + 1 );
    joint_via_ddpose = Eigen::MatrixXd::Zero( via_num , MAX_JOINT_ID + 1 );

    via_time = Eigen::MatrixXd::Zero( via_num , 1 );
}

RobotisState::~RobotisState(){}

}
