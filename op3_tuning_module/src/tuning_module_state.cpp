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

/* Author: Kayman, SCH */

#include "op3_tuning_module/tuning_module_state.h"

namespace robotis_op
{
TuningModuleState::TuningModuleState(int via_num)
{
  is_moving_ = false;
  is_generating_ = false;

  cnt_ = 0;

  mov_time_ = 1.0;
  smp_time_ = 0.008;
  all_time_steps_ = int(mov_time_ / smp_time_) + 1;

  calc_joint_tra_ = Eigen::MatrixXd::Zero(all_time_steps_, MAX_JOINT_ID + 1);

  joint_ini_pose_ = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);
  joint_pose_ = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);

  via_num_ = via_num;

  joint_via_pose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
  joint_via_dpose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
  joint_via_ddpose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);

  via_time_ = Eigen::MatrixXd::Zero(via_num_, 1);
}

TuningModuleState::~TuningModuleState()
{
}

}
