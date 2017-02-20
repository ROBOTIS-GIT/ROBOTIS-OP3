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

/* Author: sch, Jay Song */

#include "op3_kinematics_dynamics/link_data.h"

namespace robotis_op
{

LinkData::LinkData()
{
  name_ = "";

  parent_ = -1;
  sibling_ = -1;
  child_ = -1;

  mass_ = 0.0;

  relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  joint_limit_max_ = 100.0;
  joint_limit_min_ = -100.0;

  joint_angle_ = 0.0;
  joint_velocity_ = 0.0;
  joint_acceleration_ = 0.0;

  position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
  transformation_ = robotis_framework::getTransformationXYZRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

LinkData::~LinkData()
{
}

}
