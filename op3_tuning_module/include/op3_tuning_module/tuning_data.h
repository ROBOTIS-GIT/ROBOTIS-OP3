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

/* Authors: Kayman */

#ifndef TuningData_H_
#define TuningData_H_

#include <string>

namespace robotis_op
{

template<typename T>
class JointElement
{
public:
  void getValue(T &value)
  {
    if(has_value_)
      value = joint_value_;
  }
  void setValue(const T &value)
  {
    joint_value_ = value;
    has_value_ = true;
  }
  void clear()
  {
    has_value_ = false;
  }

private:
  T joint_value_;
  bool has_value_;
};

class TuningData
{
public:
  TuningData();
  ~TuningData();
  void clearData();

  JointElement<std::string> joint_name_;

  JointElement<double> position_;
  JointElement<double> velocity_;
  JointElement<double> effort_;

  JointElement<int> p_gain_;
  JointElement<int> i_gain_;
  JointElement<int> d_gain_;

};

class JointOffsetData
{
 public:
  double joint_offset_rad_;
  double goal_position_;
  int p_gain_;
  int i_gain_;
  int d_gain_;

  JointOffsetData()
  {
    joint_offset_rad_ = 0;
    goal_position_ = 0;
    p_gain_ = 800;
    i_gain_ = 0;
    d_gain_ = 0;
  }

  JointOffsetData(double joint_offset_rad, double goal_position)
  {
    this->joint_offset_rad_ = joint_offset_rad;
    this->goal_position_ = goal_position;
    p_gain_ = 800;
    i_gain_ = 0;
    d_gain_ = 0;
  }

  JointOffsetData(double joint_offset_rad, double goal_position, int p_gain, int i_gain, int d_gain)
  {
    this->joint_offset_rad_ = joint_offset_rad;
    this->goal_position_ = goal_position;
    p_gain_ = p_gain;
    i_gain_ = i_gain;
    d_gain_ = d_gain;
  }

  ~JointOffsetData()
  {
  }
};
}

#endif /* TuningData_H_ */
