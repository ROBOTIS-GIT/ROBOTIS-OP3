/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Author: SCH */

// include
#include "op3_localization/op3_localization.h"
#include "rclcpp/rclcpp.hpp"

//node main
int main(int argc, char **argv)
{
  //init ros
  rclcpp::init(argc, argv);

  //create ros wrapper object
  auto node = std::make_shared<robotis_op::OP3Localization>();

  //set node loop rate
  rclcpp::Rate loop_rate(10);

  //node loop
  while ( rclcpp::ok() )
  {
    node->process();

    //execute pending callbacks
    rclcpp::spin_some(node);

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  rclcpp::shutdown();
  return 0;
}
