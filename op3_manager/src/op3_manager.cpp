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

/* Author: Kayman */

/* ROS API Header */
#include <ros/ros.h>
#include <std_msgs/String.h>

/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"

/* Sensor Module Header */
#include "open_cr_module/open_cr_module.h"

/* Motion Module Header */
#include "op3_base_module/base_module.h"
#include "op3_head_control_module/head_control_module.h"
#include "op3_action_module/action_module.h"
#include "op3_walking_module/op3_walking_module.h"
#include "op3_direct_control_module/direct_control_module.h"
#include "op3_online_walking_module/online_walking_module.h"
#include "op3_tuning_module/tuning_module.h"

using namespace robotis_framework;
using namespace dynamixel;
using namespace robotis_op;

const int BAUD_RATE = 2000000;
const double PROTOCOL_VERSION = 2.0;
const int SUB_CONTROLLER_ID = 200;
const int DXL_BROADCAST_ID = 254;
const int DEFAULT_DXL_ID = 1;
const std::string SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
const int POWER_CTRL_TABLE = 24;
const int RGB_LED_CTRL_TABLE = 26;
const int TORQUE_ON_CTRL_TABLE = 64;

bool g_is_simulation = false;
int g_baudrate;
std::string g_offset_file;
std::string g_robot_file;
std::string g_init_file;
std::string g_device_name;

ros::Publisher g_init_pose_pub;
ros::Publisher g_demo_command_pub;

void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "user_long")
  {
    RobotisController *controller = RobotisController::getInstance();

    controller->setCtrlModule("none");

    controller->stopTimer();

    if (g_is_simulation == false)
    {
      // power and torque on
      PortHandler *port_handler = (PortHandler *) PortHandler::getPortHandler(g_device_name.c_str());
      bool set_port_result = port_handler->setBaudRate(g_baudrate);
      if (set_port_result == false)
      {
        ROS_ERROR("Error Set port");
        return;
      }
      PacketHandler *packet_handler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

      // check dxls torque.
      uint8_t torque = 0;
      packet_handler->read1ByteTxRx(port_handler, DEFAULT_DXL_ID, TORQUE_ON_CTRL_TABLE, &torque);

      if (torque != 1)
      {
        controller->initializeDevice(g_init_file);
      }
      else
      {
        ROS_INFO("Torque is already on!!");
      }
    }

    controller->startTimer();

    usleep(200 * 1000);

    // go to init pose
    std_msgs::String init_msg;
    init_msg.data = "ini_pose";

    g_init_pose_pub.publish(init_msg);
    ROS_INFO("Go to init pose");
  }
}

void dxlTorqueCheckCallback(const std_msgs::String::ConstPtr& msg)
{
  if (g_is_simulation == true)
    return;

  // check dxl torque
  uint8_t torque_result = 0;
  bool torque_on = true;
  RobotisController *controller = RobotisController::getInstance();
  //controller->robot_->port_default_device_

  for (std::map<std::string, std::string>::iterator map_it = controller->robot_->port_default_device_.begin();
       map_it != controller->robot_->port_default_device_.end(); map_it++)
  {
    std::string default_device_name = map_it->second;
    controller->read1Byte(default_device_name, TORQUE_ON_CTRL_TABLE, &torque_result);

    // if not, torque on
    if (torque_result != 1)
      torque_on = false;
  }

  if(torque_on == false)
  {
    controller->stopTimer();

    controller->initializeDevice(g_init_file);

    controller->startTimer();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op3_manager");
  ros::NodeHandle nh;

  ROS_INFO("manager->init");
  RobotisController *controller = RobotisController::getInstance();

  /* Load ROS Parameter */

  nh.param<std::string>("offset_file_path", g_offset_file, "");
  nh.param<std::string>("robot_file_path", g_robot_file, "");
  nh.param<std::string>("init_file_path", g_init_file, "");
  nh.param<std::string>("device_name", g_device_name, SUB_CONTROLLER_DEVICE);
  nh.param<int>("baud_rate", g_baudrate, BAUD_RATE);

  ros::Subscriber button_sub = nh.subscribe("/robotis/open_cr/button", 1, buttonHandlerCallback);
  ros::Subscriber dxl_torque_sub = nh.subscribe("/robotis/dxl_torque", 1, dxlTorqueCheckCallback);
  g_init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  g_demo_command_pub = nh.advertise<std_msgs::String>("/ball_tracker/command", 0);

  nh.param<bool>("gazebo", controller->gazebo_mode_, false);
  g_is_simulation = controller->gazebo_mode_;

  /* real robot */
  if (g_is_simulation == false)
  {
    // open port
    PortHandler *port_handler = (PortHandler *) PortHandler::getPortHandler(g_device_name.c_str());
    bool set_port_result = port_handler->setBaudRate(BAUD_RATE);
    if (set_port_result == false)
      ROS_ERROR("Error Set port");

    PacketHandler *packet_handler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // power on dxls
    int torque_on_count = 0;

    while (torque_on_count < 5)
    {
      int _return = packet_handler->write1ByteTxRx(port_handler, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);

      if(_return != 0)
        ROS_ERROR("Torque on DXLs! [%s]", packet_handler->getRxPacketError(_return));
      else
        ROS_INFO("Torque on DXLs!");

      if (_return == 0)
        break;
      else
        torque_on_count++;
    }

    usleep(100 * 1000);

    // set RGB-LED to GREEN
    int led_full_unit = 0x1F;
    int led_range = 5;
    int led_value = led_full_unit << led_range;
    int _return = packet_handler->write2ByteTxRx(port_handler, SUB_CONTROLLER_ID, RGB_LED_CTRL_TABLE, led_value);

    if(_return != 0)
      ROS_ERROR("Fail to control LED [%s]", packet_handler->getRxPacketError(_return));

    port_handler->closePort();
  }
  /* gazebo simulation */
  else
  {
    ROS_WARN("SET TO GAZEBO MODE!");
    std::string robot_name;
    nh.param<std::string>("gazebo_robot_name", robot_name, "");
    if (robot_name != "")
      controller->gazebo_robot_name_ = robot_name;
  }

  if (g_robot_file == "")
  {
    ROS_ERROR("NO robot file path in the ROS parameters.");
    return -1;
  }

  // initialize robot
  if (controller->initialize(g_robot_file, g_init_file) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return -1;
  }

  // load offset
  if (g_offset_file != "")
    controller->loadOffset(g_offset_file);

  usleep(300 * 1000);

  /* Add Sensor Module */
  controller->addSensorModule((SensorModule*) OpenCRModule::getInstance());

  /* Add Motion Module */
  controller->addMotionModule((MotionModule*) ActionModule::getInstance());
  controller->addMotionModule((MotionModule*) BaseModule::getInstance());
  controller->addMotionModule((MotionModule*) HeadControlModule::getInstance());
  controller->addMotionModule((MotionModule*) WalkingModule::getInstance());
  controller->addMotionModule((MotionModule*) DirectControlModule::getInstance());
  controller->addMotionModule((MotionModule*) OnlineWalkingModule::getInstance());
  controller->addMotionModule((MotionModule*) TuningModule::getInstance());

  // start timer
  controller->startTimer();

  usleep(100 * 1000);

  // go to init pose
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  g_init_pose_pub.publish(init_msg);
  ROS_INFO("Go to init pose");

  while (ros::ok())
  {
    usleep(1 * 1000);

    ros::spin();
  }

  return 0;
}
