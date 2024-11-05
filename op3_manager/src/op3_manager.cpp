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

/* ROS2 API Header */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

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

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_init_pose_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_demo_command_pub;

void buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg)
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
        RCLCPP_ERROR(controller->get_logger(), "Error Set port");
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
        RCLCPP_INFO(controller->get_logger(), "Torque is already on!!");
      }
    }

    controller->startTimer();

    usleep(200 * 1000);

    // go to init pose
    std_msgs::msg::String init_msg;
    init_msg.data = "ini_pose";

    g_init_pose_pub->publish(init_msg);
    RCLCPP_INFO(controller->get_logger(), "Go to init pose");
  }
}

void dxlTorqueCheckCallback(const std_msgs::msg::String::SharedPtr msg)
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
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("op3_manager");

  RCLCPP_INFO(node->get_logger(), "manager->init");
  RobotisController *controller = RobotisController::getInstance();

  /* Load ROS Parameter */

  node->declare_parameter<std::string>("offset_file_path", "");
  node->declare_parameter<std::string>("robot_file_path", "");
  node->declare_parameter<std::string>("init_file_path", "");
  node->declare_parameter<std::string>("device_name", SUB_CONTROLLER_DEVICE);
  node->declare_parameter<int>("baud_rate", BAUD_RATE);

  node->get_parameter("offset_file_path", g_offset_file);
  node->get_parameter("robot_file_path", g_robot_file);
  node->get_parameter("init_file_path", g_init_file);
  node->get_parameter("device_name", g_device_name);
  node->get_parameter("baud_rate", g_baudrate);

  auto button_sub = node->create_subscription<std_msgs::msg::String>("/robotis/open_cr/button", 1, buttonHandlerCallback);
  auto dxl_torque_sub = node->create_subscription<std_msgs::msg::String>("/robotis/dxl_torque", 1, dxlTorqueCheckCallback);
  g_init_pose_pub = node->create_publisher<std_msgs::msg::String>("/robotis/base/ini_pose", 10);
  g_demo_command_pub = node->create_publisher<std_msgs::msg::String>("/ball_tracker/command", 10);

  node->declare_parameter<bool>("gazebo", false);
  node->get_parameter("gazebo", controller->gazebo_mode_);
  g_is_simulation = controller->gazebo_mode_;

  /* real robot */
  if (g_is_simulation == false)
  {
    // open port
    PortHandler *port_handler = (PortHandler *) PortHandler::getPortHandler(g_device_name.c_str());
    bool set_port_result = port_handler->setBaudRate(BAUD_RATE);
    if (set_port_result == false)
      RCLCPP_ERROR(node->get_logger(), "Error Set port");

    PacketHandler *packet_handler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // power on dxls
    int torque_on_count = 0;

    while (torque_on_count < 5)
    {
      int _return = packet_handler->write1ByteTxRx(port_handler, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);

      if(_return != 0)
        RCLCPP_ERROR(node->get_logger(), "Torque on DXLs! [%s]", packet_handler->getRxPacketError(_return));
      else
        RCLCPP_INFO(node->get_logger(), "Torque on DXLs!");

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
      RCLCPP_ERROR(node->get_logger(), "Fail to control LED [%s]", packet_handler->getRxPacketError(_return));

    port_handler->closePort();
  }
  /* gazebo simulation */
  else
  {
    RCLCPP_WARN(node->get_logger(), "SET TO GAZEBO MODE!");
    std::string robot_name;
    node->declare_parameter<std::string>("gazebo_robot_name", "");
    node->get_parameter("gazebo_robot_name", robot_name);
    if (robot_name != "")
      controller->gazebo_robot_name_ = robot_name;
  }

  if (g_robot_file == "")
  {
    RCLCPP_ERROR(node->get_logger(), "NO robot file path in the ROS parameters.");
    return -1;
  }

  // initialize robot
  if (controller->initialize(g_robot_file, g_init_file) == false)
  {
    RCLCPP_ERROR(node->get_logger(), "ROBOTIS Controller Initialize Fail!");
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
  std_msgs::msg::String init_msg;
  init_msg.data = "ini_pose";

  g_init_pose_pub->publish(init_msg);
  RCLCPP_INFO(node->get_logger(), "Go to init pose");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
