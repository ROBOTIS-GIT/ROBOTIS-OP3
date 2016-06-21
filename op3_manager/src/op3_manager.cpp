/*
 * RobotisManager.cpp
 *
 *  Created on: 2016. 1. 21.
 *      Author: zerom
 */

#include "std_msgs/String.h"

#include "robotis_controller/RobotisController.h"

/* Sensor Module Header */
#include "cm_740_module/cm_740_module.h"

/* Motion Module Header */
#include "op3_base_module/base_module.h"
#include "op3_head_control_module/head_control_module.h"
#include "op3_action_module/action_module.h"
#include "op3_walking_module/op3_walking_module.h"

using namespace ROBOTIS;

std::string         _offset_file;
std::string         _robot_file;
std::string         _init_file;

ros::Publisher init_pose_pub;
ros::Publisher demo_command_pub;

void buttonHandlerCallback( const std_msgs::String::ConstPtr& msg )
{
  if(msg->data == "mode")
  {
    RobotisController  *_controller     = RobotisController::GetInstance();

    _controller->SetCtrlModule("none");

    _controller->StopTimer();

    // power on
    PortHandler *_port_h = (PortHandler *)PortHandler::GetPortHandler("/dev/ttyUSB0");
    bool _set_port = _port_h->SetBaudRate(1000000);
    if(_set_port == false)
    {
      ROS_ERROR("Error Set port");
      return;
    }
    PacketHandler *_packet_h = PacketHandler::GetPacketHandler(1.0);

    // check dxls torque.
    UINT8_T torque = 0;
    _packet_h->Read1ByteTxRx(_port_h, 200, 24, &torque);

    if(torque != 1)
    {
      int _return = _packet_h->Write1ByteTxRx(_port_h, 200, 24, 1);
      ROS_INFO("Power on DXLs! [%d]", _return);

      // _port_h->ClosePort();
      usleep(100 * 1000);

      PortHandler *_port_h2 = (PortHandler *)PortHandler::GetPortHandler("/dev/ttyUSB1");
      _set_port = _port_h2->SetBaudRate(3000000);
      if(_set_port == false)
      {
        ROS_ERROR("Error Set port");
        return;
      }
      PacketHandler *_packet_h2 = PacketHandler::GetPacketHandler(2.0);

      _return = _packet_h2->Write1ByteTxRx(_port_h2, 254, 64, 1);
      ROS_INFO("Torque on DXLs! [%d]", _return);

      // _port_h2->ClosePort();
      usleep(100 * 1000);

      _controller->InitDevice(_init_file);
    }
    else
    {
      ROS_INFO("Torque is already on!!");
    }

    _controller->StartTimer();

    usleep(100 * 1000);

    // go to init pose
    std_msgs::String init_msg;
    init_msg.data = "ini_pose";

    init_pose_pub.publish(init_msg);
    ROS_INFO("Go to init pose");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "OP2_Manager");
  ros::NodeHandle _nh;

  ROS_INFO("manager->init");
  RobotisController  *_controller     = RobotisController::GetInstance();

  /* Load ROS Parameter */

  _nh.param<std::string>("offset_table", _offset_file, "");
  _nh.param<std::string>("robot_file_path", _robot_file, "");
  _nh.param<std::string>("init_file_path", _init_file, "");

  ros::Subscriber power_on_sub = _nh.subscribe("/robotis/cm_740/button", 1, buttonHandlerCallback);
  init_pose_pub = _nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  demo_command_pub = _nh.advertise<std_msgs::String>("/ball_tracker/command", 0);

  PortHandler *_port_h = (PortHandler *)PortHandler::GetPortHandler("/dev/ttyUSB0");
  bool _set_port = _port_h->SetBaudRate(1000000);
  if(_set_port == false) ROS_ERROR("Error Set port");
  PacketHandler *_packet_h = PacketHandler::GetPacketHandler(1.0);

  int _return = _packet_h->Write1ByteTxRx(_port_h, 200, 24, 1);
  ROS_INFO("Torque on DXLs! [%d]", _return);
  _packet_h->PrintTxRxResult(_return);


  _port_h->ClosePort();

  usleep(100 * 1000);

  /* gazebo simulation */
  _nh.param<bool>("gazebo", _controller->gazebo_mode, false);
  if(_controller->gazebo_mode == true)
  {
    ROS_WARN("SET TO GAZEBO MODE!");
    std::string         _robot_name;
    _nh.param<std::string>("gazebo_robot_name", _robot_name, "");
    if(_robot_name != "")
      _controller->gazebo_robot_name  = _robot_name;
  }

  if(_robot_file == "")
  {
    ROS_ERROR("NO robot file path in the ROS parameters.");
    return -1;
  }

  if(_controller->Initialize(_robot_file, _init_file) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return -1;
  }

  if(_offset_file != "")
    _controller->LoadOffset(_offset_file);

  sleep(1);

  /* Add Sensor Module */
  _controller->AddSensorModule((SensorModule*)CM740Module::GetInstance());

  /* Add Motion Module */
  _controller->AddMotionModule((MotionModule*)ActionModule::GetInstance());
  _controller->AddMotionModule((MotionModule*)BaseModule::GetInstance());
  _controller->AddMotionModule((MotionModule*)HeadControlModule::GetInstance());
  _controller->AddMotionModule((MotionModule*)WalkingMotionModule::GetInstance());

  _controller->StartTimer();

  // go to init pose
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub.publish(init_msg);
  ROS_INFO("Go to init pose");

  while(ros::ok())
  {
    usleep(1 * 1000);

    ros::spin();
  }

  return 0;
}
