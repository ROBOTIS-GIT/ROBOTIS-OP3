/*
 * RobotisManager.cpp
 *
 *  Created on: 2016. 1. 21.
 *      Author: zerom
 */


#include "robotis_controller/RobotisController.h"

/* Sensor Module Header */
/* Motion Module Header */
#include "op2_base_module/BaseModule.h"
#include "op2_head_control_module/HeadControlModule.h"
#include "op2_action_module/ActionModule.h"
//#include "thormang3_manipulation_module/ManipulationModule.h"
//#include "thormang3_walking_module/WalkingModule.h"

using namespace ROBOTIS;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OP2_Manager");
    ros::NodeHandle _nh;

    ROS_INFO("manager->init");
    RobotisController  *_controller     = RobotisController::GetInstance();

    /* Load ROS Parameter */
    std::string         _offset_file;
    std::string         _robot_file;
    std::string         _init_file;

    _nh.param<std::string>("offset_table", _offset_file, "");
    _nh.param<std::string>("robot_file_path", _robot_file, "");
    _nh.param<std::string>("init_file_path", _init_file, "");

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
    // _controller->AddSensorModule((SensorModule*)ThorMang3FeetForceTorqueSensor::GetInstance());

    /* Add Motion Module */
    _controller->AddMotionModule((MotionModule*)ActionModule::GetInstance());
    _controller->AddMotionModule((MotionModule*)BaseModule::GetInstance());
//    _controller->AddMotionModule((MotionModule*)ManipulationModule::GetInstance());
    _controller->AddMotionModule((MotionModule*)HeadControlModule::GetInstance());
//    _controller->AddMotionModule((MotionModule*)WalkingMotionModule::GetInstance());

    _controller->StartTimer();

    while(ros::ok())
    { }

    return 0;
}
