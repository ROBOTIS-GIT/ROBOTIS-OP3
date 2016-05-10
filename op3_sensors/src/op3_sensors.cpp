/*
 * RobotisManager.cpp
 *
 *  Created on: 2016. 05. 09.
 *      Author: kayman
 */


#include "robotis_controller/RobotisController.h"

/* Sensor Module Header */
#include "cm_740_module/cm_740_module.h"
/* Motion Module Header */
//#include "op3_base_module/BaseModule.h"
//#include "op3_head_control_module/HeadControlModule.h"
//#include "op3_action_module/ActionModule.h"

using namespace ROBOTIS;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OP3_Sensor_740");
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
    _controller->AddSensorModule((SensorModule*)CM740Module::GetInstance());

    /* Add Motion Module */
    // _controller->AddMotionModule((MotionModule*)ActionModule::GetInstance());

    _controller->StartTimer();

    while(ros::ok())
    { }

    return 0;
}
