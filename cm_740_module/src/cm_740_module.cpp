/*
 * sensor_module_tutorial.cpp
 *
 *  Created on: 2016. 4. 20.
 *      Author: zerom
 */

#include <stdio.h>
#include "cm_740_module/cm_740_module.h"

using namespace ROBOTIS;

CM740Module *CM740Module::unique_instance_ = new CM740Module();

CM740Module::CM740Module()
    : control_cycle_msec_(8)
{
    module_name     = "cm_740_module"; // set unique module name

    result["gyro_x"] = 0.0;
    result["gyro_y"] = 0.0;
    result["gyro_z"] = 0.0;
    result["acc_x"] = 0.0;
    result["acc_y"] = 0.0;
    result["acc_z"] = 0.0;
    result["button"] = 0;
}

CM740Module::~CM740Module()
{
    queue_thread_.join();
}

void CM740Module::Initialize(const int control_cycle_msec, Robot *robot)
{
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_       = boost::thread(boost::bind(&CM740Module::QueueThread, this));
}

void CM740Module::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscriber */
    //sub1_ = _ros_node.subscribe("/tutorial_topic", 10, &CM740Module::TopicCallback, this);

    /* publisher */
    //pub1_ = _ros_node.advertise<std_msgs::Int16>("/tutorial_publish", 1, true);

    while(_ros_node.ok())
        _callback_queue.callAvailable();
}

void CM740Module::TopicCallback(const std_msgs::Int16::ConstPtr &msg)
{
    std_msgs::Int16 _msg;
    _msg.data = msg->data;
    pub1_.publish(_msg);
}

void CM740Module::Process(std::map<std::string, Dynamixel *> dxls)
{
    //UINT16_T ext_port_data_1 = dxls["r_leg_an_p"]->dxl_state->bulk_read_table["external_port_data_1"];
    //UINT16_T ext_port_data_2 = dxls["r_leg_an_p"]->dxl_state->bulk_read_table["external_port_data_2"];

    // ...

    result["test_sensor"] = 0.0;
}


