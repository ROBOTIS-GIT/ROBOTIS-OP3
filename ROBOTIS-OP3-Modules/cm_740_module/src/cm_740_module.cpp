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
, DEBUG(false)
, button_mode_(false)
, button_start_(false)
, present_volt_(0.0)
, volt_count_(0)
{
    module_name     = "cm_740_module"; // set unique module name

    result["gyro_x"] = 0.0;
    result["gyro_y"] = 0.0;
    result["gyro_z"] = 0.0;

    result["acc_x"] = 0.0;
    result["acc_y"] = 0.0;
    result["acc_z"] = 0.0;

    result["button_mode"] = 0;
    result["button_start"] = 0;

    result["present_voltage"] = 0.0;
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
    status_msg_pub_         = _ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
    imu_pub_                = _ros_node.advertise<sensor_msgs::Imu>("/robotis/cm_740/imu", 1);
    reset_dxl_pub_          = _ros_node.advertise<std_msgs::String>("/robotis/cm_740/button", 1);

    while(_ros_node.ok())
    {
        _callback_queue.callAvailable();

        usleep(100);
    }
}

void CM740Module::Process(std::map<std::string, Dynamixel *> dxls)
{
    if(dxls["cm_740"] == NULL) return;

    UINT16_T gyro_x = dxls["cm_740"]->dxl_state->bulk_read_table["gyro_x"];
    UINT16_T gyro_y = dxls["cm_740"]->dxl_state->bulk_read_table["gyro_y"];
    UINT16_T gyro_z = dxls["cm_740"]->dxl_state->bulk_read_table["gyro_z"];

    UINT16_T acc_x = dxls["cm_740"]->dxl_state->bulk_read_table["acc_x"];
    UINT16_T acc_y = dxls["cm_740"]->dxl_state->bulk_read_table["acc_y"];
    UINT16_T acc_z = dxls["cm_740"]->dxl_state->bulk_read_table["acc_z"];

    UINT8_T present_volt = dxls["cm_740"]->dxl_state->bulk_read_table["present_voltage"];

    result["gyro_x"] = getGyroValue(gyro_x);
    result["gyro_y"] = getGyroValue(gyro_y);
    result["gyro_z"] = getGyroValue(gyro_z);

    if(DEBUG) ROS_INFO("Gyro : %f, %f, %f", result["gyro_x"], result["gyro_y"], result["gyro_z"]);

    result["acc_x"] = getAccValue(acc_x);
    result["acc_y"] = getAccValue(acc_y);
    result["acc_z"] = getAccValue(acc_z);

    if(DEBUG) ROS_INFO("Acc : %f, %f, %f", result["acc_x"], result["acc_y"], result["acc_z"]);

    UINT8_T button_flag = dxls["cm_740"]->dxl_state->bulk_read_table["button"];
    result["button_mode"] = button_flag & 0x01;
    result["button_start"] = (button_flag & 0x02) >> 1;

    buttonMode(result["button_mode"] == 1.0);
    buttonStart(result["button_start"] == 1.0);

    result["present_voltage"] = present_volt * 0.1;
    handleVoltage(result["present_voltage"]);
}

double CM740Module::getGyroValue(int dxl_value)
{
    return (dxl_value - 512) * 500.0 * 2.0 / 1023;
}

double CM740Module::getAccValue(int dxl_value)
{
    return (dxl_value - 512) * 4.0 * 2.0 / 1023;
}

void CM740Module::fusionIMU()
{
    sensor_msgs::Imu _imu_msg;

    // fusion imu data


    imu_pub_.publish(_imu_msg);
}

void CM740Module::buttonMode(bool pushed)
{
    if(button_mode_ == pushed)
        return;

    button_mode_ = pushed;

    if(pushed == true)
        handleButton("mode");
}

void CM740Module::buttonStart(bool pushed)
{
    if(button_start_ == pushed)
        return;

    button_start_ = pushed;

    if(pushed == true)
        handleButton("start");
}

void CM740Module::handleButton(const std::string &button_name)
{
    std_msgs::String _button_msg;
    _button_msg.data = button_name;

    reset_dxl_pub_.publish(_button_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Button : " + button_name);
}

void CM740Module::handleVoltage(double present_volt)
{
    double _ratio = 0.4;
    previous_volt_ = (previous_volt_ != 0) ? previous_volt_ * (1 - _ratio) + present_volt * _ratio : present_volt;

    if(fabs(present_volt_ - previous_volt_) >= 0.1)
    {
        present_volt_ = previous_volt_;
        std::stringstream _ss;
        _ss << "Present Volt : " << present_volt_ << "V";
        publishStatusMsg((present_volt_ < 11 ? robotis_controller_msgs::StatusMsg::STATUS_WARN : robotis_controller_msgs::StatusMsg::STATUS_INFO), _ss.str());
        if(DEBUG) ROS_INFO("Present Volt : %fV, Read Volt : %fV", previous_volt_, result["present_voltage"]);
    }
}

void CM740Module::publishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg _status;
    _status.header.stamp = ros::Time::now();
    _status.type = type;
    _status.module_name = "SENSOR";
    _status.status_msg = msg;

    status_msg_pub_.publish(_status);
}

