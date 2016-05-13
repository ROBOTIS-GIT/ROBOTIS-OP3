/*
 * ThorManipulation.cpp
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#include <stdio.h>
#include "op3_io_module/IOModule.h"

using namespace ROBOTIS;

IOModule *IOModule::unique_instance_ = new IOModule();

IOModule::IOModule()
    : control_cycle_msec_(0)
    , reset_processed_(false)

{
    enable          = false;
    module_name     = "io_module";
    control_mode    = CURRENT_CONTROL;

    result["cm_740"]        = new DynamixelState();
}

IOModule::~IOModule()
{
    queue_thread_.join();
}

void IOModule::Initialize(const int control_cycle_msec, Robot *robot)
{
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_ = boost::thread(boost::bind(&IOModule::QueueThread, this));

    ros::NodeHandle _ros_node;

    /* publish topics */
    status_msg_pub_         = _ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
    set_ctrl_module_pub_	= _ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1, false);
}

void IOModule::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscribe topics */
    // for gui
    ros::Subscriber ini_pose_msg_sub = _ros_node.subscribe("/robotis/io/reset", 1, &IOModule::ResetDXLMsgCallback, this);

    while(_ros_node.ok())
    {
        _callback_queue.callAvailable();
    }
}

void IOModule::ResetDXLMsgCallback( const std_msgs::String::ConstPtr& msg )
{
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Torque On Dxl");

    // set module
    setCtrlModule(module_name);

    // wait to change module and to get goal position for init
    while(enable == false) usleep(8 * 1000);
}

bool IOModule::IsRunning()
{
    return false;
}

void IOModule::Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors)
{
    if(enable == false)
        return;

    /*----- set joint data -----*/
    for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
    {
        std::string _joint_name = state_iter->first;

        result[ _joint_name ]->goal_current = 1;
    }

    std::cout << "torque on dxl" << std::endl;

    setCtrlModule("none");
}

void IOModule::Stop()
{
    return;
}

void IOModule::setCtrlModule(std::string module)
{
    std_msgs::String _control_msg;
    _control_msg.data = module;

    set_ctrl_module_pub_.publish(_control_msg);
}

void IOModule::publishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg _status;
    _status.header.stamp = ros::Time::now();
    _status.type = type;
    _status.module_name = "IO";
    _status.status_msg = msg;

    status_msg_pub_.publish(_status);
}

void IOModule::turnOnDxl()
{
//    PortHandler *_port_h = (PortHandler *)PortHandler::GetPortHandler("/dev/ttyUSB1");
////    _port_h->SetBaudRate(1000000);
//    PacketHandler *_packet_h = PacketHandler::GetPacketHandler(1.0);
//
//    int _return = _packet_h->Write1ByteTxRx(_port_h, 200, 24, 1);
//    ROS_INFO("Torque on DXLs! [%d]", _return);
//    _packet_h->PrintTxRxResult(_return);
}
