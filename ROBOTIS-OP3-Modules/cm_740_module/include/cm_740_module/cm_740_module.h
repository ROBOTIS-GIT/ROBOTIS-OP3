/*
 * sensor_module_tutorial.h
 *
 *  Created on: 2016. 4. 20.
 *      Author: zerom
 */

#ifndef OP3_CM_740_MODULE_H_
#define OP3_CM_740_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/SensorModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

namespace ROBOTIS
{

class CM740Module : public SensorModule
{
private:
    static CM740Module *unique_instance_;

    int             control_cycle_msec_;
    boost::thread   queue_thread_;
    bool            DEBUG;
    bool            button_mode_;
    bool            button_start_;
    double          previous_volt_;
    double          present_volt_;
    int             volt_count_;

    /* sample subscriber & publisher */
    ros::Subscriber     sub1_;
    ros::Publisher      imu_pub_;
    ros::Publisher      reset_dxl_pub_;
    ros::Publisher      status_msg_pub_;

    CM740Module();

    void QueueThread();

    double getGyroValue(int dxl_value);
    double getAccValue(int dxl_value);
    void fusionIMU();

    void buttonMode(bool pushed);
    void buttonStart(bool pushed);
    void handleButton(const std::string &button_name);
    void handleVoltage(double present_volt);
    void publishStatusMsg(unsigned int type, std::string msg);

public:
    virtual ~CM740Module();

    static CM740Module *GetInstance() { return unique_instance_; }

    /* ROS Topic Callback Functions */
    // void    TopicCallback(const std_msgs::Int16::ConstPtr &msg);

    void    Initialize(const int control_cycle_msec, Robot *robot);
    void    Process(std::map<std::string, Dynamixel *> dxls);
};

}


#endif /* OP3_CM_740_MODULE_H_ */
