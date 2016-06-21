/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/


/* Author: Kayman Jung */

#ifndef OP3_CM_740_MODULE_H_
#define OP3_CM_740_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include "robotis_math/RobotisMathBase.h"
#include "robotis_math/RobotisLinearAlgebra.h"
#include "robotis_framework_common/SensorModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

namespace ROBOTIS
{

class CM740Module : public SensorModule, public Singleton<CM740Module>
{
public:
  CM740Module();
    virtual ~CM740Module();

    /* ROS Topic Callback Functions */
    // void    TopicCallback(const std_msgs::Int16::ConstPtr &msg);

    void    Initialize(const int control_cycle_msec, Robot *robot);
    void    Process(std::map<std::string, Dynamixel *> dxls,
                    std::map<std::string, Sensor *> sensors);

private:
    const double G_ACC = 9.80665;

    void QueueThread();

    double getGyroValue(int dxl_value);
    double getAccValue(int dxl_value);
    void fusionIMU();

    void buttonMode(bool pushed);
    void buttonStart(bool pushed);
    void handleButton(const std::string &button_name);
    void handleVoltage(double present_volt);
    void publishStatusMsg(unsigned int type, std::string msg);
    double lowPassFilter(double alpha, double x_new, double x_old);

    int             control_cycle_msec_;
    boost::thread   queue_thread_;
    bool            DEBUG;
    bool            button_mode_;
    bool            button_start_;
    double          previous_volt_;
    double          present_volt_;
    int             volt_count_;

    sensor_msgs::Imu imu_msg_;

    /* sample subscriber & publisher */
    ros::Subscriber     sub1_;
    ros::Publisher      imu_pub_;
    ros::Publisher      reset_dxl_pub_;
    ros::Publisher      status_msg_pub_;
};

}


#endif /* OP3_CM_740_MODULE_H_ */
