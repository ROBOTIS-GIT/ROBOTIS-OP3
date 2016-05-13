/*
 * BaseModule.h
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#ifndef IOMODULE_H_
#define IOMODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include <yaml-cpp/yaml.h>

#include "robotis_framework_common/MotionModule.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "robotis_math/RobotisMath.h"
#include "op3_kinematics_dynamics/OP3KinematicsDynamics.h"

namespace ROBOTIS
{

class IOModule : public MotionModule
{
private:
    static IOModule *unique_instance_;

    int                 control_cycle_msec_;
    bool                reset_processed_;
    bool                is_running_;
    boost::thread       queue_thread_;
    boost::thread       tra_gene_tread_;

    ros::Publisher      status_msg_pub_;
    ros::Publisher		set_ctrl_module_pub_;

    IOModule();

    void QueueThread();
    void setCtrlModule(std::string module);

    void publishStatusMsg(unsigned int type, std::string msg);
    void turnOnDxl();

public:
    virtual ~IOModule();

    static IOModule *GetInstance() { return unique_instance_; }

    /* ROS Topic Callback Functions */
    void    ResetDXLMsgCallback( const std_msgs::String::ConstPtr& msg );

    /* ROS Framework Functions */
    void    Initialize(const int control_cycle_msec, Robot *robot);
    void    Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors);

    void	Stop();
    bool	IsRunning();
};

}


#endif /* IOMODULE_H_ */
