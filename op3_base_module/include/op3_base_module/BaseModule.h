/*
 * BaseModule.h
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#ifndef BASEMODULE_H_
#define BASEMODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>

#include <yaml-cpp/yaml.h>

#include "robotis_framework_common/MotionModule.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "robotis_math/RobotisMath.h"
#include "op3_kinematics_dynamics/OP3KinematicsDynamics.h"

#include "RobotisState.h"

namespace ROBOTIS
{

class BaseJointData
{

public:
    double position;
    double velocity;
    double effort;

    int p_gain;
    int i_gain;
    int d_gain;

};

class BaseJointState
{

public:
    BaseJointData curr_joint_state[ MAX_JOINT_ID + 1 ];
    BaseJointData goal_joint_state[ MAX_JOINT_ID + 1 ];
    BaseJointData fake_joint_state[ MAX_JOINT_ID + 1 ];

};

class BaseModule : public MotionModule
{
private:
    static BaseModule *unique_instance_;

    int                 control_cycle_msec_;
    boost::thread       queue_thread_;
    boost::thread       tra_gene_tread_;

    ros::Publisher      status_msg_pub_;
    ros::Publisher		set_ctrl_module_pub_;

    std::map<std::string, int> joint_name_to_id;

    bool				has_goal_joints_;
    bool 				ini_pose_only_;

    BaseModule();

    void QueueThread();
    void setCtrlModule(std::string module);

    void parseIniPoseData(const std::string &path);

    void publishStatusMsg(unsigned int type, std::string msg);

public:
    virtual ~BaseModule();

    static BaseModule *GetInstance() { return unique_instance_; }

    /* ROS Topic Callback Functions */
    void    IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg );

    /* ROS Calculation Functions */
    void    IniposeTraGeneProc();

    void 	PoseGenProc(Eigen::MatrixXd _joint_angle_pose);
    void    PoseGenProc(std::map<std::string, double>& joint_angle_pose);

    /* ROS Framework Functions */
    void    Initialize(const int control_cycle_msec, Robot *robot);
    void    Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors);

    void	Stop();
    bool	IsRunning();

    /* Parameter */
    ROBOTIS_BASE::RobotisState *Robotis;
    BaseJointState *JointState;

};

}


#endif /* BASEMODULE_H_ */
