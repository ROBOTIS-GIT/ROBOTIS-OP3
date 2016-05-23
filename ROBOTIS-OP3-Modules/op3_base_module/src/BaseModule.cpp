/*
 * ThorManipulation.cpp
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#include <stdio.h>
#include "op3_base_module/BaseModule.h"

using namespace ROBOTIS;

BaseModule *BaseModule::unique_instance_ = new BaseModule();

BaseModule::BaseModule()
: control_cycle_msec_(0)
, has_goal_joints_(false)
, ini_pose_only_(false)
{
    enable          = false;
    module_name     = "base_module";
    control_mode    = POSITION_CONTROL;

    /* arm */
    result["r_sho_pitch"]   = new DynamixelState();
    result["l_sho_pitch"]   = new DynamixelState();
    result["r_sho_roll"]    = new DynamixelState();
    result["l_sho_roll"]    = new DynamixelState();
    result["r_el"]          = new DynamixelState();
    result["l_el"]          = new DynamixelState();

    /* leg */
    result["r_hip_pitch"]   = new DynamixelState();
    result["r_hip_roll"]    = new DynamixelState();
    result["r_hip_yaw"]     = new DynamixelState();
    result["r_knee"]        = new DynamixelState();
    result["r_ank_pitch"]   = new DynamixelState();
    result["r_ank_roll"]    = new DynamixelState();

    result["l_hip_pitch"]   = new DynamixelState();
    result["l_hip_roll"]    = new DynamixelState();
    result["l_hip_yaw"]     = new DynamixelState();
    result["l_knee"]        = new DynamixelState();
    result["l_ank_pitch"]   = new DynamixelState();
    result["l_ank_roll"]    = new DynamixelState();

    /* head */
    result["head_pan"]      = new DynamixelState();
    result["head_tilt"]     = new DynamixelState();

    /* arm */
    joint_name_to_id["r_sho_pitch"]     = 1;
    joint_name_to_id["l_sho_pitch"]     = 2;
    joint_name_to_id["r_sho_roll"]      = 3;
    joint_name_to_id["l_sho_roll"]      = 4;
    joint_name_to_id["r_el"]            = 5;
    joint_name_to_id["l_el"]            = 6;

    /* leg */
    joint_name_to_id["r_hip_yaw"]       = 7;
    joint_name_to_id["l_hip_yaw"]       = 8;
    joint_name_to_id["r_hip_roll"]      = 9;
    joint_name_to_id["l_hip_roll"]      = 10;
    joint_name_to_id["r_hip_pitch"]     = 11;
    joint_name_to_id["l_hip_pitch"]     = 12;
    joint_name_to_id["r_knee"]          = 13;
    joint_name_to_id["l_knee"]          = 14;
    joint_name_to_id["r_ank_pitch"]     = 15;
    joint_name_to_id["l_ank_pitch"]     = 16;
    joint_name_to_id["r_ank_roll"]      = 17;
    joint_name_to_id["l_ank_roll"]      = 18;

    /* head */
    joint_name_to_id["head_pan"]        = 19;
    joint_name_to_id["head_tilt"]       = 20;

    Robotis = new ROBOTIS_BASE::RobotisState();
    JointState = new BaseJointState();
}

BaseModule::~BaseModule()
{
    queue_thread_.join();
}

void BaseModule::Initialize(const int control_cycle_msec, Robot *robot)
{
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_ = boost::thread(boost::bind(&BaseModule::QueueThread, this));

    ros::NodeHandle _ros_node;

    /* publish topics */
    status_msg_pub_         = _ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
    //    set_ctrl_module_pub_	= _ros_node.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_ctrl_module", 1);
    set_ctrl_module_pub_	= _ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
}

void BaseModule::parseIniPoseData( const std::string &path )
{
    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile( path.c_str() );
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return ;
    }

    // parse movement time
    double _mov_time;
    _mov_time = doc["mov_time"].as< double >();

    Robotis->mov_time = _mov_time;

    // parse via-point number
    int _via_num;
    _via_num = doc["via_num"].as< int >();

    Robotis->via_num = _via_num;

    // parse via-point time
    std::vector<double> _via_time;
    _via_time = doc["via_time"].as< std::vector<double> >();

    Robotis->via_time.resize( _via_num , 1 );
    for( int num = 0; num < _via_num; num++ )
        Robotis->via_time.coeffRef( num , 0 ) = _via_time[ num ];

    // parse via-point pose
    Robotis->joint_via_pose.resize( _via_num , MAX_JOINT_ID + 1 );
    Robotis->joint_via_dpose.resize( _via_num , MAX_JOINT_ID + 1 );
    Robotis->joint_via_ddpose.resize( _via_num , MAX_JOINT_ID + 1 );

    Robotis->joint_via_pose.fill( 0.0 );
    Robotis->joint_via_dpose.fill( 0.0 );
    Robotis->joint_via_ddpose.fill( 0.0 );

    YAML::Node _via_pose_node = doc["via_pose"];
    for(YAML::iterator _it = _via_pose_node.begin() ; _it != _via_pose_node.end() ; ++_it)
    {
        int _id;
        std::vector<double> _value;

        _id = _it->first.as<int>();
        _value = _it->second.as< std::vector<double> >();

        for( int num = 0; num < _via_num; num++ )
            Robotis->joint_via_pose.coeffRef( num , _id ) = _value[ num ] * deg2rad;
    }

    // parse target pose
    YAML::Node _tar_pose_node = doc["tar_pose"];
    for(YAML::iterator _it = _tar_pose_node.begin() ; _it != _tar_pose_node.end() ; ++_it)
    {
        int _id;
        double _value;

        _id = _it->first.as<int>();
        _value = _it->second.as<double>();

        Robotis->joint_ini_pose.coeffRef( _id , 0 ) = _value * deg2rad;
    }

    Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;
    Robotis->calc_joint_tra.resize( Robotis->all_time_steps , MAX_JOINT_ID + 1 );
}

void BaseModule::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscribe topics */
    // for gui
    ros::Subscriber ini_pose_msg_sub = _ros_node.subscribe("/robotis/base/ini_pose", 5, &BaseModule::IniPoseMsgCallback, this);

    while(_ros_node.ok())
    {
        _callback_queue.callAvailable();

        usleep(100);
    }
}

void BaseModule::IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg )
{
    if ( Robotis->is_moving == false )
    {
        if ( msg->data == "ini_pose")
        {
            // set module of all joints -> this module
            setCtrlModule(module_name);

            // wait to change module and to get goal position for init
            while(enable == false || has_goal_joints_ == false) usleep(8 * 1000);

            // parse initial pose
            std::string _ini_pose_path = ros::package::getPath("op3_base_module") + "/data/ini_pose.yaml";
            parseIniPoseData( _ini_pose_path );

            // generate trajectory
            tra_gene_tread_ = boost::thread(boost::bind(&BaseModule::IniposeTraGeneProc, this));
        }
    }
    else
        ROS_INFO("previous task is alive");

    return;
}

void BaseModule::IniposeTraGeneProc()
{
    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
        double ini_value = JointState->goal_joint_state[ id ].position;
        double tar_value = Robotis->joint_ini_pose.coeff( id , 0 );

        Eigen::MatrixXd tra;

        if ( Robotis->via_num == 0 )
        {
            tra = minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                    tar_value , 0.0 , 0.0 ,
                    Robotis->smp_time , Robotis->mov_time );
        }
        else
        {
            Eigen::MatrixXd via_value = Robotis->joint_via_pose.col( id );
            Eigen::MatrixXd d_via_value = Robotis->joint_via_dpose.col( id );
            Eigen::MatrixXd dd_via_value = Robotis->joint_via_ddpose.col( id );

            tra = minimum_jerk_tra_via_n_qdqddq( Robotis->via_num,
                    ini_value , 0.0 , 0.0 ,
                    via_value , d_via_value, dd_via_value,
                    tar_value , 0.0 , 0.0 ,
                    Robotis->smp_time , Robotis->via_time , Robotis->mov_time );
        }

        Robotis->calc_joint_tra.block( 0 , id , Robotis->all_time_steps , 1 ) = tra;
    }

    Robotis->is_moving = true;
    Robotis->cnt = 0;
    ROS_INFO("[start] send trajectory");
}

void BaseModule::PoseGenProc(Eigen::MatrixXd _joint_angle_pose)
{
    setCtrlModule(module_name);
    while(enable == false || has_goal_joints_ == false) usleep(8 * 1000);

    Robotis->mov_time = 5.0;
    Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;

    Robotis->calc_joint_tra.resize( Robotis->all_time_steps , MAX_JOINT_ID + 1 );

    Robotis->joint_pose = _joint_angle_pose;

    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
        double ini_value = JointState->goal_joint_state[ id ].position;
        double tar_value = Robotis->joint_pose.coeff( id , 0 );

        ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);


        Eigen::MatrixXd tra = minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                tar_value , 0.0 , 0.0 ,
                Robotis->smp_time , Robotis->mov_time );


        Robotis->calc_joint_tra.block( 0 , id , Robotis->all_time_steps , 1 ) = tra;
    }

    Robotis->is_moving = true;
    Robotis->cnt = 0;
    ini_pose_only_ = true;
    ROS_INFO("[start] send trajectory");
}

void BaseModule::PoseGenProc(std::map<std::string, double>& joint_angle_pose)
{
    setCtrlModule(module_name);
    while(enable == false || has_goal_joints_ == false) usleep(8 * 1000);

    Eigen::MatrixXd _target_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1 , 1 );

    for(std::map<std::string, double>::iterator _it = joint_angle_pose.begin(); _it != joint_angle_pose.end(); _it++)
    {
        std::string _joint_name = _it->first;
        double _joint_angle_rad = _it->second;

        std::map<std::string, int>::iterator _joint_name_to_id_it = joint_name_to_id.find(_joint_name);
        if(_joint_name_to_id_it != joint_name_to_id.end()) {
            _target_pose.coeffRef(_joint_name_to_id_it->second, 0) = _joint_angle_rad;
        }
    }

    Robotis->joint_pose = _target_pose;

    Robotis->mov_time = 5.0;
    Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;

    Robotis->calc_joint_tra.resize( Robotis->all_time_steps , MAX_JOINT_ID + 1 );

    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
        double ini_value = JointState->goal_joint_state[ id ].position;
        double tar_value = Robotis->joint_pose.coeff( id , 0 );

        ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);


        Eigen::MatrixXd tra = minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
                tar_value , 0.0 , 0.0 ,
                Robotis->smp_time , Robotis->mov_time );


        Robotis->calc_joint_tra.block( 0 , id , Robotis->all_time_steps , 1 ) = tra;
    }

    Robotis->is_moving = true;
    Robotis->cnt = 0;
    ini_pose_only_ = true;
    ROS_INFO("[start] send trajectory");
}

bool BaseModule::IsRunning()
{
    return Robotis->is_moving;
}

void BaseModule::Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors)
{
    if(enable == false)
        return;

    /*----- write curr position -----*/
    for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
    {
        std::string _joint_name = state_iter->first;

        Dynamixel *_dxl = NULL;
        std::map<std::string, Dynamixel*>::iterator _dxl_it = dxls.find(_joint_name);
        if(_dxl_it != dxls.end())
            _dxl = _dxl_it->second;
        else
            continue;

        double _joint_curr_position = _dxl->dxl_state->present_position;
        double _joint_goal_position = _dxl->dxl_state->goal_position;

        JointState->curr_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_curr_position;
        JointState->goal_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_goal_position;
    }

    has_goal_joints_ = true;

    /* ----- send trajectory ----- */
    if ( Robotis->is_moving == true )
    {
        if ( Robotis->cnt == 1 )
            publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Init Pose");


        for ( int id = 1; id <= MAX_JOINT_ID; id++ )
            JointState->goal_joint_state[ id ].position = Robotis->calc_joint_tra( Robotis->cnt , id );

        Robotis->cnt++;
    }

    /*----- set joint data -----*/
    for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
    {
        std::string _joint_name = state_iter->first;

        result[ _joint_name ]->goal_position = JointState->goal_joint_state[ joint_name_to_id[_joint_name] ].position;
    }

    /*---------- initialize count number ----------*/

    if ( (Robotis->cnt >= Robotis->all_time_steps ) && (Robotis->is_moving == true ) )
    {
        ROS_INFO("[end] send trajectory");

        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish Init Pose");

        Robotis->is_moving = false;
        Robotis->cnt = 0;

        // set all joints -> none
        if(ini_pose_only_ == true)
        {
            setCtrlModule("none");
            ini_pose_only_ = false;
        }
    }
}

void BaseModule::Stop()
{
    return;
}

void BaseModule::setCtrlModule(std::string module)
{
    //    robotis_controller_msgs::JointCtrlModule _control_msg;

    //    std::map<std::string, DynamixelState *>::iterator _joint_iter;

    //    for(_joint_iter = result.begin(); _joint_iter != result.end(); ++_joint_iter)
    //    {
    //        _control_msg.joint_name.push_back(_joint_iter->first);
    //        _control_msg.module_name.push_back(module);
    //    }

    std_msgs::String _control_msg;
    _control_msg.data = module_name;

    set_ctrl_module_pub_.publish(_control_msg);
}

void BaseModule::publishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg _status;
    _status.header.stamp = ros::Time::now();
    _status.type = type;
    _status.module_name = "Base";
    _status.status_msg = msg;

    status_msg_pub_.publish(_status);
}
