/*
 * op3_walking_module.cpp
 *
 *  Created on: 2016. 5. 16.
 *      Author: JungKM
 */


#include <stdio.h>

#include "op3_walking_module/op3_walking_module.h"

using namespace ROBOTIS;

WalkingMotionModule *WalkingMotionModule::unique_instance_ = new WalkingMotionModule();

WalkingMotionModule::WalkingMotionModule()
: control_cycle_msec_(8)
{
    enable          = false;
    module_name     = "walking_module";
    control_mode    = POSITION_CONTROL;

    op3_kd_ = new ROBOTIS::OP3KinematicsDynamics(ROBOTIS::WHOLE_BODY);

    // result
    result["r_hip_yaw"  ] = new DynamixelState();
    result["r_hip_roll" ] = new DynamixelState();
    result["r_hip_pitch"] = new DynamixelState();
    result["r_knee"     ] = new DynamixelState();
    result["r_ank_pitch"] = new DynamixelState();
    result["r_ank_roll" ] = new DynamixelState();

    result["l_hip_yaw"  ] = new DynamixelState();
    result["l_hip_roll" ] = new DynamixelState();
    result["l_hip_pitch"] = new DynamixelState();
    result["l_knee"     ] = new DynamixelState();
    result["l_ank_pitch"] = new DynamixelState();
    result["l_ank_roll" ] = new DynamixelState();

    result["r_sho_pitch"] = new DynamixelState();
    result["l_sho_pitch"] = new DynamixelState();
    // result["head_pan"   ] = new DynamixelState();

    // joint table
    joint_table_["l_hip_yaw"  ] = 0;
    joint_table_["r_hip_roll" ] = 1;
    joint_table_["r_hip_pitch"] = 2;
    joint_table_["r_knee"     ] = 3;
    joint_table_["r_ank_pitch"] = 4;
    joint_table_["r_ank_roll" ] = 5;

    joint_table_["l_hip_yaw"  ] = 6;
    joint_table_["l_hip_roll" ] = 7;
    joint_table_["l_hip_pitch"] = 8;
    joint_table_["l_knee"     ] = 9;
    joint_table_["l_ank_pitch"] = 10;
    joint_table_["l_ank_roll" ] = 11;

    joint_table_["r_sho_pitch"] = 12;
    joint_table_["l_sho_pitch"] = 13;
    // joint_table_["head_pan"   ] = 14;

    goal_position_      = Eigen::MatrixXd::Zero(1, result.size());
}

WalkingMotionModule::~WalkingMotionModule()
{
    queue_thread_.join();
}

void WalkingMotionModule::Initialize(const int control_cycle_msec, Robot *robot)
{
    queue_thread_ = boost::thread(boost::bind(&WalkingMotionModule::QueueThread, this));
    control_cycle_msec_ = control_cycle_msec;

    // m, ms, deg
    // init pose
    walking_param_.init_x_offset = -0.010;
    walking_param_.init_y_offset = 0.005;
    walking_param_.init_z_offset = 0.020;
    walking_param_.init_roll_offset = 0.0;
    walking_param_.init_pitch_offset = 0.0;
    walking_param_.init_yaw_offset = 0.0;
    walking_param_.hip_pitch_offset = 13.0;
    // time
    walking_param_.period_time = 600;
    walking_param_.dsp_ratio = 0.1;
    walking_param_.step_fb_ratio = 0.28;
    // walking
    walking_param_.x_move_amplitude = 0.0;
    walking_param_.y_move_amplitude = 0.0;
    walking_param_.z_move_amplitude = 0.040;    // foot height
    walking_param_.angle_move_amplitude = 0.0;
    // walking_param_.move_aim_on = false;
    // balance
    walking_param_.balance_enable = false;
    walking_param_.balance_hip_roll_gain = 0.5;
    walking_param_.balance_knee_gain = 0.3;
    walking_param_.balance_ankle_roll_gain = 1.0;
    walking_param_.balance_ankle_pitch_gain = 0.9;
    walking_param_.y_swap_amplitude = 0.020;
    walking_param_.z_swap_amplitude = 0.005;
    walking_param_.pelvis_offset = 3.0;
    walking_param_.arm_swing_gain = 1.5;

    // member variable
    m_Body_Swing_Y = 0;
    m_Body_Swing_Z = 0;

    m_X_Swap_Phase_Shift = M_PI;
    m_X_Swap_Amplitude_Shift = 0;
    m_X_Move_Phase_Shift = M_PI / 2;
    m_X_Move_Amplitude_Shift = 0;
    m_Y_Swap_Phase_Shift = 0;
    m_Y_Swap_Amplitude_Shift = 0;
    m_Y_Move_Phase_Shift = M_PI / 2;
    m_Z_Swap_Phase_Shift = M_PI * 3 / 2;
    m_Z_Move_Phase_Shift = M_PI / 2;
    m_A_Move_Phase_Shift = M_PI / 2;

    m_Ctrl_Running = false;
    m_Real_Running = false;
    m_Time = 0;

    ros::NodeHandle _ros_node;

    std::string _path = ros::package::getPath("op3_walking_module") + "/config/param.yaml";
    _ros_node.param<std::string>("walking_param_path", param_path_, _path);

    loadWalkingParam(param_path_);

    updateTimeParam();
    updateMovementParam();
}

void    WalkingMotionModule::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* publish topics */
    status_msg_pub_ = _ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);


    /* ROS Service Callback Functions */
    ros::ServiceServer get_walking_param_server = _ros_node.advertiseService("/robotis/walking/get_params", &WalkingMotionModule::getWalkigParameterCallback, this);
    //    ros::ServiceServer get_ref_step_data_server  = _ros_node.advertiseService("/robotis/walking/get_reference_step_data",
    //                                                                                &WalkingMotionModule::GetReferenceStepDataServiceCallback, this);
    //    ros::ServiceServer add_step_data_array_sever = _ros_node.advertiseService("/robotis/walking/add_step_data",     &WalkingMotionModule::AddStepDataServiceCallback,       this);
    //    ros::ServiceServer walking_start_server      = _ros_node.advertiseService("/robotis/walking/walking_start",     &WalkingMotionModule::WalkingStartServiceCallback,      this);
    //    ros::ServiceServer is_running_server         = _ros_node.advertiseService("/robotis/walking/is_running",        &WalkingMotionModule::IsRunningServiceCallback,         this);
    //    ros::ServiceServer set_balance_param_server  = _ros_node.advertiseService("/robotis/walking/set_balance_param", &WalkingMotionModule::SetBalanceParamServiceCallback,   this);
    //    ros::ServiceServer remove_existing_step_data = _ros_node.advertiseService("/robotis/walking/remove_existing_step_data",
    //                                                                                &WalkingMotionModule::RemoveExistingStepDataServiceCallback, this);

    /* sensor topic subscribe */
    ros::Subscriber _imu_data_sub = _ros_node.subscribe("/robotis/sensor/imu/imu", 0, &WalkingMotionModule::IMUDataOutputCallback, this);
    ros::Subscriber _walking_command_sub = _ros_node.subscribe("/robotis/walking/command", 0, &WalkingMotionModule::walkingCommandCallback, this);
    ros::Subscriber _walking_param_sub = _ros_node.subscribe("/robotis/walking/set_params", 0, &WalkingMotionModule::walkingParameterCallback, this);

    while(_ros_node.ok())
    {
        _callback_queue.callAvailable();
    }
}

void    WalkingMotionModule::PublishRobotPose(void)
{
    /*
    publish_mutex_.lock();
    robot_pose_msg_.global_to_center_of_body.position.x = desired_matrix_g_to_cob_.coeff(0, 3);
    robot_pose_msg_.global_to_center_of_body.position.y = desired_matrix_g_to_cob_.coeff(1, 3);
    robot_pose_msg_.global_to_center_of_body.position.z = desired_matrix_g_to_cob_.coeff(2, 3);
    Eigen::Quaterniond quaterniond_g_to_cob(desired_matrix_g_to_cob_.block<3, 3>(0, 0));


    robot_pose_msg_.global_to_right_foot.position.x = desired_matrix_g_to_rfoot_.coeff(0, 3);
    robot_pose_msg_.global_to_right_foot.position.y = desired_matrix_g_to_rfoot_.coeff(1, 3);
    robot_pose_msg_.global_to_right_foot.position.z = desired_matrix_g_to_rfoot_.coeff(2, 3);
    Eigen::Quaterniond quaterniond_g_to_rf(desired_matrix_g_to_rfoot_.block<3, 3>(0, 0));

    robot_pose_msg_.global_to_left_foot.position.x = desired_matrix_g_to_lfoot_.coeff(0, 3);
    robot_pose_msg_.global_to_left_foot.position.y = desired_matrix_g_to_lfoot_.coeff(1, 3);
    robot_pose_msg_.global_to_left_foot.position.z = desired_matrix_g_to_lfoot_.coeff(2, 3);
    Eigen::Quaterniond quaterniond_g_to_lf(desired_matrix_g_to_lfoot_.block<3, 3>(0, 0));
    publish_mutex_.unlock();

    tf::quaternionEigenToMsg(quaterniond_g_to_cob, robot_pose_msg_.global_to_center_of_body.orientation);
    tf::quaternionEigenToMsg(quaterniond_g_to_rf,  robot_pose_msg_.global_to_right_foot.orientation);
    tf::quaternionEigenToMsg(quaterniond_g_to_lf,  robot_pose_msg_.global_to_left_foot.orientation);

    robot_pose_pub_.publish(robot_pose_msg_);
     */
}


void    WalkingMotionModule::PublishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg _status;
    _status.header.stamp = ros::Time::now();
    _status.type = type;
    _status.module_name = "Walking";
    _status.status_msg = msg;

    status_msg_pub_.publish(_status);
}

/*
int     WalkingMotionModule::StepDataMsgToStepData(thormang3_walking_module_msgs::StepData& src, StepData& des)
{
    int copy_result = STEP_DATA_ERR::NO_ERROR;
    des.TimeData.bWalkingState          = src.time_data.walking_state;
    des.TimeData.dAbsStepTime           = src.time_data.abs_step_time;
    des.TimeData.dDSPratio              = src.time_data.dsp_ratio;
    des.TimeData.sigmoid_ratio_x        = 1.0;//msg->step_data[i].TimeData.front_pause_ratio_x;
    des.TimeData.sigmoid_ratio_y        = 1.0;//msg->step_data[i].TimeData.front_pause_ratio_y;
    des.TimeData.sigmoid_ratio_z        = 1.0;//msg->step_data[i].TimeData.front_pause_ratio_z;
    des.TimeData.sigmoid_ratio_roll     = 1.0;//msg->step_data[i].TimeData.front_pause_ratio_roll;
    des.TimeData.sigmoid_ratio_pitch    = 1.0;//msg->step_data[i].TimeData.front_pause_ratio_pitch;
    des.TimeData.sigmoid_ratio_yaw      = 1.0;//msg->step_data[i].TimeData.front_pause_ratio_yaw;

    des.TimeData.sigmoid_distortion_x       = 1.0;//msg->step_data[i].TimeData.back_pause_ratio_x;
    des.TimeData.sigmoid_distortion_y       = 1.0;//msg->step_data[i].TimeData.back_pause_ratio_y;
    des.TimeData.sigmoid_distortion_z       = 1.0;//msg->step_data[i].TimeData.back_pause_ratio_z;
    des.TimeData.sigmoid_distortion_roll    = 1.0;//msg->step_data[i].TimeData.back_pause_ratio_roll;
    des.TimeData.sigmoid_distortion_pitch   = 1.0;//msg->step_data[i].TimeData.back_pause_ratio_pitch;
    des.TimeData.sigmoid_distortion_yaw     = 1.0;//msg->step_data[i].TimeData.back_pause_ratio_yaw;


    des.PositionData.bMovingFoot        = src.position_data.moving_foot;
    des.PositionData.dShoulderSwingGain = 0;
    des.PositionData.dElbowSwingGain    = 0;
    des.PositionData.dFootHeight        = src.position_data.foot_z_swap;
    des.PositionData.dWaistPitchAngle   = 0;
    des.PositionData.dWaistYawAngle     = src.position_data.torso_yaw_angle_rad;
    des.PositionData.dZ_Swap_Amplitude  = src.position_data.body_z_swap;

    des.PositionData.stBodyPosition.z           = src.position_data.body_pose.z;
    des.PositionData.stBodyPosition.roll        = src.position_data.body_pose.roll;
    des.PositionData.stBodyPosition.pitch       = src.position_data.body_pose.pitch;
    des.PositionData.stBodyPosition.yaw         = src.position_data.body_pose.yaw;
    des.PositionData.stRightFootPosition.x      = src.position_data.right_foot_pose.x;
    des.PositionData.stRightFootPosition.y      = src.position_data.right_foot_pose.y;
    des.PositionData.stRightFootPosition.z      = src.position_data.right_foot_pose.z;
    des.PositionData.stRightFootPosition.roll   = src.position_data.right_foot_pose.roll;
    des.PositionData.stRightFootPosition.pitch  = src.position_data.right_foot_pose.pitch;
    des.PositionData.stRightFootPosition.yaw    = src.position_data.right_foot_pose.yaw;
    des.PositionData.stLeftFootPosition.x       = src.position_data.left_foot_pose.x;
    des.PositionData.stLeftFootPosition.y       = src.position_data.left_foot_pose.y;
    des.PositionData.stLeftFootPosition.z       = src.position_data.left_foot_pose.z;
    des.PositionData.stLeftFootPosition.roll    = src.position_data.left_foot_pose.roll;
    des.PositionData.stLeftFootPosition.pitch   = src.position_data.left_foot_pose.pitch;
    des.PositionData.stLeftFootPosition.yaw     = src.position_data.left_foot_pose.yaw;

    if((src.time_data.walking_state != WalkingStateFlag::InWalkingStarting)
        && (src.time_data.walking_state != WalkingStateFlag::InWalking)
        && (src.time_data.walking_state != WalkingStateFlag::InWalkingEnding) )
        copy_result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;

    if((src.time_data.start_time_delay_ratio_x < 0)
        || (src.time_data.start_time_delay_ratio_y < 0)
        || (src.time_data.start_time_delay_ratio_z < 0)
        || (src.time_data.start_time_delay_ratio_roll < 0)
        || (src.time_data.start_time_delay_ratio_pitch < 0)
        || (src.time_data.start_time_delay_ratio_yaw < 0) )
        copy_result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;

    if((src.time_data.finish_time_advance_ratio_x < 0)
        || (src.time_data.finish_time_advance_ratio_y < 0)
        || (src.time_data.finish_time_advance_ratio_z < 0)
        || (src.time_data.finish_time_advance_ratio_roll < 0)
        || (src.time_data.finish_time_advance_ratio_pitch < 0)
        || (src.time_data.finish_time_advance_ratio_yaw < 0) )
        copy_result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;

    if(((src.time_data.start_time_delay_ratio_x + src.time_data.finish_time_advance_ratio_x) > 1.0)
        || ((src.time_data.start_time_delay_ratio_y     + src.time_data.finish_time_advance_ratio_y     ) > 1.0)
        || ((src.time_data.start_time_delay_ratio_z     + src.time_data.finish_time_advance_ratio_z     ) > 1.0)
        || ((src.time_data.start_time_delay_ratio_roll  + src.time_data.finish_time_advance_ratio_roll  ) > 1.0)
        || ((src.time_data.start_time_delay_ratio_pitch + src.time_data.finish_time_advance_ratio_pitch ) > 1.0)
        || ((src.time_data.start_time_delay_ratio_yaw   + src.time_data.finish_time_advance_ratio_yaw   ) > 1.0) )
        copy_result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;

    if((src.position_data.moving_foot != MovingFootFlag::NFootMove)
        && (src.position_data.moving_foot != MovingFootFlag::RFootMove)
        && (src.position_data.moving_foot != MovingFootFlag::LFootMove))
        copy_result |= STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA;

    if(src.position_data.foot_z_swap < 0)
        copy_result |= STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA;

    return copy_result;
}

int     WalkingMotionModule::StepDataToStepDataMsg(StepData& src, thormang3_walking_module_msgs::StepData& des)
{
    des.time_data.walking_state   = src.TimeData.bWalkingState;
    des.time_data.abs_step_time   = src.TimeData.dAbsStepTime;
    des.time_data.dsp_ratio       = src.TimeData.dDSPratio;

    des.time_data.start_time_delay_ratio_x      = des.time_data.finish_time_advance_ratio_x     = 0;
    des.time_data.start_time_delay_ratio_y      = des.time_data.finish_time_advance_ratio_y     = 0;
    des.time_data.start_time_delay_ratio_z      = des.time_data.finish_time_advance_ratio_z     = 0;
    des.time_data.start_time_delay_ratio_roll   = des.time_data.finish_time_advance_ratio_roll  = 0;
    des.time_data.start_time_delay_ratio_pitch  = des.time_data.finish_time_advance_ratio_pitch = 0;
    des.time_data.start_time_delay_ratio_yaw    = des.time_data.finish_time_advance_ratio_yaw   = 0;

    des.position_data.moving_foot               = src.PositionData.bMovingFoot;
    des.position_data.foot_z_swap               = src.PositionData.dFootHeight;
    des.position_data.torso_yaw_angle_rad       = src.PositionData.dWaistYawAngle;
    des.position_data.body_z_swap               = src.PositionData.dZ_Swap_Amplitude;

    des.position_data.body_pose.z             = src.PositionData.stBodyPosition.z;
    des.position_data.body_pose.roll          = src.PositionData.stBodyPosition.roll;
    des.position_data.body_pose.pitch         = src.PositionData.stBodyPosition.pitch;
    des.position_data.body_pose.yaw           = src.PositionData.stBodyPosition.yaw;
    des.position_data.right_foot_pose.x       = src.PositionData.stRightFootPosition.x;
    des.position_data.right_foot_pose.y       = src.PositionData.stRightFootPosition.y;
    des.position_data.right_foot_pose.z       = src.PositionData.stRightFootPosition.z;
    des.position_data.right_foot_pose.roll    = src.PositionData.stRightFootPosition.roll;
    des.position_data.right_foot_pose.pitch   = src.PositionData.stRightFootPosition.pitch;
    des.position_data.right_foot_pose.yaw     = src.PositionData.stRightFootPosition.yaw;
    des.position_data.left_foot_pose.x        = src.PositionData.stLeftFootPosition.x;
    des.position_data.left_foot_pose.y        = src.PositionData.stLeftFootPosition.y;
    des.position_data.left_foot_pose.z        = src.PositionData.stLeftFootPosition.z;
    des.position_data.left_foot_pose.roll     = src.PositionData.stLeftFootPosition.roll;
    des.position_data.left_foot_pose.pitch    = src.PositionData.stLeftFootPosition.pitch;
    des.position_data.left_foot_pose.yaw      = src.PositionData.stLeftFootPosition.yaw;

    return 0;
}


bool    WalkingMotionModule::GetReferenceStepDataServiceCallback(thormang3_walking_module_msgs::GetReferenceStepData::Request &req,
                                                                thormang3_walking_module_msgs::GetReferenceStepData::Response &res)
{
    PreviewControlWalking *prev_walking = PreviewControlWalkingMotionModule::GetInstance();

    StepData _refStepData;

    prev_walking->GetReferenceStepDatafotAddition(&_refStepData);

    StepDataToStepDataMsg(_refStepData, res.reference_step_data);

    return true;
}




bool    WalkingMotionModule::AddStepDataServiceCallback(thormang3_walking_module_msgs::AddStepDataArray::Request &req,
                                                        thormang3_walking_module_msgs::AddStepDataArray::Response &res)
{
    PreviewControlWalking *prev_walking = PreviewControlWalkingMotionModule::GetInstance();
    res.result = STEP_DATA_ERR::NO_ERROR;

    if(enable == false) {
        res.result |= STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE;
        std::string _status_msg = WALKING_STATUS_MSG::FAILED_TO_ADD_STEP_DATA_MSG;
        PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
        //status_msg_pub_.publish(status_msg);
        return true;
    }

    if(prev_walking->IsRunning() == true) {
        res.result |= STEP_DATA_ERR::ROBOT_IS_WALKING_NOW;
        std::string _status_msg  = WALKING_STATUS_MSG::FAILED_TO_ADD_STEP_DATA_MSG;
        PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
        return true;
    }

    StepData _stepData, _refStepData;
    std::vector<StepData> _req_stp_data_array;

    prev_walking->GetReferenceStepDatafotAddition(&_refStepData);

    for(int i = 0; i < req.step_data_array.size(); i++)
    {
        res.result |= StepDataMsgToStepData(req.step_data_array[i], _stepData);

        if(_stepData.TimeData.dAbsStepTime <= 0) {
            res.result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;
        }

        if(i != 0) {
            if(_stepData.TimeData.dAbsStepTime <= _req_stp_data_array[_req_stp_data_array.size() - 1].TimeData.dAbsStepTime) {
                res.result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;
            }
        }
        else {
            if(_stepData.TimeData.dAbsStepTime <= _refStepData.TimeData.dAbsStepTime) {
                res.result |= STEP_DATA_ERR::PROBLEM_IN_TIME_DATA;
            }
        }

        if(res.result != STEP_DATA_ERR::NO_ERROR) {
            std::string _status_msg = WALKING_STATUS_MSG::FAILED_TO_ADD_STEP_DATA_MSG;
            PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
            return true;
        }

        _req_stp_data_array.push_back(_stepData);
    }


    if(req.remove_existing_step_data == true) {
        int _exist_num_of_step_data = prev_walking->GetNumofRemainingUnreservedStepData();
        if(_exist_num_of_step_data != 0)
            for(int _remove_count  = 0; _remove_count < _exist_num_of_step_data; _remove_count++)
                prev_walking->EraseLastStepData();
    }

    for(unsigned int _i = 0; _i <_req_stp_data_array.size() ; _i++)
        prev_walking->AddStepData(_req_stp_data_array[_i]);

    if( req.auto_start == true) {
        prev_walking->Start();
    }

    return true;
}

bool    WalkingMotionModule::WalkingStartServiceCallback(thormang3_walking_module_msgs::WalkingStart::Request &req,
                                                        thormang3_walking_module_msgs::WalkingStart::Response &res)
{
    PreviewControlWalking *prev_walking = PreviewControlWalkingMotionModule::GetInstance();
    res.result = WALKING_START_ERR::NO_ERROR;

    if(enable == false) {
        res.result |= WALKING_START_ERR::NOT_ENABLED_WALKING_MODULE;
    }

    if(prev_walking->IsRunning() == true){
        res.result |= WALKING_START_ERR::ROBOT_IS_WALKING_NOW;
    }

    if(prev_walking->GetNumofRemainingUnreservedStepData() == 0){
        res.result |= WALKING_START_ERR::NO_STEP_DATA;
    }

    if(res.result == WALKING_START_ERR::NO_ERROR) {
        prev_walking->Start();
    }

    return true;
}

bool    WalkingMotionModule::IsRunningServiceCallback(thormang3_walking_module_msgs::IsRunning::Request &req,
                                                        thormang3_walking_module_msgs::IsRunning::Response &res)
{
    bool is_running = IsRunning();
    res.is_running = is_running;

    return true;
}
 */
//bool WalkingMotionModule::IsRunning()
//{
//return PreviewControlWalkingMotionModule::GetInstance()->IsRunning();
//    return false;
//}
/*
bool    WalkingMotionModule::SetBalanceParamServiceCallback(thormang3_walking_module_msgs::SetBalanceParam::Request  &req,
                                                            thormang3_walking_module_msgs::SetBalanceParam::Response &res)
{
    PreviewControlWalking *prev_walking = PreviewControlWalkingMotionModule::GetInstance();
    res.result = BALANCE_PARAM_ERR::NO_ERROR;

    if( enable == false)
        res.result |= BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE;

    if( balance_update_with_loop_ == true)  {
        res.result |= BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED;
    }

    if( ( req.balance_param.foot_roll_angle_time_constant  <= 0.0 )
            || ( req.balance_param.foot_pitch_angle_time_constant  <= 0.0 )
            || ( req.balance_param.foot_x_force_time_constant      <= 0.0 )
            || ( req.balance_param.foot_y_force_time_constant      <= 0.0 )
            || ( req.balance_param.foot_z_force_time_constant      <= 0.0 )
            || ( req.balance_param.foot_roll_torque_time_constant  <= 0.0 )
            || ( req.balance_param.foot_pitch_torque_time_constant <= 0.0 ) )
    {
        res.result |= BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE;
    }

    if(res.result == BALANCE_PARAM_ERR::NO_ERROR) {
        if(req.updating_duration <= (control_cycle_msec_ * 0.001) )
        {
            // under 8ms apply immediately
            SetBalanceParam(req.balance_param);
            std::string _status_msg = WALKING_STATUS_MSG::BALANCE_PARAM_SETTING_FINISH_MSG;
            PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
        }
        else {
            balance_update_duration_ = req.updating_duration;
            balance_update_sys_time_ = 0.0;
            balance_update_polynomial_coeff_.resize(6, 1);

            double tf = balance_update_duration_;
            Eigen::MatrixXd A(6,6), B(6, 1);
            A <<  0.0,   0.0,    0.0,   0.0,    0.0, 1.0,
                    0.0,   0.0,    0.0, 0.0,    1.0, 0.0,
                    0.0,   0.0,    0.0, 2.0,    0.0, 0.0,
                    tf*tf*tf*tf*tf,  tf*tf*tf*tf,     tf*tf*tf,     tf*tf,   tf, 1.0,
                    5.0*tf*tf*tf*tf,    4.0*tf*tf*tf,    3.0*tf*tf, 2.0*tf,     1.0, 0.0,
                    20.0*tf*tf*tf,      12.0*tf*tf,       6.0*tf,       2.0,        0.0, 0.0;

            B << 0, 0, 0, 1.0, 0, 0;
            balance_update_polynomial_coeff_ = A.inverse() * B;

            desired_balance_param_ = req.balance_param;

            previous_balance_param_.cob_x_offset_m                  = prev_walking->COB_X_MANUAL_ADJUSTMENT_M;
            previous_balance_param_.cob_y_offset_m                  = prev_walking->COB_Y_MANUAL_ADJUSTMENT_M;

            previous_balance_param_.hip_roll_swap_angle_rad         = prev_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD;

            previous_balance_param_.gyro_gain                       = prev_walking->WALK_STABILIZER_GAIN_RATIO;
            previous_balance_param_.foot_roll_angle_gain            = prev_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU;
            previous_balance_param_.foot_pitch_angle_gain           = prev_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU;

            previous_balance_param_.foot_x_force_gain               = prev_walking->BALANCE_X_GAIN_BY_FT;
            previous_balance_param_.foot_y_force_gain               = prev_walking->BALANCE_Y_GAIN_BY_FT;
            previous_balance_param_.foot_z_force_gain               = prev_walking->BALANCE_Z_GAIN_BY_FT;
            previous_balance_param_.foot_roll_torque_gain           = prev_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT;
            previous_balance_param_.foot_pitch_torque_gain          = prev_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT;


            previous_balance_param_.foot_roll_angle_time_constant   = prev_walking->BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU;
            previous_balance_param_.foot_pitch_angle_time_constant  = prev_walking->BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU;

            previous_balance_param_.foot_x_force_time_constant      = prev_walking->BALANCE_X_TIME_CONSTANT;
            previous_balance_param_.foot_y_force_time_constant      = prev_walking->BALANCE_Y_TIME_CONSTANT;
            previous_balance_param_.foot_z_force_time_constant      = prev_walking->BALANCE_Z_TIME_CONSTANT;
            previous_balance_param_.foot_roll_torque_time_constant  = prev_walking->BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT;
            previous_balance_param_.foot_pitch_torque_time_constant = prev_walking->BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT;

            balance_update_with_loop_ = true;

            std::string _status_msg = WALKING_STATUS_MSG::BALANCE_PARAM_SETTING_START_MSG;
            PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
        }
    }

    return true;
}
//
void    WalkingMotionModule::SetBalanceParam(thormang3_walking_module_msgs::BalanceParam& balance_param_msg)
{
    PreviewControlWalking *prev_walking = PreviewControlWalkingMotionModule::GetInstance();

    prev_walking->COB_X_MANUAL_ADJUSTMENT_M = balance_param_msg.cob_x_offset_m;
    prev_walking->COB_Y_MANUAL_ADJUSTMENT_M = balance_param_msg.cob_y_offset_m;
    prev_walking->HIP_ROLL_FEEDFORWARD_ANGLE_RAD = balance_param_msg.hip_roll_swap_angle_rad;

    prev_walking->WALK_STABILIZER_GAIN_RATIO = balance_param_msg.gyro_gain;
    prev_walking->BALANCE_X_GAIN     = +1.0*20.30*0.625*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
    prev_walking->BALANCE_Y_GAIN     = -1.0*20.30*0.75*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
    prev_walking->BALANCE_Z_GAIN     =    0.0*2.0*(prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

    prev_walking->BALANCE_PITCH_GAIN = -1.0*0.10*0.5 *(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;
    prev_walking->BALANCE_ROLL_GAIN  = -1.0*0.10*0.75*(1.0 - prev_walking->FORCE_MOMENT_DISTRIBUTION_RATIO)*prev_walking->WALK_STABILIZER_GAIN_RATIO;

    prev_walking->BALANCE_ANKLE_ROLL_GAIN_BY_IMU  = balance_param_msg.foot_roll_angle_gain;
    prev_walking->BALANCE_ANKLE_PITCH_GAIN_BY_IMU = balance_param_msg.foot_pitch_angle_gain;

    prev_walking->BALANCE_X_GAIN_BY_FT              = balance_param_msg.foot_x_force_gain;
    prev_walking->BALANCE_Y_GAIN_BY_FT              = balance_param_msg.foot_y_force_gain;
    prev_walking->BALANCE_Z_GAIN_BY_FT              = balance_param_msg.foot_z_force_gain;
    prev_walking->BALANCE_RIGHT_ROLL_GAIN_BY_FT     = balance_param_msg.foot_roll_torque_gain;
    prev_walking->BALANCE_LEFT_ROLL_GAIN_BY_FT      = balance_param_msg.foot_roll_torque_gain;
    prev_walking->BALANCE_RIGHT_PITCH_GAIN_BY_FT    = balance_param_msg.foot_pitch_torque_gain;
    prev_walking->BALANCE_LEFT_PITCH_GAIN_BY_FT     = balance_param_msg.foot_pitch_torque_gain;

    prev_walking->BALANCE_ANKLE_ROLL_TIME_CONSTANT_BY_IMU  = balance_param_msg.foot_roll_angle_time_constant;
    prev_walking->BALANCE_ANKLE_PITCH_TIME_CONSTANT_BY_IMU = balance_param_msg.foot_pitch_angle_time_constant;

    prev_walking->BALANCE_X_TIME_CONSTANT                   = balance_param_msg.foot_x_force_time_constant;
    prev_walking->BALANCE_Y_TIME_CONSTANT                   = balance_param_msg.foot_y_force_time_constant;
    prev_walking->BALANCE_Z_TIME_CONSTANT                   = balance_param_msg.foot_z_force_time_constant;
    prev_walking->BALANCE_RIGHT_ANKLE_ROLL_TIME_CONSTANT    = balance_param_msg.foot_roll_torque_time_constant;
    prev_walking->BALANCE_LEFT_ANKLE_ROLL_TIME_CONSTANT     = balance_param_msg.foot_roll_torque_time_constant;
    prev_walking->BALANCE_RIGHT_ANKLE_PITCH_TIME_CONSTANT   = balance_param_msg.foot_pitch_torque_time_constant;
    prev_walking->BALANCE_LEFT_ANKLE_PITCH_TIME_CONSTANT    = balance_param_msg.foot_pitch_torque_time_constant;
}

bool    WalkingMotionModule::RemoveExistingStepDataServiceCallback(thormang3_walking_module_msgs::RemoveExistingStepData::Request  &req,
                                                                    thormang3_walking_module_msgs::RemoveExistingStepData::Response &res)
{
    PreviewControlWalking *prev_walking = PreviewControlWalkingMotionModule::GetInstance();

    res.result = REMOVE_STEP_DATA_ERR::NO_ERROR;

    if(IsRunning())
        res.result |= REMOVE_STEP_DATA_ERR::ROBOT_IS_WALKING_NOW;
    else {
        int _exist_num_of_step_data = prev_walking->GetNumofRemainingUnreservedStepData();
        if(_exist_num_of_step_data != 0)
            for(int _remove_count  = 0; _remove_count < _exist_num_of_step_data; _remove_count++)
                prev_walking->EraseLastStepData();
    }
    return true;
}
 */

void WalkingMotionModule::IMUDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // PreviewControlWalkingMotionModule::GetInstance()->current_gyro_roll_rad_per_sec  = -1.0*(msg->angular_velocity.x);
    // PreviewControlWalkingMotionModule::GetInstance()->current_gyro_pitch_rad_per_sec = -1.0*(msg->angular_velocity.y);


    Eigen::Quaterniond imu_quat;
    tf::quaternionMsgToEigen(msg->orientation, imu_quat);

    Eigen::MatrixXd imu_mat = (RX_PI_3x3*(imu_quat.toRotationMatrix()))*RZ_PI_3x3;

    double roll  = atan2( imu_mat.coeff(2,1), imu_mat.coeff(2,2));
    double pitch = atan2(-imu_mat.coeff(2,0), sqrt(powDI(imu_mat.coeff(2,1), 2) + powDI(imu_mat.coeff(2,2), 2)));
    double yaw   = atan2( imu_mat.coeff(1,0), imu_mat.coeff(0,0));

    // PreviewControlWalkingMotionModule::GetInstance()->current_imu_roll_rad = roll;
    // PreviewControlWalkingMotionModule::GetInstance()->current_imu_pitch_rad = pitch;
}

void WalkingMotionModule::walkingCommandCallback(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "start")
        startWalking();
    else if(msg->data == "stop")
        Stop();
    else if(msg->data == "balance on")
        walking_param_.balance_enable = true;
    else if(msg->data == "balance off")
        walking_param_.balance_enable = false;
}

void WalkingMotionModule::walkingParameterCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr &msg)
{
    walking_param_ = *msg;

    saveWalkingParam(param_path_);
}

bool WalkingMotionModule::getWalkigParameterCallback(op3_walking_module_msgs::GetWalkingParam::Request &req, op3_walking_module_msgs::GetWalkingParam::Response &res)
{
    res.parameters = walking_param_;

    return true;
}

double WalkingMotionModule::wSin(double time, double period, double period_shift, double mag, double mag_shift)
{
    return mag * sin(2 * M_PI / period * time - period_shift) + mag_shift;
}

// m, rad
bool WalkingMotionModule::computeIK(double *out, double pos_x, double pos_y, double pos_z, double ori_roll, double ori_pitch, double ori_yaw)
{
    double THIGH_LENGTH = 93.0 * 0.001; //m
    double CALF_LENGTH = 93.0 * 0.001; //m
    double ANKLE_LENGTH = 33.5 * 0.001; //m
    double LEG_LENGTH = 219.5 * 0.001; //m (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)

    Eigen::MatrixXd Tad, Tda, Tcd, Tdc, Tac;
    Eigen::Vector3d vec;
    double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;

    // make transform matrix
    // Tad.SetTransform(Point3D(pos_x, pos_y, pos_z - LEG_LENGTH), Vector3D(ori_roll * 180.0 / M_PI, ori_pitch * 180.0 / PI, ori_yaw * 180.0 / PI));
    // Tad = transformationXYZRPY(pos_x, pos_y, pos_z - LEG_LENGTH, ori_roll, ori_pitch, ori_yaw);
    Tad = transformationXYZRPY(pos_x, pos_y, pos_z, ori_roll, ori_pitch, ori_yaw);

    vec << pos_x + Tad.coeff(0, 2) * ANKLE_LENGTH
            , pos_y + Tad.coeff(1, 2) * ANKLE_LENGTH
            , (pos_z - LEG_LENGTH) + Tad.coeff(2, 2) * ANKLE_LENGTH;

    // Get Knee
    _Rac = vec.norm();
    _Acos = acos((_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH));
    if(std::isnan(_Acos) == 1)
    {
        std::cout << "fail ik - 1" << std::endl;
        return false;
    }
    *(out + 3) = _Acos;

    // Get Ankle Roll
    Tda = InverseTransformation(Tad);
    // if(Tda.Inverse() == false)
    //     return false;
    // _k = sqrt(Tda.m[7] * Tda.m[7] + Tda.m[11] * Tda.m[11]);
    // _l = sqrt(Tda.m[7] * Tda.m[7] + (Tda.m[11] - ANKLE_LENGTH) * (Tda.m[11] - ANKLE_LENGTH));
    double _tda_y = Tda.coeff(1, 3);
    double _tda_z = Tda.coeff(2, 3);
    _k = sqrt(_tda_y * _tda_y + _tda_z * _tda_z);
    _l = sqrt(_tda_y * _tda_y + (_tda_z - ANKLE_LENGTH) * (_tda_z - ANKLE_LENGTH));
    _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2 * _l * ANKLE_LENGTH);
    if(_m > 1.0)
        _m = 1.0;
    else if(_m < -1.0)
        _m = -1.0;
    _Acos = acos(_m);
    if(std::isnan(_Acos) == 1)
    {
        std::cout << "fail ik - 2" << std::endl;
        return false;
    }
    if(_tda_y < 0.0)
        *(out + 5) = -_Acos;
    else
        *(out + 5) = _Acos;

    // Get Hip Yaw
    // Tcd.SetTransform(Point3D(0, 0, -ANKLE_LENGTH), Vector3D(*(out + 5) * 180.0 / PI, 0, 0));
    Tcd = transformationXYZRPY(0.0, 0.0, - ANKLE_LENGTH, *(out + 5), 0.0, 0.0);
    //Tdc = Tcd;
    //if(Tdc.Inverse() == false)
    //    return false;
    Tdc = InverseTransformation(Tcd);
    Tac = Tad * Tdc;
    // _Atan = atan2(-Tac.m[1] , Tac.m[5]);
    _Atan = atan2(-Tac.coeff(0, 1) , Tac.coeff(1, 1));
    if(std::isinf(_Atan) == 1)
    {
        std::cout << "fail ik - 3" << std::endl;
        return false;
    }
    *(out) = _Atan;

    // Get Hip Roll
    // _Atan = atan2(Tac.m[9], -Tac.m[1] * sin(*(out)) + Tac.m[5] * cos(*(out)));
    _Atan = atan2(Tac.coeff(2, 1), -Tac.coeff(0, 1) * sin(*(out)) + Tac.coeff(1, 1) * cos(*(out)));
    if(std::isinf(_Atan) == 1)
    {
        std::cout << "fail ik - 4" << std::endl;
        return false;
    }
    *(out + 1) = _Atan;

    // Get Hip Pitch and Ankle Pitch
    // _Atan = atan2(Tac.m[2] * cos(*(out)) + Tac.m[6] * sin(*(out)), Tac.m[0] * cos(*(out)) + Tac.m[4] * sin(*(out)));
    _Atan = atan2(Tac.coeff(0, 2) * cos(*(out)) + Tac.coeff(1, 2) * sin(*(out)), Tac.coeff(0, 0) * cos(*(out)) + Tac.coeff(1, 0) * sin(*(out)));
    if(std::isinf(_Atan) == 1)
    {
        std::cout << "fail ik - 5" << std::endl;
        return false;
    }
    _theta = _Atan;
    _k = sin(*(out + 3)) * CALF_LENGTH;
    _l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
    _m = cos(*(out)) * vec.x() + sin(*(out)) * vec.y();
    _n = cos(*(out + 1)) * vec.z() + sin(*(out)) * sin(*(out + 1)) * vec.x() - cos(*(out)) * sin(*(out + 1)) * vec.y();
    _s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
    _c = (_n - _k * _s) / _l;
    _Atan = atan2(_s, _c);
    if(std::isinf(_Atan) == 1)
    {
        std::cout << "fail ik - 6" << std::endl;
        return false;
    }
    *(out + 2) = _Atan;
    *(out + 4) = _theta - *(out + 3) - *(out + 2);

    return true;
}

void WalkingMotionModule::updateTimeParam()
{
    m_PeriodTime = walking_param_.period_time;
    m_DSP_Ratio = walking_param_.dsp_ratio;
    m_SSP_Ratio = 1 - m_DSP_Ratio;

    m_X_Swap_PeriodTime = m_PeriodTime / 2;
    m_X_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
    m_Y_Swap_PeriodTime = m_PeriodTime;
    m_Y_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
    m_Z_Swap_PeriodTime = m_PeriodTime / 2;
    m_Z_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio / 2;
    m_A_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;

    m_SSP_Time = m_PeriodTime * m_SSP_Ratio;
    m_SSP_Time_Start_L = (1 - m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_End_L = (1 + m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_Start_R = (3 - m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_End_R = (3 + m_SSP_Ratio) * m_PeriodTime / 4;

    m_Phase_Time1 = (m_SSP_Time_End_L + m_SSP_Time_Start_L) / 2;
    m_Phase_Time2 = (m_SSP_Time_Start_R + m_SSP_Time_End_L) / 2;
    m_Phase_Time3 = (m_SSP_Time_End_R + m_SSP_Time_Start_R) / 2;

    // m_Pelvis_Offset = PELVIS_OFFSET*MX28::RATIO_ANGLE2VALUE;
    m_Pelvis_Offset = walking_param_.pelvis_offset * deg2rad;
    m_Pelvis_Swing = m_Pelvis_Offset * 0.35;
    m_Arm_Swing_Gain = walking_param_.arm_swing_gain;
}

void WalkingMotionModule::updateMovementParam()
{
    // Forward/Back
    m_X_Move_Amplitude = walking_param_.x_move_amplitude;
    m_X_Swap_Amplitude = walking_param_.x_move_amplitude * walking_param_.step_fb_ratio;

    // Right/Left
    m_Y_Move_Amplitude = walking_param_.y_move_amplitude / 2;
    if(m_Y_Move_Amplitude > 0)
        m_Y_Move_Amplitude_Shift = m_Y_Move_Amplitude;
    else
        m_Y_Move_Amplitude_Shift = -m_Y_Move_Amplitude;
    m_Y_Swap_Amplitude = walking_param_.y_swap_amplitude + m_Y_Move_Amplitude_Shift * 0.04;

    m_Z_Move_Amplitude = walking_param_.z_move_amplitude / 2;
    m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2;
    m_Z_Swap_Amplitude = walking_param_.z_swap_amplitude;
    m_Z_Swap_Amplitude_Shift = m_Z_Swap_Amplitude;

    // Direction
    if(walking_param_.move_aim_on == false)
    {
        m_A_Move_Amplitude = walking_param_.angle_move_amplitude * deg2rad / 2;
        if(m_A_Move_Amplitude > 0)
            m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
        else
            m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
    }
    else
    {
        m_A_Move_Amplitude = -walking_param_.angle_move_amplitude * deg2rad / 2;
        if(m_A_Move_Amplitude > 0)
            m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
        else
            m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
    }
}

void WalkingMotionModule::updatePoseParam()
{
    m_X_Offset = walking_param_.init_x_offset;
    m_Y_Offset = walking_param_.init_y_offset;
    m_Z_Offset = walking_param_.init_z_offset;
    m_R_Offset = walking_param_.init_roll_offset  * deg2rad;
    m_P_Offset = walking_param_.init_pitch_offset * deg2rad;
    m_A_Offset = walking_param_.init_yaw_offset   * deg2rad;
    m_Hip_Pitch_Offset = walking_param_.hip_pitch_offset * deg2rad;
}

void WalkingMotionModule::startWalking()
{
    m_Ctrl_Running = true;
    m_Real_Running = true;

    PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start walking");
}

void WalkingMotionModule::Stop()
{
    m_Ctrl_Running = false;
    PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Stop walking");
}

bool WalkingMotionModule::IsRunning()
{
    return m_Real_Running;
}

// default [angle : radian, length : m]
void WalkingMotionModule::Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors)
{
    if(enable == false)
        return;

    Pose3D _swap, _right_leg_move, _left_leg_move;
    double _pelvis_offset_r, _pelvis_offset_l;
    double angle[14], ep[12];
    double offset;
    double TIME_UNIT = control_cycle_msec_;
    //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL, L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING, L_ARM_SWING
    int dir[14]          = {   -1,        -1,          1,         1,         -1,            1,          -1,        -1,         -1,         -1,         1,            1,           1,           -1      };
    double initAngle[14] = {   0.0,       0.0,        0.0,       0.0,        0.0,          0.0,         0.0,       0.0,        0.0,        0.0,       0.0,          0.0,       -48.345,       41.313    };
    double outValue[14];

    // Update walk parameters
    if(m_Time == 0)
    {
        // set present joint angle to goal
        for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
        {
            std::string _joint_name = state_iter->first;
            int _index = joint_table_[_joint_name];

            Dynamixel *_dxl = NULL;
            std::map<std::string, Dynamixel*>::iterator _dxl_it = dxls.find(_joint_name);
            if(_dxl_it != dxls.end())
                _dxl = _dxl_it->second;
            else
                continue;

            goal_position_.coeffRef(0, _index) = _dxl->dxl_state->goal_position;
            result[_joint_name]->goal_position = goal_position_.coeff(0, _index);
        }

        updateTimeParam();
        m_Phase = PHASE0;
        if(m_Ctrl_Running == false)
        {
            if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
            {
                m_Real_Running = false;
            }
            else
            {
                // init walking param
                walking_param_.x_move_amplitude = 0;
                walking_param_.y_move_amplitude = 0;
                walking_param_.angle_move_amplitude = 0;
            }
        }
    }
    else if(m_Time >= (m_Phase_Time1 - TIME_UNIT/2) && m_Time < (m_Phase_Time1 + TIME_UNIT/2))
    {
        updateMovementParam();
        m_Phase = PHASE1;
    }
    else if(m_Time >= (m_Phase_Time2 - TIME_UNIT/2) && m_Time < (m_Phase_Time2 + TIME_UNIT/2))
    {
        updateTimeParam();

        m_Time = m_Phase_Time2;
        m_Phase = PHASE2;
        if(m_Ctrl_Running == false)
        {
            if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
            {
                m_Real_Running = false;
            }
            else
            {
                // init walking param
                walking_param_.x_move_amplitude = 0;
                walking_param_.y_move_amplitude = 0;
                walking_param_.angle_move_amplitude = 0;
            }
        }
    }
    else if(m_Time >= (m_Phase_Time3 - TIME_UNIT/2) && m_Time < (m_Phase_Time3 + TIME_UNIT/2))
    {
        updateMovementParam();
        m_Phase = PHASE3;
    }

    updatePoseParam();

    // Compute endpoints
    _swap.x = wSin(m_Time, m_X_Swap_PeriodTime, m_X_Swap_Phase_Shift, m_X_Swap_Amplitude, m_X_Swap_Amplitude_Shift);
    _swap.y = wSin(m_Time, m_Y_Swap_PeriodTime, m_Y_Swap_Phase_Shift, m_Y_Swap_Amplitude, m_Y_Swap_Amplitude_Shift);
    _swap.z = wSin(m_Time, m_Z_Swap_PeriodTime, m_Z_Swap_Phase_Shift, m_Z_Swap_Amplitude, m_Z_Swap_Amplitude_Shift);
    _swap.roll = 0.0;
    _swap.pitch = 0.0;
    _swap.yaw = 0.0;

    if(m_Time <= m_SSP_Time_Start_L)
    {
        _left_leg_move.x = wSin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        _left_leg_move.y = wSin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        _left_leg_move.z = wSin(m_SSP_Time_Start_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        _left_leg_move.yaw = wSin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        _right_leg_move.x = wSin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        _right_leg_move.y = wSin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        _right_leg_move.z = wSin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        _right_leg_move.yaw = wSin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        _pelvis_offset_l = 0;
        _pelvis_offset_r = 0;
    }
    else if(m_Time <= m_SSP_Time_End_L)
    {
        _left_leg_move.x = wSin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        _left_leg_move.y = wSin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        _left_leg_move.z = wSin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        _left_leg_move.yaw = wSin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        _right_leg_move.x = wSin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        _right_leg_move.y = wSin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        _right_leg_move.z = wSin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        _right_leg_move.yaw = wSin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        _pelvis_offset_l = wSin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
        _pelvis_offset_r = wSin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, -m_Pelvis_Offset / 2, -m_Pelvis_Offset / 2);
    }
    else if(m_Time <= m_SSP_Time_Start_R)
    {
        _left_leg_move.x = wSin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        _left_leg_move.y = wSin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        _left_leg_move.z = wSin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        _left_leg_move.yaw = wSin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        _right_leg_move.x = wSin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        _right_leg_move.y = wSin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        _right_leg_move.z = wSin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        _right_leg_move.yaw = wSin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        _pelvis_offset_l = 0;
        _pelvis_offset_r = 0;
    }
    else if(m_Time <= m_SSP_Time_End_R)
    {
        _left_leg_move.x = wSin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + M_PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        _left_leg_move.y = wSin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + M_PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        _left_leg_move.z = wSin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        _left_leg_move.yaw = wSin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + M_PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        _right_leg_move.x = wSin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + M_PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        _right_leg_move.y = wSin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + M_PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        _right_leg_move.z = wSin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        _right_leg_move.yaw = wSin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + M_PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        _pelvis_offset_l = wSin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Pelvis_Offset / 2, m_Pelvis_Offset / 2);
        _pelvis_offset_r = wSin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
    }
    else
    {
        _left_leg_move.x = wSin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + M_PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        _left_leg_move.y = wSin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + M_PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        _left_leg_move.z = wSin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        _left_leg_move.yaw = wSin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + M_PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        _right_leg_move.x = wSin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * M_PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + M_PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        _right_leg_move.y = wSin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * M_PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + M_PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        _right_leg_move.z = wSin(m_SSP_Time_End_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * M_PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        _right_leg_move.yaw = wSin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * M_PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + M_PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        _pelvis_offset_l = 0;
        _pelvis_offset_r = 0;
    }

    _left_leg_move.roll = 0;
    _left_leg_move.pitch = 0;
    _right_leg_move.roll = 0;
    _right_leg_move.pitch = 0;

    double _leg_length = op3_kd_->thigh_length_m + op3_kd_->calf_length_m + op3_kd_->ankle_length_m;
    // std::cout << "Leg length : "  << _leg_length << std::endl;

    // mm, rad
    ep[0] = _swap.x + _right_leg_move.x + m_X_Offset;
    ep[1] = _swap.y + _right_leg_move.y - m_Y_Offset / 2;
    ep[2] = _swap.z + _right_leg_move.z + m_Z_Offset - _leg_length;
    ep[3] = _swap.roll + _right_leg_move.roll - m_R_Offset / 2;
    ep[4] = _swap.pitch + _right_leg_move.pitch + m_P_Offset;
    ep[5] = _swap.yaw + _right_leg_move.yaw - m_A_Offset / 2;
    ep[6] = _swap.x + _left_leg_move.x + m_X_Offset;
    ep[7] = _swap.y + _left_leg_move.y + m_Y_Offset / 2;
    ep[8] = _swap.z + _left_leg_move.z + m_Z_Offset - _leg_length;
    ep[9] = _swap.roll + _left_leg_move.roll + m_R_Offset / 2;
    ep[10] = _swap.pitch + _left_leg_move.pitch + m_P_Offset;
    ep[11] = _swap.yaw + _left_leg_move.yaw + m_A_Offset / 2;

    // Compute body swing
    if(m_Time <= m_SSP_Time_End_L)
    {
        m_Body_Swing_Y = -ep[7];
        m_Body_Swing_Z = ep[8];
    }
    else
    {
        m_Body_Swing_Y = -ep[1];
        m_Body_Swing_Z = ep[2];
    }
    m_Body_Swing_Z -= _leg_length;

    // Compute arm swing
    if(m_X_Move_Amplitude == 0)
    {
        angle[12] = 0; // Right
        angle[13] = 0; // Left
    }
    else
    {
        angle[12] = wSin(m_Time, m_PeriodTime, M_PI * 1.5, -m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
        angle[13] = wSin(m_Time, m_PeriodTime, M_PI * 1.5, m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
    }

    if(m_Real_Running == true)
    {
        m_Time += TIME_UNIT;
        if(m_Time >= m_PeriodTime)
            m_Time = 0;
    }

    // right leg
    if(op3_kd_->InverseKinematicsforRightLeg(&angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == false)
    {
        printf("IK not Solved EPR : %f %f %f %f %f %f\n", ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]);
        return;
    }

    if(op3_kd_->InverseKinematicsforLeftLeg(&angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == false)
    {
        printf("IK not Solved EPL : %f %f %f %f %f %f\n", ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]);
        return;
    }

    // Compute angles
    /*
    if(computeIK(&angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == false)
    {
        std::cout << "fail ik - right" << std::endl;
        printf("IK not Solved EPR[R] : %f %f %f %f %f %f\n", ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]);
        printf("Result : %f %f %f %f %f %f\n", angle[0], angle[1], angle[2], angle[3], angle[4], angle[5]);
        return;
    }
    else
        printf("Result[R 1] : %f %f %f %f %f %f\n", angle[0], angle[1], angle[2], angle[3], angle[4], angle[5]);

    if (computeIK(&angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == false)
    {
        std::cout << "fail ik" << std::endl;
        printf("IK not Solved EPL[L] : %f %f %f %f %f %f\n", ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]);
        printf("Result : %f %f %f %f %f %f\n", angle[6], angle[7], angle[8], angle[9], angle[10], angle[11]);
        return;
    }
    else
        printf("Result[L] : %f %f %f %f %f %f\n", angle[6], angle[7], angle[8], angle[9], angle[10], angle[11]);
     */


    // Compute dxls angle
    for(int i=0; i<14; i++)
    {
        // offset : rad
        // offset = (double)dir[i] * angle[i];
        offset = angle[i];

        if(i == 1) // R_HIP_ROLL
            offset += (double)dir[i] * _pelvis_offset_r;
        else if(i == 7) // L_HIP_ROLL
            offset += (double)dir[i] * _pelvis_offset_l;
        else if(i == 2 || i == 8) // R_HIP_PITCH or L_HIP_PITCH
            offset -= (double)dir[i] * m_Hip_Pitch_Offset;

        // outValue : rad, initAngle : deg
        outValue[i] = initAngle[i] * deg2rad + offset;
        //goal_position_.coeffRef(0, i) = initAngle[i] + (int)offset;
    }
    // printf("Out value[R] : %f %f %f %f %f %f\n", outValue[0], outValue[1], outValue[2], outValue[3], outValue[4], outValue[5]);

    //std::cout << "Gyro : [" <<  sensors["gyro_x"] << ", " << sensors["gyro_y"] << "]\n";
    // adjust balance offset
    if(walking_param_.balance_enable == true)
    {
        double rlGyroErr = sensors["gyro_x"] * deg2rad; // MotionStatus::RL_GYRO;
        double fbGyroErr = sensors["gyro_y"] * deg2rad; // MotionStatus::FB_GYRO;

        outValue[1] += (int)(dir[1] * rlGyroErr * walking_param_.balance_hip_roll_gain); // R_HIP_ROLL
        outValue[7] += (int)(dir[7] * rlGyroErr * walking_param_.balance_hip_roll_gain); // L_HIP_ROLL

        outValue[3] -= (int)(dir[3] * fbGyroErr * walking_param_.balance_knee_gain); // R_KNEE
        outValue[9] -= (int)(dir[9] * fbGyroErr * walking_param_.balance_knee_gain); // L_KNEE

        outValue[4] -= (int)(dir[4] * fbGyroErr * walking_param_.balance_ankle_pitch_gain); // R_ANKLE_PITCH
        outValue[10] -= (int)(dir[10] * fbGyroErr * walking_param_.balance_ankle_pitch_gain); // L_ANKLE_PITCH

        outValue[5] -= (int)(dir[5] * rlGyroErr * walking_param_.balance_ankle_roll_gain); // R_ANKLE_ROLL
        outValue[11] -= (int)(dir[11] * rlGyroErr * walking_param_.balance_ankle_roll_gain); // L_ANKLE_ROLL
    }

    // set goal position
    for(int idx = 0; idx < 14; idx++)
    {
        goal_position_.coeffRef(0, idx) = outValue[idx];
    }
    // head joint
    //goal_position_.coeffRef(0, joint_table_["head_pan"]) = A_MOVE_AMPLITUDE;

    // set result
    for(std::map<std::string, DynamixelState *>::iterator state_iter = result.begin(); state_iter != result.end(); state_iter++)
    {
        std::string _joint_name = state_iter->first;
        int _index = joint_table_[_joint_name];

        result[_joint_name]->goal_position = goal_position_.coeff(0, _index);
    }

    // pid gain
    // for(int id = JointData::ID_R_HIP_YAW; id <= JointData::ID_L_ANKLE_ROLL; id++)
    // {
    //     m_Joint.SetPGain(id, P_GAIN);
    //     m_Joint.SetIGain(id, I_GAIN);
    //     m_Joint.SetDGain(id, D_GAIN);
    // }
}

void WalkingMotionModule::loadWalkingParam(const std::string &path)
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
    walking_param_.init_x_offset              = doc["x_offset"].as<double>();
    walking_param_.init_y_offset              = doc["y_offset"].as<double>();
    walking_param_.init_z_offset              = doc["z_offset"].as<double>();
    walking_param_.init_roll_offset           = doc["roll_offset"].as<double>();
    walking_param_.init_pitch_offset          = doc["pitch_offset"].as<double>();
    walking_param_.init_yaw_offset            = doc["yaw_offset"].as<double>();
    walking_param_.hip_pitch_offset           = doc["hip_pitch_offset"].as<double>();
    // time
    walking_param_.period_time                = doc["period_time"].as<double>();
    walking_param_.dsp_ratio                  = doc["dsp_ratio"].as<double>();
    walking_param_.step_fb_ratio              = doc["step_forward_back_ratio"].as<double>();
    // walking
    // walking_param_.x_move_amplitude
    // walking_param_.y_move_amplitude
    walking_param_.z_move_amplitude           = doc["foot_height"].as<double>();
    // walking_param_.angle_move_amplitude
    // walking_param_.move_aim_on
    // balance
    // walking_param_.balance_enable
    walking_param_.balance_hip_roll_gain      = doc["balance_hip_roll_gain"].as<double>();
    walking_param_.balance_knee_gain          = doc["balance_knee_gain"].as<double>();
    walking_param_.balance_ankle_roll_gain    = doc["balance_ankle_roll_gain"].as<double>();
    walking_param_.balance_ankle_pitch_gain   = doc["balance_ankle_pitch_gain"].as<double>();
    walking_param_.y_swap_amplitude           = doc["swing_right_left"].as<double>();
    walking_param_.z_swap_amplitude           = doc["swing_top_down"].as<double>();
    walking_param_.pelvis_offset              = doc["pelvis_offset"].as<double>();
    walking_param_.arm_swing_gain             = doc["arm_swing_gain"].as<double>();

    // gain
    walking_param_.p_gain = doc["p_gain"].as<int>();
    walking_param_.i_gain = doc["i_gain"].as<int>();
    walking_param_.d_gain = doc["d_gain"].as<int>();
}

void WalkingMotionModule::saveWalkingParam(std::string &path)
{
    YAML::Emitter _out;

    _out << YAML::BeginMap;
    _out << YAML::Key << "x_offset"                  << YAML::Value << walking_param_.init_x_offset;
    _out << YAML::Key << "y_offset"                  << YAML::Value << walking_param_.init_y_offset;
    _out << YAML::Key << "z_offset"                  << YAML::Value << walking_param_.init_z_offset;
    _out << YAML::Key << "roll_offset"               << YAML::Value << walking_param_.init_roll_offset;
    _out << YAML::Key << "pitch_offset"              << YAML::Value << walking_param_.init_pitch_offset;
    _out << YAML::Key << "yaw_offset"                << YAML::Value << walking_param_.init_yaw_offset;
    _out << YAML::Key << "hip_pitch_offset"          << YAML::Value << walking_param_.hip_pitch_offset;
    _out << YAML::Key << "period_time"               << YAML::Value << walking_param_.period_time;
    _out << YAML::Key << "dsp_ratio"                 << YAML::Value << walking_param_.dsp_ratio;
    _out << YAML::Key << "step_forward_back_ratio"   << YAML::Value << walking_param_.step_fb_ratio;
    _out << YAML::Key << "foot_height"               << YAML::Value << walking_param_.z_move_amplitude;
    _out << YAML::Key << "swing_right_left"          << YAML::Value << walking_param_.y_swap_amplitude;
    _out << YAML::Key << "swing_top_down"            << YAML::Value << walking_param_.z_swap_amplitude;
    _out << YAML::Key << "pelvis_offset"             << YAML::Value << walking_param_.pelvis_offset;
    _out << YAML::Key << "arm_swing_gain"            << YAML::Value << walking_param_.arm_swing_gain;
    _out << YAML::Key << "balance_knee_gain"         << YAML::Value << walking_param_.balance_hip_roll_gain;
    _out << YAML::Key << "balance_ankle_pitch_gain"  << YAML::Value << walking_param_.balance_knee_gain;
    _out << YAML::Key << "balance_hip_roll_gain"     << YAML::Value << walking_param_.balance_ankle_roll_gain;
    _out << YAML::Key << "balance_ankle_roll_gain"   << YAML::Value << walking_param_.balance_ankle_pitch_gain;

    _out << YAML::Key << "p_gain"                    << YAML::Value << walking_param_.p_gain;
    _out << YAML::Key << "i_gain"                    << YAML::Value << walking_param_.i_gain;
    _out << YAML::Key << "d_gain"                    << YAML::Value << walking_param_.d_gain;
    _out << YAML::EndMap;

    // output to file
    std::ofstream fout(path.c_str());
    fout << _out.c_str();
}
