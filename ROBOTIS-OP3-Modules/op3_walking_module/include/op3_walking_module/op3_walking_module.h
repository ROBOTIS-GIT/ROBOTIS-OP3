/*
 * op3_walking_module.h
 *
 *  Created on: 2016. 5. 16.
 *      Author: JungKM
 */

#ifndef OP3_WALKING_MODULE_H_
#define OP3_WALKING_MODULE_H_

#include "op3_walking_parameter.h"

#include <math.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include "robotis_framework_common/MotionModule.h"
#include "robotis_math/RobotisMath.h"
#include "robotis_math/RobotisTrajectoryCalculator.h"
#include "op3_kinematics_dynamics/OP3KinematicsDynamics.h"

#include "robotis_controller_msgs/StatusMsg.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/SetWalkingParam.h"

namespace ROBOTIS
{

typedef struct
{
    double x, y, z;
} Position3D;

typedef struct
{
    double x, y, z, roll, pitch, yaw;
} Pose3D;

class WalkingMotionModule : public MotionModule
{
private:
    static WalkingMotionModule *unique_instance_;
    OP3KinematicsDynamics* op3_kd_;

    int             control_cycle_msec_;
    std::string     param_path_;
    boost::thread   queue_thread_;
    boost::mutex    publish_mutex_;

    Eigen::MatrixXd RX_PI_3x3, RZ_PI_3x3;
    Eigen::MatrixXd desired_matrix_g_to_cob_;
    Eigen::MatrixXd desired_matrix_g_to_rfoot_;
    Eigen::MatrixXd desired_matrix_g_to_lfoot_;

    WalkingMotionModule();

    void QueueThread();

    /* ROS Topic Publish Functions */
    ros::Publisher robot_pose_pub_;
    ros::Publisher status_msg_pub_;

    Eigen::MatrixXd calc_joint_tra_;

    Eigen::MatrixXd target_position_;
    Eigen::MatrixXd goal_position_;
    Eigen::MatrixXd init_position_;
    Eigen::MatrixXi joint_axis_direction_;
    std::map<std::string, int> joint_table_;
    int walking_state_;
    enum
    {
        WalkingDisable = 0,
        WalkingEnable = 1,
        WalkingInitPose = 2,
        WalkingReady = 3
    };
    bool DEBUG;
    int init_count_;

    op3_walking_module_msgs::WalkingParam walking_param_;

    // thormang3_walking_module_msgs::RobotPose  robot_pose_msg_;
    // bool    balance_update_with_loop_;
    // double  balance_update_duration_;
    // double  balance_update_sys_time_;
    // Eigen::MatrixXd balance_update_polynomial_coeff_;
    // thormang3_walking_module_msgs::BalanceParam previous_balance_param_;
    // thormang3_walking_module_msgs::BalanceParam current_balance_param_;
    // thormang3_walking_module_msgs::BalanceParam desired_balance_param_;

    void    PublishRobotPose(void);

    void    PublishStatusMsg(unsigned int type, std::string msg);

    /* ROS Topic Callback Functions */
    void    IMUDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void    walkingCommandCallback(const std_msgs::String::ConstPtr &msg);
    void    walkingParameterCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr &msg);
    bool    getWalkigParameterCallback(op3_walking_module_msgs::GetWalkingParam::Request &req, op3_walking_module_msgs::GetWalkingParam::Response &res);

    /* ROS Service Callback Functions */
    //    bool    GetReferenceStepDataServiceCallback(thormang3_walking_module_msgs::GetReferenceStepData::Request  &req,
    //                                                thormang3_walking_module_msgs::GetReferenceStepData::Response &res);
    //    bool    AddStepDataServiceCallback(thormang3_walking_module_msgs::AddStepDataArray::Request  &req,
    //                                        thormang3_walking_module_msgs::AddStepDataArray::Response &res);
    //    bool    WalkingStartServiceCallback(thormang3_walking_module_msgs::WalkingStart::Request  &req,
    //                                        thormang3_walking_module_msgs::WalkingStart::Response &res);
    //    bool    IsRunningServiceCallback(thormang3_walking_module_msgs::IsRunning::Request  &req,
    //                                        thormang3_walking_module_msgs::IsRunning::Response &res);
    //    bool    SetBalanceParamServiceCallback(thormang3_walking_module_msgs::SetBalanceParam::Request  &req,
    //                                            thormang3_walking_module_msgs::SetBalanceParam::Response &res);
    //    bool    RemoveExistingStepDataServiceCallback(thormang3_walking_module_msgs::RemoveExistingStepData::Request  &req,
    //                                                    thormang3_walking_module_msgs::RemoveExistingStepData::Response &res);
    //
    //    int     StepDataMsgToStepData(thormang3_walking_module_msgs::StepData& src, StepData& des);
    //    int     StepDataToStepDataMsg(StepData& src, thormang3_walking_module_msgs::StepData& des);
    //
    //    void    SetBalanceParam(thormang3_walking_module_msgs::BalanceParam& balance_param_msg);

    void processPhase(const double &time_unit);
    bool computeLegAngle(double *leg_angle);
    void computeArmAngle(double *arm_angle);
    void sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle);

    // variable for walking
    double m_PeriodTime;
    double m_DSP_Ratio;
    double m_SSP_Ratio;
    double m_X_Swap_PeriodTime;
    double m_X_Move_PeriodTime;
    double m_Y_Swap_PeriodTime;
    double m_Y_Move_PeriodTime;
    double m_Z_Swap_PeriodTime;
    double m_Z_Move_PeriodTime;
    double m_A_Move_PeriodTime;
    double m_SSP_Time;
    double m_SSP_Time_Start_L;
    double m_SSP_Time_End_L;
    double m_SSP_Time_Start_R;
    double m_SSP_Time_End_R;
    double m_Phase_Time1;
    double m_Phase_Time2;
    double m_Phase_Time3;

    double m_X_Offset;
    double m_Y_Offset;
    double m_Z_Offset;
    double m_R_Offset;
    double m_P_Offset;
    double m_A_Offset;

    double m_X_Swap_Phase_Shift;
    double m_X_Swap_Amplitude;
    double m_X_Swap_Amplitude_Shift;
    double m_X_Move_Phase_Shift;
    double m_X_Move_Amplitude;
    double m_X_Move_Amplitude_Shift;
    double m_Y_Swap_Phase_Shift;
    double m_Y_Swap_Amplitude;
    double m_Y_Swap_Amplitude_Shift;
    double m_Y_Move_Phase_Shift;
    double m_Y_Move_Amplitude;
    double m_Y_Move_Amplitude_Shift;
    double m_Z_Swap_Phase_Shift;
    double m_Z_Swap_Amplitude;
    double m_Z_Swap_Amplitude_Shift;
    double m_Z_Move_Phase_Shift;
    double m_Z_Move_Amplitude;
    double m_Z_Move_Amplitude_Shift;
    double m_A_Move_Phase_Shift;
    double m_A_Move_Amplitude;
    double m_A_Move_Amplitude_Shift;

    double m_Pelvis_Offset;
    double m_Pelvis_Swing;
    double m_Hip_Pitch_Offset;
    double m_Arm_Swing_Gain;

    bool m_Ctrl_Running;
    bool m_Real_Running;
    double m_Time;

    int    m_Phase;
    double m_Body_Swing_Y;
    double m_Body_Swing_Z;

    double wSin(double time, double period, double period_shift, double mag, double mag_shift);
    bool computeIK(double *out, double x, double y, double z, double a, double b, double c);
    void updateTimeParam();
    void updateMovementParam();
    void updatePoseParam();

    void startWalking();

    void loadWalkingParam(const std::string &path);
    void saveWalkingParam(std::string &path);

    void IniposeTraGene(double mov_time);

public:
    virtual ~WalkingMotionModule();

    double gyro_x, gyro_y;
    double orientation_roll, orientation_pitch;
    double r_foot_fx_N,  r_foot_fy_N,  r_foot_fz_N;
    double r_foot_Tx_Nm, r_foot_Ty_Nm, r_foot_Tz_Nm;
    double l_foot_fx_N,  l_foot_fy_N,  l_foot_fz_N;
    double l_foot_Tx_Nm, l_foot_Ty_Nm, l_foot_Tz_Nm;

    static WalkingMotionModule *GetInstance() { return unique_instance_; }
    void    Initialize(const int control_cycle_msec, Robot *robot);
    void    Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors);

    void    Stop();
    bool    IsRunning();

    enum
    {
        PHASE0 = 0,
        PHASE1 = 1,
        PHASE2 = 2,
        PHASE3 = 3
    };

    int GetCurrentPhase()       { return m_Phase; }
    double GetBodySwingY()      { return m_Body_Swing_Y; }
    double GetBodySwingZ()      { return m_Body_Swing_Z; }

protected:
    void OnEnable();
    void OnDisable();
};

}

#endif /* OP3_WALKING_MODULE_H_ */
