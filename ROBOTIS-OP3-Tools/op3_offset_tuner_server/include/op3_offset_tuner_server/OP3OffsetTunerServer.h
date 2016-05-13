#ifndef OP3_OFFSET_TUNER_SERVER_H_
#define OP3_OFFSET_TUNER_SERVER_H_

#include <ros/ros.h>
#include <map>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "robotis_controller/RobotisController.h"
#include "op3_base_module/BaseModule.h"
#include "op3_offset_tuner_msgs/JointOffsetData.h"
#include "op3_offset_tuner_msgs/JointTorqueOnOffArray.h"
#include "op3_offset_tuner_msgs/GetPresentJointOffsetData.h"


namespace ROBOTIS
{

class JointOffsetData
{
public:
	double	joint_offset_rad;
	double	joint_init_pos_rad;
	int 	p_gain;
	int		i_gain;
	int		d_gain;

	JointOffsetData()
	{
		joint_offset_rad = 0;
		joint_init_pos_rad = 0;
		p_gain = 32;
		i_gain = 0;
		d_gain = 0;
	}

	JointOffsetData(double joint_offset_rad, double joint_init_pose_rad)
	{
		this->joint_offset_rad = joint_offset_rad;
		this->joint_init_pos_rad = joint_init_pose_rad;
		p_gain = 32;
		i_gain = 0;
		d_gain = 0;
	}

	~JointOffsetData() { }
};

class OffsetTunerServer
{
private:
	//RobotisController* controller_;


	std::string offset_file_;
	std::string robot_file_;
	std::string init_file_;
	std::map<std::string, JointOffsetData*> RobotOffsetData;
	std::map<std::string, bool> RobotTorqueEnableData;
	RobotisController* controller_;

	OffsetTunerServer();
	void setCtrlModule(std::string module);

    static OffsetTunerServer* unique_instance_;

    ros::Subscriber		send_tra_sub_;
    ros::Subscriber		joint_offset_data_sub_;
    ros::Subscriber		joint_torque_enable_sub_;
    ros::Subscriber		command_sub_;
    ros::ServiceServer	offset_data_server_;

public:
    static OffsetTunerServer* GetInstance() { return  unique_instance_ ;}

	~OffsetTunerServer();

	bool Initialize();
	void MoveToInitPose();
	void StringMsgsCallBack(const std_msgs::String::ConstPtr& msg);
	void CommandCallback(const std_msgs::String::ConstPtr& msg);
    void JointOffsetDataCallback(const op3_offset_tuner_msgs::JointOffsetData::ConstPtr &msg);
    void JointTorqueOnOffCallback(const op3_offset_tuner_msgs::JointTorqueOnOffArray::ConstPtr& msg);
    bool GetPresentJointOffsetDataServiceCallback(op3_offset_tuner_msgs::GetPresentJointOffsetData::Request &req,
                                                op3_offset_tuner_msgs::GetPresentJointOffsetData::Response &res);

};

}

#endif /* OP3_OFFSET_TUNER_SERVER_H_ */
