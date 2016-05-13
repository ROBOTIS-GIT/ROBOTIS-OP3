/*
 * op3OffsetTunerServer.cpp
 *
 *  Created on: 2016. 2. 15.
 *      Author: HJSONG
 */

#include "op3_offset_tuner_server/OP3OffsetTunerServer.h"

#define OFFSET_ROSPARAM_KEY "offset"
#define OFFSET_INIT_POS_ROSPARAM_KEY "init_pose_for_offset_tuner"

namespace ROBOTIS
{

OffsetTunerServer *OffsetTunerServer::unique_instance_ = new OffsetTunerServer();

OffsetTunerServer::OffsetTunerServer()
{
	//controller_ = 0;

	offset_file_ = "";
	robot_file_ = "";
	init_file_ = "";
	controller_ = 0;
}

OffsetTunerServer::~OffsetTunerServer()
{

}

void OffsetTunerServer::setCtrlModule(std::string module)
{
    ros::NodeHandle _nh;
    ros::Publisher set_ctrl_module_pub_	= _nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

    std_msgs::String _control_msg;

    _control_msg.data = module;
	set_ctrl_module_pub_.publish(_control_msg);
}

bool OffsetTunerServer::Initialize()
{
	controller_ = RobotisController::GetInstance();

    ros::NodeHandle _nh;

    //Get File Path
    _nh.param<std::string>("offset_path", offset_file_, "");
    _nh.param<std::string>("robot_file_path", robot_file_, "");
	_nh.param<std::string>("init_file_path", init_file_, "");

	if((offset_file_ == "") || (robot_file_ == "")) {
		ROS_ERROR("Failed to get file path");
		return -1;
	}
	//
    //Controller Initialize with robot file info
    if(controller_->Initialize(robot_file_, init_file_) == false) {
        ROS_ERROR("ROBOTIS Controller Initialize Fail!");
        return -1;
    }
	//controller_->LoadOffset(offset_file_);
	controller_->AddMotionModule((MotionModule*)BaseModule::GetInstance());


	//Initialize RobotOffsetData
	for(std::map<std::string, Dynamixel *>::iterator _robot_it = controller_->robot->dxls.begin(); _robot_it != controller_->robot->dxls.end(); _robot_it++)
	{
		std::string _joint_name = _robot_it->first;
		RobotOffsetData[_joint_name] = new JointOffsetData(0, 0);
		RobotTorqueEnableData[_joint_name]	= true;
	}


	//Get and Initialize Offset Data
	std::map<std::string, double> _offset_data;
    //bool result = _nh.getParam(OFFSET_ROSPARAM_KEY, _offset_data);
//	if(result == true)
//	{
        //ROS_INFO("Succeed to get offset");

        //offset data initialize
        //referring to  robotis file info the data will be initialized

//
//
//
//		for(std::map<std::string, double>::iterator _it = _offset_data.begin(); _it != _offset_data.end(); _it++)
//	    {
//	        std::string _joint_name = _it->first;
//	        double 		_joint_offset = _it->second;
//
//	        RobotOffsetData[_joint_name]		= new JointOffsetData(_joint_offset, 0);
//	        RobotTorqueEnableData[_joint_name]	= true;
//	    }
//	}
//	else {
//		ROS_ERROR("Failed to get offset data");
//		return false;
//	}


	//Load Offset.yaml
	YAML::Node _offset_yaml_node = YAML::LoadFile(offset_file_.c_str());

	//Get Offset Data and Init_Pose for Offset Tuning
	YAML::Node _offset_data_node		= _offset_yaml_node[OFFSET_ROSPARAM_KEY];
	YAML::Node _offset_init_pose_node	= _offset_yaml_node[OFFSET_INIT_POS_ROSPARAM_KEY];

	//Initialize Offset Data in RobotOffsetData
	for(YAML::const_iterator _node_it = _offset_data_node.begin(); _node_it != _offset_data_node.end(); _node_it++)
	{
		std::string _joint_name = _node_it->first.as<std::string>();
		double      _offset     = _node_it->second.as<double>();

		std::map<std::string, JointOffsetData*>::iterator _robot_offset_data_it = RobotOffsetData.find(_joint_name);
		if(_robot_offset_data_it != RobotOffsetData.end())
			RobotOffsetData[_joint_name]->joint_offset_rad = _offset;
	}

	//Initialize Init Pose for Offset Tuning in RobotOffsetData
	for(YAML::const_iterator _node_it = _offset_init_pose_node.begin(); _node_it != _offset_init_pose_node.end(); _node_it++)
	{
		std::string _joint_name			= _node_it->first.as<std::string>();
		double      _offset_init_pose   = _node_it->second.as<double>();

		std::map<std::string, JointOffsetData*>::iterator _robot_offset_data_it = RobotOffsetData.find(_joint_name);
		if(_robot_offset_data_it != RobotOffsetData.end())
			RobotOffsetData[_joint_name]->joint_init_pos_rad = _offset_init_pose;
	}


//	//Get Initial Pose for Offset Tunning
//	std::map<std::string, double> _init;
//	result = _nh.getParam(OFFSET_INIT_POS_ROSPARAM_KEY, _init);
//	if(result == true)
//	{
//		std::map<std::string, JointOffsetData*>::iterator _robot_offset_data_it; // iterator for RobotOffsetData
//		ROS_INFO("Succeed to get offset init pose");
//	    for(std::map<std::string, double>::iterator _it = _init.begin(); _it != _init.end(); _it++) {
//	        std::string _joint_name = _it->first;
//	        double 		_joint_init = _it->second;
//
//	        //check the joint name exist at the offset data
//	        _robot_offset_data_it = RobotOffsetData.find(_joint_name);
//
//	        if(_robot_offset_data_it != RobotOffsetData.end())
//	        	RobotOffsetData[_joint_name]->joint_init_pos_rad = _joint_init;
//	        else
//	        	continue;
//	    }
//	}
//	else {
//		ROS_ERROR("Failed to get offset init pose");
//		return false;
//	}

	ROS_INFO(" ");
	ROS_INFO(" ");
	//For Data Check, Print All Available Data
	int i = 0;
	ROS_INFO_STREAM( "num" <<" | "<<"joint_name" << " | " << "InitPose" << ", " << "OffsetAngleRad");
    for(std::map<std::string, JointOffsetData*>::iterator _it = RobotOffsetData.begin(); _it != RobotOffsetData.end(); _it++)
    {
        std::string 		_joint_name = _it->first;
        JointOffsetData* 	_joint_data = _it->second;

        ROS_INFO_STREAM( i <<" | "<<_joint_name << " : " << _joint_data->joint_init_pos_rad << ", " << _joint_data->joint_offset_rad);
        i++;
    }

    send_tra_sub_         	 = _nh.subscribe("/robotis/base/send_tra",                  10, &OffsetTunerServer::StringMsgsCallBack,       this);
    joint_offset_data_sub_	 = _nh.subscribe("/robotis/offset_tuner/joint_offset_data", 10, &OffsetTunerServer::JointOffsetDataCallback,  this);
    joint_torque_enable_sub_ = _nh.subscribe("/robotis/offset_tuner/torque_enable",     10, &OffsetTunerServer::JointTorqueOnOffCallback, this);
    command_sub_			 = _nh.subscribe("/robotis/offset_tuner/command",            5, &OffsetTunerServer::CommandCallback,          this);
    offset_data_server_		 = _nh.advertiseService("robotis/offset_tuner/get_present_joint_offset_data",&OffsetTunerServer::GetPresentJointOffsetDataServiceCallback, this);

	return true;
}

void OffsetTunerServer::MoveToInitPose()
{

	controller_->StartTimer();
	BaseModule *_base_modue = BaseModule::GetInstance();

//	Eigen::MatrixXd _offset_init_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1 , 1 );

//    for(std::map<std::string, JointOffsetData*>::iterator _it = RobotOffsetData.begin(); _it != RobotOffsetData.end(); _it++)
//    {
//        std::string 		_joint_name = _it->first;
//        JointOffsetData* 	_joint_data = _it->second;
//
//
//        if ( controller_->robot->dxls.find(_joint_name) == controller_->robot->dxls.end() ) {
//          continue;
//        }
//        else {
//            int id = (int)(controller_->robot->dxls[_joint_name]->id);
//            //if(id > MAX_JOINT_ID) {
//            if(id > MAX_JOINT_ID) {
//            	ROS_ERROR("Invalid JointID");
//            	return;
//            }
//            else {
//            	_offset_init_pose.coeffRef(id, 0) = _joint_data->joint_init_pos_rad;
//            }
//        }
//    }

	//make map, key : joint_name, value : joint_init_pos_rad;
	std::map<std::string, double> _offset_init_pose;
	for(std::map<std::string, JointOffsetData*>::iterator _it = RobotOffsetData.begin(); _it != RobotOffsetData.end(); _it++)
	{
		std::string 		_joint_name = _it->first;
		JointOffsetData* 	_joint_data = _it->second;


		if ( controller_->robot->dxls.find(_joint_name) == controller_->robot->dxls.end() ) {
			continue;
		}
		else {
			_offset_init_pose[_joint_name] = _joint_data->joint_init_pos_rad + _joint_data->joint_offset_rad;
		}
	}

    usleep(80*1000);
    _base_modue->PoseGenProc(_offset_init_pose);
    usleep(10*000);

    while(_base_modue->IsRunning())
    	usleep(50*1000);

    controller_->StopTimer();
    while(controller_->IsTimerRunning())
    	usleep(10*1000);

    if(controller_->IsTimerRunning())
    {
    	ROS_INFO("Timer Running");
    }

    setCtrlModule("none");
}

void OffsetTunerServer::StringMsgsCallBack(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO_STREAM(msg->data);
}

void OffsetTunerServer::CommandCallback(const std_msgs::String::ConstPtr& msg)
{
	if(msg->data == "save")
	{
//		YAML::Node  _baseNode = YAML::LoadFile(offset_file_); // gets the root node
//	    for(std::map<std::string, JointOffsetData*>::iterator _it = RobotOffsetData.begin(); _it != RobotOffsetData.end(); _it++)
//	    {
//	        std::string 		_joint_name = _it->first;
//	        JointOffsetData* 	_joint_data = _it->second;
//
//	        _baseNode["offset"][_joint_name] 					 = _joint_data->joint_offset_rad; // edit one of the nodes
//	        _baseNode["init_pose_for_offset_tuner"][_joint_name] = _joint_data->joint_init_pos_rad; // edit one of the nodes
//	    }
//		std::ofstream fout(offset_file_.c_str());
//		fout << _baseNode; // dump it back into the file

		YAML::Emitter _out;
		std::map<std::string, double> _offset;
		std::map<std::string, double> _init_pose;
		for(std::map<std::string, JointOffsetData*>::iterator _it = RobotOffsetData.begin(); _it != RobotOffsetData.end(); _it++)
		{
			std::string 		_joint_name = _it->first;
			JointOffsetData* 	_joint_data = _it->second;

			_offset[_joint_name] 	= _joint_data->joint_offset_rad; // edit one of the nodes
			_init_pose[_joint_name] = _joint_data->joint_init_pos_rad; // edit one of the nodes
		}

		_out << YAML::BeginMap;
		_out << YAML::Key << "offset" << YAML::Value << _offset;
		_out << YAML::Key << "init_pose_for_offset_tuner" << YAML::Value << _init_pose;
		_out << YAML::EndMap;
		std::ofstream fout(offset_file_.c_str());
		fout << _out.c_str(); // dump it back into the file

	}
	else if(msg->data == "ini_pose")
	{
		MoveToInitPose();
	}
	else
	{
		ROS_INFO_STREAM("Invalid Command : " << msg->data);
	}
}

void OffsetTunerServer::JointOffsetDataCallback(const op3_offset_tuner_msgs::JointOffsetData::ConstPtr &msg)
{
    if(controller_->IsTimerRunning()) {
    	ROS_ERROR("Timer is running now");
    	return;
    }


	//goal position
	ROS_INFO_STREAM(msg->joint_name << " " << msg->goal_value << " " << msg->offset_value << " " << msg->p_gain <<" " << msg->i_gain <<" " << msg->d_gain);

	std::map<std::string, JointOffsetData*>::iterator _it;
	_it = RobotOffsetData.find(msg->joint_name);
	if(_it == RobotOffsetData.end()) {
		ROS_ERROR("Invalid Joint Name");
		return;
	}

	if(RobotTorqueEnableData[msg->joint_name] == false) {
		ROS_ERROR_STREAM(msg->joint_name << "is turned off the torque");
		return;
	}


	double goal_pose_rad = msg->offset_value + msg->goal_value;
	INT32_T goal_pose_value = controller_->robot->dxls[msg->joint_name]->ConvertRadian2Value(goal_pose_rad);
	UINT8_T _dxl_error = 0;
	INT32_T _comm_result = COMM_SUCCESS;
	_comm_result = controller_->WriteCtrlItem(msg->joint_name, controller_->robot->dxls[msg->joint_name]->goal_position_item->item_name, goal_pose_value, &_dxl_error);
	if(_comm_result != COMM_SUCCESS)	{
		ROS_ERROR("Failed to write goal position");
		return;
	}
	else {
		RobotOffsetData[msg->joint_name]->joint_init_pos_rad = msg->goal_value;
		RobotOffsetData[msg->joint_name]->joint_offset_rad   = msg->offset_value;
	}

	if(_dxl_error != 0 )	{
		ROS_ERROR_STREAM("goal_pos_set : " << msg->joint_name << "  has error "<< (int)_dxl_error);
	}

	RobotOffsetData[msg->joint_name]->p_gain = msg->p_gain;
	RobotOffsetData[msg->joint_name]->i_gain = msg->i_gain;
	RobotOffsetData[msg->joint_name]->d_gain = msg->d_gain;


//	UINT16_T goal_pose_addr = controller_->robot->dxls[msg->joint_name]->goal_position_address;
//	UINT8_T _error = 0;
//	int comm_result = COMM_SUCCESS;
//	//length ??
//	int id = controller_->robot->dxls[msg->joint_name]->id;
//	ROS_INFO_STREAM("ID : " << id <<" " << goal_pose_value << " " << (UINT32_T)goal_pose_value);
//	comm_result = controller_->Write4Byte(msg->joint_name, goal_pose_addr, goal_pose_value, &_error);
//
//	if(comm_result != COMM_SUCCESS)	{
//		ROS_ERROR("Failed to write goal position");
//		return;
//	}
//	else {
//		RobotOffsetData[msg->joint_name]->joint_init_pos_rad = msg->goal_value;
//		RobotOffsetData[msg->joint_name]->joint_offset_rad   = msg->offset_value;
//	}
//
//	if(_error != 0 )	{
//		fprintf(stderr, "%c %d\n", _error, _error);
//		ROS_ERROR_STREAM("goal_pos_set : " << msg->joint_name << "  has error "<< (int)_error);
//	}
//
//	UINT16_T pgain_addr = controller_->robot->dxls[msg->joint_name]->position_p_gain_address;
//	comm_result = controller_->Write2Byte(msg->joint_name, pgain_addr, msg->p_gain, &_error);
//	if(comm_result != COMM_SUCCESS)	{
//		ROS_ERROR("Failed to write p gain");
//		return;
//	}
//	else
//		RobotOffsetData[msg->joint_name]->p_gain = msg->p_gain;
//
//	if(_error != 0 )	{
//		ROS_ERROR_STREAM("pgain_set : " << msg->joint_name << "  has error "<< (int)_error);
//	}
//
//	UINT16_T igain_addr = controller_->robot->dxls[msg->joint_name]->position_i_gain_address;
//	if(igain_addr != 0 ) {
//		UINT8_T length = controller_->robot->dxls[msg->joint_name]->ctrl_table["position_I_gain"]->data_length;
//		if(length == 1)
//			comm_result = controller_->Write1Byte(msg->joint_name, igain_addr, (UINT8_T)(msg->i_gain), &_error);
//		else if(length == 2)
//			comm_result = controller_->Write2Byte(msg->joint_name, igain_addr, (UINT16_T)(msg->i_gain), &_error);
//		else if(length == 4)
//			comm_result = controller_->Write4Byte(msg->joint_name, igain_addr, (UINT32_T)(msg->i_gain), &_error);
//		else {
//			return;
//		}
//
//		if(comm_result != COMM_SUCCESS)	{
//			ROS_ERROR("Failed to write i gain");
//			return;
//		}
//		else
//			RobotOffsetData[msg->joint_name]->i_gain = msg->i_gain;
//
//		if(_error != 0 )	{
//			ROS_ERROR_STREAM("igain_set : " << msg->joint_name << "  has error "<< (int)_error);
//		}
//	}
//
//
//
//	UINT16_T dgain_addr = controller_->robot->dxls[msg->joint_name]->position_d_gain_address;
//	if(dgain_addr != 0 ) {
//		comm_result = controller_->Write2Byte(msg->joint_name, dgain_addr, msg->d_gain, &_error);
//		if(comm_result != COMM_SUCCESS)	{
//			ROS_ERROR("Failed to write d gain");
//			return;
//		}
//		else
//			RobotOffsetData[msg->joint_name]->d_gain = msg->d_gain;
//
//		if(_error != 0 )	{
//			ROS_ERROR_STREAM("dgain_set : " << msg->joint_name << "  has error "<< (int)_error);
//		}
//	}
}


void OffsetTunerServer::JointTorqueOnOffCallback(const op3_offset_tuner_msgs::JointTorqueOnOffArray::ConstPtr& msg)
{
	for(unsigned int _i = 0;  _i < msg->torque_enable_data.size(); _i++)
	{
		std::string _joint_name = msg->torque_enable_data[_i].joint_name;
		bool _torque_enable = msg->torque_enable_data[_i].torque_enable;
		ROS_INFO_STREAM(_i <<" " << _joint_name << _torque_enable);

		std::map<std::string, JointOffsetData*>::iterator _it;
		_it = RobotOffsetData.find(_joint_name);
		if(_it == RobotOffsetData.end()) {
			ROS_ERROR("Invalid Joint Name");
			continue;
		}
		else {
			INT32_T _comm_result = COMM_SUCCESS;
			UINT8_T _dxl_error = 0;
			UINT8_T _torque_enable_value = 0;

			if(_torque_enable)
				_torque_enable_value = 1;
			else
				_torque_enable_value = 0;

			_comm_result = controller_->WriteCtrlItem(_joint_name, controller_->robot->dxls[_joint_name]->torque_enable_item->item_name, _torque_enable_value, &_dxl_error);
			if(_comm_result != COMM_SUCCESS)	{
				ROS_ERROR("Failed to write goal position");
			}
			else {
				RobotTorqueEnableData[_joint_name] = _torque_enable;
			}

			if(_dxl_error != 0 )	{
				ROS_ERROR_STREAM("goal_pos_set : " << _joint_name << "  has error "<< (int)_dxl_error);
			}
		}
	}


//	for(unsigned int _i = 0;  _i < msg->torque_enable_data.size(); _i++)
//	{
//		std::string _joint_name = msg->torque_enable_data[_i].joint_name;
//
//		std::map<std::string, JointOffsetData*>::iterator _it;
//		for(_it = RobotOffsetData.begin(); _it != RobotOffsetData.end(); _it++)
//		{
//			if(_it->first == _joint_name)
//				break;
//		}
//
//		if(_it == RobotOffsetData.end())
//		{
//			ROS_ERROR_STREAM("Invalid Joint Name : " << _joint_name);
//			return;
//		}

//		UINT16_T _torque_enable_addr = controller_->robot->dxls[msg->torque_enable_data[_i].joint_name]->torque_enable_address;
//		UINT8_T  _torque_enable = 0;
//		if(msg->torque_enable_data[_i].torque_enable == true)
//			_torque_enable = 1;
//
//		UINT8_T _error = 0;
//		int comm_result = controller_->Write1Byte(msg->torque_enable_data[_i].joint_name, _torque_enable_addr, _torque_enable, &_error);
//		if((comm_result != COMM_SUCCESS))
//		{
//			ROS_ERROR("Failed to write torque enable [result : %d]", comm_result);
//			return;
//		}
//
//		if(_error != 0 )	{
//			ROS_ERROR_STREAM( msg->torque_enable_data[_i].joint_name << "  has error "<< (int)_error);
//		}
//
//		if(RobotTorqueEnableData[msg->torque_enable_data[_i].joint_name] == false)
//		{
//			UINT32_T present_pos_value = 0;
//			UINT8_T  error = 0;
//			int 	 comm_result = COMM_SUCCESS;
//			int comm_count = 0;
//			int max_comm_count = 3;
//			for(comm_count = 0; comm_count < max_comm_count; comm_count++) {
//				comm_result = controller_->Read4Byte(_joint_name, controller_->robot->dxls[_joint_name]->present_position_address, &present_pos_value, &error);
//				if(comm_result != COMM_SUCCESS)
//				{
//					ROS_ERROR("Failed to read present pos");
//					return;
//				}
//				else {
//					RobotOffsetData[msg->torque_enable_data[_i].joint_name]->joint_offset_rad = controller_->robot->dxls[_joint_name]->ConvertValue2Radian(present_pos_value);
//					break;
//				}
//			}
//
//			if(comm_count == max_comm_count)
//			{
//				ROS_ERROR("All Comm Fail");
//			}
//		}
//
//		RobotTorqueEnableData[msg->torque_enable_data[_i].joint_name] = msg->torque_enable_data[_i].torque_enable;
//	}
}

bool OffsetTunerServer::GetPresentJointOffsetDataServiceCallback(op3_offset_tuner_msgs::GetPresentJointOffsetData::Request &req,
                                                                op3_offset_tuner_msgs::GetPresentJointOffsetData::Response &res)
{

	ROS_INFO("GetPresentJointOffsetDataService Called");

	for(std::map<std::string, JointOffsetData*>::iterator _it = RobotOffsetData.begin(); _it != RobotOffsetData.end(); _it++)
	{
		std::string 		_joint_name = _it->first;
		JointOffsetData* 	_joint_data = _it->second;


        op3_offset_tuner_msgs::JointOffsetPositionData _joint_offset_pos;

		INT32_T present_pos_value = 0;
		UINT8_T  _dxl_error = 0;
		int 	 comm_result = COMM_SUCCESS;


		comm_result = controller_->ReadCtrlItem(_joint_name, controller_->robot->dxls[_joint_name]->present_position_item->item_name, (UINT32_T*)& present_pos_value, &_dxl_error);
		if(comm_result != COMM_SUCCESS)
		{
			ROS_ERROR("Failed to read present pos");
			return false;
		}
		else {
			if(_dxl_error != 0 )	{
				ROS_ERROR_STREAM( _joint_name << "  has error "<< (int)_dxl_error);
			}

			_joint_offset_pos.joint_name	= _joint_name;
			_joint_offset_pos.goal_value	= _joint_data->joint_init_pos_rad;
			_joint_offset_pos.offset_value	= _joint_data->joint_offset_rad;
			_joint_offset_pos.present_value	= controller_->robot->dxls[_joint_name]->ConvertValue2Radian(present_pos_value);
			_joint_offset_pos.p_gain		= _joint_data->p_gain;
			_joint_offset_pos.i_gain		= _joint_data->i_gain;
			_joint_offset_pos.d_gain		= _joint_data->d_gain;

			res.present_data_array.push_back(_joint_offset_pos);
		}
    }
	return true;
}



}

