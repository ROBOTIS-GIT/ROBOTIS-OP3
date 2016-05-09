/*
 * motion_module_tutorial.cpp
 *
 *  Created on: 2016. 2. 23.
 *      Author: zerom
 */

#include <stdio.h>
#include <sstream>
#include "op2_action_module/ActionModule.h"




using namespace ROBOTIS;

ActionModule *ActionModule::unique_instance_ = new ActionModule();

std::string ActionModule::IntToString(int _n) {
	std::ostringstream ostr;
	ostr << _n;
	return ostr.str();
}



ActionModule::ActionModule()
    : control_cycle_msec_(8)
{
    enable          = false;
    module_name     = "action_module"; // set unique module name
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
    joint_name_to_id_["r_sho_pitch"]     = 1;
    joint_name_to_id_["l_sho_pitch"]     = 2;
    joint_name_to_id_["r_sho_roll"]      = 3;
    joint_name_to_id_["l_sho_roll"]      = 4;
    joint_name_to_id_["r_el"]            = 5;
    joint_name_to_id_["l_el"]            = 6;

    /* leg */
    joint_name_to_id_["r_hip_yaw"]       = 7;
    joint_name_to_id_["l_hip_yaw"]       = 8;
    joint_name_to_id_["r_hip_roll"]      = 9;
    joint_name_to_id_["l_hip_roll"]      = 10;
    joint_name_to_id_["r_hip_pitch"]     = 11;
    joint_name_to_id_["l_hip_pitch"]     = 12;
    joint_name_to_id_["r_knee"]          = 13;
    joint_name_to_id_["l_knee"]          = 14;
    joint_name_to_id_["r_ank_pitch"]     = 15;
    joint_name_to_id_["l_ank_pitch"]     = 16;
    joint_name_to_id_["r_ank_roll"]      = 17;
    joint_name_to_id_["l_ank_roll"]      = 18;

    /* head */
    joint_name_to_id_["head_pan"]        = 19;
    joint_name_to_id_["head_tilt"]       = 20;

    for(std::map<std::string, int>::iterator _it = joint_name_to_id_.begin(); _it != joint_name_to_id_.end(); _it++)
    {
    	joint_id_to_name_[_it->second] = _it->first;
    }

    //////////////////////////////////
	action_file_ = 0;
	playing_ = false;
	first_driving_start_ = false;
	playing_finished = true;
	page_step_count_ = 0;
	play_page_idx_ = 0;
	stop_playing_ = true;

	previous_enable_  = false;
	present_enable_   = false;
	previous_running_ = false;
	present_running_  = false;
}

ActionModule::~ActionModule()
{
    queue_thread_.join();

    ////////////////////////////////////////
	if(action_file_ != 0)
		fclose( action_file_ );
}

void ActionModule::Initialize(const int control_cycle_msec, Robot *robot)
{
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_       = boost::thread(boost::bind(&ActionModule::QueueThread, this));


    ros::NodeHandle _ros_node;

    std::string _path = ros::package::getPath("op2_action_module") + "/data/motion_4095.bin";
    std::string _action_file_path;
    _ros_node.param<std::string>("action_file_path", _action_file_path, _path);

    LoadFile((char*)_action_file_path.c_str());


    playing_ = false;
}

void ActionModule::QueueThread()
{
    ros::NodeHandle     _ros_node;
    ros::CallbackQueue  _callback_queue;

    _ros_node.setCallbackQueue(&_callback_queue);

    /* subscriber */
    action_page_sub_ = _ros_node.subscribe("/robotis/action/page_num", 1, &ActionModule::PageNumberCallback, this);

    /* publisher */
    status_msg_pub_ = _ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 0);

    while(_ros_node.ok())
        _callback_queue.callAvailable();
}


void ActionModule::PageNumberCallback(const std_msgs::Int32::ConstPtr& _msg)
{
	if(_msg->data == -1) {
		Stop();
	}
	else if(_msg->data == -2) {
		Brake();
	}
	else {
		if(IsRunning() == true) {
			ROS_ERROR_STREAM("Previous Motion is not finished.");
			PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Previous Motion is not finished.");

		}

		if(Start(_msg->data) == true) {
	    	std::string _status_msg = "Succeed to start page " + IntToString(_msg->data);
	    	ROS_INFO_STREAM(_status_msg);
			PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
		}
		else {
	    	std::string _status_msg = "Failed to start page " + IntToString(_msg->data);
	    	ROS_ERROR_STREAM(_status_msg);
			PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
		}
	}
}


void ActionModule::Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors)
{
	previous_enable_ = present_enable_;
	present_enable_  = enable;

    if(enable == false)
        return;

    if((present_enable_ == true) && (present_enable_ != previous_enable_)) {
    	for(std::map<std::string, Dynamixel *>::iterator _it = dxls.begin() ; _it != dxls.end(); _it++)
    	{
    		std::string _joint_name = _it->first;

    		if(result.find(_joint_name) == result.end())
    			continue;
    		else {
    			result[_joint_name]->goal_position = _it->second->dxl_state->present_position;
    		}
    	}
    }


	previous_running_ = present_running_;
	present_running_  = IsRunning();

	if(present_running_ != previous_running_) {
		if(present_running_ == true) {
	    	std::string _status_msg = "Action_Start";
	    	ROS_INFO_STREAM(_status_msg);
			PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
		}
		else {
	    	std::string _status_msg = "Action_Finish";
	    	ROS_INFO_STREAM(_status_msg);
			PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, _status_msg);
		}
	}

    ActionPlayProcess(dxls);
}




void ActionModule::Stop()
{
	stop_playing_ = true;
}

bool ActionModule::IsRunning()
{
	return playing_;
}

int  ActionModule::RadTow4095(double _rad)
{
	return (int)((_rad + M_PI)*2048.0/M_PI);
}

double ActionModule::w4095ToRad(int _w4095)
{
	return (_w4095 - 2048)*M_PI/2048.0;
}


bool ActionModule::VerifyChecksum( ACTION_FILE::PAGE* _page )
{
	unsigned char _checksum = 0x00;
    unsigned char* _pt = (unsigned char*)_page;

    for(unsigned int i = 0; i < sizeof(ACTION_FILE::PAGE); i++)
    {
        _checksum += *_pt;
        _pt++;
    }

    if(_checksum != 0xff)
        return false;

	return true;
}

void ActionModule::SetChecksum( ACTION_FILE::PAGE* _page )
{
	unsigned char _checksum = 0x00;
    unsigned char* _pt = (unsigned char*)_page;

    _page->header.checksum = 0x00;

    for(unsigned int i=0; i<sizeof(ACTION_FILE::PAGE); i++)
    {
        _checksum += *_pt;
        _pt++;
    }

    _page->header.checksum = (unsigned char)(0xff - _checksum);
}



bool ActionModule::LoadFile(char* _file_name)
{
	FILE* _action = fopen( _file_name, "r+b" );
    if( _action == 0 )
	{
    	std::string _status_msg = "Can not open Action file!";
    	ROS_ERROR_STREAM(_status_msg);
		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
        return false;
	}

    fseek( _action, 0, SEEK_END );
    if( ftell(_action) != (long)(sizeof(ACTION_FILE::PAGE) * ACTION_FILE::MAXNUM_PAGE) )
    {
    	std::string _status_msg = "It's not an Action file!";
    	ROS_ERROR_STREAM(_status_msg);
		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
        fclose( _action );
        return false;
    }

	if(action_file_ != 0)
		fclose( action_file_ );

	action_file_ = _action;
	return true;
}

bool ActionModule::CreateFile(char* _file_name)
{
	FILE* _action = fopen( _file_name, "ab" );
	if( _action == 0 )
	{
		std::string _status_msg = "Can not create Action file!";
		ROS_ERROR_STREAM(_status_msg);
		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
		return false;
	}

	ACTION_FILE::PAGE page;
	ResetPage(&page);

	for(int i=0; i < ACTION_FILE::MAXNUM_PAGE; i++)
		fwrite((const void *)&page, 1, sizeof(ACTION_FILE::PAGE), _action);

	if(action_file_ != 0)
		fclose( action_file_ );

	action_file_ = _action;

	return true;
}

bool ActionModule::Start(int _page_number)
{
	if( _page_number < 1 || _page_number >= ACTION_FILE::MAXNUM_PAGE )
	{

		std::string _status_msg = "Can not play page.(" + IntToString(_page_number) + " is invalid index)";
		ROS_ERROR_STREAM(_status_msg);
		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
        return false;
	}

	ACTION_FILE::PAGE page;
	if( LoadPage(_page_number, &page) == false )
        return false;

	return Start(_page_number, &page);
}

bool ActionModule::Start(char* _page_name)
{
	int index;
	ACTION_FILE::PAGE page;

	for(index=1; index < ACTION_FILE::MAXNUM_PAGE; index++)
	{
		if(LoadPage(index, &page) == false)
			return false;

		if(strcmp(_page_name, (char*)page.header.name) == 0)
			break;
	}

	if(index == ACTION_FILE::MAXNUM_PAGE) {
		std::string _str_name_page = _page_name;
		std::string _status_msg = "Can not play page.(" + _str_name_page + " is invalid name)\n";
		ROS_ERROR_STREAM(_status_msg);
		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
        return false;
	}
	else
		return Start(index, &page);
}

bool ActionModule::Start(int _page_number, ACTION_FILE::PAGE *_page)
{
	if(enable == false)	{
		std::string _status_msg = "Action Module is disabled";
		ROS_ERROR_STREAM(_status_msg);
		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
		return false;
	}

	if(playing_ == true)
	{
		std::string _status_msg = "Can not play page " + IntToString(_page_number) + ".(Now playing)";
		ROS_ERROR_STREAM(_status_msg);
		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
        return false;
	}

	play_page_ = *_page;

    if( play_page_.header.repeat == 0 || play_page_.header.stepnum == 0 )
	{
		std::string _status_msg = "Page " + IntToString(_page_number) + " has no action\n";
		ROS_ERROR_STREAM(_status_msg);
		PublishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, _status_msg);
        return false;
	}

    play_page_idx_ = _page_number;
    first_driving_start_ = true;
    playing_ = true;

	return true;
}

void ActionModule::Brake()
{
	playing_ = false;
}

bool ActionModule::IsRunning(int* _playing_page_num, int* _playing_step_num)
{
	if(_playing_page_num != 0)
		*_playing_page_num = play_page_idx_;

	if(_playing_step_num != 0)
		*_playing_step_num = page_step_count_ - 1;

	return IsRunning();
}

bool ActionModule::LoadPage(int _page_number, ACTION_FILE::PAGE* _page)
{
	long position = (long)(sizeof(ACTION_FILE::PAGE)*_page_number);

    if( fseek( action_file_, position, SEEK_SET ) != 0 )
        return false;

    if( fread( _page, 1, sizeof(ACTION_FILE::PAGE), action_file_ ) != sizeof(ACTION_FILE::PAGE) )
        return false;

    if( VerifyChecksum( _page ) == false )
        ResetPage( _page );

	return true;
}

bool ActionModule::SavePage(int _page_number, ACTION_FILE::PAGE* _page)
{
	long position = (long)(sizeof(ACTION_FILE::PAGE)*_page_number);

	if( VerifyChecksum(_page) == false )
        SetChecksum(_page);

    if( fseek( action_file_, position, SEEK_SET ) != 0 )
        return false;

    if( fwrite( _page, 1, sizeof(ACTION_FILE::PAGE), action_file_ ) != sizeof(ACTION_FILE::PAGE) )
        return false;

	return true;
}

void ActionModule::ResetPage(ACTION_FILE::PAGE *_page)
{
	unsigned char *pt = (unsigned char*)_page;

    for(unsigned int i=0; i<sizeof(ACTION_FILE::PAGE); i++)
    {
        *pt = 0x00;
        pt++;
    }

    _page->header.schedule = ACTION_FILE::TIME_BASE_SCHEDULE; // default time base
    _page->header.repeat = 1;
    _page->header.speed = 32;
    _page->header.accel = 32;

	for(int i=0; i < 38; i++)
		_page->header.pgain[i] = 0x55;

    for(int i=0; i < ACTION_FILE::MAXNUM_STEP; i++)
    {
        for(int j=0; j < 38; j++)
            _page->step[i].position[j] = ACTION_FILE::INVALID_BIT_MASK;

        _page->step[i].pause = 0;
        _page->step[i].time = 0;
    }

    SetChecksum( _page );
}

void ActionModule::ActionPlayProcess(std::map<std::string, Dynamixel *> dxls)
{
	//////////////////// 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
    unsigned char bID;
    unsigned long ulTotalTime256T;
    unsigned long ulPreSectionTime256T;
    unsigned long ulMainTime256T;
    long lStartSpeed1024_PreTime_256T;
    long lMovingAngle_Speed1024Scale_256T_2T;
    long lDivider1,lDivider2;
    //unsigned short
    int wMaxAngle1024;
    int wMaxSpeed256;
    int wTmp;
    int wPrevTargetAngle; // Start position
    int wCurrentTargetAngle; // Target position
    int wNextTargetAngle; // Next target position
    unsigned char bDirectionChanged;

    ///////////////// Static 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
	static unsigned short wpStartAngle1024[ACTION_FILE::MAXNUM_JOINTS]; // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
    static unsigned short wpTargetAngle1024[ACTION_FILE::MAXNUM_JOINTS]; // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
    static short int ipMovingAngle1024[ACTION_FILE::MAXNUM_JOINTS]; // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌묒슌異�筌뚯슦�뮝�뜝�럡�렊
    static short int ipMainAngle1024[ACTION_FILE::MAXNUM_JOINTS]; // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌묒슌異�筌뚯슦�뮝�뜝�럡�렊
    static short int ipAccelAngle1024[ACTION_FILE::MAXNUM_JOINTS]; // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌묒슌異�筌뚯슦�뮝�뜝�럡�렊
    static short int ipMainSpeed1024[ACTION_FILE::MAXNUM_JOINTS]; // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎄괴 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥�뜝�럥占쎈Ŋ�굲�뜝�럩留뜹뜝�럥�맶占쎈쐻�뜝占�
    static short int ipLastOutSpeed1024[ACTION_FILE::MAXNUM_JOINTS]; // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈걢占쎈쐻占쎈윥�뜎�뛼�쐻占쎈윪�뤃占�(占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�)
    static short int ipGoalSpeed1024[ACTION_FILE::MAXNUM_JOINTS]; // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆�옚�삕占쎈�占쎄콬�뜝�럡�렊 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎄괴占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈걢占쎈쐻占쎈윥�뜎�뛼�쐻占쎈윪�뤃占�
    static unsigned char bpFinishType[ACTION_FILE::MAXNUM_JOINTS]; // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
    short int iSpeedN;
    static unsigned short wUnitTimeCount;
    static unsigned short wUnitTimeNum;
    static unsigned short wPauseTime;
    static unsigned short wUnitTimeTotalNum;
    static unsigned short wAccelStep;
    static unsigned char bSection;
    static unsigned char bPlayRepeatCount;
    static unsigned short wNextPlayPage;

    /////////////// Enum 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�

    /**************************************
    * Section             /----\
    *                    /|    |\
    *        /+---------/ |    | \
    *       / |        |  |    |  \
    * -----/  |        |  |    |   \----
    *      PRE  MAIN   PRE MAIN POST PAUSE
    ***************************************/
    enum{ PRE_SECTION, MAIN_SECTION, POST_SECTION, PAUSE_SECTION };
    enum{ ZERO_FINISH, NONE_ZERO_FINISH};

    if( playing_ == false )
        return;

    if( first_driving_start_ == true ) // 癲ル슪�맋�몭�굩�삕占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈윪�얠렱�쐻占쎈윪�뤃占�
    {
        first_driving_start_ = false; //First Process end
        playing_finished = false;
		stop_playing_ = false;
        wUnitTimeCount = 0;
        wUnitTimeNum = 0;
        wPauseTime = 0;
        bSection = PAUSE_SECTION;
        page_step_count_ = 0;
        bPlayRepeatCount = play_page_.header.repeat;
        wNextPlayPage = 0;


		for(unsigned int _jointIndex = 0; _jointIndex < ACTION_FILE::MAXNUM_JOINTS; _jointIndex++)
		{
			bID = _jointIndex;
			std::string _joint_name = "";

			if(joint_id_to_name_.find(bID) == joint_id_to_name_.end())
				continue;
			else
				_joint_name = joint_id_to_name_[bID];

			if(dxls.find(_joint_name) == dxls.end())
				continue;
			else {
				double _goal_joint_angle_rad = dxls[joint_id_to_name_[bID]]->dxl_state->goal_position;
				wpTargetAngle1024[bID] = RadTow4095(_goal_joint_angle_rad);
				ipLastOutSpeed1024[bID] = 0;
				ipMovingAngle1024[bID] = 0;
				ipGoalSpeed1024[bID] = 0;
			}
		}
    }


    if( wUnitTimeCount < wUnitTimeNum ) // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆蹂잙쐻占쎈윪�얠렱�쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占�
    {
        wUnitTimeCount++;
        if( bSection == PAUSE_SECTION )
        {
        }
        else
        {
            for(unsigned int _jointIndex = 0; _jointIndex < ACTION_FILE::MAXNUM_JOINTS; _jointIndex++ )
            {
				bID = _jointIndex;
				std::string _joint_name = "";

				if(joint_id_to_name_.find(bID) == joint_id_to_name_.end())
					continue;
				else
					_joint_name = joint_id_to_name_[bID];

				if(dxls.find(_joint_name) == dxls.end())
					continue;
				else {
					if( ipMovingAngle1024[bID] == 0 )
					{
						result[_joint_name]->goal_position = w4095ToRad(wpStartAngle1024[bID]);
					}
					else
					{
						if( bSection == PRE_SECTION )
						{
							iSpeedN = (short)( ( (long)(ipMainSpeed1024[bID] - ipLastOutSpeed1024[bID]) * wUnitTimeCount ) / wUnitTimeNum );
							ipGoalSpeed1024[bID] = ipLastOutSpeed1024[bID] + iSpeedN;
							ipAccelAngle1024[bID] =  (short)( ( ( (long)( ipLastOutSpeed1024[bID] + (iSpeedN >> 1) ) * wUnitTimeCount * 144 ) / 15 ) >> 9);

							result[_joint_name]->goal_position = w4095ToRad(wpStartAngle1024[bID] + ipAccelAngle1024[bID]);
						}
						else if( bSection == MAIN_SECTION )
						{
							result[_joint_name]->goal_position	= w4095ToRad(wpStartAngle1024[bID] + (short int)(((long)(ipMainAngle1024[bID])*wUnitTimeCount) / wUnitTimeNum));

							ipGoalSpeed1024[bID] = ipMainSpeed1024[bID];
						}
						else // POST_SECTION
						{
							if( wUnitTimeCount == (wUnitTimeNum-1) )
							{
								//
								result[_joint_name]->goal_position	= w4095ToRad(wpTargetAngle1024[bID]);
							}
							else
							{
								if( bpFinishType[bID] == ZERO_FINISH )
								{
									iSpeedN = (short int)(((long)(0 - ipLastOutSpeed1024[bID]) * wUnitTimeCount) / wUnitTimeNum);
									ipGoalSpeed1024[bID] = ipLastOutSpeed1024[bID] + iSpeedN;

									result[_joint_name]->goal_position
									= w4095ToRad(wpStartAngle1024[bID] +  (short)((((long)(ipLastOutSpeed1024[bID] + (iSpeedN>>1)) * wUnitTimeCount * 144) / 15) >> 9));

								}
								else // NONE_ZERO_FINISH
								{
									// MAIN Section�뜝�룞�삕 �뜝�럩逾졾뜝�럡留믣뜝�럥留�
									//
									result[_joint_name]->goal_position
									= w4095ToRad(wpStartAngle1024[bID] + (short int)(((long)(ipMainAngle1024[bID]) * wUnitTimeCount) / wUnitTimeNum));

									ipGoalSpeed1024[bID] = ipMainSpeed1024[bID];
								}
							}
						}
					}

                    // gains are excepted
                    // result[_joint_name]->position_p_gain = ( 256 >> (play_page_.header.pgain[bID] >> 4) ) << 2 ;
				}
			}
		}
	}
    else if( wUnitTimeCount >= wUnitTimeNum ) // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� Section占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆節뉗땡占쎌뒦占쎈쳮�뜝�럡�렊占쎈쐻占쎈윥�뜎�뛼�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥�댚�뀘�쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占�
    {
        wUnitTimeCount = 0;

        for(unsigned int _jointIndex = 0; _jointIndex < ACTION_FILE::MAXNUM_JOINTS; _jointIndex++)
        {
			bID = _jointIndex;
			std::string _joint_name = "";

			if(joint_id_to_name_.find(bID) == joint_id_to_name_.end())
				continue;
			else
				_joint_name = joint_id_to_name_[bID];

			if(dxls.find(_joint_name) == dxls.end())
				continue;
			else {
				double _goal_joint_angle_rad = dxls[joint_id_to_name_[bID]]->dxl_state->goal_position;
				wpStartAngle1024[bID] = RadTow4095(_goal_joint_angle_rad);
				ipLastOutSpeed1024[bID] = ipGoalSpeed1024[bID];
			}
        }

        // Section 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥獄�占� ( PRE -> MAIN -> POST -> (PAUSE or PRE) ... )
        if( bSection == PRE_SECTION )
        {
            // MAIN Section 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺕�뜝�럥���뜝�럥�뒄�뜝�럡�렊
            bSection = MAIN_SECTION;
            wUnitTimeNum =  wUnitTimeTotalNum - (wAccelStep << 1);

            for(unsigned int _jointIndex = 0; _jointIndex < ACTION_FILE::MAXNUM_JOINTS; _jointIndex++)
			{
				bID = _jointIndex;

				if( bpFinishType[bID] == NONE_ZERO_FINISH )
				{
					if( (wUnitTimeTotalNum - wAccelStep) == 0 ) // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺙癲ル슢�뀖占쎈뮝�뜝�럡�렊
						ipMainAngle1024[bID] = 0;
					else
						ipMainAngle1024[bID] = (short)((((long)(ipMovingAngle1024[bID] - ipAccelAngle1024[bID])) * wUnitTimeNum) / (wUnitTimeTotalNum - wAccelStep));
				}
				else // ZERO_FINISH
					ipMainAngle1024[bID] = ipMovingAngle1024[bID] - ipAccelAngle1024[bID] - (short int)((((long)ipMainSpeed1024[bID] * wAccelStep * 12) / 5) >> 8);
			}
        }
        else if( bSection == MAIN_SECTION )
        {
            // POST Section 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺕�뜝�럥���뜝�럥�뒄�뜝�럡�렊
            bSection = POST_SECTION;
            wUnitTimeNum = wAccelStep;

            for(unsigned int _jointIndex = 0; _jointIndex < ACTION_FILE::MAXNUM_JOINTS; _jointIndex++)
			{
				bID = _jointIndex;
				ipMainAngle1024[bID] = ipMovingAngle1024[bID] - ipMainAngle1024[bID] - ipAccelAngle1024[bID];
			}
        }
        else if( bSection == POST_SECTION )
        {
            // Pause time 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꽡占쎈쐻占쎈윪�얠렱�쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
            if( wPauseTime )
            {
                bSection = PAUSE_SECTION;
                wUnitTimeNum = wPauseTime;
            }
            else
            {
                bSection = PRE_SECTION;
            }
        }
        else if( bSection == PAUSE_SECTION )
        {
            // PRE Section 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺕�뜝�럥���뜝�럥�뒄�뜝�럡�렊
            bSection = PRE_SECTION;

            for(unsigned int _jointIndex = 0; _jointIndex < ACTION_FILE::MAXNUM_JOINTS; _jointIndex++)
			{
            	bID = _jointIndex;
				ipLastOutSpeed1024[bID] = 0;
			}
        }

        // PRE Section占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌묒궍�쐻占쎈윪占쎈�뀐옙�쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺕�뜝�럥���뜝�럥�뒄�뜝�럡�렊 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆�뮪�쐻占쎈윥占쎈꺑占쎈쐻占쎈윪�뤃占�.
        if( bSection == PRE_SECTION )
        {
            if( playing_finished == true ) // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺙癲ル슢�뀖占쎈뮝�뜝�럡�렊
            {
                playing_ = false;
                return;
            }

            page_step_count_++;

            if( page_step_count_ > play_page_.header.stepnum ) // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺙癲ル슢�뀖占쎈뮝�뜝�럡�렊
            {
                // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
                play_page_ = next_play_page_;
                if( play_page_idx_ != wNextPlayPage )
                    bPlayRepeatCount = play_page_.header.repeat;
                page_step_count_ = 1;
                play_page_idx_ = wNextPlayPage;
            }

            if( page_step_count_ == play_page_.header.stepnum ) // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆蹂잙쐻占쎈윪�얠렱�쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占�
            {
                // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆�꽒�쐻占쎈윥�뜎�뛼�쐻占쎈윪�뤃占�
                if( stop_playing_ == true ) // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺋占쎈쐻占쎈윥�젆袁ㅼ땡筌뚯슦�뮝�뜝�럡�렊
                {
                    wNextPlayPage = play_page_.header.exit; // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� Exit 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
                }
                else
                {
                    bPlayRepeatCount--;
                    if( bPlayRepeatCount > 0 ) // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꽗�뜝�럥���뜝�럥�뒄�뜝�럡�렊 占쎈쐻占쎈윪占쎈츆占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈윥�젆袁ㅼ땡筌뚯슦�뮝�뜝�럡�렊
                        wNextPlayPage = play_page_idx_; // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
                    else // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꽗�뜝�럥���뜝�럥�뒄�뜝�럡�렊占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꽣占쎈쐻占쎈윥�젆袁ㅼ땡筌뚯슦�뮝�뜝�럡�렊
                        wNextPlayPage = play_page_.header.next; // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� Next 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
                }

                if( wNextPlayPage == 0 ) // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺙癲ル슢�뀖占쎈뮝�뜝�럡�렊 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꽓嚥싲갭큔占쎈뎔�뜝�럡�렊占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆節륁삕占쎈�占쎄콬�뜝�럡�렊 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
                    playing_finished = true;
                else
                {
                    // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆�꽒�쐻占쎈윥�뜎�뛼�쐻占쎈윪�뤃占�(占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꽡癲ル슢�뀖占쎈뮝�뜝�럡�렊 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�, 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺙癲ル슢�뀖占쎈뮝�뜝�럡�렊占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆占썸퓴諛매�占쎈뎔�뜝�럡�렊)
                    if( play_page_idx_ != wNextPlayPage )
                        LoadPage( wNextPlayPage, &next_play_page_ );
                    else
                        next_play_page_ = play_page_;

                    // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺙癲ル슢�뀖占쎈뮝�뜝�럡�렊 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꽓嚥싲갭큔占쎈뎔�뜝�럡�렊占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆節륁삕占쎈�占쎄콬�뜝�럡�렊 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
                    if( next_play_page_.header.repeat == 0 || next_play_page_.header.stepnum == 0 )
                        playing_finished = true;
                }
            }

            //////// Step 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌묒뇯�쐻占쎈윪�얠렱�쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占�
            wPauseTime = (((unsigned short)play_page_.step[page_step_count_-1].pause) << 5) / play_page_.header.speed;
            wMaxSpeed256 = ((unsigned short)play_page_.step[page_step_count_-1].time * (unsigned short)play_page_.header.speed) >> 5;
            if( wMaxSpeed256 == 0 )
                wMaxSpeed256 = 1;
            wMaxAngle1024 = 0;

            ////////// Joint占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌묒뇯�쐻占쎈윪�얠렱�쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占�
            for(unsigned int _jointIndex = 0; _jointIndex < ACTION_FILE::MAXNUM_JOINTS; _jointIndex++)
			{
				bID = _jointIndex;
				// 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�, 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�, 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆蹂≪땡占쎌뒦占쎈쳮�뜝�럡�렊占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占�
				ipAccelAngle1024[bID] = 0;

				// Find current target angle
				if( play_page_.step[page_step_count_-1].position[bID] & ACTION_FILE::INVALID_BIT_MASK )
					wCurrentTargetAngle = wpTargetAngle1024[bID];
				else
					wCurrentTargetAngle = play_page_.step[page_step_count_-1].position[bID];

				// Update start, prev_target, curr_target
				wpStartAngle1024[bID] = wpTargetAngle1024[bID];
				wPrevTargetAngle = wpTargetAngle1024[bID];
				wpTargetAngle1024[bID] = wCurrentTargetAngle;

				// Find Moving offset
				ipMovingAngle1024[bID] = (int)(wpTargetAngle1024[bID] - wpStartAngle1024[bID]);

				// Find Next target angle
				if( page_step_count_ == play_page_.header.stepnum ) // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆蹂잙쐻占쎈윪�얠렱�쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占�
				{
					if( playing_finished == true ) // 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆蹂잙쐻占쎈윪�얠렱�쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占�
						wNextTargetAngle = wCurrentTargetAngle;
					else
					{
						if( next_play_page_.step[0].position[bID] & ACTION_FILE::INVALID_BIT_MASK )
							wNextTargetAngle = wCurrentTargetAngle;
						else
							wNextTargetAngle = next_play_page_.step[0].position[bID];
					}
				}
				else
				{
					if( play_page_.step[page_step_count_].position[bID] & ACTION_FILE::INVALID_BIT_MASK )
						wNextTargetAngle = wCurrentTargetAngle;
					else
						wNextTargetAngle = play_page_.step[page_step_count_].position[bID];
				}

				// Find direction change
				if( ((wPrevTargetAngle < wCurrentTargetAngle) && (wCurrentTargetAngle < wNextTargetAngle))
					|| ((wPrevTargetAngle > wCurrentTargetAngle) && (wCurrentTargetAngle > wNextTargetAngle)) )
				{
					// 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆節녴�熬곥룊占썩댙�삕�댆�뿥�쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆節륁삕占쎈�占쎄콬�뜝�럡�렊, 占쎈쐻占쎈윪繹먮ㅇ�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺙癲ル슢�뀖占쎈뮝�뜝�럡�렊(占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�, 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈짗占쎌굲占쎈쐻占쎈윪占쎈�뀐옙�쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺙癲ル슢�뀖占쎈뮝�뜝�럡�렊)
					bDirectionChanged = 0;
				}
				else
				{
					bDirectionChanged = 1;
				}

				// Find finish type
				if( bDirectionChanged || wPauseTime || playing_finished == true )
				{
					bpFinishType[bID] = ZERO_FINISH;
				}
				else
				{
					bpFinishType[bID] = NONE_ZERO_FINISH;
				}

				if( play_page_.header.schedule == ACTION_FILE::SPEED_BASE_SCHEDULE )
				{
					//MaxAngle1024 update
					if( ipMovingAngle1024[bID] < 0 )
						wTmp = -ipMovingAngle1024[bID];
					else
						wTmp = ipMovingAngle1024[bID];

					if( wTmp > wMaxAngle1024 )
						wMaxAngle1024 = wTmp;
				}

			}

            //占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌묒궪�삕占쎈�占쎄콬�뜝�럡�렊占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅곗녇占쎌뵞繹먮쨪�삕占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺙占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 7.8msec占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�(<<7)-占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺔占쎈쐻占쎈윥筌묒궪�삕占쎈�占쎄콬�뜝�럡�렊占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆�씢�쐻占쎈윪占쎈�뀐옙�쐻占쎈윪�뤃占� 7.8msec占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욌�ㅻ쨬占쎈즵�맱�슱�삕占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃�먯삕占쎈쐵�뜝�럥彛뺝뜝�럥�넅占쎈쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
            //占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윪占쎄섬占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꼦占쎈쐻占쎈윪占쎈�뀐옙�쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�/占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈걢占쎈쐻占쎈윥�뜎�뛼�쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆節륁삕占쎈�占쎄콬�뜝�럡�렊(占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌묒궪�삕占쎈�占쎄콬�뜝�럡�렊)占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌묒궪�삕占쎈�占쎄콬�뜝�럡�렊占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺙占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 7.8msec占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욌�ㅻ쨬占쎈즵�맱�슱�삕占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃�먯삕占쎈쐵�뜝�럥彛뺝뜝�럥�넅占쎈쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈ぎ占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占�
            //占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윪占쎄섬 ---  占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� :1024占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�->300占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�,  占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈걢占쎈쐻占쎈윥�뜎�뛼�쐻占쎈윪�뤃占�: 256占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� ->720占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�
            //wUnitTimeNum = ((wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) /7.8msec;
            //             = ((128*wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) ;    (/7.8msec == *128)
            //             = (wMaxAngle1024*40) /(wMaxSpeed256 *3);
            if( play_page_.header.schedule == ACTION_FILE::TIME_BASE_SCHEDULE )
                wUnitTimeTotalNum  = wMaxSpeed256; //TIME BASE 051025
            else
                wUnitTimeTotalNum  = (wMaxAngle1024 * 40) / (wMaxSpeed256 * 3);

            wAccelStep = play_page_.header.accel;
            if( wUnitTimeTotalNum <= (wAccelStep << 1) )
            {
                if( wUnitTimeTotalNum == 0 )
                    wAccelStep = 0;
                else
                {
                    wAccelStep = (wUnitTimeTotalNum - 1) >> 1;
                    if( wAccelStep == 0 )
                        wUnitTimeTotalNum = 0; //占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆蹂≪땡占쎌뒦占쎈쳮�뜝�럡�렊占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욌�ｋ쐻占쎈윥筌앾옙 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�,占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍�뜝�럥�맶占쎈쐻�뜝占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥�젆蹂잙쐻占쎈윞占쎈릊占쎈쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃轅⑤쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎈꺕占쎈쐻占쎈윥占쎄퐫占쎈쐻占쎈윪�뤃占�
                }
            }

            ulTotalTime256T = ((unsigned long)wUnitTimeTotalNum) << 1;// /128 * 256
            ulPreSectionTime256T = ((unsigned long)wAccelStep) << 1;// /128 * 256
            ulMainTime256T = ulTotalTime256T - ulPreSectionTime256T;
            lDivider1 = ulPreSectionTime256T + (ulMainTime256T << 1);
            lDivider2 = (ulMainTime256T << 1);

            if(lDivider1 == 0)
                lDivider1 = 1;

            if(lDivider2 == 0)
                lDivider2 = 1;

            for(unsigned int _jointIndex = 0; _jointIndex < ACTION_FILE::MAXNUM_JOINTS; _jointIndex++)
			{
				bID = _jointIndex;
				lStartSpeed1024_PreTime_256T = (long)ipLastOutSpeed1024[bID] * ulPreSectionTime256T; //  *300/1024 * 1024/720 * 256 * 2
				lMovingAngle_Speed1024Scale_256T_2T = (((long)ipMovingAngle1024[bID]) * 2560L) / 12;

				if( bpFinishType[bID] == ZERO_FINISH )
					ipMainSpeed1024[bID] = (short int)((lMovingAngle_Speed1024Scale_256T_2T - lStartSpeed1024_PreTime_256T) / lDivider2);
				else
					ipMainSpeed1024[bID] = (short int)((lMovingAngle_Speed1024Scale_256T_2T - lStartSpeed1024_PreTime_256T) / lDivider1);

				if( ipMainSpeed1024[bID] > 1023 )
					ipMainSpeed1024[bID] = 1023;

				if( ipMainSpeed1024[bID] < -1023 )
					ipMainSpeed1024[bID] = -1023;

			}
            wUnitTimeNum = wAccelStep; //PreSection
        }
    }
}


void ActionModule::PublishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg _status;
    _status.header.stamp = ros::Time::now();
    _status.type = type;
    _status.module_name = "Action";
    _status.status_msg = msg;

    status_msg_pub_.publish(_status);
}

