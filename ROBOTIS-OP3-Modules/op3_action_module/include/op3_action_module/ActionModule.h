/*
 * motion_module_tutorial.h
 *
 *  Created on: 2016. 2. 23.
 *      Author: zerom
 */

#ifndef ACTION_MOTION_MODULE_H_
#define ACTION_MOTION_MODULE_H_

#define _USE_MATH_DEFINES
#include <cmath>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/MotionModule.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "ActionFileDefine.h"

namespace ROBOTIS
{

class ActionModule : public MotionModule
{
private:
    static ActionModule *unique_instance_;

    int             control_cycle_msec_;
    boost::thread   queue_thread_;

    /* sample subscriber & publisher */
    ros::Subscriber action_page_sub_;

    ros::Publisher  status_msg_pub_;


    ActionModule();

    void QueueThread();


    /////////////////////////////////////////////////////////////////////////
    std::map<std::string, int> joint_name_to_id_;
    std::map<int, std::string> joint_id_to_name_;
    FILE* action_file_;
    ACTION_FILE::PAGE play_page_;
    ACTION_FILE::PAGE next_play_page_;
    ACTION_FILE::STEP current_step_;


	int  play_page_idx_;
	bool first_driving_start_;
	int  page_step_count_;

	bool playing_;
	bool stop_playing_;
	bool playing_finished;

	bool VerifyChecksum( ACTION_FILE::PAGE* _page );
	void SetChecksum( ACTION_FILE::PAGE* _page );

	int  RadTow4095(double _rad);
	double w4095ToRad(int _w4095);

	void PublishStatusMsg(unsigned int type, std::string msg);

	std::string IntToString(int _n);

	bool previous_enable_;
	bool present_enable_;
	bool previous_running_;
	bool present_running_;

//////////////////////////////////////////////////////////////
public:
    virtual ~ActionModule();

    static ActionModule *GetInstance() { return unique_instance_; }

    void    Initialize(const int control_cycle_msec, Robot *robot);
    void    Process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors);

    void	Stop();
    bool	IsRunning();

    ////////////////////////////////////////////////////////////////
    void PageNumberCallback(const std_msgs::Int32::ConstPtr& _msg);

	bool LoadFile(char* _file_name);
	bool CreateFile(char* _file_name);

	bool Start(int _page_number);
	bool Start(char* _page_name);
	bool Start(int _page_number, ACTION_FILE::PAGE *_page);

	void Brake();
	bool IsRunning(int* _playing_page_num, int* _playing_step_num);
	bool LoadPage(int _page_number, ACTION_FILE::PAGE* _page);
	bool SavePage(int _page_number, ACTION_FILE::PAGE* _page);
	void ResetPage(ACTION_FILE::PAGE* _page);

	void ActionPlayProcess(std::map<std::string, Dynamixel *> dxls);

protected:
    void OnEnable();
    void OnDisable();
};

}



#endif /* THORMANG3_ACTION_MOTION_MODULE_H_ */
