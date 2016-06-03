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

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/op3_offset_tuner_client/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op3_offset_tuner_client {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv),
	is_refresh_(false)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"op3_offset_tuner_client");

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

    // Add your ros communications here
    joint_offset_data_pub = n.advertise<op3_offset_tuner_msgs::JointOffsetData>("/robotis/offset_tuner/joint_offset_data", 0);
    torque_enable_pub = n.advertise<op3_offset_tuner_msgs::JointTorqueOnOffArray>("/robotis/offset_tuner/torque_enable", 0);
    command_pub = n.advertise<std_msgs::String>("/robotis/offset_tuner/command", 0);

    get_present_joint_offset_data_client = n.serviceClient<op3_offset_tuner_msgs::GetPresentJointOffsetData>("/robotis/offset_tuner/get_present_joint_offset_data");

    std::string _config_path = ros::package::getPath("op3_offset_tuner_client") +"/config/joint_data.yaml";
    ParseOffsetGroup(_config_path);

	start();
	return true;
}

void QNode::run() {

    ros::Rate loop_rate(125);

	while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();

	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::send_torque_enable_msg( op3_offset_tuner_msgs::JointTorqueOnOffArray msg )
{
    torque_enable_pub.publish( msg );

    log( Info , "Joint Torque On/Off" );
}

void QNode::send_joint_offset_data_msg( op3_offset_tuner_msgs::JointOffsetData msg )
{
    joint_offset_data_pub.publish( msg );

    log( Info , "Send Joint Offset Data" );
}

void QNode::send_command_msg( std_msgs::String msg )
{
    command_pub.publish( msg );

    std::stringstream log_msg;
    log_msg << "Send Command : " << msg.data ;

    log( Info , log_msg.str() );
}

void QNode::getPresentJointOffsetData()
{
    is_refresh_ = true;

    op3_offset_tuner_msgs::GetPresentJointOffsetData _get_present_joint_offset_data;

    //request


    //response
    if ( get_present_joint_offset_data_client.call( _get_present_joint_offset_data ) )
    {
        for ( int id = 0; id < _get_present_joint_offset_data.response.present_data_array.size(); id++ )
        {
            op3_offset_tuner_msgs::JointOffsetPositionData _temp = _get_present_joint_offset_data.response.present_data_array[ id ];

            Q_EMIT update_present_joint_offset_data( _temp );
        }
    }
    else
        log(Error, "Fail to get joint offset data");

    is_refresh_ = false;
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}


void QNode::ParseOffsetGroup(const std::string &path)
{
    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile(path.c_str());
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Fail to load offset config yaml.");
        return;
    }

    // parse right_arm
    YAML::Node _right_arm_node = doc["right_arm"];
    for(YAML::iterator _it = _right_arm_node.begin() ; _it != _right_arm_node.end() ; ++_it)
    {
        int _index;
        std::string _joint_name;

        _index = _it->first.as<int>();
        _joint_name = _it->second.as<std::string>();

        right_arm_offset_group[_index] = _joint_name;
    }

    YAML::Node _left_arm_node = doc["left_arm"];
    for(YAML::iterator _it = _left_arm_node.begin() ; _it != _left_arm_node.end() ; ++_it)
    {
        int _index;
        std::string _joint_name;

        _index = _it->first.as<int>();
        _joint_name = _it->second.as<std::string>();

        left_arm_offset_group[_index] = _joint_name;
    }

    YAML::Node _legs_node = doc["legs"];
    for(YAML::iterator _it = _legs_node.begin() ; _it != _legs_node.end() ; ++_it)
    {
        int _index;
        std::string _joint_name;

        _index = _it->first.as<int>();
        _joint_name = _it->second.as<std::string>();

        legs_offset_group[_index] = _joint_name;
    }

    YAML::Node _body_node = doc["body"];
    for(YAML::iterator _it = _body_node.begin() ; _it != _body_node.end() ; ++_it)
    {
        int _index;
        std::string _joint_name;

        _index = _it->first.as<int>();
        _joint_name = _it->second.as<std::string>();

        body_offset_group[_index] = _joint_name;
    }
}

}  // namespace op3_offset_tuner_client