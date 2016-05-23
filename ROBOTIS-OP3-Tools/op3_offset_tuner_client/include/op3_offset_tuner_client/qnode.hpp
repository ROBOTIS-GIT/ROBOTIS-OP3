/**
 * @file /include/op3_offset_tuner_client/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef op3_offset_tuner_client_QNODE_HPP_
#define op3_offset_tuner_client_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <yaml-cpp/yaml.h>

#include <std_msgs/String.h>

#include "op3_offset_tuner_msgs/JointOffsetData.h"
#include "op3_offset_tuner_msgs/JointOffsetPositionData.h"
#include "op3_offset_tuner_msgs/JointTorqueOnOff.h"
#include "op3_offset_tuner_msgs/JointTorqueOnOffArray.h"

#include "op3_offset_tuner_msgs/GetPresentJointOffsetData.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op3_offset_tuner_client {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

    void send_torque_enable_msg( op3_offset_tuner_msgs::JointTorqueOnOffArray msg );
    void send_joint_offset_data_msg( op3_offset_tuner_msgs::JointOffsetData msg );
    void send_command_msg( std_msgs::String msg );
    bool is_refresh() {return is_refresh_;}

    std::map<int, std::string> right_arm_offset_group;
    std::map<int, std::string> left_arm_offset_group;
    std::map<int, std::string> legs_offset_group;
    std::map<int, std::string> body_offset_group;

public Q_SLOTS:
    void getPresentJointOffsetData();

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

    void update_present_joint_offset_data( op3_offset_tuner_msgs::JointOffsetPositionData msg );

private:
	int init_argc;
	char** init_argv;
	bool is_refresh_;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    ros::Publisher joint_offset_data_pub;
    ros::Publisher torque_enable_pub;
    ros::Publisher command_pub;

    ros::ServiceClient get_present_joint_offset_data_client;

    void ParseOffsetGroup(const std::string &path);

};

}  // namespace op3_offset_tuner_client

#endif /* op3_offset_tuner_client_QNODE_HPP_ */
