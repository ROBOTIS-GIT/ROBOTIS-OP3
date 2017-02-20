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
 ** Ifdefs
 *****************************************************************************/

#ifndef OP3_DEMO_QNODE_HPP_
#define OP3_DEMO_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <sstream>

#include <QThread>
#include <QStringListModel>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/GetJointModule.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

// walking demo
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/SetWalkingParam.h"

#endif
/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace robotis_op
{

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNodeOP3 : public QThread
{
Q_OBJECT
 public:

  enum LogLevel
  {
    Debug = 0,
    Info = 1,
    Warn = 2,
    Error = 3,
    Fatal = 4
  };

//    enum ControlIndex
//    {
//        Control_None = 0,
//        Control_Walking = 1,
//        Control_Manipulation = 2,
//        Control_Head = 3,
//    };

  QNodeOP3(int argc, char** argv);
  virtual ~QNodeOP3();

  bool init();
  void run();

  QStringListModel* loggingModel()
  {
    return &logging_model_;
  }
  void log(const LogLevel &level, const std::string &msg, std::string sender = "Demo");
  void clearLog();
  void assemble_lidar();
  void setJointControlMode(const robotis_controller_msgs::JointCtrlModule &msg);
  void setControlMode(const std::string &mode);
  bool getJointNameFromID(const int &id, std::string &joint_name);
  bool getIDFromJointName(const std::string &joint_name, int &id);
  bool getIDJointNameFromIndex(const int &index, int &id, std::string &joint_name);
  std::string getModeName(const int &index);
  int getModeIndex(const std::string &mode_name);
  int getModeSize();
  int getJointSize();
  void clearUsingModule();
  bool isUsingModule(std::string module_name);
  void moveInitPose();

  void setHeadJoint(double pan, double tilt);

  // Walking
  void setWalkingCommand(const std::string &command);
  void refreshWalkingParam();
  void saveWalkingParam();
  void applyWalkingParam(const op3_walking_module_msgs::WalkingParam &walking_param);
  void initGyro();
  //        void setWalkingBalance(bool on_command);

  // Demo
  void setDemoCommand(const std::string &command);
  void setActionModuleBody();
  void setModuleToDemo();

  std::map<int, std::string> module_table_;
  std::map<int, std::string> motion_table_;
  std::map<int, int> motion_shortcut_table_;

 public Q_SLOTS:
  void getJointControlMode();
  void playMotion(int motion_index);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void updateCurrentJointControlMode(std::vector<int> mode);

  // Manipulation
//    void updateCurrJoint( double value );
//    void updateCurrPos( double x , double y , double z );
//    void updateCurrOri( double x , double y , double z , double w );

// Head
  void updateHeadAngles(double pan, double tilt);

  // Walking
  void updateWalkingParameters(op3_walking_module_msgs::WalkingParam params);

 private:
  void parseJointNameFromYaml(const std::string &path);
  void parseMotionMapFromYaml(const std::string &path);
  void refreshCurrentJointControlCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg);
  void updateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);

  int init_argc_;
  char** init_argv_;
  bool debug_;

  op3_walking_module_msgs::WalkingParam walking_param_;

  ros::Publisher init_pose_pub_;
  ros::Publisher module_control_pub_;
  ros::Publisher module_control_preset_pub_;
  ros::Publisher init_gyro_pub_;
  ros::Subscriber status_msg_sub_;
  ros::Subscriber init_ft_foot_sub_;
  ros::Subscriber both_ft_foot_sub_;
  ros::Subscriber current_module_control_sub_;
  ros::ServiceClient get_module_control_client_;

  // Head
  ros::Publisher set_head_joint_angle_pub_;
  ros::Subscriber current_joint_states_sub_;

  // Manipulation
//    ros::Publisher set_control_mode_msg_pub;
//    ros::Publisher send_ini_pose_msg_pub;
//    ros::Publisher send_des_joint_msg_pub;
//    ros::Publisher send_ik_msg_pub;
//    ros::Publisher send_pathplan_demo_pub;
//    ros::Subscriber kenematics_pose_sub;
//    ros::ServiceClient get_joint_pose_client;
//    ros::ServiceClient get_kinematics_pose_client;

// Walking
  ros::Publisher set_walking_command_pub;
//    ros::Publisher set_walking_balance_pub;
  ros::Publisher set_walking_param_pub;
  ros::ServiceClient get_walking_param_client_;

  // Action
  ros::Publisher motion_index_pub_;

  // Demo
  ros::Publisher demo_command_pub_;

  ros::Time start_time_;

  QStringListModel logging_model_;
  std::map<int, std::string> id_joint_table_;
  std::map<std::string, int> joint_id_table_;

  std::map<int, std::string> index_mode_table_;
  std::map<std::string, int> mode_index_table_;
  std::map<std::string, bool> using_mode_table_;
};

}  // namespace robotis_op

#endif /* OP3_DEMO_QNODE_HPP_ */
