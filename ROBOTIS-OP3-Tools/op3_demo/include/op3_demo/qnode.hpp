/**
 * @file /include/thor3_control/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef op3_control_QNODE_HPP_
#define op3_control_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

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

//#include "thormang3_base_module_msgs/BothWrench.h"
//#include "thormang3_base_module_msgs/CalibrationWrench.h"

// manipulation demo
//#include "thormang3_manipulation_module_msgs/JointPose.h"
//#include "thormang3_manipulation_module_msgs/DemoPose.h"
//#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
//#include "thormang3_manipulation_module_msgs/GetJointPose.h"
//#include "thormang3_manipulation_module_msgs/GetKinematicsPose.h"

// walking demo
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/SetWalkingParam.h"
//#include "thormang3_foot_step_generator/FootStepCommand.h"
//#include <std_msgs/Bool.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op3_demo {

/*****************************************************************************
** Class
*****************************************************************************/

class QNodeOP3 : public QThread {
        Q_OBJECT
    public:
        QNodeOP3(int argc, char** argv );
        virtual ~QNodeOP3();
        bool init();
        void run();

        /*********************
    ** Logging
    **********************/
        enum LogLevel {
            Debug = 0,
            Info = 1,
            Warn = 2,
            Error = 3,
            Fatal = 4
        };

        enum CONTROL_INDEX
        {
            Control_None = 0,
            Control_Walking = 1,
            Control_Manipulation = 2,
            Control_Head = 3,
        };

        std::map< int, std::string > module_table;
        std::map< int, std::string> motion_table;
        std::map< int, int> shortcut_table;

        QStringListModel* loggingModel() { return &logging_model_; }
        void log(const LogLevel &level, const std::string &msg, std::string sender="Demo");
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
        void initFTCommand(std::string command);

        void setHeadJoint(double pan, double tilt);

        // Walking
        void setWalkingCommand(const std::string &command);
        void refreshWalkingParam();
        void saveWalkingParam();
        void applyWalkingParam(const op3_walking_module_msgs::WalkingParam &walking_param);
//        void setWalkingBalance(bool on_command);

        // Demo
        void setDemoCommand(const std::string &command);
        void setActionModuleBody();
        void setModuleToDemo();

    public Q_SLOTS:
        void getJointControlMode();
        void getJointPose( std::string joint_name );
        void getKinematicsPose (std::string group_name );
        void getKinematicsPoseCallback(const geometry_msgs::Pose::ConstPtr &msg);
        void playMotion(int motion_index);

    Q_SIGNALS:
        void loggingUpdated();
        void rosShutdown();
        void updateCurrentJointControlMode(std::vector<int> mode);

        // Manipulation
        void updateCurrJoint( double value );
        void updateCurrPos( double x , double y , double z );
        void updateCurrOri( double x , double y , double z , double w );

        void updateHeadAngles(double pan, double tilt);

        // Walking
        void updateWalkingParameters(op3_walking_module_msgs::WalkingParam params);

    private:
        int init_argc;
        char** init_argv;
        bool DEBUG;

        op3_walking_module_msgs::WalkingParam walking_param_;

        ros::Publisher init_pose_pub_;
        ros::Publisher init_ft_pub_;
        ros::Publisher module_control_pub_;
        ros::Publisher module_control_preset_pub_;
        ros::Subscriber status_msg_sub_;
        ros::Subscriber init_ft_foot_sub_;
        ros::Subscriber both_ft_foot_sub_;
        ros::Subscriber current_module_control_sub_;
        ros::ServiceClient get_module_control_client_;

        // Head
        ros::Publisher move_lidar_pub_;
        ros::Publisher set_head_joint_angle_pub_;
        ros::Subscriber current_joint_states_sub_;

        // Manipulation
        ros::Publisher set_control_mode_msg_pub;
        ros::Publisher send_ini_pose_msg_pub;
        ros::Publisher send_des_joint_msg_pub;
        ros::Publisher send_ik_msg_pub;
        ros::Publisher send_pathplan_demo_pub;
        ros::Subscriber kenematics_pose_sub;
        ros::ServiceClient get_joint_pose_client;
        ros::ServiceClient get_kinematics_pose_client;

        // Walking
        ros::Publisher set_walking_command_pub;
        ros::Publisher set_walking_balance_pub;
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


        void parseJointNameFromYaml(const std::string &path);
        void parseMotionMapFromYaml(const std::string &path);
        void refreshCurrentJointControlCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg);
        void updateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
//        void initFTFootCallback(const thormang3_base_module_msgs::BothWrench::ConstPtr &msg);
//        void calibrationFTFootCallback(const thormang3_base_module_msgs::CalibrationWrench::ConstPtr &msg);
        void statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);
};

}  // namespace thor3_control

#endif /* op3_control_QNODE_HPP_ */
