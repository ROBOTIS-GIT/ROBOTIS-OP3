/**
 * @file /include/thor3_control/main_window.hpp
 *
 * @brief Qt based gui for thor3_control.
 *
 * @date November 2010
 **/
#ifndef thor3_control_MAIN_WINDOW_H
#define thor3_control_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace op3_demo {

#define deg2rad     (M_PI / 180.0)
#define rad2deg     (180.0 / M_PI)

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
        Q_OBJECT

    public:
        MainWindow(int argc, char** argv, QWidget *parent = 0);
        ~MainWindow();

        void ReadSettings(); // Load up qt program settings at startup
        void WriteSettings(); // Save qt program settings when closing

        void closeEvent(QCloseEvent *event); // Overloaded function
        void showNoMasterMessage();

    public Q_SLOTS:

        /******************************************
        ** Auto-connections (connectSlotsByName())
        *******************************************/
        void on_actionAbout_triggered();
        void on_button_clear_log_clicked(bool check);
        void on_button_init_pose_clicked(bool check);

        /*
        // Manipulation
        void on_inipose_button_clicked( bool check );
        void on_currjoint_button_clicked( bool check );
        void on_desjoint_button_clicked( bool check );
        void on_currpos_button_clicked( bool check );
        void on_despos_button_clicked( bool check );
        void on_button_grip_on_clicked(bool check);
        void on_button_grip_off_clicked(bool check);
        void on_button_pathplanning_ini_clicked(bool check);
        void on_button_pathplanning_a_clicked(bool check);
        void on_button_pathplanning_b_clicked(bool check);
        */

        // Walking
        void on_button_walking_start_clicked(bool check);
        void on_button_walking_stop_clicked(bool check);

        void on_button_param_refresh_clicked(bool check);
        void on_button_param_apply_clicked(bool check);
        void on_button_param_save_clicked(bool check);

        void on_checkBox_balance_on_clicked(bool check);
        void on_checkBox_balance_off_clicked(bool check);

        // Head Control
        void on_head_center_button_clicked(bool check);

        // Demo
        void on_button_demo_start_clicked(bool check);
        void on_button_demo_stop_clicked(bool check);
        void on_button_r_kick_clicked(bool check);
        void on_button_l_kick_clicked(bool check);
        void on_button_getup_front_clicked(bool check);
        void on_button_getup_back_clicked(bool check);

        /******************************************
        ** Manual connections
        *******************************************/
        void updateLoggingView(); // no idea why this can't connect automatically
        void setMode(bool check);
        void updateCurrentJointMode(std::vector<int> mode);
        void setMode(QString mode_name);

        // Head Control
        void updateHeadAngles(double pan, double tilt);

        // Manipulation
        void updateCurrJointSpinbox( double value );
        void updateCurrPosSpinbox( double x , double y , double z  );
        void updateCurrOriSpinbox( double x , double y , double z , double w );

        // Walking
        void updateWalkingParams(op3_walking_module_msgs::WalkingParam params);
        void walkingCommandShortcut();

    private:
        Ui::MainWindowDesign ui;
        QNodeOP3 qnode_op3;
        bool DEBUG;
        void setUserShortcut();
        void initModeUnit();
        void initMotionUnit();
        bool is_updating_;
        bool is_walking_;
        std::map< std::string, QList<QWidget *> > module_ui_table_;
        void updateModuleUI();
        void setHeadAngle(double pan, double tilt);
        void applyWalkingParams();

        //void sendWalkingCommand(const std::string &command);
        //void sendDemoMsg(const std::string &demo_command);

        /******************************************
        ** Transformation
        *******************************************/
        Eigen::MatrixXd rx( double s );
        Eigen::MatrixXd ry( double s );
        Eigen::MatrixXd rz( double s );
        Eigen::MatrixXd rpy2rotation(double r, double p, double y);
        Eigen::Quaterniond rpy2quaternion(double r, double p, double y);

    protected Q_SLOTS:
        void setHeadAngle();
};

}  // namespace thor3_control

#endif // thor3_control_MAIN_WINDOW_H
