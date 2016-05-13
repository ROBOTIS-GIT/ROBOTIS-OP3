/**
 * @file /include/op3_offset_tuner_client/main_window.hpp
 *
 * @brief Qt based gui for op3_offset_tuner_client.
 *
 * @date November 2010
 **/
#ifndef op3_offset_tuner_client_MAIN_WINDOW_H
#define op3_offset_tuner_client_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QList>
#include <QSpinBox>
#include <QtGui/QMainWindow>

#include <math.h>

#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace op3_offset_tuner_client {

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

	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();

    void on_save_button_clicked( bool check );
    void on_refresh_button_clicked( bool check );
    void on_inipose_button_clicked( bool checck );


    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    void update_joint_offset_data_spinbox( op3_offset_tuner_msgs::JointOffsetPositionData msg );

    void spinBox_valueChanged(QString joint_name);
    void torque_checkbox_clicked(QWidget *widget);
    void all_torque_on_button_clicked(QObject *button_group );
    void all_torque_off_button_clicked(QObject *button_group );

private:
	Ui::MainWindowDesign ui;
	QNode qnode;

    bool all_torque_on;

    QButtonGroup *right_arm_button_group_;
    QButtonGroup *left_arm_button_group_;
    QButtonGroup *legs_button_group_;
    QButtonGroup *body_button_group_;
    std::vector<std::string> spinBox_list_;

    void MakeUI();
    void MakeTabUI(QGroupBox *joint_widget, QGroupBox *torque_widget, QButtonGroup *button_group, std::map<int, std::string> &offset_group);
    void publish_torque_msgs(std::string &joint_name, bool torque_on );

    std::map<std::string, QList<QAbstractSpinBox *> > joint_spinbox_map_;
};

}  // namespace op3_offset_tuner_client

#endif // op3_offset_tuner_client_MAIN_WINDOW_H
