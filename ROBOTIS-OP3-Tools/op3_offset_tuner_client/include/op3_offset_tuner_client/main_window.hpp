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
