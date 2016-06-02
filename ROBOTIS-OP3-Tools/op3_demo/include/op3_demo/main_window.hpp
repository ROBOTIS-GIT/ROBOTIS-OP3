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

namespace op3_demo
{

#define deg2rad     (M_PI / 180.0)
#define rad2deg     (180.0 / M_PI)

/*****************************************************************************
 ** Interface [MainWindow]
 *****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
Q_OBJECT

 public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void ReadSettings();  // Load up qt program settings at startup
  void WriteSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent *event);  // Overloaded function
  void showNoMasterMessage();

 public Q_SLOTS:

  /******************************************
   ** Auto-connections (connectSlotsByName())
   *******************************************/
  void on_actionAbout_triggered();
  void on_button_clear_log_clicked(bool check);
  void on_button_init_pose_clicked(bool check);

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

  /******************************************
   ** Manual connections
   *******************************************/
  void updateLoggingView();  // no idea why this can't connect automatically
  void setMode(bool check);
  void updateCurrentJointMode(std::vector<int> mode);
  void setMode(QString mode_name);

  // Head Control
  void updateHeadAngles(double pan, double tilt);

  // Manipulation
  void updateCurrJointSpinbox(double value);
  void updateCurrPosSpinbox(double x, double y, double z);
  void updateCurrOriSpinbox(double x, double y, double z, double w);

  // Walking
  void updateWalkingParams(op3_walking_module_msgs::WalkingParam params);

 protected Q_SLOTS:
  void setHeadAngle();

 private:
  void setUserShortcut();
  void initModeUnit();
  void initMotionUnit();
  void updateModuleUI();
  void setHeadAngle(double pan, double tilt);
  void applyWalkingParams();

  /******************************************
   ** Transformation
   *******************************************/
  Eigen::MatrixXd rx(double s);
  Eigen::MatrixXd ry(double s);
  Eigen::MatrixXd rz(double s);
  Eigen::MatrixXd rpy2rotation(double r, double p, double y);
  Eigen::Quaterniond rpy2quaternion(double r, double p, double y);

  Ui::MainWindowDesign ui;
  QNodeOP3 qnode_op3;
  bool DEBUG;
  bool is_updating_;
  std::map<std::string, QList<QWidget *> > module_ui_table_;
};

}  // namespace op3_demo

#endif // thor3_control_MAIN_WINDOW_H
