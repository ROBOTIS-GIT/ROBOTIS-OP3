/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/op3_demo/main_window.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace op3_demo {

using namespace Qt;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode_op3(argc,argv)
  , is_updating_(false)
  , is_walking_(false)
{
  // code to DEBUG
  DEBUG = false;

  if(argc >= 2)
  {
    std::string _debug_code(argv[1]);
    if(_debug_code == "debug")
      DEBUG = true;
    else
      DEBUG = false;
  }

  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode_op3, SIGNAL(rosShutdown()), this, SLOT(close()));

  qRegisterMetaType< std::vector<int> >("std::vector<int>");
  QObject::connect(&qnode_op3, SIGNAL(updateCurrentJointControlMode(std::vector<int>)), this, SLOT(updateCurrentJointMode(std::vector<int>)));
  QObject::connect(&qnode_op3, SIGNAL(updateHeadAngles(double,double)), this, SLOT(updateHeadAngles(double,double)));

  QObject::connect(ui.head_pan_slider, SIGNAL(valueChanged(int)), this, SLOT(setHeadAngle()));
  QObject::connect(ui.head_tilt_slider, SIGNAL(valueChanged(int)), this, SLOT(setHeadAngle()));

  QObject::connect(&qnode_op3, SIGNAL(updateCurrJoint(double)), this, SLOT(updateCurrJointSpinbox(double)));
  QObject::connect(&qnode_op3, SIGNAL(updateCurrPos(double , double , double)), this, SLOT(updateCurrPosSpinbox(double , double , double)));
  QObject::connect(&qnode_op3, SIGNAL(updateCurrOri(double , double , double, double)), this, SLOT(updateCurrOriSpinbox(double , double , double , double)));

  qRegisterMetaType<op3_walking_module_msgs::WalkingParam>("op_walking_params");
  QObject::connect(&qnode_op3, SIGNAL(updateWalkingParameters(op3_walking_module_msgs::WalkingParam)), this, SLOT(updateWalkingParams(op3_walking_module_msgs::WalkingParam)));

  /*********************
     ** Logging
     **********************/
  ui.view_logging->setModel(qnode_op3.loggingModel());
  QObject::connect(&qnode_op3, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  /*********************
     ** Auto Start
     **********************/
  qnode_op3.init();
  initModeUnit();
  setUserShortcut();
  updateModuleUI();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/

void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

//void MainWindow::on_button_assemble_lidar_clicked(bool check) { qnode_thor3.assemble_lidar(); }
void MainWindow::on_button_clear_log_clicked(bool check) { qnode_op3.clearLog(); }
void MainWindow::on_button_init_pose_clicked(bool check) { qnode_op3.moveInitPose(); }

//void MainWindow::on_button_ft_air_clicked(bool check) { qnode_thor3.initFTCommand("ft_air"); }
//void MainWindow::on_button_ft_gnd_clicked(bool check) { qnode_thor3.initFTCommand("ft_gnd"); }
//void MainWindow::on_button_ft_calc_clicked(bool check)
//{
//    qnode_thor3.initFTCommand("ft_send");
//    qnode_thor3.log(QNodeOP3::Info, "Apply new FT config");
//}
//void MainWindow::on_button_ft_save_clicked(bool check)
//{
//    qnode_thor3.initFTCommand("ft_save");
//    qnode_thor3.log(QNodeOP3::Info, "Save FT config data.");
//}

/*
// Manipulation
void MainWindow::on_inipose_button_clicked( bool check )
{
    std_msgs::String msg;

    msg.data = "ini_pose";

    qnode_thor3.sendInitPoseMsg( msg );
}

void MainWindow::on_currjoint_button_clicked( bool check )
{
    qnode_thor3.getJointPose( ui.joint_combobox->currentText().toStdString() );
}

void MainWindow::on_desjoint_button_clicked( bool check )
{
    thormang3_manipulation_module_msgs::JointPose msg;

    msg.name = ui.joint_combobox->currentText().toStdString();
    msg.value = ui.joint_spinbox->value() * M_PI / 180.0 ;

    qnode_thor3.sendDestJointMsg( msg );
}

void MainWindow::on_currpos_button_clicked( bool check )
{
    qnode_thor3.getKinematicsPose( ui.group_combobox->currentText().toStdString() );
}

void MainWindow::on_despos_button_clicked( bool check )
{
    thormang3_manipulation_module_msgs::KinematicsPose msg;

    msg.name = ui.group_combobox->currentText().toStdString();

    msg.pose.position.x = ui.pos_x_spinbox->value();
    msg.pose.position.y = ui.pos_y_spinbox->value();
    msg.pose.position.z = ui.pos_z_spinbox->value();

    double roll = ui.ori_roll_spinbox->value() * M_PI / 180.0;
    double pitch = ui.ori_pitch_spinbox->value() * M_PI / 180.0;
    double yaw = ui.ori_yaw_spinbox->value() * M_PI / 180.0;

    Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

    msg.pose.orientation.x = QR.x();
    msg.pose.orientation.y = QR.y();
    msg.pose.orientation.z = QR.z();
    msg.pose.orientation.w = QR.w();

    qnode_thor3.sendIkMsg( msg );
}

void MainWindow::on_button_grip_on_clicked(bool check)
{
    thormang3_manipulation_module_msgs::JointPose _msg;

    _msg.name = ui.gripper_comboBox->currentText().toStdString();
    _msg.value = 60 * M_PI / 180.0 ;

    qnode_thor3.sendDestJointMsg( _msg );
}

void MainWindow::on_button_grip_off_clicked(bool check)
{
    thormang3_manipulation_module_msgs::JointPose _msg;

    _msg.name = ui.gripper_comboBox->currentText().toStdString();
    _msg.value = 0 * M_PI / 180.0 ;

    qnode_thor3.sendDestJointMsg( _msg );
}

void MainWindow::on_button_pathplanning_ini_clicked(bool check) { sendDemoMsg("ini_pose"); }
void MainWindow::on_button_pathplanning_a_clicked(bool check) { sendDemoMsg("line"); }
void MainWindow::on_button_pathplanning_b_clicked(bool check) { sendDemoMsg("circle"); }
 */

// Walking

void MainWindow::on_button_walking_start_clicked(bool check)
{
  is_walking_ = true;
  qnode_op3.setWalkingCommand("start");
}
void MainWindow::on_button_walking_stop_clicked(bool check)
{
  is_walking_ = false;
  qnode_op3.setWalkingCommand("stop");
}

void MainWindow::on_button_param_refresh_clicked(bool check)
{
  qnode_op3.refreshWalkingParam();
}

void MainWindow::on_button_param_save_clicked(bool check)
{
  qnode_op3.setWalkingCommand("save");
}

void MainWindow::on_button_param_apply_clicked(bool check)
{
  applyWalkingParams();
}

void MainWindow::on_checkBox_balance_on_clicked(bool check) { }
void MainWindow::on_checkBox_balance_off_clicked(bool check) { }


void MainWindow::on_head_center_button_clicked(bool check)
{
  //    is_updating_ == true;
  //    ui.head_pan_slider->setValue(0.0);
  //    ui.head_tilt_slider->setValue(0.0);
  //    is_updating_ == false;

  qnode_op3.log(QNodeOP3::Info, "Go Head init position");
  setHeadAngle(0, 0);
}

void MainWindow::on_button_demo_start_clicked(bool check)
{
  qnode_op3.setModuleToDemo();

  usleep(10 * 1000);

  qnode_op3.setDemoCommand("start");
}

void MainWindow::on_button_demo_stop_clicked(bool check)
{
  qnode_op3.setDemoCommand("stop");
}

void MainWindow::on_button_r_kick_clicked(bool check)
{
  // temporary code
  qnode_op3.setActionModuleBody();

  usleep(10 * 1000);

  qnode_op3.playMotion(83);
}

void MainWindow::on_button_l_kick_clicked(bool check)
{
  // temporary code
  qnode_op3.setActionModuleBody();

  usleep(10 * 1000);

  qnode_op3.playMotion(84);

}

void MainWindow::on_button_getup_front_clicked(bool check)
{
  // temporary code
  qnode_op3.setActionModuleBody();

  usleep(10 * 1000);

  qnode_op3.playMotion(81);

}

void MainWindow::on_button_getup_back_clicked(bool check)
{
  // temporary code
  qnode_op3.setActionModuleBody();

  usleep(10 * 1000);

  qnode_op3.playMotion(82);

}

/*****************************************************************************
 ** Implemenation [Slots][manually connected]
 *****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
  ui.view_logging->scrollToBottom();
}

// user shortcut
void MainWindow::setUserShortcut()
{
  // Setup a signal mapper to avoid creating custom slots for each tab
  QSignalMapper *_sig_map = new QSignalMapper(this);

  // Setup the shortcut for the first tab : Mode
  QShortcut *_short_tab1 = new QShortcut(QKeySequence("F1"), this);
  connect(_short_tab1, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab1, 0);

  // Setup the shortcut for the second tab : Manipulation
  QShortcut *_short_tab2 = new QShortcut(QKeySequence("F2"), this);
  connect(_short_tab2, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab2, 1);

  // Setup the shortcut for the third tab : Walking
  QShortcut *_short_tab3 = new QShortcut(QKeySequence("F3"), this);
  connect(_short_tab3, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab3, 2);

  // Setup the shortcut for the fouth tab : Head control
  QShortcut *_short_tab4 = new QShortcut(QKeySequence("F4"), this);
  connect(_short_tab4, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab4, 3);

  // Setup the shortcut for the fouth tab : Motion
  QShortcut *_short_tab5 = new QShortcut(QKeySequence("F5"), this);
  connect(_short_tab5, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab5, 4);

  // Wire the signal mapper to the tab widget index change slot
  connect(_sig_map, SIGNAL(mapped(int)), ui.tabWidget_control, SLOT(setCurrentIndex(int)));

  QShortcut *walking_shortcut = new QShortcut(QKeySequence(Qt::Key_Space), this);
  connect(walking_shortcut, SIGNAL(activated()), this, SLOT(walkingCommandShortcut()));
}

// mode control
// it's not used now
void MainWindow::setMode(bool check)
{
  robotis_controller_msgs::JointCtrlModule _control_msg;

  QList<QComboBox *> _combo_children = ui.widget_mode->findChildren<QComboBox *>();
  for(int ix = 0; ix < _combo_children.length(); ix++)
  {
    std::stringstream _stream;
    std::string _joint;
    int _id;

    int _control_index = _combo_children.at(ix)->currentIndex();
    // if(_control_index == QNodeThor3::Control_None) continue;

    std::string _control_mode = _combo_children.at(ix)->currentText().toStdString();

    if(qnode_op3.getIDJointNameFromIndex(ix, _id, _joint) == true)
    {
      _stream << "[" << (_id < 10 ? "0" : "") << _id << "] "<< _joint <<" : " << _control_mode;

      _control_msg.joint_name.push_back(_joint);
      _control_msg.module_name.push_back(_control_mode);
    }
    else
    {
      _stream << "id " << ix << " : " << _control_mode;
    }

    qnode_op3.log(QNodeOP3::Info, _stream.str());
  }

  // no control
  if(_control_msg.joint_name.size() == 0) return;

  qnode_op3.log(QNodeOP3::Info, "set mode");

  qnode_op3.setJointControlMode(_control_msg);
}

void MainWindow::updateCurrentJointMode(std::vector<int> mode)
{
  QList<QComboBox *> _combo_children = ui.widget_mode->findChildren<QComboBox *>();
  for(int ix = 0; ix < _combo_children.length(); ix++)
  {
    int _control_index = mode.at(ix);
    _combo_children.at(ix)->setCurrentIndex(_control_index);

    if(DEBUG)
    {
      std::stringstream _stream;
      std::string _joint;
      int _id;

      std::string _control_mode = _combo_children.at(ix)->currentText().toStdString();

      if(qnode_op3.getIDJointNameFromIndex(ix, _id, _joint) == true)
      {
        _stream << "[" << (_id < 10 ? "0" : "") << _id << "] "<< _joint <<" : " << _control_mode;
      }
      else
      {
        _stream << "id " << ix << " : " << _control_mode;
      }

      qnode_op3.log(QNodeOP3::Info, _stream.str());
    }
  }

  // set module UI
  updateModuleUI();
}

void MainWindow::updateModuleUI()
{
  if(DEBUG) return;

  for(int index = 0; index < qnode_op3.getModeSize(); index++)
  {
    std::string _mode = qnode_op3.getModeName(index);
    if(_mode == "") continue;

    std::map< std::string, QList<QWidget *> >::iterator _module_iter = module_ui_table_.find(_mode);
    if(_module_iter == module_ui_table_.end()) continue;

    bool _is_enable = qnode_op3.isUsingModule(_mode);

    QList<QWidget *> _list = _module_iter->second;
    for(int ix = 0; ix < _list.size(); ix++)
    {
      _list.at(ix)->setEnabled(_is_enable);
    }
  }

  // refresh walking parameter
  if(qnode_op3.isUsingModule("walking_module"))
    qnode_op3.refreshWalkingParam();
}

void MainWindow::updateHeadAngles(double pan, double tilt)
{
  if(ui.head_pan_slider->underMouse() == true) return;
  if(ui.head_pan_spinbox->underMouse() == true) return;
  if(ui.head_tilt_slider->underMouse() == true) return;
  if(ui.head_tilt_spinbox->underMouse() == true) return;

  is_updating_ = true;

  ui.head_pan_slider->setValue( pan * 180.0 / M_PI );
  // ui.head_pan_spinbox->setValue( pan * 180.0 / M_PI );
  ui.head_tilt_slider->setValue( tilt * 180.0 / M_PI );
  // ui.head_tilt_spinbox->setValue( tilt * 180.0 / M_PI );

  is_updating_ = false;
}

void MainWindow::setHeadAngle()
{
  if(is_updating_ == true) return;
  qnode_op3.setHeadJoint(ui.head_pan_slider->value() * M_PI / 180, ui.head_tilt_slider->value() * M_PI / 180);
}

void MainWindow::setHeadAngle(double pan, double tilt)
{
  qnode_op3.setHeadJoint(pan * M_PI / 180, tilt * M_PI / 180);
}


// manipulation
void MainWindow::updateCurrJointSpinbox( double value )
{
  ui.joint_spinbox->setValue( value * 180.0 / M_PI );
}

void MainWindow::updateCurrPosSpinbox( double x, double y, double z )
{
  ui.pos_x_spinbox->setValue( x );
  ui.pos_y_spinbox->setValue( y );
  ui.pos_z_spinbox->setValue( z );
}

void MainWindow::updateCurrOriSpinbox( double x , double y , double z , double w )
{
  Eigen::Quaterniond QR(w,x,y,z);

  Eigen::MatrixXd R = QR.toRotationMatrix();

  double roll = atan2( R.coeff(2,1), R.coeff(2,2) ) * 180.0 / M_PI;
  double pitch = atan2( -R.coeff(2,0), sqrt( pow(R.coeff(2,1),2) + pow(R.coeff(2,2),2) ) ) * 180.0 / M_PI;
  double yaw = atan2 ( R.coeff(1,0) , R.coeff(0,0) ) * 180.0 / M_PI;

  ui.ori_roll_spinbox->setValue( roll );
  ui.ori_pitch_spinbox->setValue( pitch );
  ui.ori_yaw_spinbox->setValue( yaw );
}

/*
void MainWindow::sendDemoMsg(const std::string &demo_command)
{
    thormang3_manipulation_module_msgs::DemoPose _msg;

    std::string _group_name = ui.demo_group_combobox->currentText().toStdString() + (demo_command == "ini_pose" ? "" : "_with_torso");
    _msg.name = _group_name;
    _msg.demo = demo_command;

    _msg.pose.position.x = 0.0;
    _msg.pose.position.y = 0.0;
    _msg.pose.position.z = 0.0;

    _msg.pose.orientation.x = 0.0;
    _msg.pose.orientation.y = 0.0;
    _msg.pose.orientation.z = 0.0;
    _msg.pose.orientation.w = 1.0;

    qnode_thor3.sendDemoMsg( _msg );
}
 */


// walking
void MainWindow::updateWalkingParams(op3_walking_module_msgs::WalkingParam params)
{
  // init pose
  ui.dSpinBox_init_offset_x->setValue(params.init_x_offset);
  ui.dSpinBox_init_offset_y->setValue(params.init_y_offset);
  ui.dSpinBox_init_offset_z->setValue(params.init_z_offset);
  ui.dSpinBox_init_offset_roll->setValue(params.init_roll_offset * rad2deg);
  ui.dSpinBox_init_offset_pitch->setValue(params.init_pitch_offset * rad2deg);
  ui.dSpinBox_init_offset_yaw->setValue(params.init_yaw_offset * rad2deg);
  ui.dSpinBox_hip_pitch_offset->setValue(params.hip_pitch_offset * rad2deg);
  // time
  ui.dSpinBox_period_time->setValue(params.period_time * 1000);       // s -> ms
  ui.dSpinBox_dsp_ratio->setValue(params.dsp_ratio);
  ui.dSpinBox_step_fb_ratio->setValue(params.step_fb_ratio);;
  // walking
  ui.dSpinBox_x_move_amplitude->setValue(params.x_move_amplitude);
  ui.dSpinBox_y_move_amplitude->setValue(params.y_move_amplitude);
  ui.dSpinBox_z_move_amplitude->setValue(params.z_move_amplitude);
  ui.dSpinBox_y_move_amplitude->setValue(params.angle_move_amplitude);
  ui.checkBox_move_aim_on->setChecked(params.move_aim_on);
  ui.checkBox_move_aim_off->setChecked(!params.move_aim_on);
  // balance
  ui.checkBox_balance_on->setChecked(params.balance_enable);
  ui.checkBox_balance_off->setChecked(!params.balance_enable);
  ui.dSpinBox_hip_roll_gain->setValue(params.balance_hip_roll_gain);
  ui.dSpinBox_knee_gain->setValue(params.balance_knee_gain);
  ui.dSpinBox_ankle_roll_gain->setValue(params.balance_ankle_roll_gain);
  ui.dSpinBox_ankle_pitch_gain->setValue(params.balance_ankle_pitch_gain);
  ui.dSpinBox_y_swap_amplitude->setValue(params.y_swap_amplitude);
  ui.dSpinBox_z_swap_amplitude->setValue(params.z_swap_amplitude);
  ui.dSpinBox_pelvis_offset->setValue(params.pelvis_offset * rad2deg);
  ui.dSpinBox_arm_swing_gain->setValue(params.arm_swing_gain);
}

void MainWindow::applyWalkingParams()
{
  op3_walking_module_msgs::WalkingParam _walking_param;

  // init pose
  _walking_param.init_x_offset            = ui.dSpinBox_init_offset_x->value();
  _walking_param.init_y_offset            = ui.dSpinBox_init_offset_y->value();
  _walking_param.init_z_offset            = ui.dSpinBox_init_offset_z->value();
  _walking_param.init_roll_offset         = ui.dSpinBox_init_offset_roll->value() * deg2rad;
  _walking_param.init_pitch_offset        = ui.dSpinBox_init_offset_pitch->value() * deg2rad;
  _walking_param.init_yaw_offset          = ui.dSpinBox_init_offset_yaw->value() * deg2rad;
  _walking_param.hip_pitch_offset         = ui.dSpinBox_hip_pitch_offset->value() * deg2rad;
  // time
  _walking_param.period_time              = ui.dSpinBox_period_time->value() * 0.001;     // ms -> s
  _walking_param.dsp_ratio                = ui.dSpinBox_dsp_ratio->value();
  _walking_param.step_fb_ratio            = ui.dSpinBox_step_fb_ratio->value();;
  // walking
  _walking_param.x_move_amplitude         = ui.dSpinBox_x_move_amplitude->value();
  _walking_param.y_move_amplitude         = ui.dSpinBox_y_move_amplitude->value();
  _walking_param.z_move_amplitude         = ui.dSpinBox_z_move_amplitude->value();
  _walking_param.angle_move_amplitude     = ui.dSpinBox_a_move_amplitude->value() * deg2rad;
  _walking_param.move_aim_on              = ui.checkBox_move_aim_on->isChecked();
  // balance
  _walking_param.balance_enable           = ui.checkBox_balance_on->isChecked();
  _walking_param.balance_hip_roll_gain    = ui.dSpinBox_hip_roll_gain->value();
  _walking_param.balance_knee_gain        = ui.dSpinBox_knee_gain->value();
  _walking_param.balance_ankle_roll_gain  = ui.dSpinBox_ankle_roll_gain->value();
  _walking_param.balance_ankle_pitch_gain = ui.dSpinBox_ankle_pitch_gain->value();
  _walking_param.y_swap_amplitude         = ui.dSpinBox_y_swap_amplitude->value();
  _walking_param.z_swap_amplitude         = ui.dSpinBox_z_swap_amplitude->value();
  _walking_param.pelvis_offset            = ui.dSpinBox_pelvis_offset->value() * deg2rad;
  _walking_param.arm_swing_gain           = ui.dSpinBox_arm_swing_gain->value();

  qnode_op3.applyWalkingParam(_walking_param);
}

void MainWindow::walkingCommandShortcut()
{
  if(is_walking_ == true)
  {
    is_walking_ = false;
    qnode_op3.setWalkingCommand("stop");
  }
  else
  {
    is_walking_ = true;
    qnode_op3.setWalkingCommand("start");
  }
}

/*****************************************************************************
 ** Implementation [Menu]
 *****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
 ** Implementation [Configuration]
 *****************************************************************************/

void MainWindow::initModeUnit()
{
  // std::string _module[] = {"None", "Walking", "Manipulation", "Action", "Head", "Direct"};
  int _number_joint = qnode_op3.getJointSize();

  // preset button
  QHBoxLayout *_preset_layout = new QHBoxLayout;
  QSignalMapper *_signalMapper = new QSignalMapper(this);

  // yaml preset
  for(std::map< int, std::string >::iterator _iter = qnode_op3.module_table.begin(); _iter != qnode_op3.module_table.end(); ++_iter)
  {
    std::string _preset_name = _iter->second;
    QPushButton *_preset_button = new QPushButton(tr(_preset_name.c_str()));
    if(DEBUG) std::cout << "name : " <<  _preset_name << std::endl;

    _preset_layout->addWidget(_preset_button);

    _signalMapper->setMapping(_preset_button, _preset_button->text());
    QObject::connect(_preset_button, SIGNAL(clicked()), _signalMapper, SLOT(map()));
  }

  // QObject::connect(_signalMapper, SIGNAL(mapped(QString)), this, SLOT(setPreset(QString)));
  QObject::connect(_signalMapper, SIGNAL(mapped(QString)), this, SLOT(setMode(QString)));

  ui.widget_mode_preset->setLayout(_preset_layout);

  // joints
  QGridLayout *_grid_mod = new QGridLayout;
  for(int ix = 0; ix < _number_joint; ix++)
  {
    std::stringstream _stream;
    std::string _joint;
    int _id;

    if(qnode_op3.getIDJointNameFromIndex(ix, _id, _joint) == false) continue;

    _stream << "[" << (_id < 10 ? "0" : "") << _id << "] "<< _joint ;
    QLabel *_label = new QLabel(tr(_stream.str().c_str()));

    QStringList _list;
    for(int index = 0; index < qnode_op3.getModeSize(); index++)
    {
      std::string _mode = qnode_op3.getModeName(index);
      if(_mode != "")
        _list << _mode.c_str();
    }

    QComboBox *_combo = new QComboBox();
    _combo->setObjectName(tr(_joint.c_str()));
    _combo->addItems(_list);
    _combo->setEnabled(false);      // not changable
    int _row = ix / 2 + 1;
    int _col = (ix % 2) * 3;
    _grid_mod->addWidget(_label, _row, _col, 1, 1);
    _grid_mod->addWidget(_combo, _row, _col + 1, 1, 2);
  }

  // get/set buttons
  QPushButton *_get_mode_button = new QPushButton(tr("Get Mode"));
  _grid_mod->addWidget(_get_mode_button, (_number_joint / 2) + 2, 0, 1, 3);
  QObject::connect(_get_mode_button, SIGNAL(clicked(bool)), &qnode_op3, SLOT(getJointControlMode()));

  // QPushButton *_set_mode_button = new QPushButton(tr("Set Mode"));
  // _grid_mod->addWidget(_set_mode_button, (_number_joint / 2) + 2, 3, 1, 3);
  // QObject::connect(_set_mode_button, SIGNAL(clicked(bool)), this, SLOT(setMode(bool)));

  ui.widget_mode->setLayout(_grid_mod);

  // make module widget table
  for(int index = 0; index < qnode_op3.getModeSize(); index++)
  {
    std::string _mode = qnode_op3.getModeName(index);
    if(_mode == "") continue;
    std::string _mode_reg = "*_" + _mode;

    QRegExp _rx(QRegExp(tr(_mode_reg.c_str())));
    _rx.setPatternSyntax(QRegExp::Wildcard);

    QList<QWidget *> _list = ui.centralwidget->findChildren<QWidget *>(_rx);
    module_ui_table_[_mode] = _list;

    if(DEBUG) std::cout << "Module widget : " << _mode << " [" << _list.size() << "]" << std::endl;
  }

  // make motion tab
  if(qnode_op3.getModeIndex("action_module") != -1) initMotionUnit();
}

void MainWindow::initMotionUnit()
{
  // preset button
  QGridLayout *_motion_layout = new QGridLayout;
  QSignalMapper *_signalMapper = new QSignalMapper(this);

  // yaml preset
  int _index = 0;
  for(std::map< int, std::string >::iterator _iter = qnode_op3.motion_table.begin(); _iter != qnode_op3.motion_table.end(); ++_iter)
  {
    int _motion_index = _iter->first;
    std::string _motion_name = _iter->second;
    QString _q_motion_name = QString::fromStdString(_motion_name);
    QPushButton *_motion_button = new QPushButton(_q_motion_name);

    // if(DEBUG) std::cout << "name : " <<  _motion_name << std::endl;

    int _size = (_motion_index < 0) ? 2 : 1;
    int _row = _index / 4;
    int _col = _index % 4;
    _motion_layout->addWidget(_motion_button, _row, _col, 1, _size);

    //hotkey
    std::map<int, int>::iterator _short_iter = qnode_op3.shortcut_table.find(_motion_index);
    if(_short_iter != qnode_op3.shortcut_table.end())
      _motion_button->setShortcut(QKeySequence(_short_iter->second));

    _signalMapper->setMapping(_motion_button, _motion_index);
    QObject::connect(_motion_button, SIGNAL(clicked()), _signalMapper, SLOT(map()));

    _index += _size;
  }

  int _row = _index / 4;
  _row = (_index % 4 == 0) ? _row : _row + 1;
  QSpacerItem *_verticalSpacer = new QSpacerItem(20, 400, QSizePolicy::Minimum, QSizePolicy::Expanding);
  _motion_layout->addItem(_verticalSpacer, _row, 0, 1, 4);

  QObject::connect(_signalMapper, SIGNAL(mapped(int)), &qnode_op3, SLOT(playMotion(int)));

  ui.scroll_widget_motion->setLayout(_motion_layout);
}

void MainWindow::setMode(QString mode_name)
{
  qnode_op3.setControlMode(mode_name.toStdString());
}

void MainWindow::ReadSettings()
{
  QSettings settings("Qt-Ros Package", "thor3_control");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings()
{
  QSettings settings("Qt-Ros Package", "thor3_control");
  //settings.setValue("master_url",ui.line_edit_master->text());
  //settings.setValue("host_url",ui.line_edit_host->text());
  //settings.setValue("topic_name",ui.line_edit_topic->text());
  //settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
  QMainWindow::closeEvent(event);
}

/*****************************************************************************
 ** Implementation [Util]
 *****************************************************************************/

Eigen::MatrixXd MainWindow::rx( double s )
{
  Eigen::MatrixXd R(3,3);

  R << 1.0, 	 0.0, 	  0.0,
      0.0, cos(s), -sin(s),
      0.0, sin(s),  cos(s);

  return R;
}

Eigen::MatrixXd MainWindow::ry( double s )
{
  Eigen::MatrixXd R(3,3);

  R << cos(s), 0.0, sin(s),
      0.0, 	 1.0, 	 0.0,
      -sin(s), 0.0, cos(s);

  return R;
}

Eigen::MatrixXd MainWindow::rz( double s )
{
  Eigen::MatrixXd R(3,3);

  R << cos(s), -sin(s), 0.0,
      sin(s),  cos(s), 0.0,
      0.0,     0.0, 1.0;

  return R;
}

Eigen::MatrixXd MainWindow::rpy2rotation( double r, double p, double y )
{
  Eigen::MatrixXd R = rz(y)*ry(p)*rx(r);

  return R;
}

Eigen::Quaterniond MainWindow::rpy2quaternion( double r, double p, double y )
{
  Eigen::MatrixXd R = rpy2rotation(r, p, y);

  Eigen::Matrix3d R_plus;
  R_plus = R.block(0,0,3,3);

  Eigen::Quaterniond QR;
  QR = R_plus;

  return QR;
}

}  // namespace thor3_control

