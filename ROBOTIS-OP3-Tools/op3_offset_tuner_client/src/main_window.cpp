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

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/op3_offset_tuner_client/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op3_offset_tuner_client {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    all_torque_on = false;

    spinBox_list_.push_back("goal");
    spinBox_list_.push_back("offset");
    spinBox_list_.push_back("mod");
    spinBox_list_.push_back("present");
    spinBox_list_.push_back("p_gain");
    spinBox_list_.push_back("i_gain");
    spinBox_list_.push_back("d_gain");

    /****************************
    ** Connect
    ****************************/

    qRegisterMetaType< op3_offset_tuner_msgs::JointOffsetPositionData >("op3_offset_tuner_msgs::JointOffsetPositionData");
    QObject::connect(&qnode , SIGNAL(update_present_joint_offset_data(op3_offset_tuner_msgs::JointOffsetPositionData)), this, SLOT(update_joint_offset_data_spinbox(op3_offset_tuner_msgs::JointOffsetPositionData)));

    /*********************
    ** Logging
    **********************/
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this,
                     SLOT(updateLoggingView()));

    /****************************
    ** Connect
    ****************************/

    /*********************
    ** Auto Start
    **********************/
    qnode.init();

    // make ui
    MakeUI();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::on_save_button_clicked( bool check )
{
    std_msgs::String _msg;
    _msg.data = "save";

    qnode.send_command_msg( _msg );
}

void MainWindow::on_inipose_button_clicked( bool checck )
{
    std_msgs::String _msg;
    _msg.data = "ini_pose";

    qnode.send_command_msg( _msg );
}

void MainWindow::on_refresh_button_clicked( bool check )
{
    qnode.getPresentJointOffsetData();
}

void MainWindow::all_torque_on_button_clicked(QObject *button_group)
{
    all_torque_on = true;

    QButtonGroup* _button_group = qobject_cast<QButtonGroup*>(button_group);
    if (!_button_group) // this is just a safety check
        return;

    QList<QAbstractButton *> _torque_buttons = _button_group->buttons();
    for(int ix = 0; ix < _torque_buttons.size(); ix++)
    {
        if( _torque_buttons[ix]->isChecked() == false )
            _torque_buttons[ix]->click();
    }

    qnode.getPresentJointOffsetData();

    all_torque_on = false;
}

void MainWindow::all_torque_off_button_clicked(QObject *button_group)
{
    QButtonGroup* _button_group = qobject_cast<QButtonGroup*>(button_group);
    if (!_button_group) // this is just a safety check
        return;

    QList<QAbstractButton *> _torque_buttons = _button_group->buttons();
    for(int ix = 0; ix < _torque_buttons.size(); ix++)
    {
        if( _torque_buttons[ix]->isChecked() == true )
            _torque_buttons[ix]->click();
    }
}

//void MainWindow::checkbox_clicked(QString joint_name)
void MainWindow::torque_checkbox_clicked(QWidget *widget)
{
    QCheckBox* _checkBox = qobject_cast<QCheckBox*>(widget);
    if (!_checkBox) // this is just a safety check
        return;

    std::string _joint_name = _checkBox->text().toStdString();
    bool _is_on = _checkBox->isChecked();

    QList<QAbstractSpinBox *> _spinbox_list = joint_spinbox_map_[_joint_name];

    for(int ix = 0; ix < _spinbox_list.size(); ix++)
    {
        _spinbox_list[ix]->setEnabled(_is_on);
    }

    publish_torque_msgs(_joint_name, _is_on);

}

void MainWindow::publish_torque_msgs(std::string &joint_name, bool torque_on )
{
    op3_offset_tuner_msgs::JointTorqueOnOffArray _msgs;
    op3_offset_tuner_msgs::JointTorqueOnOff _msg;

    _msg.joint_name = joint_name;
    _msg.torque_enable = torque_on;

    _msgs.torque_enable_data.push_back( _msg );

    qnode.send_torque_enable_msg( _msgs );

    if ( all_torque_on == false )
        qnode.getPresentJointOffsetData();
}

void MainWindow::spinBox_valueChanged(QString joint_name)
{
    if(qnode.is_refresh() == true) return;

    op3_offset_tuner_msgs::JointOffsetData _msg;
    std::string _joint_name = joint_name.toStdString();

    QList<QAbstractSpinBox *> _spinbox_list = joint_spinbox_map_[_joint_name];
    QDoubleSpinBox *_mod_spinBox;

    _msg.joint_name = _joint_name;

    for(int ix = 0; ix < _spinbox_list.size(); ix++)
    {
        if(_spinbox_list[ix]->whatsThis().toStdString() == "goal")
        {
            QDoubleSpinBox* _spinBox = qobject_cast<QDoubleSpinBox*>(_spinbox_list[ix]);
            if (!_spinBox) // this is just a safety check
                continue;

            _msg.goal_value = _spinBox->value() * M_PI / 180.0;
        }
        else if(_spinbox_list[ix]->whatsThis().toStdString() == "offset")
        {
            QDoubleSpinBox* _spinBox = qobject_cast<QDoubleSpinBox*>(_spinbox_list[ix]);
            if (!_spinBox) // this is just a safety check
                continue;

            _msg.offset_value = _spinBox->value() * M_PI / 180.0;
        }
        else if(_spinbox_list[ix]->whatsThis().toStdString() == "mod")
        {
            _mod_spinBox = qobject_cast<QDoubleSpinBox*>(_spinbox_list[ix]);
        }
        else if(_spinbox_list[ix]->whatsThis().toStdString() == "p_gain")
        {
            QSpinBox* _spinBox = qobject_cast<QSpinBox*>(_spinbox_list[ix]);
            if (!_spinBox) // this is just a safety check
                continue;

            _msg.p_gain = _spinBox->value();
        }
        else if(_spinbox_list[ix]->whatsThis().toStdString() == "i_gain")
        {
            QSpinBox* _spinBox = qobject_cast<QSpinBox*>(_spinbox_list[ix]);
            if (!_spinBox) // this is just a safety check
                continue;

            _msg.i_gain = _spinBox->value();
        }
        else if(_spinbox_list[ix]->whatsThis().toStdString() == "d_gain")
        {
            QSpinBox* _spinBox = qobject_cast<QSpinBox*>(_spinbox_list[ix]);
            if (!_spinBox) // this is just a safety check
                continue;

            _msg.d_gain = _spinBox->value();
        }
    }

    if (_mod_spinBox) // this is just a safety check
        _mod_spinBox->setValue( (_msg.goal_value + _msg.offset_value) * 180.0 / M_PI );

    qnode.send_joint_offset_data_msg( _msg );
}

void MainWindow::update_joint_offset_data_spinbox( op3_offset_tuner_msgs::JointOffsetPositionData msg )
{
    std::string _joint_name = msg.joint_name;

    QList<QAbstractSpinBox *> _spinbox_list = joint_spinbox_map_[_joint_name];

    for(int ix = 0; ix < _spinbox_list.size(); ix++)
    {
        if(_spinbox_list[ix]->whatsThis().toStdString() == "goal")
        {
            QDoubleSpinBox* _spinBox = qobject_cast<QDoubleSpinBox*>(_spinbox_list[ix]);
            if (!_spinBox) // this is just a safety check
                continue;

            _spinBox->setValue(msg.goal_value * 180.0 / M_PI);
        }
        else if(_spinbox_list[ix]->whatsThis().toStdString() == "offset")
        {
            QDoubleSpinBox* _spinBox = qobject_cast<QDoubleSpinBox*>(_spinbox_list[ix]);
            if (!_spinBox) // this is just a safety check
                continue;

            _spinBox->setValue(msg.offset_value * 180.0 / M_PI);
        }
        else if(_spinbox_list[ix]->whatsThis().toStdString() == "present")
        {
            QDoubleSpinBox* _spinBox = qobject_cast<QDoubleSpinBox*>(_spinbox_list[ix]);
            if (!_spinBox) // this is just a safety check
                continue;

            _spinBox->setValue(msg.present_value * 180.0 / M_PI);
        }
        else if(_spinbox_list[ix]->whatsThis().toStdString() == "p_gain")
        {
            QSpinBox* _spinBox = qobject_cast<QSpinBox*>(_spinbox_list[ix]);
            if (!_spinBox) // this is just a safety check
                continue;

            _spinBox->setValue(msg.p_gain);
        }
        else if(_spinbox_list[ix]->whatsThis().toStdString() == "i_gain")
        {
            QSpinBox* _spinBox = qobject_cast<QSpinBox*>(_spinbox_list[ix]);
            if (!_spinBox) // this is just a safety check
                continue;

            _spinBox->setValue(msg.i_gain);
        }
        else if(_spinbox_list[ix]->whatsThis().toStdString() == "d_gain")
        {
            QSpinBox* _spinBox = qobject_cast<QSpinBox*>(_spinbox_list[ix]);
            if (!_spinBox) // this is just a safety check
                continue;

            _spinBox->setValue(msg.d_gain);
        }
    }
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

void MainWindow::MakeUI()
{
    MakeTabUI(ui.right_arm_group, ui.right_arm_torque, right_arm_button_group_, qnode.right_arm_offset_group);
    MakeTabUI(ui.left_arm_group, ui.left_arm_torque, left_arm_button_group_, qnode.left_arm_offset_group);
    MakeTabUI(ui.leg_group, ui.leg_torque, legs_button_group_, qnode.legs_offset_group);
    MakeTabUI(ui.body_group, ui.body_torque, body_button_group_, qnode.body_offset_group);
}

void MainWindow::MakeTabUI(QGroupBox *joint_widget, QGroupBox *torque_widget, QButtonGroup *button_group, std::map<int, std::string> &offset_group)
{
    int _ix = 0;

    QSignalMapper *_torque_checkbox_signalMapper = new QSignalMapper(this);

    QGridLayout *_grid_layout = (QGridLayout *) joint_widget->layout();
    QGridLayout *_torque_layout = (QGridLayout *) torque_widget->layout();

    button_group = new QButtonGroup();
    button_group->setExclusive(false);

    std::cout << "ERROR : " << _ix++ << std::endl;

    int _row = 3;
    int _torque_checkbox_index = 0;
    int _torque_row = 1;
    int _torque_col = 0;
    for(std::map< int, std::string >::iterator _iter = offset_group.begin(); _iter != offset_group.end(); ++_iter)
    {
        QSignalMapper *_spingox_signalMapper = new QSignalMapper(this);
        QList<QAbstractSpinBox *> _spinbox_list;

        // spin_box
        int _col = 0;
        int _size = 1;
        std::string _joint_name = _iter->second;
        QString _q_joint_name = QString::fromStdString(_joint_name);

        // label
        QLabel *_joint_label = new QLabel(_q_joint_name);
        _grid_layout->addWidget(_joint_label, _row, _col++, 1, _size);

        // double spin box
        for(int ix = 0; ix < 4; ix++)
        {
            QDoubleSpinBox *_spin_box = new QDoubleSpinBox();
            _spin_box->setWhatsThis(tr(spinBox_list_[ix].c_str()));
            _spin_box->setMinimum(-360);
            _spin_box->setMaximum(360);
            _spin_box->setSingleStep(0.05);

            switch(ix)
            {
            case 2:
            case 3:
                _spin_box->setReadOnly(true);
                break;

            default:
                _spingox_signalMapper->setMapping(_spin_box, _q_joint_name);
                QObject::connect(_spin_box, SIGNAL(valueChanged(QString)), _spingox_signalMapper, SLOT(map()));
                break;
            }

            _grid_layout->addWidget(_spin_box, _row, _col++, 1, _size);

            _spinbox_list.append(_spin_box);
        }
        std::cout << "ERROR 2 : " << _ix++ << std::endl;

        // spin box
        for(int ix = 0; ix < 3; ix++)
        {
            QSpinBox *_spin_box = new QSpinBox();
            _spin_box->setWhatsThis(tr(spinBox_list_[ix + 4].c_str()));
            _spin_box->setMinimum(0);
            _spin_box->setMaximum(1000);
            _spin_box->setSingleStep(1);

            switch(ix)
            {
            case 0:
                _spin_box->setValue(32);

                _spingox_signalMapper->setMapping(_spin_box, _q_joint_name);
                QObject::connect(_spin_box, SIGNAL(valueChanged(QString)), _spingox_signalMapper, SLOT(map()));
                break;

            default:
                _spin_box->setReadOnly(true);
                break;
            }

            _grid_layout->addWidget(_spin_box, _row, _col++, 1, _size);

            _spinbox_list.append(_spin_box);
        }

        // spinbox
        joint_spinbox_map_[_joint_name] = _spinbox_list;
        QObject::connect(_spingox_signalMapper, SIGNAL(mapped(QString)), this, SLOT(spinBox_valueChanged(QString)));

        _row += 1;

        // torque checkbox
        _torque_row = _torque_checkbox_index / 6;
        _torque_col = _torque_checkbox_index % 6;

        QCheckBox *_check_box = new QCheckBox(_q_joint_name);
        _check_box->setChecked(true);
        _torque_layout->addWidget(_check_box, _torque_row, _torque_col, 1, _size);
        button_group->addButton(_check_box);

        _torque_checkbox_signalMapper->setMapping(_check_box, _check_box);
        QObject::connect(_check_box, SIGNAL(clicked()), _torque_checkbox_signalMapper, SLOT(map()));

        _torque_checkbox_index += 1;
    }

    // all torque on
    QSignalMapper *_torque_on_signalMapper = new QSignalMapper(this);
    QPushButton *_on_button = new QPushButton(tr("All torque ON"));
    _torque_layout->addWidget(_on_button, _torque_row + 1, 4, 1, 1);
    _torque_on_signalMapper->setMapping(_on_button, button_group);
    QObject::connect(_on_button, SIGNAL(clicked()), _torque_on_signalMapper, SLOT(map()));
    QObject::connect(_torque_on_signalMapper, SIGNAL(mapped(QObject*)), this, SLOT(all_torque_on_button_clicked(QObject*)));

    // all torque off
    QSignalMapper *_torque_off_signalMapper = new QSignalMapper(this);
    QPushButton *_off_button = new QPushButton(tr("All torque OFF"));
    _torque_layout->addWidget(_off_button, _torque_row + 1, 5, 1, 1);
    _torque_off_signalMapper->setMapping(_off_button, button_group);
    QObject::connect(_off_button, SIGNAL(clicked()), _torque_off_signalMapper, SLOT(map()));
    QObject::connect(_torque_off_signalMapper, SIGNAL(mapped(QObject*)), this, SLOT(all_torque_off_button_clicked(QObject*)));

    QObject::connect(_torque_checkbox_signalMapper, SIGNAL(mapped(QWidget*)), this, SLOT(torque_checkbox_clicked(QWidget*)));
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


void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

}  // namespace op3_offset_tuner_client

