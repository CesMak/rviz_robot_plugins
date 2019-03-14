/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Consulting nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Rviz display panel for controlling and debugging ROS applications
*/

#include <cstdio>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

#include <QSlider> // https://doc.qt.io/qt-5/qslider.html

#include "robot_control.h"

namespace rviz_robot_plugins
{
RobotControl::RobotControl(QWidget* parent) : rviz::Panel(parent)
{
  // Create a push button
  btn_next_ = new QPushButton(this);
  btn_next_->setText("Next");
  connect(btn_next_, SIGNAL(clicked()), this, SLOT(moveNext()));

  // Create a push button
  btn_auto_ = new QPushButton(this);
  btn_auto_->setText("Continue");
  connect(btn_auto_, SIGNAL(clicked()), this, SLOT(moveAuto()));

  // Create a push button
  btn_full_auto_ = new QPushButton(this);
  btn_full_auto_->setText("Break");
  connect(btn_full_auto_, SIGNAL(clicked()), this, SLOT(moveFullAuto()));

  // Create a push button
  btn_stop_ = new QPushButton(this);
  btn_stop_->setText("Stop");
  connect(btn_stop_, SIGNAL(clicked()), this, SLOT(moveStop()));

  // Create slider back_motors
  slid_back_ = new QSlider(Qt::Horizontal, this);
  slid_back_->setMaximum(250);
  slid_back_->setMinimum(-250);
  slid_back_->setValue(0);
  connect(slid_back_, SIGNAL(valueChanged(int)), this, SLOT(valueBack(int)));

  // forward motors:
  slid_steering_ = new QSlider(Qt::Horizontal, this);
  slid_steering_->setMaximum(250);
  slid_steering_->setMinimum(-250);
  slid_steering_->setValue(0);
  connect(slid_steering_, SIGNAL(valueChanged(int)), this, SLOT(valueForward(int)));

  // servo:
  slid_servo_ = new QSlider(Qt::Horizontal, this);
  slid_servo_->setMaximum(150);
  slid_servo_->setMinimum(60);
  slid_servo_->setValue(90);
  connect(slid_servo_, SIGNAL(valueChanged(int)), this, SLOT(valueServo(int)));


  // Timer for callback updates:
  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateLabels()));
  timer_->start(1000); //time specified in ms

  // Label:
  label_back_value_ = new QLabel(this);
  label_back_value_->setText("Back: ");

  label_forward_value_ = new QLabel(this);
  label_forward_value_->setText("Forw: ");

  label_servo_value_ = new QLabel(this);
  label_servo_value_->setText("Serv: ");

  label_battvoltage_value_ = new QLabel(this);
  label_battvoltage_value_->setText("Battery Voltage: >3.2V <4.1V");

  // Horizontal Layout Buttons
  auto* hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(btn_next_);
  hlayout1->addWidget(btn_auto_);
  hlayout1->addWidget(btn_full_auto_);
  hlayout1->addWidget(btn_stop_);


  // Horizontal Layout Slider
  auto* hlayout2 = new QHBoxLayout;
  hlayout2->addWidget(slid_back_);
  hlayout2->addWidget(label_back_value_);

  auto* hlayout3 = new QHBoxLayout;
  hlayout3->addWidget(slid_steering_);
  hlayout3->addWidget(label_forward_value_);

  auto* hlayout4 = new QHBoxLayout;
  hlayout4->addWidget(slid_servo_);
  hlayout4->addWidget(label_servo_value_);

  auto* hlayout5 = new QHBoxLayout;
  hlayout5->addWidget(label_battvoltage_value_);

  // Verticle layout
  auto* layout = new QVBoxLayout;
  layout->addLayout(hlayout2);
  layout->addLayout(hlayout3);
  layout->addLayout(hlayout4);
  layout->addLayout(hlayout5);
  layout->addLayout(hlayout1);

  setLayout(layout);

  btn_next_->setEnabled(true);
  btn_auto_->setEnabled(true);
  btn_full_auto_->setEnabled(true);
}

void RobotControl::moveNext()
{
  remote_reciever_.publishNext();
}

void RobotControl::moveAuto()
{
  remote_reciever_.publishContinue();
}

void RobotControl::moveFullAuto()
{
  remote_reciever_.publishBreak();
}

void RobotControl::moveStop()
{
    slid_steering_->setValue(0);
    slid_servo_->setValue(105);
    slid_back_->setValue(0);
}

void RobotControl::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void RobotControl::valueBack(int value)
{
  
  label_back_value_->setText(QString::number(value));
  remote_reciever_.publishBack(value);
}

void RobotControl::updateLabels()
{
  label_battvoltage_value_->setText("Battery: "+QString::number(ROUND2(remote_reciever_.battery_voltage_))+"V, "+ QString::number(ROUND2(remote_reciever_.battery_percentage_))+"%");
}

// sliders:
void RobotControl::valueForward(int value)
{
  
  label_forward_value_->setText(QString::number(value));
  remote_reciever_.publishForward(value);
}

void RobotControl::valueServo(int value)
{
  
  label_servo_value_->setText(QString::number(value));
  remote_reciever_.publishServo(value);
}

void RobotControl::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
}  // end namespace rviz_visual_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_robot_plugins::RobotControl, rviz::Panel)
