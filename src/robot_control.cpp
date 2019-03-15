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
  control_mode_ = new QPushButton(this);
  control_mode_->setText("Control Mode: ");
  connect(control_mode_, SIGNAL(clicked()), this, SLOT(changeControl()));

  // Create a push button
  reset_ = new QPushButton(this);
  reset_->setText("Reset");
  connect(reset_, SIGNAL(clicked()), this, SLOT(reset()));

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
  timer_->start(200); //time specified in ms

  // Label:
  label_by_hand_ = new QLabel(this);
  label_by_hand_->setText("By Hand Control:");

  label_back_value_ = new QLabel(this);
  label_back_value_->setText("Back: ");

  label_forward_value_ = new QLabel(this);
  label_forward_value_->setText("Forw: ");

  label_servo_value_ = new QLabel(this);
  label_servo_value_->setText("Serv: ");

  label_battvoltage_value_ = new QLabel(this);
  label_battvoltage_value_->setText("Battery Voltage: >3.2V <4.1V, Distance 10cm to 80cm");

  label_automatic_ = new QLabel(this);
  label_automatic_->setText("Automatic:");

  label_x_des_ = new QLabel(this);
  label_x_des_->setText("xdes: ");

  label_y_des_ = new QLabel(this);
  label_y_des_->setText("ydes: ");


  slid_xdes_ = new QSlider(Qt::Horizontal, this);
  slid_xdes_->setMaximum(250);
  slid_xdes_->setMinimum(-10);
  slid_xdes_->setValue(0);
  connect(slid_xdes_, SIGNAL(valueChanged(int)), this, SLOT(xDes(int)));

  slid_ydes_ = new QSlider(Qt::Horizontal, this);
  slid_ydes_->setMaximum(250);
  slid_ydes_->setMinimum(-10);
  slid_ydes_->setValue(0);
  connect(slid_ydes_, SIGNAL(valueChanged(int)), this, SLOT(yDes(int)));


  // Horizontal Layout Buttons
  auto* hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(control_mode_);
  hlayout1->addWidget(reset_);

  // ue1:
  auto* hlayout0 = new QHBoxLayout;
  hlayout0->addWidget(label_by_hand_);

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

  auto* hlayout6 = new QHBoxLayout;
  hlayout6->addWidget(label_automatic_);

  auto* hlayout7 = new QHBoxLayout;
  hlayout7->addWidget(label_x_des_);
  hlayout7->addWidget(slid_xdes_);
  hlayout7->addWidget(label_y_des_);
  hlayout7->addWidget(slid_ydes_);

  // Verticle layout
  auto* layout = new QVBoxLayout;
  layout->addLayout(hlayout1);
  layout->addLayout(hlayout0);
  layout->addLayout(hlayout2);
  layout->addLayout(hlayout3);
  layout->addLayout(hlayout4);
  layout->addLayout(hlayout5);
  layout->addLayout(hlayout6);
  layout->addLayout(hlayout7);
  setLayout(layout);
}

void RobotControl::changeControl()
{
 //TODO
}

void RobotControl::xDes(int value)
{
   label_x_des_->setText("xdes: "+QString::number(value));
}

void RobotControl::yDes(int value)
{
   label_y_des_->setText("ydes: "+QString::number(value));
}

void RobotControl::reset()
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
  label_battvoltage_value_->setText("Battery: "+QString::number(ROUND2(remote_reciever_.battery_voltage_))+
  "V, "+ QString::number(ROUND2(remote_reciever_.battery_percentage_))+"%"
  +" Distance: "+QString::number(ROUND3(remote_reciever_.distance_))+" m");
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
