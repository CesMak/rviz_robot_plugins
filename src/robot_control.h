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

// TODO(dave): convert to flow layout:
// http://doc.qt.io/qt-5/qtwidgets-layouts-flowlayout-example.html

#ifndef RVIZ_VISUAL_TOOLS__RVIZ_VISUAL_TOOLS_GUI_H
#define RVIZ_VISUAL_TOOLS__RVIZ_VISUAL_TOOLS_GUI_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#endif

#include <QPushButton>
#include <QComboBox>

#include "remote_reciever.h"

#define ROUND2(x) std::round(x * 100) / 100
#define ROUND3(x) std::round(x * 1000) / 1000

class QLineEdit;
class QSpinBox;
class QLabel;

namespace rviz_robot_plugins
{
class RobotControl : public rviz::Panel
{
  Q_OBJECT
public:
  explicit RobotControl(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:

protected Q_SLOTS:


  void changeControl();
  void reset();
  void yDes(int);
  void xDes(int);

  void valueBack(int);
  void valueForward(int);
  void valueServo(int);
  void updateLabels();

protected:
  QPushButton* control_mode_;
  QPushButton* reset_;
  QTimer *timer_;

  QSlider* slid_back_;
  QLabel* label_back_value_;
  QLabel* label_by_hand_;
  QLabel* label_automatic_;
  QLabel* label_x_des_;
  QLabel* label_y_des_;

  QSlider* slid_steering_;
  QSlider* slid_xdes_;
  QSlider* slid_ydes_;
  QLabel* label_forward_value_;

  QSlider* slid_servo_;
  QLabel* label_servo_value_;

  QLabel* label_battvoltage_value_;

  RemoteReciever remote_reciever_;
};

}  // end namespace rviz_visual_tools

#endif  // RVIZ_VISUAL_TOOLS__RVIZ_VISUAL_TOOLS_GUI_H
