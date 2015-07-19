/*
 * Filename: TeleoperatorInstanceWidget.h
 *   Author: Igor Makhtes
 *     Date: Dec 5, 2013
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Cogniteam Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef TELEOPERATORINSTANCEWIDGET_H_
#define TELEOPERATORINSTANCEWIDGET_H_

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <QtGui/QDockWidget>

#include <geometry_msgs/Twist.h>
#include <mrm_control_panel/VelocityWidget.h>
#include <ui_TeleoperatorInstance.h>
#include <mrm_control_panel/KeyboardTeleop.h>

using namespace std;

namespace mrm_control_panel {

class TeleoperatorInstanceWidget : public QWidget, public Ui::teleoperatorInstance {

Q_OBJECT

public:

    enum InputMethod {
    	Mouse = 0 , Keyboard = 1, Joystick = 2
    };

    TeleoperatorInstanceWidget(ros::NodeHandle&, InputMethod, const string&, string, string, string, QWidget* parent = 0);

    virtual ~TeleoperatorInstanceWidget();

    boost::function <void(QKeyEvent*)> keyPressed;
    boost::function <void(QKeyEvent*)> keyReleased;

    void shutdown();

public slots:
    void speedControlPressed(void);


private:
    void initJoystick(InputMethod);
	void initTopics(InputMethod, string, string, string);
	void initOperationTopic();
	void uiJoystickCallback(double, double);


	string inputMethodStr(InputMethod);

    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);

    string robot_id_;
    InputMethod input_method_;

    ros::Time last_ui_joystick_publish_;

    ros::NodeHandle& node_handle_;

    std::string velocity_topic_;
    std::string velocity_feedback_topic_;

    ros::Publisher publisher_;
    ros::Publisher ackermannPublisher_;
    ros::Subscriber subscriber_;

    mrm_control_panel::VelocityWidget* velocity_widget_;

    volatile bool closed_;

    ros::Publisher operation_publisher_;
};

}

#endif /* TELEOPERATORINSTANCEWIDGET_H_ */
