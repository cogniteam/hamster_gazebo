#ifndef TELEOPERATORWIDGET_H_
#define TELEOPERATORWIDGET_H_

#include <iostream>
#include <set>

#include <boost/foreach.hpp>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include <QListView>
#include <QPainter>
#include <QStandardItemModel>
#include <QDockWidget>
#include <QLayoutItem>

#include <mrm_control_panel/TeleoperatorInstanceWidget.h>
#include <ui_mrm_operator_widget.h>
#include <mrm_control_panel/VelocityWidget.h>
#include <mrm_control_panel/KeyboardTeleop.h>

using namespace std;

namespace mrm_control_panel
{

class TeleoperatorWidget : public QWidget, public Ui::MrmOperatorWidget
{
Q_OBJECT

public:
	TeleoperatorWidget(QWidget* parent = 0);
	virtual ~TeleoperatorWidget();

	bool setupInputOuput(const std::string& topic, const std::string& ackermann_topic, const std::string& robot_id);

	void subscribeToSelection(const std::string&);

	void shutdown();

	boost::function <void(QKeyEvent*)> keyPressed;
	boost::function <void(QKeyEvent*)> keyReleased;

public Q_SLOTS:
	void selectionChanged(std_msgs::StringConstPtr);

Q_SIGNALS:
	void selectionChangedSignal(std_msgs::StringConstPtr);


private:

	void handleKeyPressEvent(QKeyEvent*);
	void handleKeyReleaseEvent(QKeyEvent*);

	void onSelectionMessage(std_msgs::StringConstPtr);

	void clear();
	std::string getVelocityTopic(const std::string&) const;
	std::string getAckermannTopic(const std::string&) const;

	VelocityWidget* _velocity_widget;
	KeyboardTeleop* _keyboard_teleop;
	TeleoperatorInstanceWidget* _controller;
	vector<string> _outputTopics;

	ros::NodeHandle _node;

	ros::Subscriber _conrollable_robot_subscriber;

	std::string _selected;
};

} /* namespace mrm_control_panel */

#endif /* MRMRQT_H_ */
