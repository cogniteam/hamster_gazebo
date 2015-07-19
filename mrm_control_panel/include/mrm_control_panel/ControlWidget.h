/*
 * ControlPanel.h
 *
 *  Created on: Feb 10, 2014
 *      Author: xaero
 */

#ifndef CONTROLPANEL_H_
#define CONTROLPANEL_H_

#include <ros/ros.h>
#include <QEvent>
#include <QKeyEvent>
#include <mrm_control_panel/MapWidget.h>
#include <ui_mrm_control_widget.h>
#include <mrm_control_panel/RobotWidget.h>
#include <mrm_control_panel/VideoWidget.h>
#include <mrm_control_panel/TeleoperatorWidget.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <boost/regex.hpp>

#include <mrm_control_panel/RobotSelectionManager.h>

// #define KEYBOARD_MOVE_SPEED_SCALE 9.0 // 0.5
// #define KEYBOARD_TURN_SPEED_SCALE 7.5 // 0.5

namespace mrm_control_panel {;

class ControlWidget : public QWidget, public Ui::MrmControlWidget {
	Q_OBJECT

public:
	ControlWidget();
	virtual ~ControlWidget();

	void moveKeyPressed(int key);
	void moveKeyReleased(int key);


protected:
	void keyPressEvent(QKeyEvent *event);
	void keyReleaseEvent(QKeyEvent *event);

private:
	void addRobotWidgets();
	void addVideoWidget();
	void addMapWidget();
	void addTeleoperatorWidget();

	void onSelectedRobotChangedMessage(const std_msgs::StringConstPtr&);


    void setOperationMode(int mode);
	
	void subscribeToRobotsMove(ros::NodeHandle*);

	void continueSendMoveCommand(string);

	std::vector<boost::shared_ptr<RobotWidget> > robot_widgets_;

	geometry_msgs::Twist _cmd_vel;
	ackermann_msgs::AckermannDriveStamped _ackermann_msg;

	ros::Subscriber focus_subscriber_;

	ros::Publisher focus_publisher_;
	ros::Publisher operation_publisher_;


	map<std::string, ros::Publisher> _move_robot_publishers;
	map<std::string, ros::Publisher> _ackermann_move_robot_publishers;

	VideoWidget* video_widget_;
	MapWidget* map_widget_;
	TeleoperatorWidget* operator_widget_;


	boost::mutex mutex_;

	boost::mutex move_cmd_mutex_;

};

} /* namespace mrm_control_panel */

#endif /* CONTROLPANEL_H_ */
