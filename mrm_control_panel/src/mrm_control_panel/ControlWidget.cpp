/*
 * ControlPanel.cpp
 *
 *  Created on: Feb 10, 2014
 *      Author: xaero
 */

#include <mrm_control_panel/ControlWidget.h>

namespace mrm_control_panel {

ControlWidget::ControlWidget():map_widget_(NULL)
{
	setupUi(this);

	ros::NodeHandle node_handle;

	map_widget_ = new MapWidget(node_handle, this);
	operator_widget_ = new TeleoperatorWidget(this);
	video_widget_ = new VideoWidget(node_handle);

	addRobotWidgets();
	addMapWidget();
	addVideoWidget();
	addTeleoperatorWidget();

	map_widget_->start();

	map_widget_->keyPressed = boost::function<void (QKeyEvent*)>(boost::bind(&ControlWidget::keyPressEvent,this,_1));
	map_widget_->keyReleased = boost::function<void (QKeyEvent*)>(boost::bind(&ControlWidget::keyReleaseEvent,this,_1));

	operator_widget_->keyPressed = boost::function<void (QKeyEvent*)>(boost::bind(&ControlWidget::keyPressEvent,this,_1));
	operator_widget_->keyReleased = boost::function<void (QKeyEvent*)>(boost::bind(&ControlWidget::keyReleaseEvent,this,_1));

	video_widget_->subscribeToSelectedRobot(node_handle, "/mrm/control_panel/focus");
	operator_widget_->subscribeToSelection("/mrm/control_panel/focus");


	focus_publisher_ = node_handle.advertise<std_msgs::String>("/mrm/control_panel/focus", 10);

	focus_subscriber_ = node_handle.subscribe("/mrm/control_panel/focus", 10, &ControlWidget::onSelectedRobotChangedMessage, this);


	subscribeToRobotsMove(&node_handle);

	RobotSelectionManager::getInstance();
}

ControlWidget::~ControlWidget() {
}

void ControlWidget::onSelectedRobotChangedMessage(const std_msgs::StringConstPtr& message)
{
	if (map_widget_->getSelectedRobotId() == message->data)
	{
		map_widget_->setViewOnRobot(message->data);
	}
}

void ControlWidget::keyPressEvent(QKeyEvent *event)
{
	boost::mutex::scoped_lock lock(mutex_);

	if (!focus_publisher_) {
		ROS_WARN("Publisher invalid!");
		return;
	}

	std_msgs::String message;

    switch(event->key())
    {
        case Qt::Key_0:
				message.data = "agent10";
            break;
        case Qt::Key_1:  
				message.data = "agent1";
            break;
        case Qt::Key_2:  
				message.data = "agent2";
            break;
        case Qt::Key_3:  
				message.data = "agent3";
            break;
        case Qt::Key_4:  
				message.data = "agent4";
            break;
        case Qt::Key_5:  
				message.data = "agent5";
            break;
        case Qt::Key_6:  
				message.data = "agent6";
            break;
        case Qt::Key_7:  
				message.data = "agent7";
            break;
        case Qt::Key_8:  
				message.data = "agent8";
            break;
        case Qt::Key_9:  
				message.data = "agent9";				
            break;


        case Qt::Key_Space:
				if (map_widget_->getSelectedRobotId() != "")
				{
					message.data = map_widget_->getSelectedRobotId();				
					map_widget_->setViewOnRobot(map_widget_->getSelectedRobotId());
				}
				else
				{
					return;
				}
            break;


	    case Qt::Key_Up:
	    case Qt::Key_Down:
	    case Qt::Key_Left:
	    case Qt::Key_Right:
            moveKeyPressed(event->key());
        return;

        default:
        	return;
    }

    if (map_widget_->getSelectedRobotId() == message.data)
    {
    	map_widget_->setViewOnRobot(message.data);
    	return;
    }

	focus_publisher_.publish(message);

	operator_widget_->setFocus();
}


void ControlWidget::keyReleaseEvent(QKeyEvent *event)
{
	switch(event->key())
	{
		case Qt::Key_Up:
		case Qt::Key_Down:
		case Qt::Key_Left:
		case Qt::Key_Right:
			moveKeyReleased(event->key());
		return;

		default:
			return;
	}
}

void ControlWidget::moveKeyPressed(int key)
{
	std::string selected_robot_id = map_widget_->getSelectedRobotId();

	setOperationMode(0);

	short int reverse_scale = 1;

	switch (key)
	{
		case Qt::Key_Up:
			_cmd_vel.linear.x = 1.0 * KEYBOARD_MOVE_SPEED_SCALE;
			_ackermann_msg.drive.speed = _cmd_vel.linear.x;

			// _move_robot_publishers[selected_robot_id].publish(_cmd_vel);
		break;

		case Qt::Key_Down:
			_cmd_vel.linear.x = -1.0 * KEYBOARD_MOVE_SPEED_SCALE;
			_ackermann_msg.drive.speed = _cmd_vel.linear.x;

			// _move_robot_publishers[selected_robot_id].publish(_cmd_vel);
		break;

		case Qt::Key_Left:

			if (_cmd_vel.linear.x < 0)
			{
				reverse_scale = -1;
			}

			_cmd_vel.angular.z = reverse_scale * 1.0 * KEYBOARD_TURN_SPEED_SCALE;
			_ackermann_msg.drive.steering_angle = _cmd_vel.angular.z;

			// _move_robot_publishers[selected_robot_id].publish(_cmd_vel);
		break;

		case Qt::Key_Right:

			if (_cmd_vel.linear.x < 0)
			{
				reverse_scale = -1;
			}

			_cmd_vel.angular.z = reverse_scale * (-1.0) * KEYBOARD_TURN_SPEED_SCALE;
			_ackermann_msg.drive.steering_angle = _cmd_vel.angular.z;

			// _move_robot_publishers[selected_robot_id].publish(_cmd_vel);
		break;

		default:
			return;
	}

	boost::thread(boost::bind(&ControlWidget::continueSendMoveCommand, this,selected_robot_id));
}

void ControlWidget::continueSendMoveCommand(string selected_robot_id)
{
	boost::mutex::scoped_lock lock(move_cmd_mutex_, boost::try_to_lock);

	if (!lock.owns_lock())
	{
	    return;
	}

	if ("" == selected_robot_id)
	{
		return;
	}

	if(!_move_robot_publishers[selected_robot_id] || !_ackermann_move_robot_publishers[selected_robot_id]) 
	{
		ROS_ERROR("Publisher invalid!");
		return;
	}

	for (int i = 0; i < 5; ++i)
	{
		_move_robot_publishers[selected_robot_id].publish(_cmd_vel);
		_ackermann_move_robot_publishers[selected_robot_id].publish(_ackermann_msg);

		ROS_ERROR("Sending move command %d",i);

		usleep(10000);
	}
}

void ControlWidget::moveKeyReleased(int key)
{
	std::string selected_robot_id = map_widget_->getSelectedRobotId();

	switch (key)
	{
		case Qt::Key_Up:
			_cmd_vel.linear.x = 0.0;
			_ackermann_msg.drive.speed = 0.0;
		break;

		case Qt::Key_Down:			
			_cmd_vel.linear.x = 0.0;
			_ackermann_msg.drive.speed = 0.0;
		break;

		case Qt::Key_Left:			
			_cmd_vel.angular.z = 0.0;
			_ackermann_msg.drive.steering_angle = 0.0;
		break;

		case Qt::Key_Right:			
			_cmd_vel.angular.z = 0.0;
			_ackermann_msg.drive.steering_angle = 0.0;
		break;

		default:
			return;
	}

	boost::thread(boost::bind(&ControlWidget::continueSendMoveCommand, this,selected_robot_id));
}

void ControlWidget::setOperationMode(int mode)
{
}

void ControlWidget::subscribeToRobotsMove(ros::NodeHandle* node_handle) 
{	
	for (int i = 0; i < 10; ++i)
	{
		QString robot_id = QString("agent");
		robot_id.append(QString::number(i + 1));

		QString topic = QString("/agent");
		topic.append(QString::number(i + 1));
		topic.append("/cmd_vel");

		QString ackermann_topic = QString("/agent");
		ackermann_topic.append(QString::number(i + 1));
		ackermann_topic.append("/ackermann_cmd");

		_move_robot_publishers[robot_id.toStdString()] = node_handle->advertise<geometry_msgs::Twist>(topic.toStdString(), 1);
		_ackermann_move_robot_publishers[robot_id.toStdString()] = node_handle->advertise<ackermann_msgs::AckermannDriveStamped>(ackermann_topic.toStdString(), 1);
	}

}

void ControlWidget::addRobotWidgets() {
	ros::NodeHandle node_handle;
	boost::shared_ptr<RobotWidget> robot;

	for (unsigned int row = 0; row < 1; row++) 
	{
		for (unsigned int col = 0; col < 5; col++)
		{
			QString id = QString("agent");
			id.append(QString::number(robot_widgets_.size() + 1));

			robot = boost::shared_ptr<RobotWidget>(new RobotWidget(id, node_handle, this/*,io[col]*/));

			QString topic = QString("/");
			topic.append(id);
			topic.append("/image");
			robot->subscribeToCamera(topic.toStdString());
			robot->subscribeToSelectedRobot("/mrm/control_panel/focus");
			robot->subscribeToBattery("/" + id.toStdString() + "/battery");
			robot->subscribeToWifi("/" + id.toStdString() + "/rssi");
			robot_widgets_.push_back(robot);

			camerasLayout->addWidget(robot.get(), row, col, (Qt::AlignLeft | Qt::AlignTop));//

		}
	}

}

void ControlWidget::addVideoWidget() {
	controlPanelCenterLayout_Middle->addWidget(video_widget_->getWidget());
}

void ControlWidget::addMapWidget() {
	controlPanelLeftLayout->addWidget(map_widget_);
}

void ControlWidget::addTeleoperatorWidget() {
	controlPanelRightLayout->addWidget(operator_widget_);
}

} /* namespace mrm_control_panel */
