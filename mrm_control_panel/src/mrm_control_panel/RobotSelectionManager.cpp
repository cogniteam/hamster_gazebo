/*
 * RobotSelectionManager.cpp
 *
 *  Created on: Jun 22, 2014
 *      Author: blackpc
 */

#include <mrm_control_panel/RobotSelectionManager.h>

const RobotSelectionManager& RobotSelectionManager::getInstance() {
	static const RobotSelectionManager* instance = new RobotSelectionManager("/mrm/control_panel/focus");
	return *instance;
}

string RobotSelectionManager::selectedRobotId() const {
	return selected_robot_id_;
}

RobotSelectionManager::RobotSelectionManager(string focus_topic_name) {
	selected_robot_subscriber_ = ros::NodeHandle().subscribe(
			focus_topic_name, 1, &RobotSelectionManager::focusCallback, this);
}

void RobotSelectionManager::focusCallback(
		const std_msgs::String::ConstPtr& message)
{
	selected_robot_id_ = message->data;
}
