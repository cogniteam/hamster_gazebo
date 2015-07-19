/*
 * RobotSelectionManager.h
 *
 *  Created on: Jun 22, 2014
 *      Author: blackpc
 */

#ifndef ROBOTSELECTIONMANAGER_H_
#define ROBOTSELECTIONMANAGER_H_

#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;

/*
 * Holds the information about selected robot
 */
class RobotSelectionManager {
public:

	static const RobotSelectionManager& getInstance();

	string selectedRobotId() const;

private:

	string selected_robot_id_;
	ros::Subscriber selected_robot_subscriber_;

	RobotSelectionManager(string focus_topic_name);

	void focusCallback(const std_msgs::String::ConstPtr& message);
};

#endif /* ROBOTSELECTIONMANAGER_H_ */
