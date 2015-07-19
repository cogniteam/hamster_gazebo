/*
 * ros_Types.hpp
 *
 *  Created on: Apr 1, 2014
 *      Author: dan
 */

#ifndef ROS_TYPES_HPP_
#define ROS_TYPES_HPP_

#include "types.hpp"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

namespace poly{


inline
Position convert(const nav_msgs::Odometry& msg){
	Position pos;
	pos.x = msg.pose.pose.position.x;
	pos.y = msg.pose.pose.position.y;
	pos.heading = tf::getYaw(msg.pose.pose.orientation);
	return pos;
}

inline
LaserScan convert(const sensor_msgs::LaserScan& msg){
	LaserScan scan;
	scan.angle_increment = msg.angle_increment;
	scan.angle_max = msg.angle_max;
	scan.angle_min = msg.angle_min;
	scan.intensities = msg.intensities;
	scan.range_max = msg.range_max;
	scan.range_min = msg.range_min;
	scan.ranges = msg.ranges;
	scan.scan_time = msg.scan_time;
	scan.time_increment = msg.time_increment;
	return scan;
}

}

#endif /* ROS_TYPES_HPP_ */
