/*
 * Types.h
 *
 *  Created on: Mar 25, 2014
 *      Author: dan
 */

#ifndef TYPES_H_
#define TYPES_H_

#include "includes.hpp"

namespace poly{

class LaserScan{
public:
	float_t angle_min;
	float_t angle_max;
	float_t angle_increment;
	float_t time_increment;
	float_t scan_time;
	float_t range_min;
	float_t range_max;
	std::vector<float_t> ranges;
	std::vector<float_t> intensities;
};

class Position{
public:
	float_t x,y,heading;
};

class LocalizedScan{
public:
	std::string robot_id;
	LaserScan scan;
	Position pos;
};


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

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

#endif /* TYPES_H_ */
