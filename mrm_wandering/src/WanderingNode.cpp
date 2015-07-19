/*
 * Filename: WanderingNode.cpp
 *   Author: Igor Makhtes
 *     Date: Dec 10, 2013
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

#include <iostream>
#include <time.h>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/date_time.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


#include "Wandering.h"

using namespace std;

#define foreach BOOST_FOREACH

ros::Subscriber laserSub;
ros::Publisher  velocityPub;
ros::NodeHandle* node;

volatile bool publisherEnabled = false;

double linearVelocity = 0;
double angularVelocity = 0;
double driveSpeed = 0;
double driveBackSpeed = 0;
double turnRightSpeed = 0;
double turnRightAngle = 0;
double turnLeftSpeed = 0;
double turnLeftAngle = 0;
double turnRandomSpeed = 0;
double turnRandomAngle = 0;

string _baseLinkFrame;
tf::StampedTransform _robotPose;

void publishVelocity(double linear, double angular) {
    linearVelocity = linear;
    angularVelocity = angular;
}

/**
 * Preemptive wait
 * @param ms Milliseconds to wait
 * @param queue EventQueue
 * @return True if preempted
 */
bool preemptiveWait(double ms, decision_making::EventQueue& queue) {
    for (int i = 0; i < 1000 && !queue.isTerminated(); i++)
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms / 1000.0));

    return queue.isTerminated();
}

/**
 * Preemptive wait
 * @param ms Milliseconds to wait
 * @param queue EventQueue
 * @return True if preempted
 */
bool preemptiveWait(double minMs, double maxMs, decision_making::EventQueue& queue) {
    double msToWait = ((double)rand() / (double)RAND_MAX) * (maxMs - minMs) + minMs;
    return preemptiveWait(msToWait, queue);
}

double getAverage(int centerRange, int rangeSize, const sensor_msgs::LaserScan::Ptr& scan) {
    double sum = std::accumulate(scan->ranges.begin() + centerRange - rangeSize / 2.0,
                                 scan->ranges.begin() + centerRange + rangeSize / 2.0, 0.0);

    return sum / (double)rangeSize;
}

int countRanges(const sensor_msgs::LaserScan::Ptr& scan, int center, int range, double lessThan) {

	int count = 0;
	for(vector<float>::iterator it = scan->ranges.begin() + center - range / 2.0;
			it != scan->ranges.begin() + center + range / 2.0;
					it++) {
		if (*it != 0 && *it < lessThan)
		{
			count ++;
		}
	}

	return count;
}

void onLaserScan(const sensor_msgs::LaserScan::Ptr scan, EventQueue* q) {

	int countFrontRange = countRanges(scan, scan->ranges.size() / 2, 45, 0.35);
    int countFrontCloseRange = countRanges(scan, scan->ranges.size() / 2, 90, 0.30);
    int countRightRange = countRanges(scan, (scan->ranges.size() / 8) * 2.3, 90, 0.4);
    int countLeftRange  = countRanges(scan, (scan->ranges.size() / 8) * 4.7, 90, 0.4);
    int countBackCloseRange =
    		countRanges(scan, 25 , 50 , 0.30) +
    		countRanges(scan, scan->ranges.size() - 25 , 50 , 0.30);

    bool front = false;
    bool back = false;

    if (countFrontCloseRange > 1 || countBackCloseRange > 1)
    {
		if (countFrontCloseRange > 1) {
			front = true;
			q->riseEvent("/FRONT_TOO_CLOSE");
		}
		if (countBackCloseRange > 1) {
			back = true;
			q->riseEvent("/BACK_TOO_CLOSE");
			}

		if (front && back){
			   q->riseEvent("/FRONT_AND_BACK_OBSTACLE");
			   return;
		}
		return;
    }
    if (countLeftRange > 1) {
            q->riseEvent("/LEFT_OBSTACLE");
        }

    else if (countRightRange > 1) {
            q->riseEvent("/RIGHT_OBSTACLE");
        }

    else if (countFrontRange > 1) {
        	q->riseEvent("/FRONT_OBSTACLE");
    }


}

decision_making::TaskResult stopRobot(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    publishVelocity(0, 0);
    ROS_INFO("Stop");
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    publisherEnabled = false;
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult stopRobotForSec(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    publishVelocity(0, 0);
    ROS_INFO("Stoping for a sec");

    if (preemptiveWait(1500, 3500, e))
        return decision_making::TaskResult::TERMINATED();

    e.riseEvent("/TIMEOUT_STOP");
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult turnRight(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    publishVelocity(turnRightSpeed, turnRightAngle);
    ROS_INFO("Turning Right");

    if (preemptiveWait(2500, 6500, e))
        return decision_making::TaskResult::TERMINATED();

    e.riseEvent("/TIMEOUT_TURN");
    return decision_making::TaskResult::SUCCESS();
}


decision_making::TaskResult turnLeft(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    publishVelocity(turnLeftSpeed, turnLeftAngle);

    ROS_INFO("Turning left");
    if (preemptiveWait(2500, 6500, e))
        return decision_making::TaskResult::TERMINATED();

    e.riseEvent("/TIMEOUT_TURN");
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult turnRandom(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    publisherEnabled = true;

    bool turnRight = rand() % 2;
    publishVelocity(turnRandomSpeed, (turnRight * -2 + 1) * turnRandomAngle); //modification
    ROS_INFO("Turning randomly");

    if (preemptiveWait(4500, 8500, e))
        return decision_making::TaskResult::TERMINATED();

    e.riseEvent("/TIMEOUT_TURN");
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult drive(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    publishVelocity(driveSpeed, 0);
    ROS_INFO("Driving");
    if (preemptiveWait(9000, 25000, e))
        return decision_making::TaskResult::TERMINATED();

    e.riseEvent("/TIMEOUT_DRIVE");
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult driveBackward(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    publishVelocity(driveBackSpeed, 0);
    ROS_INFO("Driving Backward");

    if (preemptiveWait(1500, 4500, e)) //modification (preemptiveWait(500, 1500, e))
        return decision_making::TaskResult::TERMINATED();

    e.riseEvent("/TIMEOUT_BACKWARD");
    return decision_making::TaskResult::SUCCESS();
}

void velocityPublishLoop() {

    while (ros::ok()) {
        if (::publisherEnabled) {
            geometry_msgs::Twist velocity;
            velocity.linear.x = linearVelocity;
            velocity.angular.z = angularVelocity;
            velocityPub.publish(velocity);
//            ROS_INFO("Publishing...");

        } else {
//            ROS_WARN("Not publishing");
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(200));
    }

}

void positionUpdateLoop() {

	tf::TransformListener _tfListener;
	while (ros::ok()) {

		try {
			tf::StampedTransform baseLinkTf;
			_tfListener.lookupTransform("map", _baseLinkFrame,
					ros::Time(0), baseLinkTf);
			_robotPose = baseLinkTf;

		} catch (tf::TransformException& e) {
			ROS_WARN("Couldn't find transform from 'map' to '%s'", _baseLinkFrame.c_str());
		}

		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / 5));
	}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wandering_node", ros::init_options::AnonymousName);
    ros_decision_making_init(argc, argv);

    boost::posix_time::ptime epoch(boost::posix_time::min_date_time);
    boost::posix_time::ptime now(boost::posix_time::microsec_clock::local_time());
    unsigned int seed = (now - epoch).total_nanoseconds();
    srand(seed);
    node = new ros::NodeHandle();
    double tmp = 0.3;
//  Default values are for the simulation

    node->param("base_link", _baseLinkFrame, string("/agent1/base_link"));

    ros::NodeHandle* nodePrivate = new ros::NodeHandle("~");
    nodePrivate->param<double>("drive_speed", driveSpeed, 0.1);
    nodePrivate->param<double>("drive_back_speed", driveBackSpeed, -0.20);
    nodePrivate->param<double>("turn_right_speed", turnRightSpeed, 0.0);
    nodePrivate->param<double>("turn_right_angle", turnRightAngle, -2);
    nodePrivate->param<double>("turn_left_speed", turnLeftSpeed, 0.0);
    nodePrivate->param<double>("turn_left_angle", turnLeftAngle, 2);
    nodePrivate->param<double>("turn_random_speed", turnRandomSpeed, 0.0);
    nodePrivate->param<double>("turn_random_angle", turnRandomAngle, 2);

    RosEventQueue* q = new RosEventQueue();
    laserSub    = node->subscribe<void>("scan", 10,
            boost::function<void(const sensor_msgs::LaserScan::Ptr)>(boost::bind(onLaserScan, _1, q)));

    velocityPub = node->advertise<geometry_msgs::Twist>("cmd_vel_raw", 10, false);

    LocalTasks::registrate("TurnRight", turnRight);
    LocalTasks::registrate("TurnLeft", turnLeft);
    LocalTasks::registrate("TurnRandom", turnRandom);
    LocalTasks::registrate("Drive", drive);
    LocalTasks::registrate("DriveBackward", driveBackward);
    LocalTasks::registrate("StopRobot", stopRobot);
    LocalTasks::registrate("StopRobotForSec", stopRobotForSec);

    boost::thread(velocityPublishLoop).detach();
    boost::thread(positionUpdateLoop).detach();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    FsmWandering(NULL, q, "Wandering");

    delete node;
	return 0;
}
