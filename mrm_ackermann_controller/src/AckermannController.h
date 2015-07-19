/*
 * AckermannController.h
 *
 *  Created on: Apr 13, 2014
 *      Author: blackpc
 */

#ifndef ACKERMANNCONTROLLER_H_
#define ACKERMANNCONTROLLER_H_

#include <iostream>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>

using namespace std;

struct VehicleDescription {
	string inputTwistTopic;
    string inputTopic;
    string robotNamespace;
    string left_front_axle_command;
    string left_rear_axle_command;
    string left_steering_command;
    string right_front_axle_command;
    string right_rear_axle_command;
    string right_steering_command;
};

class AckermannController {
public:
    AckermannController(VehicleDescription description);
    virtual ~AckermannController();

private:
    ros::Subscriber _commandSub;
    ros::Subscriber _twistSub;
    ros::Publisher _leftFrontAxlePub;
    ros::Publisher _leftRearAxlePub;
    ros::Publisher _leftSteeringPub;
    ros::Publisher _rightFrontAxlePub;
    ros::Publisher _rightRearAxlePub;
    ros::Publisher _rightSteeringAxlePub;

    ackermann_msgs::AckermannDrive _currentCmd;
    ackermann_msgs::AckermannDrive _lastCmd;
    boost::posix_time::ptime _lastUpdate;

    float _last_accel_limit;
    float _last_speed;
    float _last_steer_ang;
    float _linearZeroCounter;

    void limitSteeringAngle(float& angle);
    std_msgs::Float64 toFloatMsg(float f);

    void ackermannMessageCallback(const ackermann_msgs::AckermannDriveStamped::Ptr msg);
    void twistMessageCallback(const geometry_msgs::Twist::Ptr msg);
    void spin();
};

#endif /* ACKERMANNCONTROLLER_H_ */
