/*
 * Filename: AckermannController.cpp
 *   Author: Igor Makhtes
 *     Date: Apr 13, 2014
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Cogniteam Ltd.
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

#include "AckermannController.h"

AckermannController::AckermannController(VehicleDescription description)
	: _linearZeroCounter(0), _last_steer_ang(0), _last_accel_limit(0), _last_speed(0)
{
    ros::NodeHandle node;

    _commandSub = node.subscribe(description.robotNamespace + "/" + description.inputTopic, 1, &AckermannController::ackermannMessageCallback, this);
    _twistSub = node.subscribe(description.robotNamespace + "/" + description.inputTwistTopic, 1, &AckermannController::twistMessageCallback, this);

    _leftFrontAxlePub = node.advertise<std_msgs::Float64>(description.robotNamespace + "/" + description.left_rear_axle_command, 1, false);
    _leftRearAxlePub = node.advertise<std_msgs::Float64>(description.robotNamespace + "/" + description.left_front_axle_command, 1, false);
    _leftSteeringPub = node.advertise<std_msgs::Float64>(description.robotNamespace + "/" + description.left_steering_command, 1, false);
    _rightFrontAxlePub = node.advertise<std_msgs::Float64>(description.robotNamespace + "/" + description.right_rear_axle_command, 1, false);
    _rightRearAxlePub = node.advertise<std_msgs::Float64>(description.robotNamespace + "/" + description.right_front_axle_command, 1, false);
    _rightSteeringAxlePub = node.advertise<std_msgs::Float64>(description.robotNamespace + "/" + description.right_steering_command, 1, false);

    _lastUpdate = boost::posix_time::microsec_clock::local_time();
    boost::thread(&AckermannController::spin, this);
}

AckermannController::~AckermannController() {
}

void AckermannController::ackermannMessageCallback(
        const ackermann_msgs::AckermannDriveStamped::Ptr msg) {
    _currentCmd = msg->drive;
}



std_msgs::Float64 AckermannController::toFloatMsg(float f) {
    std_msgs::Float64 msg;
    msg.data = f;
    return msg;
}

#define ANGLE_LIMIT (M_PI_4-0.01)

void AckermannController::limitSteeringAngle(float& angle) {
    angle = fmax(fmin(angle, ANGLE_LIMIT), -ANGLE_LIMIT);
}

void AckermannController::twistMessageCallback(
		const geometry_msgs::Twist::Ptr msg) {

	double linear = msg->linear.x;
	double angular = msg->angular.z;

	ackermann_msgs::AckermannDriveStamped message;
	message.header.stamp = ros::Time::now();

	if (msg->linear.x == 0 && fabs(msg->angular.z) > 0.001) {
		_linearZeroCounter += 0.15;

		linear = 0.2 * sin(_linearZeroCounter);
	}
	else
		_linearZeroCounter = 0;

	message.drive.speed = linear;
	message.drive.acceleration = 2;
	message.drive.jerk = 0.1;
	message.drive.steering_angle = fabs(angular) < 0.0001 ? 0 : atan2(0.1, 0.0001 + fabs(linear / angular));
	message.drive.steering_angle *= angular > 0 ? 1 : -1;
	message.drive.steering_angle *= linear > 0 ? 1 : -1;
	message.drive.steering_angle *= 4;

	limitSteeringAngle(message.drive.steering_angle);
	message.drive.steering_angle_velocity = 10;

	_currentCmd = message.drive;
}

void AckermannController::spin() {
    std_msgs::Float64 speedMsg, steeringMsg;

    float _joint_dist_div_2 = 0.20; // TODO Distance between steering joints
//    float left_dist = 0.00; // TODO
//    float right_dist = 0.00; // TODO
    float _wheelbase = 0.18;
    float _wheelbase_sqr = _wheelbase * _wheelbase;
    float _inv_wheelbase = 1.0 / _wheelbase;
    float _left_front_inv_circ = 1.0 / (M_PI * 0.20605); // 0.20605 Wheel dia
    float _right_front_inv_circ = 1.0 / (M_PI * 0.20605);
    float _left_rear_inv_circ = 1.0 / (M_PI * 0.20605);
    float _right_rear_inv_circ = 1.0 / (M_PI * 0.20605);


#define _get_steer_ang( phi ) (phi >= 0.0) ? (M_PI / 2) - phi : (-M_PI / 2) - phi


    while (ros::ok()) {

        double delta_t = (boost::posix_time::microsec_clock::local_time() - _lastUpdate).total_milliseconds() / 1000.0;

        float accel_limit = _currentCmd.acceleration;
        float speed = _currentCmd.speed;
        float jerk_limit = _currentCmd.jerk;
        float veh_speed;

        if (accel_limit > 0.0) {
            // Limit the vehicle's acceleration.
            float accel_limit_2;

            if (jerk_limit > 0.0) {
                if (_last_accel_limit > 0.0) {
                    float jerk = (accel_limit - _last_accel_limit) / delta_t;
                    jerk = max(-jerk_limit, min(jerk, jerk_limit));
                    accel_limit_2 = _last_accel_limit + jerk * delta_t;
                }
                else
                    accel_limit_2 = accel_limit;
            }
            else
                accel_limit_2 = accel_limit;

            _last_accel_limit = accel_limit_2;

            float accel = (speed - _last_speed) / delta_t;
            accel = max(-accel_limit_2, min(accel, accel_limit_2));
            veh_speed = _last_speed + accel * delta_t;
        }
        else {
            _last_accel_limit = accel_limit;
            veh_speed = speed;
        }


        float theta;
        float steer_ang = _currentCmd.steering_angle;
        float steer_ang_vel_limit = _currentCmd.steering_angle_velocity;

        if (steer_ang_vel_limit > 0.0) {
            // Limit the steering velocity.
            float ang_vel = (steer_ang - _last_steer_ang) / delta_t;
            ang_vel = max(-steer_ang_vel_limit, min(ang_vel, steer_ang_vel_limit));
            theta = _last_steer_ang + ang_vel * delta_t;
        }
        else
            theta = steer_ang;


        float _theta_left; // Left steer
        float _theta_right; // Right steer

        float center_y = _wheelbase * tan((M_PI / 2) - theta);
        bool steer_ang_changed = theta != _last_steer_ang;
        if (steer_ang_changed) {
            _last_steer_ang = theta;
            _theta_left =
                _get_steer_ang(atan(_inv_wheelbase *
                                         (center_y - _joint_dist_div_2)));
            _theta_right =
                _get_steer_ang(atan(_inv_wheelbase *
                                         (center_y + _joint_dist_div_2)));
        }


        float _left_front_ang_vel;
        float _right_front_ang_vel;
        float _left_rear_ang_vel;
        float _right_rear_ang_vel;

        if (veh_speed != _last_speed) {
            _last_speed = veh_speed;
            float left_dist = center_y - _joint_dist_div_2;
            float right_dist = center_y + _joint_dist_div_2;

            // Front
            float gain = (2 * M_PI) * veh_speed / abs(center_y);
            float r = sqrt(left_dist * left_dist + _wheelbase_sqr);

            _left_front_ang_vel = gain * r * _left_front_inv_circ;
            r = sqrt(right_dist * right_dist + _wheelbase_sqr);
            _right_front_ang_vel = gain * r * _right_front_inv_circ;
            // Rear
            gain = (2 * M_PI) * veh_speed / center_y;
            _left_rear_ang_vel = gain * left_dist * _left_rear_inv_circ;
            _right_rear_ang_vel = gain * right_dist * _right_rear_inv_circ;
        }

//        _theta_left = max(-M_PI_4*0.99, min((double)_theta_left, M_PI_4*0.99));
//        _theta_right = max(-M_PI_4*0.99, min((double)_theta_right, M_PI_4*0.99));

        if (_currentCmd.steering_angle > 0 && _theta_left < 0) _theta_left *= -1;
        if (_currentCmd.steering_angle > 0 && _theta_right < 0) _theta_right *= -1;
        if (_currentCmd.steering_angle < 0 && _theta_left > 0) _theta_left *= -1;
        if (_currentCmd.steering_angle < 0 && _theta_right > 0) _theta_right *= -1;

        speedMsg.data = speed;
        steeringMsg.data = _currentCmd.steering_angle;

        /**
         * Speed
         */
        _leftFrontAxlePub.publish(toFloatMsg(_left_front_ang_vel));
        _leftRearAxlePub.publish(toFloatMsg(_left_rear_ang_vel));
        _rightFrontAxlePub.publish(toFloatMsg(_right_front_ang_vel));
        _rightRearAxlePub.publish(toFloatMsg(_right_rear_ang_vel));

        /**
         * Steer
         */
        _leftSteeringPub.publish(toFloatMsg(_theta_left));
        _rightSteeringAxlePub.publish(toFloatMsg(_theta_right));

        _lastCmd = _currentCmd;
        _lastUpdate = boost::posix_time::microsec_clock::local_time();

        boost::this_thread::sleep(boost::posix_time::milliseconds(200));
    }
}
