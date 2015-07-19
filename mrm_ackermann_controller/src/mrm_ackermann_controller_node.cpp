/*
 * Filename: controller.cpp
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

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include <ros/ros.h>

#include "AckermannController.h"

vector<boost::shared_ptr<AckermannController> > _controllers;

using namespace std;

#define foreach BOOST_FOREACH

int main(int argc, char **argv) {
    ros::init(argc, argv, "mrm_ackermann_controller");

    ros::NodeHandle nodePrivate("~");
    VehicleDescription desc;

    int robotsCount;
    nodePrivate.param("robots_count", robotsCount, 1);
    ROS_INFO("Ackermann Controller For %d Robots", robotsCount);

    for (int i = 0; i < robotsCount; ++i) {
        desc.robotNamespace = "/agent" + boost::lexical_cast<string>(i + 1);
        desc.inputTwistTopic = "cmd_vel_raw";
        desc.inputTopic = "ackermann_cmd";
        desc.left_front_axle_command = "left_front_axle_ctrlr/command";
        desc.left_rear_axle_command = "left_rear_axle_ctrlr/command";
        desc.left_steering_command = "left_steering_ctrlr/command";
        desc.right_front_axle_command = "right_front_axle_ctrlr/command";
        desc.right_rear_axle_command = "right_rear_axle_ctrlr/command";
        desc.right_steering_command = "right_steering_ctrlr/command";

        _controllers.push_back(
                boost::shared_ptr<AckermannController>(
                        new AckermannController(desc)));
    }

    ros::spin();
	return 0;
}
