/*
 * AgentTransformer.h
 *
 *  Created on: Jul 15, 2015
 *      Author: ofir
 */

#ifndef INCLUDE_MRM_TRANSFORMER_AGENTTRANSFORMER_H_
#define INCLUDE_MRM_TRANSFORMER_AGENTTRANSFORMER_H_
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
class AgentTransformer
{
private:
	std::string agent_namespace;
	tf::TransformBroadcaster br;
	ros::Subscriber ground_truth;
	ros::NodeHandle nh;

	void groundTruthCB(const nav_msgs::OdometryConstPtr & odom);
public:
	AgentTransformer(ros::NodeHandle nh, std::string agent_namespace);
	virtual ~AgentTransformer();
};

#endif /* INCLUDE_MRM_TRANSFORMER_AGENTTRANSFORMER_H_ */
