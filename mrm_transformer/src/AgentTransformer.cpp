/*
 * AgentTransformer.cpp
 *
 *  Created on: Jul 15, 2015
 *      Author: ofir
 */

#include <mrm_transformer/AgentTransformer.h>

AgentTransformer::AgentTransformer(ros::NodeHandle nh, std::string agent_namespace):nh(nh),agent_namespace(agent_namespace)
{
	ROS_INFO("Registered Transform for : %s", agent_namespace.data());
	ground_truth = nh.subscribe(agent_namespace + "/odom_ground_truth",1,&AgentTransformer::groundTruthCB,this);
}

AgentTransformer::~AgentTransformer()
{

}

void AgentTransformer::groundTruthCB(const nav_msgs::OdometryConstPtr & odom)
{
	tf::Transform mapTF;
	geometry_msgs::Point position = odom->pose.pose.position;
	geometry_msgs::Quaternion orientation = odom->pose.pose.orientation;
	mapTF.setOrigin(tf::Vector3(position.x, position.y, position.z));
	mapTF.setRotation(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
	br.sendTransform(tf::StampedTransform(mapTF, ros::Time::now(), "map", agent_namespace + "/base_link"));
}

