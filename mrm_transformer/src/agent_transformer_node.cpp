/*
 * agent_transformer_node.cpp
 *
 *  Created on: Jul 15, 2015
 *      Author: ofir
 */
#include <mrm_transformer/AgentTransformer.h>

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"agent_transformer_node");
	ros::NodeHandle nh("~");
	std::string agent_namespace;
	nh.getParam("agent_namespace", agent_namespace);
	ROS_INFO("Agent Transformer NS : %s", agent_namespace.data());
	AgentTransformer at(nh, agent_namespace);
	ros::spin();
}


