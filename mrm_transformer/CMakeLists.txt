cmake_minimum_required(VERSION 2.8.3)
project(mrm_transformer)

find_package(catkin REQUIRED
	tf
	nav_msgs
	geometry_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include)

add_executable(agent_transformer_node 
	src/agent_transformer_node.cpp
	src/AgentTransformer.cpp
)

target_link_libraries(agent_transformer_node
  ${catkin_LIBRARIES}
)
