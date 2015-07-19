/*
 * Parameters.cpp
 *
 *  Created on: Mar 6, 2014
 *      Author: xaero
 */

#include <mrm_control_panel/Parameters.h>

namespace mrm_control_panel {

Parameters* Parameters::instance_ = NULL;
boost::mutex Parameters::lock_;


Parameters::~Parameters() {
	delete instance_;
}

void Parameters::initialize(float map_resolution, const QPointF& map_origin) {
	boost::mutex::scoped_lock lock(lock_);

	map_resolution_ = map_resolution;
	map_origin_ 	= map_origin;

	is_initialized_ = true;

	ROS_INFO("Map parameters initialized: resolution %f, origin (%f, %f)",
			map_resolution_, map_origin_.x(), map_origin_.y());
}

bool Parameters::isInitialized() const {
	boost::mutex::scoped_lock lock(lock_);

	return is_initialized_;
}

float Parameters::mapResolution() const {
	return map_resolution_;
}

QPointF Parameters::mapOrigin() const {
	return map_origin_;
}

} /* namespace mrm_control_panel */
