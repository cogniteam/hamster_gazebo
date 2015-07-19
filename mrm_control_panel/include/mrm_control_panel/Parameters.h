/*
 * Parameters.h
 *
 *  Created on: Mar 6, 2014
 *      Author: xaero
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <stddef.h>

#include <boost/thread/mutex.hpp>

#include <qpoint.h>

#include <ros/ros.h>

namespace mrm_control_panel {

class Parameters {
public:

	static Parameters* getInstance() {
		return createInstance();
	}

	virtual ~Parameters();

	void initialize(float, const QPointF&);
	bool isInitialized() const;

	float mapResolution() const;
	QPointF mapOrigin() const;
private:

	Parameters()
		: is_initialized_(false)
		, map_resolution_(0.05)
		, map_origin_(0.0, 0.0) {
	}

	static Parameters*  instance_;
	static boost::mutex lock_;

	static Parameters* createInstance() {
		boost::mutex::scoped_lock lock(lock_);

		if (instance_ == NULL)
			instance_ = new Parameters();

		return instance_;
	}

	bool 	is_initialized_;
	float	map_resolution_;
	QPointF map_origin_;
};

} /* namespace mrm_control_panel */

#endif /* PARAMETERS_H_ */
