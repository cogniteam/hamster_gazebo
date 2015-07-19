/*
 * ControlPlugin.h
 *
 *  Created on: Feb 19, 2014
 *      Author: xaero
 */

#ifndef CONTROLPLUGIN_H_
#define CONTROLPLUGIN_H_

#include <mrm_control_panel/ControlWidget.h>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <pluginlib/class_list_macros.h>

namespace mrm_control_panel {

class ControlPlugin : public rqt_gui_cpp::Plugin {
public:
	ControlPlugin();
	virtual ~ControlPlugin();

	virtual void initPlugin(qt_gui_cpp::PluginContext& context);
	virtual void shutdownPlugin();
	virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
							  qt_gui_cpp::Settings& instance_settings) const;
	virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
								 const qt_gui_cpp::Settings& instance_settings);

private:
	void spin();

	ControlWidget* control_widget_;

	ros::AsyncSpinner spinner_;

};

} /* namespace mrm_control_panel */

#endif /* CONTROLPLUGIN_H_ */
