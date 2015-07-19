/*
 * ControlPlugin.cpp
 *
 *  Created on: Feb 19, 2014
 *      Author: xaero
 */

#include <mrm_control_panel/ControlPlugin.h>

namespace mrm_control_panel {

ControlPlugin::ControlPlugin()
	: control_widget_(NULL)
	, spinner_(2) {
	spinner_.start();
}

ControlPlugin::~ControlPlugin() {
}

void ControlPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
	control_widget_ = new ControlWidget();
	context.addWidget(control_widget_);
}

void ControlPlugin::shutdownPlugin() {
	spinner_.stop();
}

void ControlPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
		qt_gui_cpp::Settings& instance_settings) const {
}

void ControlPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
		const qt_gui_cpp::Settings& instance_settings) {
}


} /* namespace mrm_control_panel */

PLUGINLIB_EXPORT_CLASS(mrm_control_panel::ControlPlugin, rqt_gui_cpp::Plugin)
