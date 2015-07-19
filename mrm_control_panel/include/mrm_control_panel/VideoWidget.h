/*
 * Video.h
 *
 *  Created on: Feb 11, 2014
 *      Author: xaero
 */

#ifndef VIDEO_H_
#define VIDEO_H_

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>

#include <ui_mrm_video_widget.h>

#define ROBOT_CRITICAL_BATTERY_VALUE 10
#define ROBOT_CRITICAL_WIFI_VALUE 5

namespace mrm_control_panel {

class VideoWidget : QObject {
	Q_OBJECT
public:
	VideoWidget(ros::NodeHandle&);
	virtual ~VideoWidget();

	QWidget* getWidget() const;

	void shutdown();

	void subscribeToSelectedRobot(ros::NodeHandle&, const std::string&);
	void subscribeToCameraImage(ros::NodeHandle&, const std::string&);

	void onSelectedRobotChangedMessage(std_msgs::StringConstPtr);
	void onCameraImageMessage(const sensor_msgs::ImageConstPtr);




public slots:
	void redraw();

private Q_SLOTS:
	void batteryValueChanged(double);
	void wifiSignalValueChanged(double);

	void cameraSourceChanged(std_msgs::StringConstPtr);
	void cameraImageChanged(QImage);

Q_SIGNALS:
	void cameraSourceChangedSignal(std_msgs::StringConstPtr);
	void cameraImageChangedSignal(QImage);

	void batteryValueChangedSignal(double);
	void wifiSignalValueChangedSignal(double);
	
	void redrawSignal(void);
private:

	const static QString BATTERY_STATUS_FULL;
	const static QString BATTERY_STATUS_ALMOST_FULL;
	const static QString BATTERY_STATUS_HALF;
	const static QString BATTERY_STATUS_CRITICAL;

	const static QString WIFI_STATUS_0;
	const static QString WIFI_STATUS_1;
	const static QString WIFI_STATUS_2;
	const static QString WIFI_STATUS_3;
	const static QString WIFI_STATUS_FULL;

	bool setInt(QLabel* label, double value);
	void onVelocityMessage(const geometry_msgs::Twist::Ptr velocityMessage);


	ros::NodeHandle node_handle_;
	Ui_MrmVideoWidget _mrm_video_widget;
	QWidget* _widget;

	ros::Subscriber velocity_subscriber_;
	ros::Subscriber cur_view_robot_subscriber_;

	ros::Subscriber _cameraSourceSubscriber;
	image_transport::Subscriber image_subscriber_;

	std::string current_source_;

	double linear_velocity_;
	double angular_velocity_;
};

} /* namespace mrm_control_panel */

#endif /* VIDEO_H_ */
