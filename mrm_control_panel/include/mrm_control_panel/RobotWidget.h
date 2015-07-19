/*
 * RobotStatus.h
 *
 *  Created on: Feb 10, 2014
 *      Author: xaero
 */

#ifndef ROBOTSTATUS_H_
#define ROBOTSTATUS_H_

#include <math.h>
#include <std_msgs/String.h>
#include <set>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/regex.hpp>
#include <QFocusEvent>
#include <qmessagebox.h>
#include <QColor>

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/Float32.h>
#include <ui_mrm_robot_widget.h>

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#define ROBOT_CRITICAL_BATTERY_VALUE 10
#define ROBOT_CRITICAL_WIFI_VALUE 5

#define TIMES_TO_BLINK 5
#define SLEEP_MILLISECONDS_BETWEEN_BLINKS 500

namespace mrm_control_panel {



class RobotWidget : public QWidget, public Ui::MrmRobotWidget {

	Q_OBJECT
public:
	RobotWidget(QString ,ros::NodeHandle&, QWidget* parent);
	virtual ~RobotWidget();

	QWidget* getWidget() const;

	void shutdown();

	void subscribeToCamera(const std::string&);
	void subscribeToSelectedRobot(const std::string&);

	void subscribeToBattery(const std::string&);
	void subscribeToWifi(const std::string&);

	void onCameraImageMessage(const sensor_msgs::ImageConstPtr);
	void onSelectedRobotChangedMessage(std_msgs::StringConstPtr);

	void onRobotBatteryMessage(const std_msgs::Float32&);
	void onRobotWifiMessage(const std_msgs::Float32&);

private Q_SLOTS:

	void modeButtonClicked(void);

	void updateOperationIcon(int);

	void cameraImageChanged(QImage);

	void batteryValueChanged(double);
	void wifiSignalValueChanged(double);
	void selectedRobotChanged(std_msgs::StringConstPtr);
	void setBlackListed(void);

Q_SIGNALS:
	void cameraImageChangedSignal(QImage);
	void batteryValueChangedSignal(double);
	void wifiSignalValueChangedSignal(double);
	void selectedRobotChangedSignal(std_msgs::StringConstPtr);

	void updateOperationIconSignal(int);

private:
    const static QString FRAME_STYLE_NORMAL;
    const static QString FRAME_STYLE_SELECTED;

    const static QString BATTERY_STATUS_FULL;
    const static QString BATTERY_STATUS_ALMOST_FULL;
    const static QString BATTERY_STATUS_HALF;
    const static QString BATTERY_STATUS_CRITICAL;

    const static QString WIFI_STATUS_0;
    const static QString WIFI_STATUS_1;
    const static QString WIFI_STATUS_2;
    const static QString WIFI_STATUS_3;
    const static QString WIFI_STATUS_FULL;

    boost::thread blinking_thread_;
    	
    void mousePressEvent(QMouseEvent* event);
    
    void setOperationMode(int mode);

    bool setInt(QLabel*, double);
    bool isPositive(double) const;
    bool isCritical(double) const;

    void updateFrameStyle();

    void blinkWithColors();

    ros::NodeHandle node_handle_;

    image_transport::Subscriber camera_subscriber_;

	ros::Subscriber selection_subscriber_;
	ros::Publisher  selection_publisher_;

	ros::Subscriber state_subscriber_;
	ros::Subscriber battery_subscriber_;
	ros::Subscriber wifi_subscriber_;


	ros::Publisher operation_publisher_;

	bool selected_;

	boost::mutex lock_;

	int current_mode_;
};


} /* namespace mrm_control_panel */

#endif /* ROBOTSTATUS_H_ */
