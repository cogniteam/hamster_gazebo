/*
 * Video.cpp
 *
 *  Created on: Feb 11, 2014
 *      Author: xaero
 */

#include <mrm_control_panel/VideoWidget.h>

namespace mrm_control_panel {

const QString VideoWidget::BATTERY_STATUS_FULL = 	QString(".QLabel {image: url(:/images/interactive_icons/battery/b3.png);}");
const QString VideoWidget::BATTERY_STATUS_ALMOST_FULL = 		QString(".QLabel {image: url(:/images/interactive_icons/battery/b2.png);}");
const QString VideoWidget::BATTERY_STATUS_HALF = 		QString(".QLabel {image: url(:/images/interactive_icons/battery/b1.png);}");
const QString VideoWidget::BATTERY_STATUS_CRITICAL =QString(".QLabel {image: url(:/images/interactive_icons/battery/b0.png);}");

const QString VideoWidget::WIFI_STATUS_FULL = 		QString(".QLabel {image: url(:/images/interactive_icons/signal/s4.png);}");
const QString VideoWidget::WIFI_STATUS_3 = 		QString(".QLabel {image: url(:/images/interactive_icons/signal/s3.png);}");
const QString VideoWidget::WIFI_STATUS_2 = 		QString(".QLabel {image: url(:/images/interactive_icons/signal/s2.png);}");
const QString VideoWidget::WIFI_STATUS_1 = 		QString(".QLabel {image: url(:/images/interactive_icons/signal/s1.png);}");
const QString VideoWidget::WIFI_STATUS_0 = 		QString(".QLabel {image: url(:/images/interactive_icons/signal/s0.png);}");


VideoWidget::VideoWidget(ros::NodeHandle& node_handle)
	: node_handle_(node_handle)
	, _widget(new QWidget()) {

	_mrm_video_widget.setupUi(_widget);

	qRegisterMetaType<std_msgs::StringConstPtr>("std_msgs::StringConstPtr");

	connect(this, SIGNAL(cameraSourceChangedSignal(std_msgs::StringConstPtr)),
			this, SLOT(cameraSourceChanged(std_msgs::StringConstPtr)));

	connect(this, SIGNAL(cameraImageChangedSignal(QImage)),
			this, SLOT(cameraImageChanged(QImage)));

	connect(this, SIGNAL(redrawSignal()), this, SLOT(redraw()), Qt::QueuedConnection);

	connect(this, SIGNAL(batteryValueChangedSignal(double)),
			this, SLOT(batteryValueChanged(double)));

	connect(this, SIGNAL(wifiSignalValueChangedSignal(double)),
			this, SLOT(wifiSignalValueChanged(double)));
}

VideoWidget::~VideoWidget() {

}



void VideoWidget::onVelocityMessage(const geometry_msgs::Twist::Ptr velocityMessage) 
{
	double linear = velocityMessage->linear.x;
	double angular = velocityMessage->angular.z;

	linear_velocity_ = roundf(linear * 100) / 100;
	angular_velocity_ = roundf(angular * 100) / 100;

	emit redrawSignal();
}

void VideoWidget::redraw() {
	_mrm_video_widget.videoLcdLinearVelocity->display(linear_velocity_);
	_mrm_video_widget.videoLcdAngularVelocity->display(angular_velocity_);
}

void VideoWidget::batteryValueChanged(double value) 
{
	if (setInt(_mrm_video_widget.batteryValueLabel, value)) 
	{
		if (value < ROBOT_CRITICAL_BATTERY_VALUE) 
		{
			_mrm_video_widget.batteryIconLabel->setStyleSheet(VideoWidget::BATTERY_STATUS_CRITICAL);
			return;
		}

		if (value < 50.0) 
		{
			_mrm_video_widget.batteryIconLabel->setStyleSheet(VideoWidget::BATTERY_STATUS_HALF);
			return;
		}

		if (value < 80.0) 
		{
			_mrm_video_widget.batteryIconLabel->setStyleSheet(VideoWidget::BATTERY_STATUS_ALMOST_FULL);
			return;
		}

		_mrm_video_widget.batteryIconLabel->setStyleSheet(VideoWidget::BATTERY_STATUS_FULL);
	}
}

void VideoWidget::wifiSignalValueChanged(double value) 
{
	if (setInt(_mrm_video_widget.wifiValueLabel, value)) 
	{		
		if (value < ROBOT_CRITICAL_WIFI_VALUE) 
		{
			_mrm_video_widget.wifiIconLabel->setStyleSheet(VideoWidget::WIFI_STATUS_0);
			return;
		}

		if (value < 30.0) 
		{
			_mrm_video_widget.wifiIconLabel->setStyleSheet(VideoWidget::WIFI_STATUS_1);
			return;
		}

		if (value < 60.0) 
		{
			_mrm_video_widget.wifiIconLabel->setStyleSheet(VideoWidget::WIFI_STATUS_2);
			return;
		}

		if (value < 85.0) 
		{
			_mrm_video_widget.wifiIconLabel->setStyleSheet(VideoWidget::WIFI_STATUS_3);
			return;
		}

		_mrm_video_widget.wifiIconLabel->setStyleSheet(VideoWidget::WIFI_STATUS_FULL);
	}
}

bool VideoWidget::setInt(QLabel* label, double value) 
{
	bool converted = false;

	double current = label->text().replace(QString("%"), QString("")).toDouble(&converted);

	// Conversion to double failed
	if (!converted) 
	{
		ROS_ERROR("Cannot convert received value to double, update has been aborted");
		return false;
	}

	// Invalid range
	if (value < 0.0) 
	{
		ROS_ERROR("Non positive values are not allowed, update has been aborted");
		ROS_ERROR("%f", value);
		return false;
	}

	// Value has been changed
	if (current != value) 
	{
		QString displayed_value = QString::number(ceil(value));
		label->setText(displayed_value+"%");
		return true;
	}

	// No update needed
	return false;
}

QWidget* VideoWidget::getWidget() const {
	return _widget;
}

void VideoWidget::shutdown() {
	_cameraSourceSubscriber.shutdown();
	image_subscriber_.shutdown();
}

void VideoWidget::subscribeToSelectedRobot(ros::NodeHandle& node_handle, const std::string& topic) {
	_cameraSourceSubscriber = node_handle.subscribe(topic, 1, &VideoWidget::onSelectedRobotChangedMessage, this);
}

void VideoWidget::subscribeToCameraImage(ros::NodeHandle& node_handle, const std::string& robot) {
	std::string topic;
	topic.append("/");
	topic.append(robot);
	topic.append("/image");

	// image_subscriber_.shutdown();
	boost::function<void(const sensor_msgs::ImageConstPtr)> imageCallBack
			= boost::bind(&VideoWidget::onCameraImageMessage, this, _1);
	image_transport::ImageTransport image_transport_(node_handle);

	image_subscriber_ = image_transport_.subscribe(topic, 1, imageCallBack, ros::VoidPtr(),
												   image_transport::TransportHints("compressed", ros::TransportHints().unreliable()));
}

void VideoWidget::onCameraImageMessage(const sensor_msgs::ImageConstPtr message) {
	int width = _mrm_video_widget.videoLabel->width();
	int height = _mrm_video_widget.videoLabel->height();

	QImage source = QImage((uchar*)message->data.data(), message->width, message->height, QImage::Format_RGB888);

	if (message->encoding == "bgr8")
		source = source.rgbSwapped();
	
	if (width == message->width || height == message->height)
		emit cameraImageChangedSignal(source.copy());
	else
		emit cameraImageChangedSignal(source.scaled(width, height, Qt::KeepAspectRatio, Qt::FastTransformation));

}

void VideoWidget::onSelectedRobotChangedMessage(std_msgs::StringConstPtr message) {
	if (current_source_ != message->data) {
		emit cameraSourceChangedSignal(message);
		current_source_ = message->data;

		QString title = QString::fromStdString(current_source_);
		_mrm_video_widget.current_robot_text->setText(title);

		std::stringstream stream;

		stream << "/"<< current_source_ << "/cmd_vel";

		velocity_subscriber_ = node_handle_.subscribe(stream.str(), 1,&VideoWidget::onVelocityMessage, this);
	}
}

void VideoWidget::cameraSourceChanged(std_msgs::StringConstPtr message) {
	ros::NodeHandle node;
	subscribeToCameraImage(node , message->data);
}

void VideoWidget::cameraImageChanged(QImage image) {
	_mrm_video_widget.videoLabel->setPixmap(QPixmap::fromImage(image));
}

} /* namespace mrm_control_panel */
