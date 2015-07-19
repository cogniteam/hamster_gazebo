/*
 * RobotStatus.cpp
 *
 *  Created on: Feb 10, 2014
 *      Author: xaero
 */

#include <mrm_control_panel/RobotWidget.h>

namespace mrm_control_panel {
const QString RobotWidget::FRAME_STYLE_NORMAL =QString(
		".QFrame {\
		background-color: rgb(247, 247, 247);\
		border:1px solid rgb(189, 190, 194);\
		border-radius:0px;\
		padding: 5px; \
	}");
const QString RobotWidget::FRAME_STYLE_SELECTED =QString(
		".QFrame {\
			background-color: rgba(76, 230, 100, 150);\
			border:1px solid rgb(76, 150, 100);\
			border-radius:0px;\
			padding: 5px; \
		}");

const QString RobotWidget::BATTERY_STATUS_FULL = 	QString(".QLabel {image: url(:/images/interactive_icons/battery/b3.png);background-color: rgb(240,120,80);}");
const QString RobotWidget::BATTERY_STATUS_ALMOST_FULL = 		QString(".QLabel {image: url(:/images/interactive_icons/battery/b2.png);background-color: rgb(240,120,80);}");
const QString RobotWidget::BATTERY_STATUS_HALF = 		QString(".QLabel {image: url(:/images/interactive_icons/battery/b1.png);background-color: rgb(240,120,80);}");
const QString RobotWidget::BATTERY_STATUS_CRITICAL =QString(".QLabel {image: url(:/images/interactive_icons/battery/b0.png);background-color: rgb(240,120,80);}");

const QString RobotWidget::WIFI_STATUS_FULL = 		QString(".QLabel {image: url(:/images/interactive_icons/signal/s4.png);border-top-left-radius:4px;background-color: rgb(240,120,80);}");
const QString RobotWidget::WIFI_STATUS_3 = 		QString(".QLabel {image: url(:/images/interactive_icons/signal/s3.png);border-top-left-radius:4px;background-color: rgb(240,120,80);}");
const QString RobotWidget::WIFI_STATUS_2 = 		QString(".QLabel {image: url(:/images/interactive_icons/signal/s2.png);border-top-left-radius:4px;background-color: rgb(240,120,80);}");
const QString RobotWidget::WIFI_STATUS_1 = 		QString(".QLabel {image: url(:/images/interactive_icons/signal/s1.png);border-top-left-radius:4px;background-color: rgb(240,120,80);}");
const QString RobotWidget::WIFI_STATUS_0 = 		QString(".QLabel {image: url(:/images/interactive_icons/signal/s0.png);border-top-left-radius:4px;background-color: rgb(240,120,80);}");

RobotWidget::RobotWidget(QString id, ros::NodeHandle& node_handle, QWidget* parent/*,boost::asio::io_service& io*/)
	: QWidget(parent)
	// , timer_(io, boost::posix_time::milliseconds(700))
	, node_handle_(node_handle)
	, current_mode_(0)
	, selected_(false)
{
	setupUi(this);
	idLabel->setText(id);

	connect(this, SIGNAL(cameraImageChangedSignal(QImage)),
			this, SLOT(cameraImageChanged(QImage)));

	connect(this, SIGNAL(batteryValueChangedSignal(double)),
			this, SLOT(batteryValueChanged(double)));

	connect(this, SIGNAL(wifiSignalValueChangedSignal(double)),
			this, SLOT(wifiSignalValueChanged(double)));

	connect(this, SIGNAL(selectedRobotChangedSignal(std_msgs::StringConstPtr)),
			this, SLOT(selectedRobotChanged(std_msgs::StringConstPtr)));



	connect(operationModeButton, SIGNAL(clicked()), this, SLOT(modeButtonClicked()));

	connect(this, SIGNAL(updateOperationIconSignal(int)), this, SLOT(updateOperationIcon(int)));

	operation_publisher_ = node_handle_.advertise<std_msgs::String>("/decision_making/" + id.toStdString() + "/events", 10, true);

	updateFrameStyle();


	qRegisterMetaType<std_msgs::StringConstPtr>("std_msgs::StringConstPtr");
}

RobotWidget::~RobotWidget() {
}

void RobotWidget::shutdown() {
}

void RobotWidget::updateOperationIcon(int mode)
{	
	QIcon icon;
	
	if (mode > 0)
	{
		icon.addFile(QString::fromUtf8(":/images/auto.png"), QSize(), QIcon::Normal, QIcon::Off);
		operationModeButton->setIcon(icon);
	} 
	else 
	{
		icon.addFile(QString::fromUtf8(":/images/manual.png"), QSize(), QIcon::Normal, QIcon::Off);
		operationModeButton->setIcon(icon);
	}
}

void RobotWidget::modeButtonClicked()
{

	if (current_mode_ > 0)
	{
		current_mode_ = 0;
	} 
	else 
	{
		current_mode_ = 1;
	}

	emit updateOperationIconSignal(current_mode_);

	setOperationMode(current_mode_);
}

void RobotWidget::setOperationMode(int mode)
{
	ROS_INFO("Changing operation mode to %d", mode);

	if (!operation_publisher_)
	{
		ROS_WARN("Failed to send operation mode instruction");
		return;
	}

	std_msgs::String message;

	if (mode == 0)
		message.data = "PAUSE";
	else
		message.data = "RESUME";

	operation_publisher_.publish(message);
}

void RobotWidget::subscribeToCamera(const std::string& topic) {
	boost::function<void(const sensor_msgs::ImageConstPtr)> imageCallBack
				= boost::bind(&RobotWidget::onCameraImageMessage, this, _1);
	image_transport::ImageTransport image_transport_(node_handle_);

	camera_subscriber_ = image_transport_.subscribe(topic, 10, imageCallBack, ros::VoidPtr(),
													 image_transport::TransportHints("compressed", ros::TransportHints().unreliable()));
}

void RobotWidget::subscribeToSelectedRobot(const std::string& topic) {
	selection_publisher_ = node_handle_.advertise<std_msgs::String>(topic, 1, false);
	selection_subscriber_ = node_handle_.subscribe(topic, 10, &RobotWidget::onSelectedRobotChangedMessage, this);
}

void RobotWidget::subscribeToBattery(const std::string& topic) {
	battery_subscriber_ = node_handle_.subscribe(topic, 10, &RobotWidget::onRobotBatteryMessage, this);
}

void RobotWidget::subscribeToWifi(const std::string& topic) {
	wifi_subscriber_ = node_handle_.subscribe(topic, 10, &RobotWidget::onRobotWifiMessage, this);
}

void RobotWidget::onCameraImageMessage(const sensor_msgs::ImageConstPtr message) {
	int width = imageLabel->width();
	int height = imageLabel->height();

	QImage source = QImage((uchar*) message->data.data(), message->width, message->height, message->step, QImage::Format_RGB888);
	QImage scaled = source.scaled(width, height, Qt::KeepAspectRatio, Qt::FastTransformation);

	if (message->encoding == "bgr8")
		scaled = scaled.rgbSwapped();
	
	emit cameraImageChangedSignal(scaled);
}

void RobotWidget::onSelectedRobotChangedMessage(std_msgs::StringConstPtr message) {
	emit selectedRobotChangedSignal(message);
}

void RobotWidget::onRobotBatteryMessage(const std_msgs::Float32& message) {
	emit batteryValueChangedSignal(message.data);
}

void RobotWidget::onRobotWifiMessage(const std_msgs::Float32& message) {
	emit wifiSignalValueChangedSignal(message.data);
}

void RobotWidget::cameraImageChanged(QImage image) {
	imageLabel->setPixmap(QPixmap::fromImage(image));
}

void RobotWidget::batteryValueChanged(double value) {

	/**
	 * Normalization from [3.2..4.2] to [0..100]
	 */
	value = (value - 3.2) * 100.0;

	if (setInt(batteryValueLabel, value)) {

		if (value < ROBOT_CRITICAL_BATTERY_VALUE) {
			batteryIconLabel->setStyleSheet(RobotWidget::BATTERY_STATUS_CRITICAL);
			return;
		}

		if (value < 50.0) {
			batteryIconLabel->setStyleSheet(RobotWidget::BATTERY_STATUS_HALF);
			return;
		}

		if (value < 80.0) {
			batteryIconLabel->setStyleSheet(RobotWidget::BATTERY_STATUS_ALMOST_FULL);
			return;
		}

		batteryIconLabel->setStyleSheet(RobotWidget::BATTERY_STATUS_FULL);
	}
}

void RobotWidget::wifiSignalValueChanged(double value) {

	if (setInt(wifiValueLabel, value)) {
		
		if (value < ROBOT_CRITICAL_WIFI_VALUE) {
			wifiIconLabel->setStyleSheet(RobotWidget::WIFI_STATUS_0);
			return;
		}

		if (value < 30.0) {
			wifiIconLabel->setStyleSheet(RobotWidget::WIFI_STATUS_1);
			return;
		}

		if (value < 60.0) {
			wifiIconLabel->setStyleSheet(RobotWidget::WIFI_STATUS_2);
			return;
		}

		if (value < 85.0) {
			wifiIconLabel->setStyleSheet(RobotWidget::WIFI_STATUS_3);
			return;
		}

		wifiIconLabel->setStyleSheet(RobotWidget::WIFI_STATUS_FULL);
	}
}

bool RobotWidget::setInt(QLabel* label, double value) 
{
	bool converted = false;

	double current = label->text().replace(QString("%"), QString("")).toDouble(&converted);

	// Conversion to double failed
	if (!converted) {
		ROS_ERROR("Cannot convert received value to double, update has been aborted");
		return false;
	}

	// Invalid range
	if (!isPositive(value)) {
		ROS_ERROR("Non positive values are not allowed, update has been aborted");
		ROS_ERROR("%f", value);
		return false;
	}

	// Value has been changed
	if (current != value) {
		QString displayed_value = QString::number(ceil(value));
		label->setText(displayed_value+"%");
		return true;
	}

	// No update needed
	return false;
}

bool RobotWidget::isPositive(double value) const {
	return value >= 0.00000;
}

bool RobotWidget::isCritical(double value) const {
	return value < 0.001 && value > -0.001;
}

void RobotWidget::selectedRobotChanged(std_msgs::StringConstPtr message) {
	if (!this->isEnabled())
		return;

	if (idLabel->text().toStdString() == message->data) {
		selected_=true;
		updateFrameStyle();
	} else {
		selected_=false;
		updateFrameStyle();
	}
}

void RobotWidget::mousePressEvent(QMouseEvent* event)
{
	if (!this->isEnabled())
		return;

	if (!selection_publisher_) {
	  ROS_WARN("Publisher invalid!");
	  return;
	}

	std_msgs::String message;
	message.data = idLabel->text().toStdString();

	selection_publisher_.publish(message);
}

//Detect the state and set style to the frame accordingly.
void RobotWidget::updateFrameStyle() 
{

	if (!selected_) {
		frame->setStyleSheet(RobotWidget::FRAME_STYLE_NORMAL);
		return;
	}
	if (selected_) {
		frame->setStyleSheet(RobotWidget::FRAME_STYLE_SELECTED);
		return;
	}

}
} /* namespace mrm_control_panel */
