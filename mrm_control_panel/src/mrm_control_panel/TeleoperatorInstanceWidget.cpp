#include <mrm_control_panel/TeleoperatorInstanceWidget.h>

namespace mrm_control_panel {

TeleoperatorInstanceWidget::TeleoperatorInstanceWidget(ros::NodeHandle& node_handle,
	InputMethod input, const string& robot_id, string velocity_topic, string ackermann_topic,
	string velocity_feedback_topic, QWidget* parent)
            : QWidget(parent)
            , robot_id_(robot_id)
    		, input_method_(input)
            , node_handle_(node_handle)
    		, velocity_topic_(velocity_topic)
    		, velocity_feedback_topic_(velocity_feedback_topic)
{

		setupUi(this);

        closed_ = false;

        initTopics(input, velocity_feedback_topic, velocity_topic, ackermann_topic);

        velocity_widget_ = new mrm_control_panel::VelocityWidget(joystickContainer);
        velocity_widget_->setMinimumSize(200, 200);
        velocity_widget_->setMaximumSize(200, 200);
        velocity_widget_->setCallback(
                boost::bind(&TeleoperatorInstanceWidget::uiJoystickCallback, this, _1, _2));


        connect(velocity_widget_, SIGNAL(pressedSignal(void)), this, SLOT(speedControlPressed(void)));

        initJoystick(input);
}

TeleoperatorInstanceWidget::~TeleoperatorInstanceWidget() {
	shutdown();
}

void TeleoperatorInstanceWidget::shutdown() {
	closed_ = true;
	delete velocity_widget_;
	velocity_widget_ = NULL;
}

void TeleoperatorInstanceWidget::initJoystick(InputMethod method) {
	switch (method) {
		case TeleoperatorInstanceWidget::Joystick:
			velocity_widget_->disableMouseControl();
			break;

		case TeleoperatorInstanceWidget::Mouse:
			velocity_widget_->enableMouseControl();
			break;

		case TeleoperatorInstanceWidget::Keyboard:
			velocity_widget_->disableMouseControl();
			break;
	}
}

void TeleoperatorInstanceWidget::initTopics(InputMethod method, string feedbackTopicName, string velocityTopic, string ackermann_topic) {
	publisher_ = node_handle_.advertise<geometry_msgs::Twist>(velocityTopic, 1, false);
	ackermannPublisher_ = node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>(ackermann_topic, 1, false);
	initOperationTopic();
}

void TeleoperatorInstanceWidget::initOperationTopic() {
}

void TeleoperatorInstanceWidget::uiJoystickCallback(double linearPercent, double angularPercent) {
	if (closed_)
		return;

	if (!publisher_ || !ackermannPublisher_) 
	{
		ROS_WARN("TeleoperatorInstanceWidget::uiJoystickCallback publisher is not ready");
		return;
	}

	if ((ros::Time::now() - last_ui_joystick_publish_) < ros::Duration(0.1) &&
			(linearPercent != 0 || angularPercent != 0))
		return;

	geometry_msgs::Twist cmdVel;

	linearPercent = fmin(100, fmax(-100, linearPercent));
	angularPercent = fmin(100, fmax(-100, angularPercent));

	cmdVel.linear.x = linearPercent / 50;
	cmdVel.angular.z = angularPercent / 200;
	publisher_.publish(cmdVel);

	ackermann_msgs::AckermannDriveStamped ackermannMsg;

	ackermannMsg.drive.speed = cmdVel.linear.x;
	ackermannMsg.drive.steering_angle = cmdVel.angular.z;

	ackermannPublisher_.publish(ackermannMsg);

	last_ui_joystick_publish_ = ros::Time::now();
}

void TeleoperatorInstanceWidget::speedControlPressed(void) {
}


void TeleoperatorInstanceWidget::keyPressEvent(QKeyEvent *event)
{	
	if (keyPressed)
	{
		keyPressed(event);
	}
}
void TeleoperatorInstanceWidget::keyReleaseEvent(QKeyEvent *event)
{
	if (keyReleased)
	{
		keyReleased(event);
	}
}

}
