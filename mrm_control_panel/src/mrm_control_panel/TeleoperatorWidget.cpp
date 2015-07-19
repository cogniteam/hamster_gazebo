#include <mrm_control_panel/TeleoperatorWidget.h>

namespace mrm_control_panel
{

TeleoperatorWidget::TeleoperatorWidget(QWidget* parent)
	: QWidget(parent)
	, _velocity_widget(NULL)
	, _keyboard_teleop(NULL)
	, _controller(NULL) {

	setupUi(this);

	qRegisterMetaType<std_msgs::StringConstPtr>("std_msgs::StringConstPtr");

	QObject::connect(this, SIGNAL(selectionChangedSignal(std_msgs::StringConstPtr)),
			         this, SLOT(selectionChanged(std_msgs::StringConstPtr)));
}

TeleoperatorWidget::~TeleoperatorWidget() {
}


bool TeleoperatorWidget::setupInputOuput(const std::string& topic, const std::string& ackermann_topic, const std::string& robot_id)
{
	clear();

	TeleoperatorInstanceWidget::InputMethod input = TeleoperatorInstanceWidget::Mouse;
    string feedback_topic = input == TeleoperatorInstanceWidget::Joystick ? "/joy_cmd_vel" : topic;
    _controller = new TeleoperatorInstanceWidget(_node, input, robot_id, topic, ackermann_topic, feedback_topic, this);

	_controller->keyPressed = boost::function<void (QKeyEvent*)>(boost::bind(&TeleoperatorWidget::handleKeyPressEvent,this,_1));
	_controller->keyReleased = boost::function<void (QKeyEvent*)>(boost::bind(&TeleoperatorWidget::handleKeyReleaseEvent,this,_1));

    horizontalLayoutContainer->addWidget(_controller);
    _selected = robot_id;

	return true;
}

void TeleoperatorWidget::shutdown() {
	_conrollable_robot_subscriber.shutdown();
}

void TeleoperatorWidget::subscribeToSelection(const std::string& topic) {
	_conrollable_robot_subscriber =
			_node.subscribe(topic, 1, &TeleoperatorWidget::onSelectionMessage, this);
}

void TeleoperatorWidget::selectionChanged(std_msgs::StringConstPtr message) {
	std::string topic = getVelocityTopic(message->data);
	std::string ackermannTopic = getAckermannTopic(message->data);

	setupInputOuput(topic, ackermannTopic, message->data);
}

void TeleoperatorWidget::onSelectionMessage(std_msgs::StringConstPtr message) {
	emit selectionChangedSignal(message);
}

void TeleoperatorWidget::clear() {
	QLayoutItem* widget;
	while ((widget = horizontalLayoutContainer->takeAt(0)) != NULL) {
		delete widget->widget();
		delete widget;
	}

	_controller = NULL;
}

std::string TeleoperatorWidget::getVelocityTopic(const std::string& robot_id) const {
	std::stringstream stream;

	stream << "/" << robot_id << "/cmd_vel";

	return stream.str();
}


std::string TeleoperatorWidget::getAckermannTopic(const std::string& robot_id) const {
	std::stringstream stream;

	stream << "/" << robot_id << "/ackermann_cmd";

	return stream.str();
}

void TeleoperatorWidget::handleKeyPressEvent(QKeyEvent* event)
{
	if (keyPressed)
	{
		keyPressed(event);
	}

}
void TeleoperatorWidget::handleKeyReleaseEvent(QKeyEvent* event)
{
	if (keyReleased)
	{
		keyReleased(event);
	}
}


} /* namespace mrm_control_panel */
