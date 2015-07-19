/*
 * MapWidget.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: xaero
 */

#include <mrm_control_panel/MapWidget.h>

namespace mrm_control_panel {


MapWidget::MapWidget(ros::NodeHandle& node_handle, QWidget* parent)
	: QWidget(parent)
	, graphicsView_(NULL)
	, _move_robot_publishers(NULL)
	, scene_(NULL)
	, map_(NULL)
	, map_resolutuion_(0.05)
	, map_origin_(0, 0)
	, selected_robot_(NULL)
	// , _selected_robot_id("")
	, selected_position_(0, 0)
	, node_handle_(node_handle)
	, map_size_(0)
	, mode_(Normal)
	, active_view_on_robot_("") {

	setupUi(this);

	connectSignals();
}

MapWidget::~MapWidget() {
	shutdown();

	tf_listener_thread_.interrupt();
	tf_listener_thread_.join();
}

void MapWidget::connectSignals() {
	qRegisterMetaType<std::string>("std::string");
	qRegisterMetaType<nav_msgs::OccupancyGrid>("nav_msgs::OccupancyGrid");

	connect(this, SIGNAL(mapChangedSignal(QImage)),
			this, SLOT(mapChanged(QImage)));

	connect(this, SIGNAL(tfChangedSignal(const std::string&, const QPointF&, double)),
			this, SLOT(tfChanged(const std::string&, const QPointF&, double)));

	connect(this, SIGNAL(objectFoundSignal(mrm_msgs::ObjectFoundConstPtr)),
			this, SLOT(objectFound(mrm_msgs::ObjectFoundConstPtr)));

	connect(this, SIGNAL(objectApproveSignal(mrm_msgs::ObjectApprovalConstPtr)),
			this, SLOT(objectApprove(mrm_msgs::ObjectApprovalConstPtr)));

	connect(this, SIGNAL(setGoalSignal(int, QPointF&)),
				this, SLOT(setGoalSlot(int, QPointF&)));

	connect(this, SIGNAL(removeGoalFlagSignal(const std::string&)),
				this, SLOT(removeGoalFlagSlot(const std::string&)));
}

void MapWidget::shutdown() {
	map_subscriber_.shutdown();
	focus_subscriber_.shutdown();
	focus_publisher_.shutdown();
}

void MapWidget::start() {
	map_subscriber_ = node_handle_.subscribe("/map", 1, &MapWidget::initMapParameters, this);
}

void MapWidget::initMapParameters(nav_msgs::OccupancyGridConstPtr message) {
	map_subscriber_.shutdown();

	float resolution = message->info.resolution;
	QPointF origin = QPointF(message->info.origin.position.x,
							 message->info.origin.position.y);
	Parameters::getInstance()->initialize(resolution, origin);

	unsigned int width = message->info.width;
	unsigned int height = message->info.height;

	onMapMessage(message);

	map_subscriber_ 			= node_handle_.subscribe("/map", 1,
											 	 	 	 &MapWidget::onMapMessage, this);

	focus_subscriber_ = node_handle_.subscribe("/mrm/control_panel/focus", 10,
											   &MapWidget::onFocusMessage, this);
	focus_publisher_ = node_handle_.advertise<std_msgs::String>("/mrm/control_panel/focus", 10);

}

void MapWidget::onMapMessage(nav_msgs::OccupancyGridConstPtr message) {
	QImage image = getMapImage(message);

	emit mapChangedSignal(image);
}

QImage MapWidget::getMapImage(nav_msgs::OccupancyGridConstPtr message) {
	unsigned int width = message->info.width;
	unsigned int height = message->info.height;

	QImage image = QImage(width, height, QImage::Format_RGB888);
	for (unsigned int i = 0; i < width; i++) {
		for (unsigned int y = 0; y < height; y++) {
			unsigned char pixel = message->data[i * width + y] + 1;

			// if (0 == map_shadow_[i][y])
			// {
			// 	// Free Visited
			// 	image.setPixel(y, i, qRgb(0, 240, 0));
			// 	continue;
			// }

			switch (pixel) {
				case 0: // Unknown
					image.setPixel(i, y, qRgb(189, 189, 189));
					break;
				case 1: // Free
						image.setPixel(i, y, qRgb(INDOOR_FLOOR_COLOR));
					// if (0 == map_shadow_[i][y])
					// {
					// 	// Free Visited
					// 	image.setPixel(i, y, qRgb(0, 240, 0));
					// }
					// else
					// {
					// 	// Free Not visited
					// }
					break;
				default: // Occupied
					// image.setPixel(i, y, qRgb(99, 99, 99));
					image.setPixel(i, y, qRgb(44, 44, 44));
					break;
			}
		}
	}

	map_shadow_ = image;

	return image;
}

void MapWidget::handleKeyPressEvent(QKeyEvent* event)
{
	if (keyPressed)
	{
		keyPressed(event);
	}

}
void MapWidget::handleKeyReleaseEvent(QKeyEvent* event)
{
	if (keyReleased)
	{
		keyReleased(event);
	}
}

void MapWidget::mapChanged(QImage map) {
	lock_.lock();
	if (map_ == NULL) {
		Parameters* parameters = Parameters::getInstance();

		graphicsView_ = new MapGraphicsView(frame);

		graphicsView_->keyPressed = boost::function<void (QKeyEvent*)>(boost::bind(&MapWidget::handleKeyPressEvent,this,_1));
		graphicsView_->keyReleased = boost::function<void (QKeyEvent*)>(boost::bind(&MapWidget::handleKeyReleaseEvent,this,_1));

		graphicsView_->setCacheMode(QGraphicsView::CacheNone);
		graphicsView_->setAlignment(Qt::AlignCenter);
		graphicsView_->setHorizontalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );
		graphicsView_->setVerticalScrollBarPolicy ( Qt::ScrollBarAlwaysOff );

		frameGridLayout->addWidget(graphicsView_, 0, 0, 1, 1);

		graphicsView_->scale(1, -1);
		graphicsView_->scale(1 / parameters->mapResolution(),
							 1 / parameters->mapResolution());

		scene_ = new QGraphicsScene(graphicsView_);
		graphicsView_->setScene(scene_);

		map_ = new MapGraphicsItem(QPixmap::fromImage(map), parameters->mapOrigin(), 0, scene_);
		graphicsView_->centerOn(map_);

		connect(map_, SIGNAL(mousePressed(QPointF , Qt::MouseButton)), this, SLOT(onMapPressedSlot(QPointF , Qt::MouseButton)));

		map_size_ = map.width() * parameters->mapResolution();

		square_image_ = QImage(map.width(), map.height(), QImage::Format_RGB888);

		tf_listener_thread_ = boost::thread(&MapWidget::tfListenerProcess, this);
	} else {
		map_->setPixmap(QPixmap::fromImage(map));
	}


	lock_.unlock();
}

MapGraphicsView* MapWidget::getGraphicsView()
{
	return graphicsView_;
}

void MapWidget::setViewOnRobot(std::string robot_id)
{
	if (active_view_on_robot_ == robot_id)
	{
		return;
	}

	RobotGraphicsItem* robot = getRobot(robot_id);

	QPointF position;
	double heading;

	getRobotPosition("/"+robot_id+"/base_link", position, heading);

	graphicsView_->setTransform(QTransform());

	graphicsView_->centerOn(position.x(),position.y());

	// graphicsView_->scale(1, -1);
	graphicsView_->scale(56, -56);

	active_view_on_robot_ = robot_id;
}

std::string MapWidget::getSelectedRobotId()
{
	if (NULL == selected_robot_) {
		return "";
	}

	return selected_robot_->Id();
}

void MapWidget::setViewPoint(int x,int y) /*not used yet*/
{
	if (NULL == selected_robot_)
	{
		ROS_ERROR("In MapWidget::setViewPoint: selected_robot_ is NULL. aborting set view.");
		return;
	}

	graphicsView_->centerOn(selected_robot_->x(),selected_robot_->y());
}

void MapWidget::tfChanged(const std::string& robot_id, const QPointF& position, double heading) {
	boost::mutex::scoped_lock lock(lock_);

	RobotGraphicsItem* robot = getRobot(robot_id);
	if (robot == NULL) {
		robot = registrateRobot(robot_id, position);
		if (robot == NULL) {
			ROS_WARN("Robot registration failed. Key was present");
			return;
		}
	}

	robot->updatePosition(position, heading);
}

RobotGraphicsItem* MapWidget::getRobot(const std::string& robot_id) const {
	std::map<std::string, RobotGraphicsItem*>::const_iterator robot = robots_.find(robot_id);

	if (robot == robots_.end())
		return NULL;

	return robot->second;
}

RobotGraphicsItem* MapWidget::registrateRobot(const std::string& robot_id,
											  const QPointF& position) {
	boost::mutex::scoped_lock lock = boost::mutex::scoped_lock(registration_lock_);

	RobotGraphicsItem* robot = getRobot(robot_id);
	if (robot != NULL)
		return robot;

	robot = new RobotGraphicsItem(robot_id, position, 0, scene_);
	connect(robot, SIGNAL(selected(const std::string&)),
			this, SLOT(selected(const std::string&)));

	robot_pair_t entry = std::make_pair<std::string, RobotGraphicsItem*>(robot_id, robot);

	if (!robots_.insert(entry).second) {
		return NULL;
	}

	ROS_INFO("Robot '%s' has been registered", robot_id.c_str());

	return robot;
}

bool MapWidget::getRobotPosition(const std::string& robot_tf_topic, QPointF& position, double& heading) {
	try {
		tf::StampedTransform tf;

		tf_listener_.lookupTransform("/map", robot_tf_topic, ros::Time(0), tf);

		position = QPointF(tf.getOrigin().x(), tf.getOrigin().y());

		heading = -tf::getYaw(tf.getRotation()) * 180.0 / 3.14159265358979323846;
	} catch(tf::TransformException& ex) {
//		ROS_WARN("> %s", ex.what());
		return false;
	}

	return true;
}

void MapWidget::drawShadow(const QPointF& position, const double& heading)
{
	unsigned int x_img_pos = ceil((position.x()+17.9375)/0.035);
	unsigned int y_img_pos = ceil((position.y()+17.9375)/0.035);

	map_shadow_.setPixel(y_img_pos, x_img_pos, qRgb(SHADOW_COLOR));

	
	// Draw shadow of robot on square image	

	QPainter painter(&square_image_);
	painter.setPen(qRgb(SHADOW_COLOR));
	QBrush brush(qRgb(SHADOW_COLOR));
	painter.setBrush(brush);

	painter.drawPie(y_img_pos - 42, x_img_pos -45, 80, 100, -(heading + 120) * 16, 960);
}

void MapWidget::tfListenerProcess() 
{
	QPointF position;
	double heading = 0.0;

	bool shadow_changed = false;

	std::stringstream stream;

	while (ros::ok() && !tf_listener_thread_.interruption_requested()) 
	{
		if (getRobotPosition("/agent1/base_link", position, heading)) {

			drawShadow(position,heading);

			shadow_changed = true;

			emit tfChangedSignal("agent1", position, heading);
		}

		if (getRobotPosition("/agent2/base_link", position, heading)) {

			drawShadow(position,heading);

			shadow_changed = true;

			emit tfChangedSignal("agent2", position, heading);
		}

		if (getRobotPosition("/agent3/base_link", position, heading)) {

			drawShadow(position,heading);

			shadow_changed = true;

			emit tfChangedSignal("agent3", position, heading);
		}

		if (getRobotPosition("/agent4/base_link", position, heading)) {

			drawShadow(position,heading);

			shadow_changed = true;

			emit tfChangedSignal("agent4", position, heading);
		}

		if (getRobotPosition("/agent5/base_link", position, heading)) {

			drawShadow(position,heading);

			shadow_changed = true;

			emit tfChangedSignal("agent5", position, heading);
		}

		if (getRobotPosition("/agent6/base_link", position, heading)) {

			drawShadow(position,heading);

			shadow_changed = true;

			emit tfChangedSignal("agent6", position, heading);
		}

		if (getRobotPosition("/agent7/base_link", position, heading)) {

			drawShadow(position,heading);

			shadow_changed = true;

			emit tfChangedSignal("agent7", position, heading);
		}

		if (getRobotPosition("/agent8/base_link", position, heading)) {

			drawShadow(position,heading);

			shadow_changed = true;

			emit tfChangedSignal("agent8", position, heading);
		}

		if (getRobotPosition("/agent9/base_link", position, heading)) {

			drawShadow(position,heading);

			shadow_changed = true;

			emit tfChangedSignal("agent9", position, heading);
		}

		if (getRobotPosition("/agent10/base_link", position, heading)) {

			drawShadow(position,heading);

			shadow_changed = true;

			emit tfChangedSignal("agent10", position, heading);
		}

		if (true == shadow_changed)
		{
			shadow_changed = false;

			emit mapChangedSignal(map_shadow_);
		}

		boost::this_thread::sleep(boost::posix_time::milliseconds(200));
	}//while
}

void MapWidget::onMapPressedSlot(QPointF position, Qt::MouseButton button) {
//	if (selected_goal_marker_ != NULL) {
//		scene_->removeItem(selected_goal_marker_);
//		delete selected_goal_marker_;
//		selected_goal_marker_ = NULL;
//	}

	//handle left button clicks only
	if(button != Qt::LeftButton && button != Qt::RightButton)
	{
		return;
	}

	if (Qt::RightButton == button)
	{
		if (NULL == selected_robot_) {
			return;
		}

		selected_position_ = position;

		return;
	}

	switch (mode_) 
	{
		case Normal:
			break;
		case Grab:
			break;
	}
}

void MapWidget::onFocusMessage(std_msgs::StringConstPtr message) {
	
	boost::mutex::scoped_lock lock(lock_);

	for(std::map<std::string, RobotGraphicsItem*>::iterator iterator = robots_.begin();
			iterator != robots_.end(); iterator++) 
	{
		if (iterator->first == message->data) 
		{
			selected_robot_ = iterator->second;
			// _selected_robot_id = selected_robot_->Id();//todo
			iterator->second->emitUpdateFocusSignal(true, false);
		} 
		else 
		{
			iterator->second->emitUpdateFocusSignal(false, false);
		}
	}
}

void MapWidget::selected(const std::string& robot_id) {

	boost::mutex::scoped_lock lock(lock_);

	if (!focus_publisher_) {
	  ROS_WARN("Publisher invalid!");
	  return;
	}

	if (NULL != selected_robot_ &&
		selected_robot_->Id() == robot_id)
	{
		// this selection is excess
		return;
	}

	std_msgs::String message;
	message.data = robot_id;

	focus_publisher_.publish(message);

	active_view_on_robot_ = "";
}

} /* namespace mrm_control_panel */
