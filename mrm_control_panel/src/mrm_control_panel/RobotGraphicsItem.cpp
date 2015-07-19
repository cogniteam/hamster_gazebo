/*
 * RobotGraphicsItem.cpp
 *
 *  Created on: Feb 23, 2014
 *      Author: xaero
 */

#include <mrm_control_panel/RobotGraphicsItem.h>

namespace mrm_control_panel {

RobotGraphicsItem::RobotGraphicsItem(const std::string& id, QPointF position, QGraphicsItem* parent, QGraphicsScene* scene)
	: QGraphicsItemGroup(parent, scene)
	, id_(id)
	, marker_(NULL)
	, marker_size_(0.5)
	, label_(NULL)
	, operation_mode(-1)
	, update_time_stamp_(boost::posix_time::microsec_clock::local_time())
	, update_interval_(1000 / 5) {

	setPosition(position);

	// Marker
	marker_ = new RobotGraphicsMarker(marker_size_);


	QColor fill_color = QColor::fromRgb(TRIANGLE_STYLE_NORMAL);
	updateStyle(fill_color);

	// QPen ellipse_pen = QPen();
	// ellipse_pen.setBrush(QColor::fromRgb(80, 80, 80));
	// marker_->setPen(ellipse_pen);

	// QBrush ellipse_brush = QBrush();
	// ellipse_brush.setColor(QColor::fromRgb(120, 120, 120));
	// ellipse_brush.setStyle(Qt::SolidPattern);
	// marker_->setBrush(ellipse_brush);


	marker_->setToolTip(QString::fromStdString(id_));
	marker_->setParentItem(this);
	addToGroup(marker_);


	// Text label
	label_ = new RobotGraphicsLabel();
	label2_ = new RobotGraphicsLabel();

	QString robot_number = QString::fromStdString(id_);
	robot_number.replace("agent",QString::fromStdString(""));

	QString html = QString::fromStdString("<p style='color: rgb(0,0,0); font: 5pt;'>#robot_id#</p>");
	html.replace("#robot_id#", robot_number);
	label_->setHtml(html);
	label_->setParentItem(marker_);
	addToGroup(label_);
	label_->align();

	label2_->setHtml(QString::fromStdString("<p style='color: rgb(0,0,0); font: 4pt;'></p>"));
	label2_->setParentItem(marker_);
	addToGroup(label2_);
	// label2_->align();

	setCacheMode(QGraphicsItem::ItemCoordinateCache);
	setHandlesChildEvents(false);

	connect(this, SIGNAL(updateFocusSignal(bool,bool)), this, SLOT(updateFocus(bool,bool)));
	connect(this, SIGNAL(updateOperationModeSignal(const std::string&)), this, SLOT(updateOperationMode(const std::string&)));
	connect(marker_, SIGNAL(selected(void)), this, SLOT(selected(void)));
}

RobotGraphicsItem::~RobotGraphicsItem() {
}

void RobotGraphicsItem::updatePosition(QPointF position, double heading) {
	boost::mutex::scoped_lock lock(lock_);

	if (!isUpdateAllowed())
		return;

	update_time_stamp_ = boost::posix_time::microsec_clock::local_time();

	setPosition(position);
	setRotation(heading);
}

bool RobotGraphicsItem::isUpdateAllowed() const {
	boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration difference = now - update_time_stamp_;

	return difference.total_milliseconds() > update_interval_;
}

void RobotGraphicsItem::updateFocus(bool focussed,bool malfunctioned) {
	if (NULL == marker_) {
		ROS_ERROR("Robot marker is not initialized");
		return;
	}

	QColor fill_color;

	if (malfunctioned && !focussed)
	{
		fill_color = QColor::fromRgb(TRIANGLE_STYLE_MALFUNCTIONED);
		updateStyle(fill_color);
		return;
	}

	if (malfunctioned && focussed)
	{
		fill_color = QColor::fromRgb(TRIANGLE_STYLE_MALFUNCTIONED_FOCUSSED);
		updateStyle(fill_color);
		return;
	}

	if (!malfunctioned && !focussed)
	{
		fill_color = QColor::fromRgb(TRIANGLE_STYLE_NORMAL);
		updateStyle(fill_color);
		return;
	}

	if (!malfunctioned && focussed)
	{
		fill_color = QColor::fromRgb(TRIANGLE_STYLE_FOCUSSED);
		updateStyle(fill_color);
		return;
	}

}

void RobotGraphicsItem::updateStyle(QColor& brush_color)
{
	QBrush brush = QBrush();
	QPen pen = QPen();

	pen.setBrush(QColor::fromRgb(80, 80, 80));
	pen.setStyle(Qt::SolidLine);

	brush.setColor(brush_color);
	brush.setStyle(Qt::SolidPattern);


	marker_->setPen(pen);
	marker_->setBrush(brush);

// // GREEN
// 	pen.setBrush(QColor::fromRgb(80, 80, 80));
// 	pen.setStyle(Qt::SolidLine);

// 	brush.setStyle(Qt::SolidPattern);
// 	brush.setColor(QColor::fromRgb(0, 250, 0));


// 	// GRAY
// 	pen.setBrush(QColor::fromRgb(80, 80, 80));
// 	pen.setStyle(Qt::SolidLine);

// 	brush.setColor(QColor::fromRgb(120, 120, 120));
// 	brush.setStyle(Qt::SolidPattern);

}

void RobotGraphicsItem::emitUpdateFocusSignal(bool highlighted, bool malfunctioned) {
	emit updateFocusSignal(highlighted,malfunctioned);
}

void RobotGraphicsItem::selected() {
	emit selected(id_);
}

void RobotGraphicsItem::setPosition(QPointF position) {
	removePosition();
	translate(position.x(), position.y());
	scale(1, -1);
}

void RobotGraphicsItem::setRotation(double angle) {
	removeRotation();
	marker_->setRotation(angle);
}

void RobotGraphicsItem::removeRotation() {
	marker_->setRotation(-marker_->rotation());
}

void RobotGraphicsItem::removePosition() {
	resetTransform();
}

std::string RobotGraphicsItem::Id() const {
	return id_;
}

// todo:delete
// void RobotGraphicsItem::changeItemColor(std::string color)
// {
// 	QPen ellipse_pen = QPen();
// 	ellipse_pen.setBrush(QColor::fromRgb(255, 0, 0));
// 	marker_->setPen(ellipse_pen);

// 	QBrush ellipse_brush = QBrush();
// 	ellipse_brush.setColor(QColor::fromRgb(255, 0, 0));
// 	ellipse_brush.setStyle(Qt::SolidPattern);
// 	marker_->setBrush(ellipse_brush);
// }

void RobotGraphicsItem::emitUpdateOperationModeSignal(const std::string& mode) {
	emit updateOperationModeSignal(mode);
}

void RobotGraphicsItem::updateOperationMode(const std::string& mode) {

	// Remove the "agent" text on robot's label
	QString robot_number = QString::fromStdString(id_);
	robot_number.replace("agent",QString::fromStdString(""));

	QString html = QString::fromStdString("<p style='color: rgb(0,0,0); font: 5pt;'>#robot_id#</p>");
	// QString html = QString::fromStdString("<p style='color: rgb(0,0,0); font: 4pt;'>#robot_id# [#mode#]</p>");

	html.replace("#robot_id#", robot_number);
	// html.replace("#mode#", QString::fromStdString(mode));

	label_->setHtml(html);
	label2_->setHtml(QString::fromStdString("<p style='color: rgb(0,0,0); font: 4pt;'>["+mode+"]</p>"));
}


/*
 * RobotGraphicsMarker
 */
RobotGraphicsMarker::RobotGraphicsMarker(double size, QGraphicsItem* parent, QGraphicsScene* scene)
	: QGraphicsPolygonItem( parent, scene) {
	QPolygonF polygon;

	polygon << QPointF(0.15+size * 0.5, 0);
	polygon << QPointF(0.15+-size * 0.5, size * 0.3);
	polygon << QPointF(0.15+-size * 0.5, -size * 0.3);

	setPolygon(polygon);
}

RobotGraphicsMarker::~RobotGraphicsMarker() {
}

void RobotGraphicsMarker::mousePressEvent(QGraphicsSceneMouseEvent* event) {
	if(event->button()==Qt::LeftButton)
		emit selected();
}


/*
 * RobotGraphicsLabel
 */
RobotGraphicsLabel::RobotGraphicsLabel(QGraphicsItem* parent, QGraphicsScene* scene)
	: QGraphicsTextItem(parent, scene) {
	float resolution = Parameters::getInstance()->mapResolution();
	scale(resolution, resolution);
}

RobotGraphicsLabel::~RobotGraphicsLabel() {
}

void RobotGraphicsLabel::align() {
	QRectF bounding_box = boundingRect();

	translate(-bounding_box.center().x(), -bounding_box.center().y());
	// translate(-bounding_box.center().x(), -bounding_box.center().y() + bounding_box.height());
}


} /* namespace mrm_director_panel */

