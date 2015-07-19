/*
 * MapGraphicsItem.cpp
 *
 *  Created on: Feb 23, 2014
 *      Author: xaero
 */

#include <mrm_control_panel/MapGraphicsItem.h>
#include <mrm_control_panel/RobotGraphicsItem.h>

namespace mrm_control_panel {

MapGraphicsItem::MapGraphicsItem(const QPixmap &pixmap,
								 const QPointF& origin,
								 QGraphicsItem* parent,
								 QGraphicsScene *scene)
	: QGraphicsPixmapItem(pixmap, parent, scene) {
	float resolution = Parameters::getInstance()->mapResolution();

	// ROS_ERROR("origin.x() = %f, origin.y() = %f",origin.x(), origin.y());
	translate(origin.x(), origin.y());
	scale(resolution, resolution);
	// ROS_ERROR("resolution = %f",resolution);
	scale(-1, 1);
	rotate(90);
}

MapGraphicsItem::~MapGraphicsItem() {
}

void MapGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent* event) {
	emit mousePressed(mapToScene(event->pos()) , event->button());
}

} /* namespace mrm_control_panel */
