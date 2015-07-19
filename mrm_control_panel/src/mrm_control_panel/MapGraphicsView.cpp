/*
 * MapGraphicsView.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: xaero
 */

#include <mrm_control_panel/MapGraphicsView.h>
#include <mrm_control_panel/RobotGraphicsItem.h>
namespace mrm_control_panel {

MapGraphicsView::MapGraphicsView(QWidget *parent)
	: QGraphicsView(parent),
	  _enableDrag(false),
	  _drag(false),
	  _dragStartY(0),
	  _dragStartX(0)  {
}

MapGraphicsView::MapGraphicsView(QGraphicsScene *scene, QWidget *parent)
	: QGraphicsView(scene, parent),
	  _enableDrag(false),
	  _drag(false),
	  _dragStartY(0),
	  _dragStartX(0) {
}


MapGraphicsView::~MapGraphicsView() {
}

void MapGraphicsView::keyPressEvent(QKeyEvent *event)
{
	if (keyPressed)
	{
		keyPressed(event);
	}

	// QGraphicsView::keyPressEvent(event);
}

void MapGraphicsView::keyReleaseEvent(QKeyEvent *event)
{
	if (keyReleased)
	{
		keyReleased(event);
	}

	// QGraphicsView::keyReleaseEvent(event);
}

void MapGraphicsView::setDraggable(bool newValue){
	_enableDrag=newValue;
}
void MapGraphicsView::mousePressEvent(QMouseEvent *event) {
	QGraphicsView::mousePressEvent(event);

	//Configurations for left-click and middle-click dragging
	if (true or _enableDrag) {
		if (event->button() == Qt::MiddleButton ||
			event->button() == Qt::LeftButton) {
			_drag = true;
			_dragStartX = event->x();
			_dragStartY = event->y();
			setCursor(Qt::ClosedHandCursor);
			event->accept();
			return;
		}
		event->ignore();
	}
}

void MapGraphicsView::mouseReleaseEvent(QMouseEvent *event) {
	QGraphicsView::mouseReleaseEvent(event);

	//Configurations for left-click and middle-click dragging
	if (true or _enableDrag) {
		if (event->button() == Qt::MiddleButton ||
			event->button() == Qt::LeftButton) {
			_drag = false;
			setCursor(Qt::ArrowCursor);
			event->accept();
			return;
		}
		event->ignore();
	}
}

void MapGraphicsView::mouseDoubleClickEvent(QMouseEvent* event) {
	QGraphicsView::mouseDoubleClickEvent(event);
}

void MapGraphicsView::mouseMoveEvent(QMouseEvent *event)
{
	//Configurations for right-click dragging
	if (true or _enableDrag) {
		if (_drag) {
			horizontalScrollBar()->setValue(horizontalScrollBar()->value() - (event->x() - _dragStartX));
			verticalScrollBar()->setValue(verticalScrollBar()->value() - (event->y() - _dragStartY));
			_dragStartX = event->x();
			_dragStartY = event->y();
			event->accept();
			return;
		}
		event->ignore();
	}
}

void MapGraphicsView::wheelEvent(QWheelEvent* event) {
	const QPointF scene_point = mapToScene(event->pos());

	double factor = pow(1.001, event->delta());
	scale(factor, factor);

//	const QPointF mouse_point = mapFromScene(scene_point);
//	const QPointF destination_point = mouse_point - event->pos();
//
//	horizontalScrollBar()->setValue(destination_point.x() + horizontalScrollBar()->value());
//	verticalScrollBar()->setValue(destination_point.y() + verticalScrollBar()->value());
}

} /* namespace mrm_control_panel */
