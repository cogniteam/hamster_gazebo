/*
 * MapGraphicsView.h
 *
 *  Created on: Feb 26, 2014
 *      Author: xaero
 */

#ifndef MAPGRAPHICSVIEW_H_
#define MAPGRAPHICSVIEW_H_

#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <qevent.h>
#include <qgraphicsview.h>
#include <QPointF>
#include <QScrollBar>



#define KEYBOARD_MOVE_SPEED_SCALE 0.5 //3.0 // 3.0 // 0.5
#define KEYBOARD_TURN_SPEED_SCALE 0.5 //3.0 // 0.5

namespace mrm_control_panel {

class MapGraphicsView : public QGraphicsView {
	// Q_OBJECT
public:
	MapGraphicsView(QWidget *parent = 0);
	MapGraphicsView(QGraphicsScene *scene, QWidget *parent = 0);

	void setDraggable(bool newValue);
	virtual ~MapGraphicsView();

	void setParent(QWidget* parent);

	// std::map<std::string, ros::Publisher>* _move_robot_publishers;

	// RobotGraphicsItem* selected_robot_;
	// std::string MapWidget::getSelectedRobotId()

	boost::function <void(QKeyEvent*)> keyPressed;
	boost::function <void(QKeyEvent*)> keyReleased;

	// std::string* _selected_robot_id;

	// void (*moveKeyPressed)(int);

// Q_SIGNALS:
	// void moveKeyPressedSignal(int key);

protected:
	void mousePressEvent(QMouseEvent* event);
	void mouseReleaseEvent(QMouseEvent* event);
	void mouseDoubleClickEvent(QMouseEvent* event);
	void wheelEvent(QWheelEvent* event);
    void mouseMoveEvent(QMouseEvent *event);

    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);

    // void moveKeyPressed(int key);
    // void moveKeyReleased(int key);

	geometry_msgs::Twist _cmd_vel;

	// QKeyEvent* _key_event;
	//For dragging with the right button
	bool _enableDrag;
    bool _drag;
    int _dragStartX, _dragStartY;
};

} /* namespace mrm_control_panel */

#endif /* MAPGRAPHICSVIEW_H_ */
