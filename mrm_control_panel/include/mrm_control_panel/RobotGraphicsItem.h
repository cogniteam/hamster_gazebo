/*
 * RobotGraphicsItem.h
 *
 *  Created on: Feb 23, 2014
 *      Author: xaero
 */

#ifndef ROBOTGRAPHICSITEM_H_
#define ROBOTGRAPHICSITEM_H_

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <QRectF>
#include <QPointF>
#include <QPen>
#include <QBrush>
#include <QGraphicsSceneMouseEvent>
#include <QFocusEvent>
#include <QGraphicsTextItem>

#include <qgraphicsscene.h>
#include <qgraphicsitem.h>

#include <mrm_control_panel/Parameters.h>

#define TRIANGLE_STYLE_NORMAL 120, 120, 120
#define TRIANGLE_STYLE_MALFUNCTIONED 240, 35, 35
#define TRIANGLE_STYLE_MALFUNCTIONED_FOCUSSED 230, 130, 30
#define TRIANGLE_STYLE_FOCUSSED 0, 250, 0

namespace mrm_control_panel {

class RobotGraphicsMarker;
class RobotGraphicsLabel;

class RobotGraphicsItem: public QObject, public QGraphicsItemGroup {
	Q_OBJECT
public:
	// void changeItemColor(std::string color);
	
	RobotGraphicsItem(const std::string& id, QPointF position,
					  QGraphicsItem* parent = 0, QGraphicsScene* scene = 0);
	virtual ~RobotGraphicsItem();

	std::string Id() const;

	int operation_mode;

public Q_SLOTS:
	void updatePosition(QPointF, double);
	void updateFocus(bool focussed,bool malfunctioned);
	void updateOperationMode(const std::string&);

	void selected(void);

	void emitUpdateFocusSignal(bool,bool);
	void emitUpdateOperationModeSignal(const std::string&);
Q_SIGNALS:
	void updatePositionSignal(QPointF);
	void updateFocusSignal(bool,bool);
	void updateOperationModeSignal(const std::string&);


	void selected(const std::string&);

private:
	void updateStyle(QColor& brush_color);

    bool isUpdateAllowed() const;

    void setPosition(QPointF);
    void removePosition();
    void setRotation(double);
    void removeRotation();

    std::string id_;
    
    RobotGraphicsMarker* marker_;
	double marker_size_;

	RobotGraphicsLabel* label_;
	RobotGraphicsLabel* label2_;

	boost::mutex lock_;
	boost::posix_time::ptime update_time_stamp_;
	double update_interval_;

};




class RobotGraphicsMarker : public QObject, public QGraphicsPolygonItem {
	Q_OBJECT
public:
	RobotGraphicsMarker(double, QGraphicsItem *parent = 0, QGraphicsScene* scene = 0);
	virtual ~RobotGraphicsMarker();
Q_SIGNALS:
	void selected();

protected:
	void mousePressEvent(QGraphicsSceneMouseEvent*);
};




class RobotGraphicsLabel : public QGraphicsTextItem {
	Q_OBJECT
public:
	RobotGraphicsLabel(QGraphicsItem* parent = 0, QGraphicsScene* scene = 0);
	virtual ~RobotGraphicsLabel();

	void align();
};

} /* namespace mrm_director_panel */

#endif /* ROBOTGRAPHICSITEM_H_ */
