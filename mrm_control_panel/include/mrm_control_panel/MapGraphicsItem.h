/*
 * MapGraphicsItem.h
 *
 *  Created on: Feb 23, 2014
 *      Author: xaero
 */

#ifndef MAPGRAPHICSITEM_H_
#define MAPGRAPHICSITEM_H_

#include <ros/ros.h>

#include <QPoint>
#include <QPointF>
#include <QPen>
#include <QBrush>
#include <QPainter>
#include <qimage.h>
#include <qgraphicsscene.h>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsItem>
#include <qevent.h>

#include <mrm_control_panel/Parameters.h>

namespace mrm_control_panel {

class MapGraphicsItem: public QObject, public QGraphicsPixmapItem {
	Q_OBJECT
public:
	MapGraphicsItem(const QPixmap &pixmap, const QPointF& origin,
					QGraphicsItem* parent = 0, QGraphicsScene *scene = 0);
	virtual ~MapGraphicsItem();

Q_SIGNALS:
	void mousePressed(QPointF pos, Qt::MouseButton button);
protected:
	void mousePressEvent(QGraphicsSceneMouseEvent *event);
};


} /* namespace mrm_control_panel */

#endif /* MAPGRAPHICSITEM_H_ */
