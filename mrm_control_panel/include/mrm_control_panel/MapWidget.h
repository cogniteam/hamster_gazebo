/*
 * MapWidget.h
 *
 *  Created on: Feb 12, 2014
 *      Author: xaero
 */

#ifndef MAPWIDGET_H_
#define MAPWIDGET_H_

#include <iostream>
#include <map>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_msgs/TFMessage.h>

#include <qgraphicsview.h>
#include <qgraphicsitem.h>
#include <qgraphicsscene.h>
#include <QGraphicsPixmapItem>

#include <QScrollBar>
#include <QPainter>

#include <sensor_msgs/LaserScan.h>

#include <ui_mrm_map_widget.h>

#include <mrm_control_panel/MapGraphicsItem.h>
#include <mrm_control_panel/RobotGraphicsItem.h>
#include <mrm_control_panel/MapGraphicsView.h>
#include <mrm_control_panel/Parameters.h>

#define INDOOR_FLOOR_COLOR 240, 240, 240
#define SHADOW_COLOR 43, 204, 240

using namespace std;

namespace mrm_control_panel {

class MapWidget : public QWidget, public Ui::MrmMapWidget {
	Q_OBJECT
public:


	boost::function <void(QKeyEvent*)> keyPressed;
	boost::function <void(QKeyEvent*)> keyReleased;
	
	typedef std::pair<std::string, RobotGraphicsItem*> robot_pair_t;

	enum Mode {
		Normal,
		Grab,
		GoalGoAndStop,
		GoalGoAndContinue,
		RestrictedZoneAdding
	};

	MapWidget(ros::NodeHandle& node_handle, QWidget* parent);
	virtual ~MapWidget();

	void shutdown();
	void start();

	void initMapParameters(nav_msgs::OccupancyGridConstPtr);
	void onMapMessage(nav_msgs::OccupancyGridConstPtr);

	MapGraphicsView* getGraphicsView();

private slots:

	QImage getMapImage(nav_msgs::OccupancyGridConstPtr);
	void mapChanged(QImage);
	void tfChanged(const std::string&, const QPointF&, double);

	void onFocusMessage(std_msgs::StringConstPtr);
	void selected(const std::string&);

public slots:

	void onMapPressedSlot(QPointF pos, Qt::MouseButton button);

	void setViewPoint(int,int);
	void setViewOnRobot(std::string robot_id);

	std::string getSelectedRobotId();

signals:

	void mapChangedSignal(QImage);
	void tfChangedSignal(const std::string&, const QPointF&, double);

private:

	void drawShadow(const QPointF& position, const double& heading);

	void handleKeyPressEvent(QKeyEvent*);
	void handleKeyReleaseEvent(QKeyEvent*);

	RobotGraphicsItem* getRobot(const std::string&) const;
	RobotGraphicsItem* registrateRobot(const std::string&, const QPointF&);

	bool getRobotPosition(const std::string&, QPointF&, double&);
	void tfListenerProcess(void);

	void connectSignals();

	QImage map_shadow_;
	QImage square_image_;

	MapGraphicsView* graphicsView_;
	QGraphicsScene* scene_;
	MapGraphicsItem* map_;

	float map_resolutuion_;
	QPointF map_origin_;


	RobotGraphicsItem* selected_robot_;
	std::string active_view_on_robot_;
	
	QPointF selected_position_;

	ros::NodeHandle node_handle_;
	ros::Subscriber map_subscriber_;

	ros::Publisher shadow_counter_publisher_;

	ros::Subscriber shadow_counter_subscriber_;

	ros::Subscriber focus_subscriber_;
	ros::Publisher focus_publisher_;

	std::map<std::string, RobotGraphicsItem*> robots_;

	map<std::string, ros::Publisher>* _move_robot_publishers;

	boost::mutex lock_;
	boost::mutex flags_arr_lock_;
	boost::mutex registration_lock_;

	tf::TransformListener tf_listener_;

	double map_size_;
	Mode mode_;


	boost::thread tf_listener_thread_;
};

} /* namespace mrm_control_panel */

#endif /* MAPWIDGET_H_ */
