/*
 * UI.h
 *
 *  Created on: Mar 30, 2014
 *      Author: dan
 */

#ifndef UI_H_
#define UI_H_

#include "clipper.hpp"

namespace poly {

class UI {
public:
	struct View{
		double zoom;
		double offset_x,offset_y;
		View(double z, double x, double y):zoom(z),offset_x(x),offset_y(y){}
	};

	View view;
public:
	UI();
	virtual ~UI();

	void show(std::string title,const Polygons& p);
	void show(std::string title,const PolyTree& p);
	void show(std::string title,const Polygon& p);
	void waitKey(int=0);
};

} /* namespace poly */

#endif /* UI_H_ */
