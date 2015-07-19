///*
// * Polygon2d.h
// *
// *  Created on: Mar 25, 2014
// *      Author: dan
// */
//
#ifndef POLYGON2D_H_
#define POLYGON2D_H_

#include "types.hpp"

#include "clipper.hpp"
#include "UI.hpp"
#include "cv_extra.hpp"
#include <math.h>


namespace poly{

typedef ClipperLib::IntPoint PolygonPoint;
typedef ClipperLib::Polygons PolygonScan;
typedef ClipperLib::Polygons PolygonMap;

inline
PolygonPoint point_poliar(double a, double r){
	return PolygonPoint(cos(a)*r, sin(a)*r);
}
inline
double angle(const PolygonPoint& p){
	return atan2(p.Y,p.X);
}
inline
double length(const PolygonPoint& p){
	return hypot(p.X,p.Y);
}
inline
PolygonPoint scale(const PolygonPoint& p,double kx,double ky){
	return PolygonPoint(p.X*kx,p.Y*ky);
}
inline
PolygonPoint add(const PolygonPoint& p,const PolygonPoint& o){
	return PolygonPoint(p.X+o.X,p.Y+o.Y);
}
inline
PolygonPoint sub(const PolygonPoint& p,const PolygonPoint& o){
	return PolygonPoint(p.X-o.X,p.Y-o.Y);
}

inline
PolygonScan tr_move(const PolygonScan& p, const PolygonPoint& o){
	PolygonScan scan;scan.resize(p.size());
	for(size_t i=0;i<scan.size();i++){
		scan[i].resize(p[i].size());
		for(size_t j=0;j<scan[i].size();j++)scan[i][j] = add(p[i][j],o);
	}
	return scan;
}

inline
float_t put_in_range(float_t min, float_t max, float_t value){
	return fmax(fmin(value,max),min);
}
inline
float_t put_angle_in_range(float_t ang){
	const float f = (float)2.0*3.14159265359;
	const float r2d = 180.0/3.14159265359;
	float t = ::fmod((float)ang,f);
	//cout<<"t "<<t<<" "<<ang<<endl;
	if( t < 3.14159265359 ) return t;
	return t- f;
}

}

#endif /* POLYGON2D_H_ */
