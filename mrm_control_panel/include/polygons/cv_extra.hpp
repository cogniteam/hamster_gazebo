/*
 * cv_extra.hpp
 *
 *  Created on: Apr 1, 2014
 *      Author: dan
 */

#ifndef CV_EXTRA_HPP_
#define CV_EXTRA_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cv{

	void fillPoly(
			cv::Mat& mat,
			std::vector< std::vector<Point> >& pts,
			const cv::Scalar& color,
			int lineType=8,
			int shift=0,
			Point offset=Point()
	){
		using namespace std;
		using namespace cv;
		struct Polygons{
			int c;
			int* n;
			Point ** p;
			Polygons(const std::vector< std::vector<Point> >& pts)
				: c(pts.size()), n(new int[pts.size()]), p(new Point* [pts.size()])
			{
				for(size_t i=0;i<pts.size();i++){
					n[i]=pts[i].size();
					p[i]=new Point[n[i]];
					for(size_t j=0;j<pts[i].size();j++){
						(p[i])[j] = (pts[i])[j];
					}
					cout<<endl;
				}
			}
			~Polygons(){
				for(size_t i=0;i<c;i++){ delete[] p[i]; }
				delete[] p;
				delete[] n;
			}
			//size_t points_count(size_t i)const{ return n[i]; }
			//const Point* polygon(size_t i)const{ return p[i]; }
			//const Point& point(size_t i, size_t j)const{ return p[i][j]; }
			const Point** polygons()const{ return (const Point**)p; }
			size_t polygons_count()const{ return c; }
			const int* points_counts()const{ return n; }
		} poly(pts);
		cv::fillPoly(mat, poly.polygons(), poly.points_counts(), poly.polygons_count(), color, lineType, shift, offset);
	}
}


#endif /* CV_EXTRA_HPP_ */
