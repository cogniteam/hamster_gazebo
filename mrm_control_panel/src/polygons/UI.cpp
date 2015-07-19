/*
 * UI.cpp
 *
 *  Created on: Mar 30, 2014
 *      Author: dan
 */

#include <polygons/UI.hpp>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"



namespace poly {

UI::UI()
:view(0.2,0,0)
{

}

UI::~UI() {
}

void UI::show(std::string name, const Polygons& poly){
	cv::Mat image (700,700,CV_8UC3,cv::Scalar( 255, 255, 255 ));
	cv::Mat zoomed (700,700,CV_8UC3);

	cv::namedWindow(name);
	struct Painter{
		void drawLine( cv::Mat img, cv::Point start, cv::Point end )
		{
		  int thickness = 1;
		  int lineType = 8;
		  cv::line( img,
				start,
				end,
				cv::Scalar( 0, 0, 0 ),
				thickness,
				lineType );
		}
	} painter;
	struct Localizer{
		const View& view;
		cv::Size size;
		Localizer(const View& v,cv::Size _size):view(v),size(_size){}
		cv::Point point(const IntPoint& p){ return cv::Point(p.X*view.zoom+size.width/2.0+view.offset_x,p.Y*view.zoom+size.height/2.0+view.offset_y); }
		cv::Point opoint(const IntPoint& p){ return cv::Point(p.X,p.Y); }
	} localizer(view, image.size());

	for(size_t i=0;i<poly.size();i++){
		const Polygon& p = poly[i];
		for(size_t j=0;j<p.size();j++){
			const IntPoint& cp = p[j];
			const IntPoint& np = p[(j+1)%p.size()];
			painter.drawLine(image, localizer.point(cp),localizer.point(np));
		}
	}

	cv::resize(image, zoomed, zoomed.size());
	cv::flip(zoomed,zoomed, 0);
	cv::imshow(name, zoomed);
	cv::waitKey(3);
}


void UI::waitKey(int i){
	cv::waitKey(i);
}


void UI::show(std::string title,const PolyTree& p){
	Polygons pl; copy(p, pl); show(title,pl);
}
void UI::show(std::string title,const Polygon& p){
	Polygons pl; pl<<p; show(title,pl);
}








} /* poly */
