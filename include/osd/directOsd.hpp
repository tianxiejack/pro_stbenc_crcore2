/*
 * directOsd.hpp
 *
 *  Created on: Nov 13, 2018
 *      Author: wzk
 */

#ifndef DIRECTOSD_HPP_
#define DIRECTOSD_HPP_

class IDirectOSD
{
public:
	virtual void begin(cv::Scalar& color, int thickness=1) = 0;
	virtual void end(void) = 0;
	virtual void line(const std::vector<cv::Point>& pts, int flag = 0) = 0;
	virtual void polygon(const std::vector<cv::Point>& pts, int flag = 0) = 0;
	virtual void rectangle(const cv::Rect& rec, int flag = 0) = 0;
	virtual void cross(const cv::Point& center, const cv::Size2f& scale, int flag = 0) = 0;
	virtual void numberedBox(const cv::Rect& rec, int number, int flag = 0) = 0;
	virtual void ellipse(const cv::RotatedRect& box, int number=0, int flag = 0) = 0;
};



#endif /* DIRECTOSD_HPP_ */
