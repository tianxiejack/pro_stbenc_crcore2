/*
 * gluVideoWindowSecondScreen.hpp
 *
 *  Created on: Feb 18, 2019
 *      Author: ubuntu
 */

#ifndef GLUVIDEOWINDOWSECONDSCREEN_HPP_
#define GLUVIDEOWINDOWSECONDSCREEN_HPP_

#include "gluVideoWindow.hpp"

class CGluVideoWindowSecond : public CGluVideoWindow
{
public:
	CGluVideoWindowSecond(const cv::Rect& rc, int parent = 0, int nRender = 2);
	virtual ~CGluVideoWindowSecond();
	virtual void Display();
};


#endif /* GLUVIDEOWINDOWSECONDSCREEN_HPP_ */
