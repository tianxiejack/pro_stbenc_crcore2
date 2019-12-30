/*
 * gluWindow.hpp
 *
 *  Created on: Dec 17, 2018
 *      Author: wzk
 */

#ifndef GLUWINDOW_HPP_
#define GLUWINDOW_HPP_

#include <opencv2/opencv.hpp>
#include <osa_buf.h>
#include <glew.h>
#include <glut.h>
#include <freeglut_ext.h>
#include "renderBase.hpp"

class CGLVideo;
class CGluWindow
{
	static int count;
public:
	CGluWindow(const cv::Rect& rc, int parent = 0);
	virtual ~CGluWindow();
	int Create(bool bFullScreen = true);
	virtual void Destroy();
	virtual void Display();
	virtual void Display2();
	virtual void Reshape(int width, int height);

	int m_index;
	int m_winId;
	cv::Rect m_rc, m_rcReal;
	int m_parent;

protected:
	static void _reshape(int width, int height);
	static void _display(void);

};


#endif /* GLUWINDOW_HPP_ */
