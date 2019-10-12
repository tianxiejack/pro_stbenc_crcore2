/*
 * crcoreSecondScreen.hpp
 *
 *  Created on: Feb 21, 2019
 *      Author: ubuntu
 */

#ifndef CRCORESECONDSCREEN_HPP_
#define CRCORESECONDSCREEN_HPP_

//core vision 1.0.10++

class CSecondScreenBase
{
public:
	CSecondScreenBase(const cv::Rect& rc, int fps, bool bFull = true, int fontSize = 45, const char* fontFile = NULL);
	virtual ~CSecondScreenBase();
	virtual void renderCall(int stepIdx, int stepSub, int context);
	virtual int set(int winId, int chId, const cv::Rect& rc, const cv::Matx44f& matric);
	virtual int set(int winId, int chId);
	virtual int set(int winId, const cv::Rect& rc);
	virtual int set(int winId, const cv::Matx44f& matric);
	virtual void OnRender(int stepIdx, int stepSub, int context){};
	void *m_context;
};


#endif /* CRCORESECONDSCREEN_HPP_ */
