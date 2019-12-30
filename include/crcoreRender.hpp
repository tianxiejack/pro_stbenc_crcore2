/*
 * crcoreRender.hpp
 *
 *  Created on: Jul 5, 2019
 *      Author: ubuntu
 */

#ifndef CRCORERENDER_HPP_
#define CRCORERENDER_HPP_

//core vision 2.0.3++

class CCoreRenderBase
{
public:
	CCoreRenderBase(int winId = -1, const cv::Scalar& bgColor = cv::Scalar(255,0,0));
	virtual ~CCoreRenderBase();
	int Draw(const cv::Rect& pos);
	int Draw(const cv::Rect& pos, const cv::Mat& img);
	int Draw(const cv::Rect& pos, const cv::Rect& roi);
	int Draw(const cv::Rect& pos, int videoId, const cv::Rect& roi);
	virtual void OnRender(void) = 0;
	void *m_render;
};

#endif /* CRCORERENDER_HPP_ */
