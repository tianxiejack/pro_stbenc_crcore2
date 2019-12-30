/*
 * gltext.hpp
 *
 *  Created on: Nov 8, 2018
 *      Author: wzk
 */

#ifndef GLTEXT_HPP_
#define GLTEXT_HPP_
namespace cr_osd
{

class GLTXT
{
	void *m_freetype;
public:
	GLTXT(const char* faceName = NULL, int fontSize = 45, int nWidth = 1920, int nHeight = 1080);
	virtual ~GLTXT(void);
	void putText(const cv::Size& viewSize, const wchar_t* text, const cv::Point& pt, const cv::Scalar& color);

protected:
};

};//namespace cr_osd



#endif /* GLTEXT_HPP_ */
