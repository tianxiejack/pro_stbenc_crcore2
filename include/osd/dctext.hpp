/*
 * dctext.hpp
 *
 *  Created on: Nov 8, 2018
 *      Author: wzk
 */

#ifndef DCTEXT_HPP_
#define DCTEXT_HPP_
namespace cr_osd
{

class DCTXT
{
	void *m_freetype;
public:
	DCTXT(const char* faceName = NULL, int fontSize = 45);
	virtual ~DCTXT(void);
	void putText(const cv::Mat& img, const wchar_t* text, const cv::Point& pt, const cv::Scalar& color);

protected:
};

};//namespace cr_osd



#endif /* DCTEXT_HPP_ */
