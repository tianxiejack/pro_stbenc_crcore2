/*
 * crosd.hpp
 *
 *  Created on: Nov 20, 2018
 *      Author: wzk
 */

#ifndef CROSD_HPP_
#define CROSD_HPP_

#include <opencv2/opencv.hpp>
#include <glew.h>

#include <GLBatchMini.h>
#include <GLFrustum.h>
#include <GLFrame.h>
#include <GLMatrixStack.h>
#include <GLGeometryTransform.h>
#include <GLShaderManagerMini.h>

namespace cr_osd
{
extern GLShaderManagerMini		glShaderManager;
extern void bind(int index = -1);
extern void put(const wchar_t* share, const cv::Point& pos, const cv::Scalar& color);
extern void put(const char* share, const cv::Point& pos, const cv::Scalar& color);
extern void put(const unsigned char* share, const wchar_t* format, const cv::Point& pos, const cv::Scalar& color);
extern void put(const int* share, const wchar_t* format, const cv::Point& pos, const cv::Scalar& color);
extern void put(const unsigned int* share, const wchar_t* format, const cv::Point& pos, const cv::Scalar& color);
extern void put(const float* share, const wchar_t* format, const cv::Point& pos, const cv::Scalar& color);
extern void put(const int *share, const int nStat, const cv::Point& pos, const cv::Scalar& color, ...);
extern void set(const void *share, const cv::Scalar& color);
extern void set(const void *share, const int yuv);
extern void set(const cv::Scalar& color);
extern void set(const int yuv);
extern void set(const void *share, const cv::Point& pos);
extern void erase(const void *share);
extern void clear(void);
extern void shprintf(wchar_t *__restrict share, size_t n, const wchar_t *__restrict format, ...);

class IPattern
{
public :
	static IPattern* Create(const cv::Mat& mat, const cv::Rect& rcVeiw, const cv::Scalar& color = cv::Scalar::all(255));
	static IPattern* Create(const std::vector<float>* vArray, const cv::Rect& rcVeiw, const cv::Scalar& color = cv::Scalar::all(255));
	static void Destroy(IPattern *pattern);
	virtual void draw(bool bShow) = 0;
	virtual void updatedata(void) = 0;
	virtual void draw_(void) = 0;
};

class Pattern
{
	std::vector<void *> vbases;
protected:
	Pattern(int nVert, GLenum primitive = GL_LINES, int bindId = -1);
	virtual ~Pattern(void);
	virtual int update(const std::vector<cv::Point>& vpts, const cv::Scalar& color = cv::Scalar::all(255), int thickness = 1);
public:
	virtual void set(const cv::Scalar& color);
	virtual void set(const int yuv);
};

class Line : public Pattern
{
public:
	Line(int bindId = -1):Pattern(2, GL_LINES, bindId){};
	virtual ~Line(void){};
	void draw(const cv::Point& pt1, const cv::Point& pt2, const cv::Scalar& color, int thickness=1){
		std::vector<cv::Point> vpts;
		vpts.push_back(pt1);
		vpts.push_back(pt2);
		update(vpts, color, thickness);
	}
};

class Polygon : public Pattern
{
public:
	Polygon(int nPts = 3, GLenum primitive = GL_POLYGON, int bindId = -1):Pattern(nPts, primitive, bindId){};
	virtual ~Polygon(void){};
	void draw(const std::vector<cv::Point>& vpts, const cv::Scalar& color, int thickness=1){
		update(vpts, color, thickness);
	}
};

};//namespace cr_osd



#endif /* CROSD_HPP_ */
