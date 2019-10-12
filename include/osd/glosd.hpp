/*
 * glosd.hpp
 *
 *  Created on: Nov 6, 2018
 *      Author: wzk
 */

#ifndef GLOSD_HPP_
#define GLOSD_HPP_

#include <GLShaderManagerMini.h>
#include <math3d.h>
#include <opencv2/opencv.hpp>
#include "directOsd.hpp"
#include "gltext.hpp"
#include "dctext.hpp"
#include "crosd.hpp"
#include "osa_mutex.h"

namespace cr_osd
{
class GLOSDFactory;
class GLOSDTxt
{
protected:
	friend class GLOSDFactoryBase;
	friend class GLOSD;
	friend class DCOSD;
	cv::Size2f m_posScale;
	int m_index;
	bool m_bUpdate;
	bool m_bAlloc;
	cv::Point m_pos;
	M3DVector4f m_vColors;
	wchar_t *m_text;
	std::vector<wchar_t *>m_format;
	size_t m_textLen;
	size_t m_memSize;
	OSA_MutexHndl *m_mutexlock;
	const wchar_t *m_sValue;
	const char *m_strValue;
	const int* m_iValue;
	const unsigned int* m_uiValue;
	const unsigned char* m_byValue;
	const float* m_fValue;
	const int* m_statValue;
public:
	GLOSDFactory *m_factory;
	const void* m_share;
	cv::Size m_viewport;
	cv::Point2f m_center;
	int m_fontSize;
	GLOSDTxt(GLOSDFactory *factory, const cv::Size2f& posScale = cv::Size2f(1.0, 1.0));
	virtual ~GLOSDTxt(void);
	virtual void update(void);
	void setcolor(const cv::Scalar& color);
	void setpos(const cv::Point& pt);
	void txt(const cv::Point& pt, const wchar_t* text, const cv::Scalar& color);
	void txt(const cv::Point& pt, const char* text, const cv::Scalar& color);
	void txt(const cv::Point& pt, const unsigned char *byValue, const wchar_t* format, const cv::Scalar& color);
	void txt(const cv::Point& pt, const int *iValue, const wchar_t* format, const cv::Scalar& color);
	void txt(const cv::Point& pt, const unsigned int *uiValue, const wchar_t* format, const cv::Scalar& color);
	void txt(const cv::Point& pt, const float *fValue, const wchar_t* format, const cv::Scalar& color);
	void txt(const cv::Point& pt, const cv::Scalar& color, const int *statValue, const int nStat, __gnuc_va_list __arg);
};

class GLOSDUNITBase
{
protected:
	friend class GLOSDFactoryBase;
	friend class GLOSD;
	friend class DCOSD;
	friend class Pattern;
	GLOSDFactory *m_factory;
	int m_index;
	bool m_bUpdate;
	std::vector<cv::Point> m_vtps;
	GLfloat *vVertexPos;
	GLfloat *vColorPos;
	int m_nVert;
	int m_thickness;
	GLenum m_primitive;
	GLenum m_dtype;
	bool m_bRect;
	OSA_MutexHndl *m_mutexlock;
	GLOSDUNITBase(GLOSDFactory *factory, int nVert, GLenum primitive = GL_LINES);
	virtual ~GLOSDUNITBase(void);
public:
	cv::Size m_viewport;
	cv::Point2f m_center;
	inline cv::Point2f normalized(const cv::Point& pt)
	{
		cv::Point2f rpt(((float)pt.x-m_center.x)/m_center.x, (m_center.y - (float)pt.y)/m_center.y);
		//OSA_printf("%s %d: c(%f, %f) pt(%d, %d) rpt(%f, %f)", __func__, __LINE__,
		//		m_center.x, m_center.y, pt.x, pt.y, rpt.x, rpt.y);
		return rpt;
	}
};

class GLOSDLine : public GLOSDUNITBase
{
public:
	GLOSDLine(GLOSDFactory *factory);
	virtual ~GLOSDLine(void);
	void line(const cv::Point& pt1, const cv::Point& pt2, const cv::Scalar& color, int thickness=1);
};

class GLOSDPolygon : public GLOSDUNITBase
{
public:
	GLOSDPolygon(GLOSDFactory *factory, int npts);
	virtual ~GLOSDPolygon(void);
	void polygon(const cv::Point* pts, const cv::Scalar& color, int thickness=1);
};

class GLOSDRect : public GLOSDPolygon
{
public:
	GLOSDRect(GLOSDFactory *factory);
	virtual ~GLOSDRect(void);
	void rect(const cv::Rect& rec, const cv::Scalar& color, int thickness=1);
};

class GLOSDRectangle : public GLOSDUNITBase
{
public:
	GLOSDRectangle(GLOSDFactory *factory);
	virtual ~GLOSDRectangle(void);
	void rectangle(const cv::Rect& rec, const cv::Scalar& color, int thickness=1);
};

/****************************************************
 * group
 *
 */
class GLOSDCross
{
	GLOSDLine *lines[5];
public:
	GLOSDCross(GLOSDFactory *factory){
		for(int i=0; i<5; i++)
			lines[i] = new GLOSDLine(factory);
	}
	virtual ~GLOSDCross(void){
		for(int i=0; i<5; i++)
			delete lines[i];
	}
	void cross(const cv::Point& pt, const cv::Size& length, const cv::Scalar& color, int thickness=1){
		cv::Point end(length.width*0.3, length.height*0.3);
		lines[0]->line(pt+cv::Point(-2,0), pt+cv::Point(2,0), color, thickness);
		lines[1]->line(pt+cv::Point(-length.width,0), pt+cv::Point(-end.x,0), color, thickness);
		lines[2]->line(pt+cv::Point(end.x,0), pt+cv::Point(length.width,0), color, thickness);
		lines[3]->line(pt+cv::Point(0,-length.height), pt+cv::Point(0,-end.y), color, thickness);
		lines[4]->line(pt+cv::Point(0,end.y), pt+cv::Point(0,length.height), color, thickness);
	}
};

class GLOSDNumberedBox
{
	GLOSDRect *box;
	GLOSDTxt *number;
	wchar_t wsNumber[16];
public:
	GLOSDNumberedBox(GLOSDFactory *factory){
		box = new GLOSDRect(factory);
		number = new GLOSDTxt(factory);
	}
	virtual ~GLOSDNumberedBox(void){
		delete box;
		delete number;
	}
	void numbox(int num, const cv::Rect& rc, const cv::Scalar& color, int thickness=1){
		swprintf(wsNumber, 16, L"%d", num+1);
		box->rect(rc, color, thickness);
		number->txt(cv::Point(rc.x+rc.width+2, rc.y-number->m_fontSize-2), wsNumber, color);
	}
};

/****************************************************
 *
 *
 */
class GLOSDFactory : public IDirectOSD
{
public:
	cv::Rect m_curViewport;
	cv::Rect m_viewport;
	virtual void sharelock(void) = 0;
	virtual void shareunlock(void) = 0;
	virtual void Draw(void) = 0;
	virtual void Add(IPattern* ptt) = 0;
	virtual void Erase(IPattern* ptt) = 0;
	virtual void Add(GLOSDTxt* txt) = 0;
	virtual void Erase(GLOSDTxt* txt) = 0;
	virtual void Add(GLOSDUNITBase* unit) = 0;
	virtual void Erase(GLOSDUNITBase* unit) = 0;
	virtual void SetThickness(GLOSDUNITBase* unit, int thickness) = 0;
};

typedef std::vector<GLOSDTxt*> vTXT;
typedef std::vector<GLOSDTxt*>::iterator viTXT;
typedef std::vector<GLOSDUNITBase*> vUNIT;
typedef std::vector<GLOSDUNITBase*>::iterator viUNIT;
typedef std::vector<vUNIT> aUNIT;
typedef std::vector<IPattern*> vPtt;
typedef std::vector<IPattern*>::iterator viPtt;
class GLOSDFactoryBase : public GLOSDFactory
{
protected:
	cv::Point2f m_center;
	int m_fontSize;
	OSA_MutexHndl m_mutexlock;
	vTXT vecTxts;
	vUNIT vecUnits;
	aUNIT mapUnits;
	vPtt vPatterns;
	std::vector<int> vecCnt;
	cv::Scalar m_color;
	int m_thickness;
protected:
	void GLOSDFactoryBase_(const cv::Rect& viewport, int fontSize);

public:
	GLOSDFactoryBase(int vWidth = 1920, int vHeight = 1080, int fontSize = 45);
	GLOSDFactoryBase(const cv::Rect& viewport, int fontSize = 45);
	virtual ~GLOSDFactoryBase(void);
	//virtual void Draw(void){};
	virtual void Add(IPattern* ptt);
	virtual void Erase(IPattern* ptt);
	virtual void Add(GLOSDTxt* txt);
	virtual void Erase(GLOSDTxt* txt);
	virtual void Add(GLOSDUNITBase* unit);
	virtual void Erase(GLOSDUNITBase* unit);
	virtual void SetThickness(GLOSDUNITBase* unit, int thickness);
	virtual void sharelock(void){OSA_mutexLock(&m_mutexlock);};
	virtual void shareunlock(void){OSA_mutexUnlock(&m_mutexlock);};
};
class GLOSD : public GLOSDFactoryBase
{
protected:
	GLTXT *m_gltxt;
	std::vector<cv::Point2f> m_vPts;
	std::vector<cv::Point> m_vNumPts;
	std::vector<int> m_vNums;
	inline cv::Point2f normalized(const cv::Point& pt)
	{
		cv::Point2f rpt(((float)pt.x-m_center.x)/m_center.x, (m_center.y - (float)pt.y)/m_center.y);
		//OSA_printf("%s %d: c(%f, %f) pt(%d, %d) rpt(%f, %f)", __func__, __LINE__,
		//		m_center.x, m_center.y, pt.x, pt.y, rpt.x, rpt.y);
		//OSA_printf("%s: (%d,%d)(%f %f)(%f %f)", __func__, pt.x, pt.y, rpt.x,rpt.y,m_center.x, m_center.y);
		return rpt;
	}
	void directDraw(const cv::Scalar& norColor, float  thickness);

public:
	GLOSD(int vWidth = 1920, int vHeight = 1080, int fontSize = 45, const char* faceName = NULL);
	GLOSD(const cv::Rect& viewport = cv::Rect(0, 0, 1920, 1080), int fontSize = 45, const char* faceName = NULL);
	virtual ~GLOSD(void);
	virtual void Draw(void);
	virtual void begin(cv::Scalar& color, int thickness=1);
	virtual void end(void);
	virtual void line(const std::vector<cv::Point>& pts, int flag = 0);
	virtual void polygon(const std::vector<cv::Point>& pts, int flag = 0);
	virtual void rectangle(const cv::Rect& rec, int flag = 0);
	virtual void cross(const cv::Point& center, const cv::Size2f& scale, int flag = 0);
	virtual void numberedBox(const cv::Rect& rec, int number, int flag = 0);
	virtual void ellipse(const cv::RotatedRect& box, int number=0, int flag = 0);
};

typedef struct _DC_unit_info{
	void *context;
	GLenum primitive;
	bool bRect;
	bool bTxt;
	int thickness;
	int lineType;
	wchar_t txt[128];
	std::vector<cv::Point> vdrawPos;
	//cv::Rect  drawRC;
	//cv::RotatedRect drawRRC;
}DCU;
typedef std::vector<DCU*> vDCU;
typedef std::vector<DCU*>::iterator viDCU;
class DCOSD : public GLOSDFactoryBase
{
protected:
	DCTXT *m_dctxt;
	cv::Mat *m_dc;
	DCU *m_dcuMem;
	vDCU m_vdcus[2];
	int m_idcu;
	std::vector<cv::Point> m_vPts;
	std::vector<cv::Point> m_vNumPts;
	std::vector<int> m_vNums;
	void dcuDraw(const DCU& dcu, const cv::Scalar& color);
	void HideDCU(vDCU& vdcus, const void *context);
	void HideDCU(vDCU& vdcus);
	void directDraw(void);
public:
	DCOSD(cv::Mat *dc, int fontSize = 45, const char* faceName = NULL);
	virtual ~DCOSD(void);
	virtual void Draw(void);
	virtual void begin(cv::Scalar& color, int thickness=1);
	virtual void end(void);
	virtual void line(const std::vector<cv::Point>& pts, int flag = 0);
	virtual void polygon(const std::vector<cv::Point>& pts, int flag = 0);
	virtual void rectangle(const cv::Rect& rec, int flag = 0);
	virtual void cross(const cv::Point& center, const cv::Size2f& scale, int flag = 0);
	virtual void numberedBox(const cv::Rect& rec, int number, int flag = 0);
	virtual void ellipse(const cv::RotatedRect& box, int number=0, int flag = 0);
};

};//namespace cr_local_osd

#endif /* GLOSD_HPP_ */
