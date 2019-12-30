/*
 * processBase.hpp
 *
 *  Created on: Sep 19, 2018
 *      Author: wzk
 */

#ifndef PROCESSBASE_HPP_
#define PROCESSBASE_HPP_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include "directOsd.hpp"
#include "osa.h"
#include "osa_thr.h"
#include "osa_buf.h"
#include "osa_sem.h"
#include "osa_msgq.h"
#include "osa_mutex.h"
#include "osa_image_queue.h"

using namespace std;
using namespace cv;

#define MAX_CHAN 4
#define MAX_NFOV_PER_CHAN	16
#define MAX_NFOV	(MAX_CHAN*MAX_NFOV_PER_CHAN)

#define MAX_TARGET_COUNT		(32)
#define VP_IDENTIFY_BASE		0
#define VP_IDENTIFY_TRK			1
#define VP_IDENTIFY_MMTD		2
#define VP_IDENTIFY_MONTION		3
#define VP_IDENTIFY_BKGD		4
#define VP_IDENTIFY_BLOB		5
#define VP_IDENTIFY_SVM			6
#define VP_IDENTIFY_SCENE		7
#define VP_IDENTIFY_INTELL      8

#define VP_CFG_BASE			(0x0000)
#define VP_CFG_TRK_BASE		(0x1000 + VP_CFG_BASE)
#define VP_CFG_MMTD_BASE	(0x2000 + VP_CFG_BASE)
#define VP_CFG_MONTION_BASE	(0x3000 + VP_CFG_BASE)
#define VP_CFG_BKGD_BASE	(0x4000 + VP_CFG_BASE)
#define VP_CFG_BLOB_BASE	(0x5000 + VP_CFG_BASE)
#define VP_CFG_SVM_BASE     (0x6000 + VP_CFG_BASE)
#define VP_CFG_SCENE_BASE   (0x7000 + VP_CFG_BASE)
#define VP_CFG_INTELL_BASE  (0x8000 + VP_CFG_BASE)


typedef struct _OSD_unit_info{
	bool bNeedDraw;
	int orgValue;
	cv::Point orgPos;
	cv::Rect  orgRC;
	cv::RotatedRect orgRRC;
	cv::Rect  drawRC;
	cv::RotatedRect drawRRC;
}OSDU_Info;

typedef struct{
	int valid;
	cv::Rect Box;
}PROC_TARGETINFO;

class IProcess
{
public:
	virtual int process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp) = 0;
	virtual int dynamic_config(int type, int iPrm, void* pPrm = NULL, int prmSize = 0) = 0;
	virtual int OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp) = 0;
};

#define MAX_SHARE_TGT_NUM 64

class CProcessBase : public IProcess
{
	IProcess *m_proc;
public:
	OSA_MutexHndl m_mutexlock;
	unsigned int m_identify;
	bool m_bHide;
	bool m_bDebug;
	uint64_t m_frameTimestamp[MAX_CHAN];
	static PROC_TARGETINFO m_shTargets[MAX_SHARE_TGT_NUM];
	static PROC_TARGETINFO m_shTrkTarget;
	CProcessBase(IProcess *proc, unsigned int identify = 0):m_proc(proc),m_identify(identify),m_bHide(false),m_bDebug(false){
		OSA_mutexCreate(&m_mutexlock);
		for(int i=0; i<MAX_CHAN; i++){
			m_frameTimestamp[i] = 0l;
		}
	};
	virtual ~CProcessBase(){
		OSA_mutexUnlock(&m_mutexlock);
		OSA_mutexLock(&m_mutexlock);
		OSA_mutexDelete(&m_mutexlock);
	};
	virtual int process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp){
		//cout<<"CProcessBase process"<<endl;
		//m_frameTimestamp[chId] = timestamp;
		if(m_proc != NULL)
			return m_proc->process(chId, fovId, ezoomx, frameOrg, frameGray, timestamp);
		return 0;
	};
	enum{
		VP_CFG_MainChId = VP_CFG_BASE,
		VP_CFG_MainFov,
		VP_CFG_GetTargetInfo,
	};
	virtual int dynamic_config(int type, int iPrm, void* pPrm = NULL, int prmSize = 0)
	{
		//cout<<"CProcessBase dynamic_config"<<endl;
		if(m_proc != NULL)
			return m_proc->dynamic_config(type, iPrm, pPrm, prmSize);
		return 0;
	};
	virtual int OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp)
	{
		if(m_proc != NULL)
			return m_proc->OnOSD(chId, fovId, ezoomx, dc, osd, timestamp);
	}
};

__inline__ cv::Point tPosScale(cv::Point pos, cv::Size imgSize, float fscale)
{
	float x = imgSize.width/2.0f - (imgSize.width/2.0f - pos.x)*fscale;
	float y = imgSize.height/2.0f - (imgSize.height/2.0f - pos.y)*fscale;
	return cv::Point(int(x+0.5), int(y+0.5));
}

__inline__ cv::Point2f tPosScale(cv::Point2f pos, cv::Size imgSize, float fscale)
{
	float x = imgSize.width/2.0f - (imgSize.width/2.0f - pos.x)*fscale;
	float y = imgSize.height/2.0f - (imgSize.height/2.0f - pos.y)*fscale;
	return cv::Point2f(x, y);
}

__inline__ cv::Point2f tPosScale(cv::Point2f pos, cv::Size orgSize, cv::Size scaleSize)
{
	cv::Point2f fscale((float)scaleSize.width/orgSize.width, (float)scaleSize.height/orgSize.height);
	float x = scaleSize.width/2.0f - (orgSize.width/2.0f - pos.x)*fscale.x;
	float y = scaleSize.height/2.0f - (orgSize.height/2.0f - pos.y)*fscale.y;
	return cv::Point2f(x, y);
}

__inline__ cv::Point2f tRectCenter(Rect rc)
{
	cv::Point2f point(rc.x+rc.width/2.0, rc.y+rc.height/2.0);
	return point;
}

__inline__ Rect tRectScale(Rect rc, cv::Size imgSize, float fscale)
{
	cv::Point2f point(rc.x, rc.y);
	cv::Size size((int)(rc.width*fscale+0.5), (int)(rc.height*fscale+0.5));
	point = tPosScale(point, imgSize, fscale);
	return Rect((int)point.x+0.5, (int)point.y+0.5, size.width, size.height);
}

__inline__ Rect tRectScale(Rect rc, cv::Size orgSize, cv::Size toSize)
{
	Rect ret;
	cv::Point2f fscale((float)toSize.width/orgSize.width, (float)toSize.height/orgSize.height);
	ret.x = (int)(toSize.width/2.0f - (orgSize.width/2.0f - rc.x)*fscale.x + 0.5f);
	ret.y = (int)(toSize.height/2.0f - (orgSize.height/2.0f - rc.y)*fscale.y+ 0.5f);
	ret.width = (int)(rc.width*fscale.x+0.5f);
	ret.height = (int)(rc.height*fscale.y+0.5f);
	return ret;
}

__inline__ RotatedRect tRectScale(cv::RotatedRect rc, cv::Size orgSize, cv::Size toSize)
{
	cv::RotatedRect ret;
	cv::Point2f fscale((float)toSize.width/orgSize.width, (float)toSize.height/orgSize.height);
	ret = rc;
	ret.center = cv::Point2f((toSize.width/2.0f - (orgSize.width/2.0f - rc.center.x)*fscale.x),
			(toSize.height/2.0f - (orgSize.height/2.0f - rc.center.y)*fscale.y));
	ret.size.width = rc.size.width*fscale.x;
	ret.size.height = rc.size.height*fscale.y;
	return ret;
}

typedef cv::Rect_<float> Rect2f;
__inline__ Rect2f tRectScale(Rect2f rc, cv::Size imgSize, float fscale)
{
	cv::Point2f point(rc.x, rc.y);
	cv::Size2f size(rc.width*fscale, rc.height*fscale);
	return Rect2f(tPosScale(point, imgSize, fscale), size);
}

__inline__ Rect2f tRectScale(Rect2f rc, cv::Size orgSize, cv::Size toSize)
{
	Rect2f ret;
	cv::Point2f fscale((float)toSize.width/orgSize.width, (float)toSize.height/orgSize.height);
	ret.x = toSize.width/2.0f - (orgSize.width/2.0f - rc.x)*fscale.x;
	ret.y = toSize.height/2.0f - (orgSize.height/2.0f - rc.y)*fscale.y;
	ret.width = rc.width*fscale.x;
	ret.height = rc.height*fscale.y;
	return ret;
}

__inline__ bool isRectOverlap(const cv::Rect& rc1, const cv::Rect& rc2)
{
	return !( rc1.x+rc1.width < rc2.x || rc1.y+rc1.height < rc2.y || rc2.x+rc2.width < rc1.x || rc2.y+rc2.height < rc1.y);
}


#endif /* PROCESSBASE_HPP_ */
