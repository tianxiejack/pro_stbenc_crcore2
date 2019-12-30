/*
 * crCore.hpp
 *
 *  Created on: Sep 27, 2018
 *      Author: wzk
 */

#ifndef CRCORE_HPP_
#define CRCORE_HPP_

/***********************************************
 * core version 2.0.4
 *
 * 20/12/2018 motify: CORE1001_INIT_PARAM::renderHook
 *	enum{ RUN_ENTER = 0, RUN_WIN, RUN_SWAP, RUN_LEAVE };
 * 11/01/2019 motify: int ICore_1001::setHideSysOsd(bool bHideOSD);
 * 20/02/2019 motify: cv::Rect renderRC;
 * 26/02/2019 motify: CORE1001_INIT_PARAM2;
 * 07/03/2019 motify: int enableMMTD(bool enable, int nTarget, int nSel = 0, const cv::Rect& roi);
 *            motify: CORE1001_CHN_INIT_PARAM::int notFullRender;//0;1;
 *            motify: int setTrackForce(bool enable = false);CORE1001_STATS::trackForceFlag;
 * 19/03/2019 motify: int ICore_1001::setHideSysOsd(unsigned int mask);
 *                    bit0: gdb;
 *                    bit1: trk;
 *                    bit2: mmtd;
 *                    bit3: montion;
 *                    bit4: bkgd;
 *                    bit5: blob;
 *                    bit6: svm;
 *                    bit7: scene;
 * 28/03/2019 motify: CORE1001_STATS::trackPosFilter; CORE1001_STATS::frameTimestampRender[CORE_CHN_MAX]
 * 18/06/2019 motify: v2.0.1
 * 				      int enableStab(bool enable, float cropMargin, bool bScale);
 *                    int enableStab(int chId, bool enable, float cropMargin, bool bScale);
 * 04/07/2019 motify: v2.0.2
 * 				      int enableStab(bool enable, float cropMargin, bool bScale, bool bBorderTransparent);
 *                    int enableStab(int chId, bool enable, float cropMargin, bool bScale, bool bBorderTransparent);
 * 09/07/2019 motify: v2.0.3
 *                    void* QureyObj(int objId)
 *                    crcoreRender.hpp CCoreRenderBase
 * 11/07/2019 motify: v2.0.4
 *
 */
#include "osa.h"
#include "osa_sem.h"

#define CORE_1001_VERSION_  "2.0.4"

#define COREID_1001			(0x10010000)

class ICore
{
public:
	int coreId;
	static ICore* Qury(int coreID=COREID_1001);
	static void Release(ICore* core);
	static void* QureyObj(int objId);
	virtual int init(void *pParam, int paramSize) = 0;
	virtual int uninit() = 0;
	virtual void processFrame(int chId, unsigned char *data, struct v4l2_buffer capInfo, int format) = 0;
};

////////////////////////////////////////////////////////////
//   core stats
#define CORE_CHN_MAX	(4)
#define CORE_TGT_NUM_MAX	(32)
typedef struct{
	int valid;
	int index;
	cv::Rect Box;
	cv::Point2f pos;
}CORE_TGT_INFO;
typedef struct _core_1001_chn_stats{
	cv::Size imgSize;
	int fovId;
	cv::Point2f axis;
	bool enableEnh;
	int iEZoomx;
	bool enableEncoder;
	uint64_t frameTimestamp;
	int blendBindId;
	cv::Matx44f blendMatric;
	bool enableStabilizer;
}CORE1001_CHN_STATS;

typedef struct _core_1001_stats{
	int mainChId;
	int subChId;
	bool enableTrack;
	bool enableMMTD;
	bool enableINTELL;
	bool enableMotionDetect;
	bool enableBlob;
	bool enableOSD;
	cv::Size acqWinSize;
	int iTrackorStat;
	cv::Point2f trackPos;
	cv::Point2f trackPosFilter;
	cv::Size trackWinSize;
	CORE_TGT_INFO tgts[CORE_TGT_NUM_MAX];
	CORE1001_CHN_STATS chn[CORE_CHN_MAX];
	CORE_TGT_INFO blob;
	unsigned int lossCoastFrames;
	unsigned int lossCoastTelapse;//ms
	cv::Rect subRc;
	cv::Matx44f subMatric;
	int colorYUV;
	int transLevel;
	int trackForceFlag;
	uint64_t frameTimestampRender[CORE_CHN_MAX];
	float renderSched;
}CORE1001_STATS;

//////////////////////////////////////////////////////////////
// core init paramers
typedef struct _core_1001_chnInfo_init{
	cv::Size imgSize;
	int format;
	int fps;
	bool bZoomRender;//false: rendering according to screen size; true: rendering according to the scale ratio of image size;
	cv::Point2f zoomRatio;
}CORE1001_CHN_INIT_PARAM;
typedef struct _core_1001_init{
	CORE1001_CHN_INIT_PARAM chnInfo[CORE_CHN_MAX];
	int nChannels;
	OSA_SemHndl *notify;
	void (*renderHook)(int displayId, int stepIdx, int stepSub, int context);
	bool bEncoder;
	bool bRender;
	bool bHideOSD;
	cv::Rect renderRC;
	int renderFPS;
	float renderSched;
	char *encStreamIpaddr;
	int *encoderParamTab[3];
	int *encoderParamTabMulti[CORE_CHN_MAX][3];
}CORE1001_INIT_PARAM;

class CGluVideoWindow;
typedef struct _core_1001_init2{
	CORE1001_CHN_INIT_PARAM chnInfo[CORE_CHN_MAX];
	int nChannels;
	OSA_SemHndl *notify;
	bool bEncoder;
	bool bHideOSD;
	char *encStreamIpaddr;
	int *encoderParamTab[3];
	int *encoderParamTabMulti[CORE_CHN_MAX][3];
	CGluVideoWindow *videoWindow;
}CORE1001_INIT_PARAM2;

/////////////////////////////////////////////////////////////
//
typedef struct _core_stab_param
{
	enum MotionModel
	{
	    MM_TRANSLATION = 0,
	    MM_TRANSLATION_AND_SCALE = 1,
	    MM_ROTATION = 2,
	    MM_RIGID = 3,
	    MM_SIMILARITY = 4,
	    MM_AFFINE = 5,
	    MM_HOMOGRAPHY = 6,
	    MM_STABILIZER = 7,
	    MM_UNKNOWN = 8
	};
	MotionModel mm;
	double noise_cov;
	bool bBorderTransparent;
	float cropMargin;
	bool bCropMarginScale;
	bool bFixedPos;
	bool bPreProcess;
	_core_stab_param(){
		mm = MM_TRANSLATION;
		noise_cov = 1E-4;
		bBorderTransparent = false;
		cropMargin = -1.0f;
		bCropMarginScale = false;
		bFixedPos = false;
		bPreProcess = false;
	}
}CORE_STAB_PARAM;

enum{	 RENDER_HOOK_RUN_ENTER = 0, RENDER_HOOK_RUN_WIN,	RENDER_HOOK_RUN_SWAP,	RENDER_HOOK_RUN_LEAVE };
typedef cv::Rect_<float> Rect2f;

class ICore_1001 : public ICore
{
public:
	virtual int setMainChId(int chId, int fovId, int ndrop, const cv::Size& acqSize) = 0;
	virtual int setSubChId(int chId) = 0;
	virtual int enableTrack(bool enable, cv::Size winSize, bool bFixSize = false) = 0;
	virtual int enableTrack(bool enable, Rect2f winRect, bool bFixSize = false) = 0;
	virtual int enableTrackByMMTD(int index, cv::Size *winSize = NULL, bool bFixSize = false) = 0;
	virtual int enableMMTD(bool enable, int nTarget, int nSel = 0, const cv::Rect& roi=cv::Rect(0,0,0,0)) = 0;
	virtual int enableIntellDetect(bool enable, int nTarget, int nSel = 0, const cv::Rect& roi=cv::Rect(0,0,0,0)) = 0;
	virtual int enableMotionDetect(bool enable) = 0;
	virtual int enableEnh(bool enable) = 0;
	virtual int enableEnh(int chId, bool enable) = 0;
	virtual int enableStab(bool enable, const CORE_STAB_PARAM& params) = 0;
	virtual int enableStab(int chId, bool enable, const CORE_STAB_PARAM& params) = 0;
	virtual int enableBlob(bool enable) = 0;
	virtual int bindBlend(int blendchId, const cv::Matx44f& matric) = 0;
	virtual int bindBlend(int chId, int blendchId, const cv::Matx44f& matric) = 0;
	virtual int enableOSD(bool enable) = 0;
	virtual int enableEncoder(int chId, bool enable) = 0;
	virtual int setAxisPos(cv::Point pos) = 0;
	virtual int saveAxisPos() = 0;
	virtual int setTrackPosRef(cv::Point2f ref) = 0;
	virtual int setTrackCoast(int nFrames) = 0;
	virtual int setTrackForce(bool enable = false) = 0;
	virtual int setEZoomx(int value) = 0;
	virtual int setEZoomx(int chId, int value) = 0;
	virtual int setWinPos(int winId, const cv::Rect& rc) = 0;//! (0,0):(left,bottom)
	virtual int setWinMatric(int winId, const cv::Matx44f& matric) = 0;
	virtual int setOSDColor(int yuv, int thickness = 2) = 0;
	virtual int setOSDColor(cv::Scalar color, int thickness = 2) = 0;
	virtual int setEncTransLevel(int iLevel) = 0;
	virtual int setHideSysOsd(bool bHideOSD) = 0;
	virtual int setHideSysOsd(unsigned int mask = 0) = 0;//bit0: gdb; bit1: trk; bit2: mmtd; bit3: montion; bit4: bkgd; bit5: blob; bit6: svm; bit7: scene; bit8: intell;

	CORE1001_STATS m_stats;
	cv::Mat m_dc[CORE_CHN_MAX];
};

#endif /* CRCORN_HPP_ */
