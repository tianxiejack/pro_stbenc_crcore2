/*
 * crCore.cpp
 *
 *  Created on: Sep 27, 2018
 *      Author: wzk
 */

#include "osa_image_queue.h"
#include "intelligentDetectProcess.hpp"
#include "SceneProcess.hpp"
#include "blobDetectProcess.hpp"
#include "backgroundProcess.hpp"
#include "motionDetectProcess.hpp"
#include "mmtdProcess.hpp"
#include "GeneralProcess.hpp"
//#include "Displayer.hpp"
#include "glvideo.hpp"
#include "gluVideoWindow.hpp"

#include "encTrans.hpp"
#include "cuda_convert.cuh"
#include "crvxMotionComp_lib.hpp"
#include "thread.h"
#include "crCore.hpp"
#include "glosd.hpp"

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <memory>

//#include <NVX/nvx.h>
//#include <NVX/nvx_timer.hpp>

namespace cr_osd
{
extern void put(wchar_t* s, const cv::Point& pos, const cv::Scalar& color, size_t n, const wchar_t* format, ...);
//extern void put(const wchar_t* s, const cv::Point& pos, const cv::Scalar& color);
//extern void clear(void);
extern GLShaderManagerMini		glShaderManager;
extern std::vector<cr_osd::GLOSDFactory *> vosdFactorys;
};
//using namespace cr_osd;
#define isEqual(a,b) (fabs((a)-(b))<=1.0e-6)

namespace cr_local
{
static cr_osd::GLOSDFactory *vOSDs[CORE_CHN_MAX];
static cr_osd::GLOSD *glosdFront = NULL;
static CEncTrans *enctran = NULL;
static CGluVideoWindow *gluWindow = NULL;
static IProcess *proc = NULL;
static CINTELLProcess *intell = NULL;
static CSceneProcess *scene = NULL;
static CBlobDetectProcess *blob = NULL;
static CBkgdDetectProcess *bkgd = NULL;
static CMotionDetectProcess *motion = NULL;
static CMMTDProcess *mmtd = NULL;
static CGeneralProc *general = NULL;
static std::unique_ptr<MotionComp> motionCompensator[CORE_CHN_MAX];
static int curChannelFlag = 0;
static int curSubChannelIdFlag = -1;
static int curFovIdFlag[CORE_CHN_MAX];
static bool curFixSizeFlag = false;
static bool enableTrackFlag = false;
static bool enableMMTDFlag = false;
static bool enableIntellFlag = false;
static bool enableMotionDetectFlag = false;
static bool enableEnhFlag[CORE_CHN_MAX];
static bool enableStabSideBySideFlag[CORE_CHN_MAX];
static bool enableStabFlag[CORE_CHN_MAX];
static CORE_STAB_PARAM stabParams[CORE_CHN_MAX];
static unsigned int nSabCount[CORE_CHN_MAX];
static bool enableBlobFlag = false;
static int bindBlendFlag[CORE_CHN_MAX];
static int bindBlendTFlag[CORE_CHN_MAX];
static cv::Matx44f blendMatric[CORE_CHN_MAX];
static bool enableEncoderFlag[CORE_CHN_MAX];
static bool enableOSDFlag = true;
static int ezoomxFlag[CORE_CHN_MAX];
static float scaleFlag[CORE_CHN_MAX];
static int colorYUVFlag = WHITECOLOR;
static cv::Scalar colorRGBAFlag = cv::Scalar::all(255);
static int curThicknessFlag = 2;
static int nValidChannels = CORE_CHN_MAX;
static cv::Size channelsImgSize[CORE_CHN_MAX];
static int channelsFormat[CORE_CHN_MAX];
static int channelsFPS[CORE_CHN_MAX];
static int channelsRenderMode[CORE_CHN_MAX];
static cv::Point2f channelsRenderRatio[CORE_CHN_MAX];
static char fileNameFont[CORE_CHN_MAX][256];
static char fileNameFontRender[256];
static int fontSizeVideo[CORE_CHN_MAX];
static int fontSizeRender = 45;
static int renderFPS = 30;
static int curTransLevel = 1;
static void (*renderHook)(int displayId, int stepIdx, int stepSub, int context) = NULL;
static cv::Rect subRc;
static cv::Matx44f subMatric;
static cv::Rect mainWinRenderRC[CORE_CHN_MAX];
static bool trackForceFlag = false;

static int defaultEncParamTab0[CORE_CHN_MAX*3][8] = {
	//bitrate; minQP; maxQP;minQI;maxQI;minQB;maxQB;
	{1400000,  -1,    -1,   -1,   -1,   -1,   -1, },//2M
	{2800000,  -1,    -1,   -1,   -1,   -1,   -1, },//4M
	{5600000,  -1,    -1,   -1,   -1,   -1,   -1, }, //8M
	//bitrate; minQP; maxQP;minQI;maxQI;minQB;maxQB;
	{1400000,  -1,    -1,   -1,   -1,   -1,   -1, },//2M
	{2800000,  -1,    -1,   -1,   -1,   -1,   -1, },//4M
	{5600000,  -1,    -1,   -1,   -1,   -1,   -1, }, //8M
	//bitrate; minQP; maxQP;minQI;maxQI;minQB;maxQB;
	{1400000,  -1,    -1,   -1,   -1,   -1,   -1, },//2M
	{2800000,  -1,    -1,   -1,   -1,   -1,   -1, },//4M
	{5600000,  -1,    -1,   -1,   -1,   -1,   -1, }, //8M
	//bitrate; minQP; maxQP;minQI;maxQI;minQB;maxQB;
	{1400000,  -1,    -1,   -1,   -1,   -1,   -1, },//2M
	{2800000,  -1,    -1,   -1,   -1,   -1,   -1, },//4M
	{5600000,  -1,    -1,   -1,   -1,   -1,   -1, } //8M
};
static int defaultEncParamTab1[CORE_CHN_MAX*3][8] = {
	//bitrate; minQP; maxQP;minQI;maxQI;minQB;maxQB;
	{700000,  -1,    -1,   -1,   -1,   -1,   -1, },//2M
	{1400000,  -1,    -1,   -1,   -1,   -1,   -1, },//4M
	{2800000,  -1,    -1,   -1,   -1,   -1,   -1, }, //8M
	//bitrate; minQP; maxQP;minQI;maxQI;minQB;maxQB;
	{700000,  -1,    -1,   -1,   -1,   -1,   -1, },//2M
	{1400000,  -1,    -1,   -1,   -1,   -1,   -1, },//4M
	{2800000,  -1,    -1,   -1,   -1,   -1,   -1, }, //8M
	//bitrate; minQP; maxQP;minQI;maxQI;minQB;maxQB;
	{700000,  -1,    -1,   -1,   -1,   -1,   -1, },//2M
	{1400000,  -1,    -1,   -1,   -1,   -1,   -1, },//4M
	{2800000,  -1,    -1,   -1,   -1,   -1,   -1, }, //8M
	//bitrate; minQP; maxQP;minQI;maxQI;minQB;maxQB;
	{700000,  -1,    -1,   -1,   -1,   -1,   -1, },//2M
	{1400000,  -1,    -1,   -1,   -1,   -1,   -1, },//4M
	{2800000,  -1,    -1,   -1,   -1,   -1,   -1, } //8M
};

//static int *userEncParamTab[3] = {NULL, NULL, NULL};
static int *userEncParamTab0[CORE_CHN_MAX][3];
static int *userEncParamTab1[CORE_CHN_MAX][3];
static unsigned char *gDeviceMem = NULL;

static __inline__ int* getEncParamTab(int chId, int level)
{
	int nEnc = 0;
	for(int i = 0; i < nValidChannels; i++)
		nEnc += enableEncoderFlag[i];
	if(nEnc>1)
		return userEncParamTab1[chId][level];
	return userEncParamTab0[chId][level];
}

static int ReadCfgFile_OSD(int nChannels)
{
	int iret = -1;
	string cfgFile;
	cfgFile = "ConfigOSDFile.yml";
	FILE *fp = fopen(cfgFile.c_str(), "rt");
	if(fp != NULL)
	{
		fseek(fp, 0, SEEK_END);
		int len = ftell(fp);
		fclose(fp);

		if(len > 10)
		{
			FileStorage fr(cfgFile, FileStorage::READ);
			if(fr.isOpened())
			{
				int ivalue;
				string szfile;
				for(int chId=0; chId<nChannels; chId++){
					string strkey;
					char keyStr[256];
					sprintf(keyStr, "ch%02d_font_filename", chId);
					strkey = keyStr;
					szfile = (string)fr[strkey];
					if(szfile.length()>0){
						strcpy(fileNameFont[chId], szfile.data());
					}
					sprintf(keyStr, "ch%02d_font_size", chId);
					strkey = keyStr;
					ivalue = (int)fr[strkey];
					if(ivalue>10 && ivalue<200){
						fontSizeVideo[chId] = ivalue;
					}
				}

				szfile = (string)fr["Front_font_filename"];
				if(szfile.length()>0){
					strcpy(fileNameFontRender, szfile.data());
				}
				ivalue = (int)fr["Front_font_size"];
				if(ivalue>10 && ivalue<200){
					fontSizeRender = ivalue;
				}

				iret = 0;
			}
		}
	}

	for(int chId=0; chId<nChannels; chId++)
		OSA_printf("OSD: video%02d fontSize = %d [%s]",chId, fontSizeVideo[chId], fileNameFont[chId]);
	OSA_printf("OSD: Front fontSize = %d [%s]\n\n",fontSizeRender, fileNameFontRender);
	return iret;
}

static void localInit(int nChannels, bool bEncoder)
{
	curChannelFlag = 0;
	memset(curFovIdFlag, 0, sizeof(curFovIdFlag));
	enableTrackFlag = false;
	enableMMTDFlag = false;
	renderHook = NULL;
	memset(nSabCount, 0, sizeof(nSabCount));
	memset(enableStabSideBySideFlag, 0, sizeof(enableStabSideBySideFlag));
	memset(enableEnhFlag, 0, sizeof(enableEnhFlag));
	memset(enableStabFlag, 0, sizeof(enableStabFlag));
	memset(bindBlendFlag, 0, sizeof(bindBlendFlag));
	memset(userEncParamTab0, 0, sizeof(userEncParamTab0));
	memset(userEncParamTab1, 0, sizeof(userEncParamTab1));
	for(int i=0; i<CORE_CHN_MAX; i++){
		motionCompensator[i] = nullptr;
		vOSDs[i] = NULL;
		enableEncoderFlag[i] = true;
		ezoomxFlag[i] = 1;
		scaleFlag[i] = 1.0f;
		bindBlendTFlag[i] = -1;
		blendMatric[i] = cv::Matx44f::eye();
		channelsFPS[i] = 30;
		userEncParamTab0[i][0] = defaultEncParamTab0[i*3+0];
		userEncParamTab0[i][1] = defaultEncParamTab0[i*3+1];
		userEncParamTab0[i][2] = defaultEncParamTab0[i*3+2];
		userEncParamTab1[i][0] = defaultEncParamTab1[i*3+0];
		userEncParamTab1[i][1] = defaultEncParamTab1[i*3+1];
		userEncParamTab1[i][2] = defaultEncParamTab1[i*3+2];
		//sprintf(fileNameFont[i], "/usr/share/fonts/truetype/abyssinica/AbyssinicaSIL-R.ttf");
		//sprintf(fileNameFont[i], "/usr/share/fonts/truetype/freefont/FreeMono.ttf");
		//sprintf(fileNameFont[i], "/usr/share/fonts/truetype/freefont/FreeMonoBold.ttf");
		//sprintf(fileNameFont[i], "/usr/share/fonts/truetype/freefont/FreeSans.ttf");
		//sprintf(fileNameFont[i], "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf");
		//sprintf(fileNameFont[i], "/usr/share/fonts/truetype/freefont/FreeSansBoldOblique.ttf");
		//sprintf(fileNameFont[i], "/usr/share/fonts/truetype/freefont/FreeSansOblique.ttf");
		//sprintf(fileNameFont[i], "/usr/share/fonts/truetype/freefont/FreeSerif.ttf");
		//sprintf(fileNameFont[i],"/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-L.ttf");
		//sprintf(fileNameFont[i],"/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-M.ttf");
		//sprintf(fileNameFont[i],"/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-R.ttf");
		//sprintf(fileNameFont[i],"/usr/share/fonts/truetype/fonts-japanese-gothic.ttf");
		//sprintf(fileNameFont[i],"/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf");
		sprintf(fileNameFont[i],"/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf");
		//sprintf(fileNameFont[i],"/usr/share/fonts/truetype/liberation/LiberationSansNarrow-Regular.ttf");
		//sprintf(fileNameFont[i],"/usr/share/fonts/truetype/liberation/LiberationSerif-Regular.ttf");
		fontSizeVideo[i] = 45;
		channelsRenderRatio[i].x = 1.0f;
		channelsRenderRatio[i].y = 1.0f;
	}
	glosdFront = NULL;
	enableOSDFlag = true;
	colorYUVFlag = WHITECOLOR;
	colorRGBAFlag = cv::Scalar::all(255);
	curThicknessFlag = 2;
	if(bEncoder)
		curThicknessFlag = 1;
	//sprintf(fileNameFontRender, "/usr/share/fonts/truetype/abyssinica/simsun.ttc");
	sprintf(fileNameFontRender, "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf");
	fontSizeRender = 45;
	renderFPS = 30;
	curTransLevel = 1;
	nValidChannels = nChannels;
	memset(channelsRenderMode, 0, sizeof(channelsRenderMode));
	ReadCfgFile_OSD(nChannels);
	OSA_assert(nChannels>0 && nChannels<=CORE_CHN_MAX);
}

/************************************************************
 *
 *
 */
static int setEncTransLevel(int iLevel)
{
	ENCTRAN_encPrm encPrm;
	int iret = OSA_SOK;

	if(enctran == NULL)
		return iret;

	enctran->dynamic_config(CEncTrans::CFG_TransLevel, iLevel, NULL);
	for(int chId=0; chId<nValidChannels; chId++){
		int* params = getEncParamTab(chId, iLevel);
		encPrm.bitrate = params[0];
		encPrm.minQP = params[1];
		encPrm.maxQP = params[2];
		encPrm.minQI = params[3];
		encPrm.maxQI = params[4];
		encPrm.minQB = params[5];
		encPrm.maxQB = params[6];

		enctran->dynamic_config(CEncTrans::CFG_EncPrm, chId, &encPrm);
	}
	curTransLevel = iLevel;

	return iret;
}

static int setMainChId(int chId, int fovId, int ndrop, UTC_SIZE acqSize)
{
	VPCFG_MainChPrm mcPrm;
	int iret = OSA_SOK;
	if(chId<0||chId>=nValidChannels)
		return OSA_EFAIL;
	if(fovId<0||fovId>=MAX_NFOV_PER_CHAN)
		return OSA_EFAIL;
	mcPrm.fovId = fovId;
	mcPrm.iIntervalFrames = ndrop;
	proc->dynamic_config(CTrackerProc::VP_CFG_AcqWinSize, curFixSizeFlag, &acqSize, sizeof(acqSize));
	proc->dynamic_config(CProcessBase::VP_CFG_MainChId, chId, &mcPrm, sizeof(mcPrm));
	if(gluWindow!= NULL){
		gluWindow->dynamic_config(CGluVideoWindow::VWIN_CFG_ChId, 0, &chId);
		if(channelsRenderMode[chId] == 1 || channelsRenderMode[curChannelFlag] == 1)
		gluWindow->dynamic_config(CGluVideoWindow::VWIN_CFG_ViewPos, 0, &mainWinRenderRC[chId]);
	}
	curChannelFlag = chId;
	curFovIdFlag[chId] = fovId;
	return iret;
}

static int setSubChId(int chId)
{
	curSubChannelIdFlag = chId;
	if(gluWindow!= NULL){
		gluWindow->dynamic_config(CGluVideoWindow::VWIN_CFG_ChId, 1, &chId);
	}
	return OSA_SOK;
}

static int enableTrack(bool enable, UTC_SIZE winSize, bool bFixSize)
{
	int iret = OSA_SOK;
	proc->dynamic_config(CTrackerProc::VP_CFG_AcqWinSize, bFixSize, &winSize, sizeof(winSize));
	proc->dynamic_config(CTrackerProc::VP_CFG_TrkEnable, enable);
	enableTrackFlag = enable;
	curFixSizeFlag = bFixSize;
	return iret;
}

static int enableTrack(bool enable, UTC_RECT_float winRect, bool bFixSize)
{
	int iret = OSA_SOK;
	proc->dynamic_config(CTrackerProc::VP_CFG_TrkEnable, enable, &winRect, sizeof(winRect));
	proc->dynamic_config(CTrackerProc::VP_CFG_AcqWinSize, bFixSize);
	enableTrackFlag = enable;
	curFixSizeFlag = bFixSize;
	return iret;
}

//static int enableMMTD(bool enable, int nTarget, int nSel)
static int enableMMTD(bool enable, int nTarget, int nSel, const cv::Rect& roi)
{
	int iret = OSA_SOK;
	if(nSel <= 0)
		nSel = nTarget;
	if(enable)
		proc->dynamic_config(CMMTDProcess::VP_CFG_MMTDTargetCount, nTarget, &nSel, sizeof(nSel));
	cv::Rect tmprc = roi;
	proc->dynamic_config(CMMTDProcess::VP_CFG_MMTDEnable, enable, &tmprc, sizeof(tmprc));
	enableMMTDFlag = enable;
	return iret;
}

static int enableIntellDetect(bool enable, int nTarget, int nSel, const cv::Rect& roi)
{
	int iret = OSA_SOK;
	if(nSel <= 0)
		nSel = nTarget;
	if(enable)
		proc->dynamic_config(CINTELLProcess::VP_CFG_INTELLTargetCount, nTarget, &nSel, sizeof(nSel));
	cv::Rect tmprc = roi;
	proc->dynamic_config(CINTELLProcess::VP_CFG_INTELLEnable, enable, &tmprc, sizeof(tmprc));
	enableIntellFlag = enable;
	return iret;
}

static int enableMotionDetect(bool enable)
{
	proc->dynamic_config(CMotionDetectProcess::VP_CFG_MONTIONEnable, enable);
	enableMotionDetectFlag = enable;
	return OSA_SOK;
}

static int enableTrackByMMTD(int index, cv::Size *winSize, bool bFixSize)
{
	PROC_TARGETINFO tgt;
	tgt.valid = 0;
	//OSA_printf("%s: CProcessBase::VP_CFG_GetTargetInfo index = %d", __func__, index);
	int ret = mmtd->dynamic_config(CProcessBase::VP_CFG_GetTargetInfo, index, &tgt, sizeof(tgt));
	if(ret != OSA_SOK || !tgt.valid){
		//OSA_printf("%s: CProcessBase::VP_CFG_GetTargetInfo index = %d ret %d valid %d", __func__, index, ret, tgt.valid);
		return OSA_EFAIL;
	}
	UTC_RECT_float acqrc;
	acqrc.x = tgt.Box.x;
	acqrc.y = tgt.Box.y;
	acqrc.width = tgt.Box.width;
	acqrc.height = tgt.Box.height;

	if(winSize != NULL){
		acqrc.x += (acqrc.width*0.5);
		acqrc.y += (acqrc.height*0.5);
		acqrc.width = winSize->width;
		acqrc.height = winSize->height;
		acqrc.x -= (acqrc.width*0.5);
		acqrc.y -= (acqrc.height*0.5);
	}

	proc->dynamic_config(CTrackerProc::VP_CFG_AcqWinSize, bFixSize);
	curFixSizeFlag = bFixSize;
	enableTrackFlag = true;
	return proc->dynamic_config(CTrackerProc::VP_CFG_TrkEnable, true, &acqrc, sizeof(acqrc));
}

static int enableEnh(bool enable)
{
	enableEnhFlag[curChannelFlag] = enable;
	return OSA_SOK;
}

static int enableEnh(int chId, bool enable)
{
	if(chId<0 || chId >=CORE_CHN_MAX)
		return OSA_EFAIL;
	enableEnhFlag[chId] = enable;
	return OSA_SOK;
}


static int enableSideBySide(bool enable)
{
	enableStabSideBySideFlag[curChannelFlag] = enable;
	nSabCount[curChannelFlag] = 0;
	return OSA_SOK;
}


static int enableSideBySide(int chId, bool enable)
{
	if(chId<0 || chId >=CORE_CHN_MAX)
		return OSA_EFAIL;
	enableStabSideBySideFlag[chId] = enable;
	nSabCount[chId] = 0;
	return OSA_SOK;
}


static int enableStab(bool enable, const CORE_STAB_PARAM& params)
{
	stabParams[curChannelFlag] = params;
	nSabCount[curChannelFlag] = 0;
	enableStabFlag[curChannelFlag] = enable;
	return OSA_SOK;
}

static int enableStab(int chId, bool enable, const CORE_STAB_PARAM& params)
{
	if(chId<0 || chId >=CORE_CHN_MAX)
		return OSA_EFAIL;
	stabParams[chId] = params;
	nSabCount[chId] = 0;
	enableStabFlag[chId] = enable;
	return OSA_SOK;
}

static int enableBlob(bool enable)
{
	proc->dynamic_config(CBlobDetectProcess::VP_CFG_BLOBTargetCount, 1);
	proc->dynamic_config(CBlobDetectProcess::VP_CFG_BLOBEnable, enable);
	//proc->dynamic_config(CSVMDetectProcess::VP_CFG_SVMTargetCount, 8);
	//proc->dynamic_config(CSVMDetectProcess::VP_CFG_SVMEnable, enable);
	enableBlobFlag = enable;
	return OSA_SOK;
}

static int bindBlend(int chId, int blendchId, cv::Matx44f matric)
{
	int ret = OSA_SOK;
	if(chId<0 || chId >=CORE_CHN_MAX)
		return OSA_EFAIL;
	if(blendchId>=0 && blendchId < CORE_CHN_MAX){
		bindBlendFlag[blendchId] |= 1<<chId;
		bindBlendTFlag[chId] = blendchId;
	}else{
		int curBlendchId = bindBlendTFlag[chId];
		if(curBlendchId>=0)
			bindBlendFlag[curBlendchId] &= ~(1<<chId);
		bindBlendTFlag[chId] = -1;
	}
	blendMatric[chId] = matric;
	if(gluWindow!= NULL){
		if(blendchId>=0 && blendchId < CORE_CHN_MAX){
			ret = gluWindow->dynamic_config(CGluVideoWindow::VWIN_CFG_BlendTransMat, chId*VWIN_CHAN_MAX+blendchId, matric.val);
			ret = proc->dynamic_config(CBkgdDetectProcess::VP_CFG_BkgdDetectEnable, true, &blendchId, sizeof(blendchId));
		}else{
			ret = proc->dynamic_config(CBkgdDetectProcess::VP_CFG_BkgdDetectEnable, false);
		}
		ret = gluWindow->dynamic_config(CGluVideoWindow::VWIN_CFG_BlendChId, chId, &blendchId);
	}
	return ret;
}

static int bindBlend(int blendchId, cv::Matx44f matric)
{
	return bindBlend(curChannelFlag, blendchId, matric);
}

static int setWinPos(int winId, cv::Rect rc)
{
	int ret = OSA_SOK;
	if(gluWindow!= NULL){
		ret = gluWindow->dynamic_config(CGluVideoWindow::VWIN_CFG_ViewPos, winId, &rc);
		if(winId == 1){
			subRc = gluWindow->m_vvideoRenders[1]->m_viewPort;//gluWindow->m_renders[1].displayrect;
			subMatric = gluWindow->m_vvideoRenders[1]->m_matrix;//gluWindow->m_renders[1].transform;
		}
		if(winId == 0){
			for(int chId=0; chId<nValidChannels; chId++){
				vOSDs[chId]->m_curViewport = rc;
			}
		}
	}
	return ret;
}

static int setWinMatric(int winId, cv::Matx44f matric)
{
	int ret = OSA_SOK;
	if(gluWindow!= NULL){
		ret = gluWindow->dynamic_config(CGluVideoWindow::VWIN_CFG_ViewTransMat, winId, matric.val);
		if(winId == 1){
			subRc = gluWindow->m_vvideoRenders[1]->m_viewPort;//gluWindow->m_renders[1].displayrect;
			subMatric = gluWindow->m_vvideoRenders[1]->m_matrix;//gluWindow->m_renders[1].transform;
		}
	}
	return ret;
}

static int enableOSD(bool enable)
{
	enableOSDFlag = enable;
	return OSA_SOK;
}

static int enableEncoder(int chId, bool enable)
{
	int ret = OSA_SOK;
	if(enctran == NULL)
		return OSA_SOK;
	enableEncoderFlag[chId] = enable;
	ret = enctran->dynamic_config(CEncTrans::CFG_Enable, chId, &enable);

	for(int chId=0; chId<nValidChannels; chId++){
		ENCTRAN_encPrm encPrm;
		int* params = getEncParamTab(chId, curTransLevel);
		encPrm.bitrate = params[0];
		encPrm.minQP = params[1];
		encPrm.maxQP = params[2];
		encPrm.minQI = params[3];
		encPrm.maxQI = params[4];
		encPrm.minQB = params[5];
		encPrm.maxQB = params[6];

		ret |= enctran->dynamic_config(CEncTrans::CFG_EncPrm, chId, &encPrm);
	}
	return ret;
}

static int setAxisPos(cv::Point pos)
{
	cv::Point2f setPos(pos.x, pos.y);
	int ret = proc->dynamic_config(CTrackerProc::VP_CFG_Axis, 0, &setPos, sizeof(setPos));
	OSA_assert(ret == OSA_SOK);
	ret = proc->dynamic_config(CTrackerProc::VP_CFG_SaveAxisToArray, 0);
	return ret;
}

static int saveAxisPos()
{
	return proc->dynamic_config(CTrackerProc::VP_CFG_SaveAxisToFile, 0);
}

static int setEZoomx(int value)
{
	if(enctran != NULL){
		if(value != 1 && value != 2 && value != 4)
			return OSA_EFAIL;
	}
	ezoomxFlag[curChannelFlag] = value;
	return OSA_SOK;
}

static int setEZoomx(int chId, int value)
{
	if(enctran != NULL){
		if(value != 1 && value != 2 && value != 4)
			return OSA_EFAIL;
	}
	if(chId<0 || chId >=CORE_CHN_MAX)
		return OSA_EFAIL;
	ezoomxFlag[chId] = value;
	return OSA_SOK;
}

static int setTrackCoast(int nFrames)
{
	return proc->dynamic_config(CTrackerProc::VP_CFG_TrkCoast, nFrames);
}

static int setTrackForce(bool enable)
{
	trackForceFlag = enable;
	return proc->dynamic_config(CTrackerProc::VP_CFG_TrkForce, enable);
}

static int setTrackPosRef(cv::Point2f ref)
{
	return proc->dynamic_config(CTrackerProc::VP_CFG_TrkPosRef, 0, &ref, sizeof(ref));
}

static int setOSDColor(int value, int thickness)
{
	int Y,U,V,R,G,B;
	Y = value & 0xff; U = (value>>8) & 0xff; V = (value>>16) & 0xff;
	R = Y+((360*(V-128))>>8);
	G = Y-((88*(U-128)+184*(V-128))>>8);
	B = Y+((455*(U-128))>>8);

	switch(value)
	{
	case WHITECOLOR:
		R = 255; G = 255; B = 255;
		break;
	case YELLOWCOLOR:
		R = 255; G = 255; B = 0;
		break;
	case CRAYCOLOR:
		break;
	case GREENCOLOR:
		R = 0; G = 255; B = 0;
		break;
	case MAGENTACOLOR:
		break;
	case REDCOLOR:
		R = 255; G = 0; B = 0;
		break;
	case BLUECOLOR:
		R = 0; G = 0; B = 255;
		break;
	case BLACKCOLOR:
		R = 0; G = 0; B = 0;
		break;
	case BLANKCOLOR:
		break;
	}

	colorYUVFlag = value;
	colorRGBAFlag = cv::Scalar(R,G,B,255);
	curThicknessFlag = thickness;
	return OSA_SOK;
}

static int setOSDColor(cv::Scalar rgba, int thickness)
{
	int Y,U,V,R,G,B;
	R = rgba.val[0];G = rgba.val[1];B = rgba.val[2];
	Y = (77*R+150*G+29*B)>>8;
	U = ((-44*R-87*G+131*B)>>8)+128;
	V = ((131*R-110*G-21*B)>>8)+128;
	colorYUVFlag = (Y&0xff)|((U&0xff)<<8)|((V&0xff)<<16);
	colorRGBAFlag = rgba;
	curThicknessFlag = thickness;
	return OSA_SOK;
}

static int setHideSysOsd(bool bHideOSD)
{
	if(blob)
	blob->m_bHide = bHideOSD;
	if(motion)
	motion->m_bHide = bHideOSD;
	if(intell)
	intell->m_bHide = bHideOSD;
	if(mmtd)
	mmtd->m_bHide = bHideOSD;
	general->m_bHide = bHideOSD;
	return OSA_SOK;
}

static int setHideSysOsd(unsigned int mask)
{
	if(blob)
	blob->m_bHide = (((mask>>blob->m_identify)&1) == 1);
	if(motion)
	motion->m_bHide = (((mask>>motion->m_identify)&1) == 1);
	if(intell)
	intell->m_bHide = (((mask>>intell->m_identify)&1) == 1);
	if(mmtd)
	mmtd->m_bHide = (((mask>>mmtd->m_identify)&1) == 1);
	general->m_bHide = (((mask>>general->m_identify)&1) == 1);

	bool bDebug = !((mask&1) > 0);
	if(blob)
	blob->m_bDebug = bDebug;
	if(motion)
	motion->m_bDebug = bDebug;
	if(intell)
	intell->m_bDebug = bDebug;
	if(mmtd)
	mmtd->m_bDebug = bDebug;
	general->m_bDebug = bDebug;

	return OSA_SOK;
}


/************************************************************************
 *      process unit
 *
 */

class InputQueue
{
public:
	InputQueue(){};
	virtual ~InputQueue(){destroy();};
	int create(int nChannel, int nBuffer = 2)
	{
		int ret = OSA_SOK;
		OSA_assert(nChannel > 0);
		OSA_assert(queue.size() == 0);
		for(int chId=0; chId<nChannel; chId++){
			OSA_BufHndl* hndl = new OSA_BufHndl;
			OSA_assert(hndl != NULL);
			ret = image_queue_create(hndl, nBuffer, 0, memtype_null);
			OSA_assert(ret == OSA_SOK);
			for(int i=0; i<hndl->numBuf; i++)
			{
				OSA_BufInfo* bufInfo = &hndl->bufInfo[i];
				struct v4l2_buffer *vbuf = new struct v4l2_buffer;
				OSA_assert(vbuf != NULL);
				memset(vbuf, 0, sizeof(vbuf));
				bufInfo->resource = vbuf;
			}
			queue.push_back(hndl);
		}
		return ret;
	}
	int destroy()
	{
		for(int chId=0; chId<queue.size(); chId++){
			OSA_BufHndl* hndl = queue[chId];
			OSA_assert(hndl != NULL);
			for(int i=0; i<hndl->numBuf; i++){
				struct v4l2_buffer *vbuf = (struct v4l2_buffer *)hndl->bufInfo[i].resource;
				if(vbuf != NULL)
					delete vbuf;
				hndl->bufInfo[i].resource = NULL;
			}
			image_queue_delete(hndl);
			delete hndl;
			queue[chId] = NULL;
		}
		queue.clear();
		return OSA_SOK;
	}
	__inline__ uint64 getCurrentTime(void)
	{
		uint64 ret = 0l;
		//struct timeval now;
		//gettimeofday(&now, NULL);
		//ret = (uint64)now.tv_sec * 1000000000ul + (uint64)now.tv_usec*1000ul;
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		ret = (uint64)ts.tv_sec * 1000000000ul + (uint64)ts.tv_nsec;
		return ret;
	}

	int input(int chId,unsigned char *data, const struct v4l2_buffer *vbuf, cv::Size vSize, int format, int channels)
	{
		int ret = OSA_SOK;
		//OSA_printf("%s %d: ch%d(%ld) %d x %d %d", __func__, __LINE__,
		//		chId, queue.size(), vSize.width, vSize.height, channels);
		if(chId>=queue.size()) return OSA_EFAIL;
		OSA_assert(data != NULL);
		OSA_assert(vbuf != NULL);

		OSA_BufHndl* hndl = queue[chId];
		OSA_assert(hndl != NULL);
		OSA_BufInfo* bufInfo = image_queue_getEmpty(hndl);
		if(bufInfo != NULL){
			memcpy(bufInfo->resource, vbuf, sizeof(struct v4l2_buffer));
			bufInfo->chId = chId;
			bufInfo->width = vSize.width;
			bufInfo->height = vSize.height;
			bufInfo->format = format;
			bufInfo->channels = channels;
			//OSA_assert(bufInfo->channels == 2);
			bufInfo->flags = (int)vbuf->flags;
			bufInfo->timestampCap = (uint64)vbuf->timestamp.tv_sec*1000000000ul
					+ (uint64)vbuf->timestamp.tv_usec*1000ul;
			struct timespec timestamp;
			clock_gettime(CLOCK_MONOTONIC, &timestamp);
			bufInfo->timestamp = timestamp.tv_sec*1000000000ULL+timestamp.tv_nsec;
			//OSA_printf("bufInfo->timestampCap %ld %ld %ld",bufInfo->timestampCap, bufInfo->timestamp, getCurrentTime());
			bufInfo->virtAddr = data;
			image_queue_putFull(hndl, bufInfo);
			//OSA_printf("%s %d: %d x %d c%d", __func__, __LINE__, bufInfo->width, bufInfo->height, bufInfo->channels);
		}else{
			//if(chId != 1)
			//OSA_printf("%s %d: InputQueue %s ch%d over flow", __FILE__, __LINE__, __func__, chId);
		}

		return ret;
	}
public:
	OSA_BufInfo* getFullQueue(int chId){
		if(chId>=queue.size())
			return NULL;
		return image_queue_getFull(queue[chId]);
	}
	int getFullQueueCount(int chId){
		if(chId>=queue.size())
			return -1;
		return image_queue_fullCount(queue[chId]);
	}
	int putEmptyQueue(OSA_BufInfo* info){
		OSA_assert(info->chId < queue.size());
		return image_queue_putEmpty(queue[info->chId], info);
	}
protected:
	std::vector<OSA_BufHndl*> queue;
};

static InputQueue *inputQ = NULL;
static OSA_BufHndl *imgQRender[CORE_CHN_MAX] = {NULL,};
static OSA_BufHndl *imgQEnc[CORE_CHN_MAX] = {NULL,};
static OSA_SemHndl *imgQEncSem[CORE_CHN_MAX] = {NULL,};
static unsigned char *memsI420[CORE_CHN_MAX] = {NULL,};
static cv::Mat imgOsd[CORE_CHN_MAX];
static OSA_MutexHndl *cumutex = NULL;
static void glosdInit(void);
static void renderCall(int displayId, int stepIdx, int stepSub, int context);

static int init(CORE1001_INIT_PARAM *initParam, OSA_SemHndl *notify = NULL, CGluVideoWindow *videoWindow = NULL)
{
	int ret = OSA_SOK;
	int chId;
	unsigned char *mem = NULL;
	OSA_assert(initParam != NULL);
	int channels = initParam->nChannels;
	bool bEncoder = initParam->bEncoder;
	bool bRender = initParam->bRender;
	bool bHideOSD = initParam->bHideOSD;
	localInit(channels, bEncoder);
	renderHook = initParam->renderHook;
	if(initParam->encoderParamTab[0]!=NULL){
		for(chId=0; chId<channels; chId++){
			userEncParamTab0[chId][0] = initParam->encoderParamTab[0];
			userEncParamTab1[chId][0] = initParam->encoderParamTab[0];
		}
	}
	if(initParam->encoderParamTab[1]!=NULL){
		for(chId=0; chId<channels; chId++){
			userEncParamTab0[chId][1] = initParam->encoderParamTab[1];
			userEncParamTab1[chId][1] = initParam->encoderParamTab[1];
		}
	}
	if(initParam->encoderParamTab[2]!=NULL){
		for(chId=0; chId<channels; chId++){
			userEncParamTab0[chId][2] = initParam->encoderParamTab[2];
			userEncParamTab1[chId][2] = initParam->encoderParamTab[2];
		}
	}
	for(chId=0; chId<channels; chId++){
		if(initParam->encoderParamTabMulti[chId][0]!=NULL)
			userEncParamTab1[chId][0] = initParam->encoderParamTabMulti[chId][0];
		if(initParam->encoderParamTabMulti[chId][1]!=NULL)
			userEncParamTab1[chId][1] = initParam->encoderParamTabMulti[chId][1];
		if(initParam->encoderParamTabMulti[chId][2]!=NULL)
			userEncParamTab1[chId][2] = initParam->encoderParamTabMulti[chId][2];
	}

	cumutex = new OSA_MutexHndl;
	OSA_mutexCreate(cumutex);
	cuConvertInit(channels, cumutex);

	for(chId=0; chId<channels; chId++){
		CORE1001_CHN_INIT_PARAM *chInf = &initParam->chnInfo[chId];
		int width = chInf->imgSize.width;
		int height = chInf->imgSize.height;
		channelsImgSize[chId] = chInf->imgSize;
		channelsFormat[chId] = chInf->format;
		channelsFPS[chId] = chInf->fps;
		channelsRenderMode[chId] = (chInf->bZoomRender == false) ? 0 : 1;//chInf->notFullRender;
		channelsRenderRatio[chId].x = (chInf->zoomRatio.x < 0.01) ? 1.0f : chInf->zoomRatio.x;
		channelsRenderRatio[chId].y = (chInf->zoomRatio.y < 0.01) ? 1.0f : chInf->zoomRatio.y;
		ret = cudaHostAlloc((void**)&memsI420[chId], width*height*3/2, cudaHostAllocDefault);
		ret = cudaHostAlloc((void**)&mem, width*height, cudaHostAllocDefault);
		imgOsd[chId] = Mat(height, width, CV_8UC1, mem);
		memset(imgOsd[chId].data, 0, imgOsd[chId].cols*imgOsd[chId].rows*imgOsd[chId].channels());

#ifdef USER_VXIO
		std::unique_ptr<MotionComp> compensator(MotionComp::create(MotionComp::MM_HOMOGRAPHY, true));
		motionCompensator[chId] = std::move(compensator);
#endif
	}

	if(bEncoder)
	{
		int iLevel = curTransLevel;
		ENCTRAN_InitPrm enctranInit;
		memset(&enctranInit, 0, sizeof(enctranInit));
		enctranInit.iTransLevel = iLevel;
		enctranInit.nChannels = channels;
		for(chId=0; chId<channels; chId++){
			CORE1001_CHN_INIT_PARAM *chInf = &initParam->chnInfo[chId];
			int* params = getEncParamTab(chId, iLevel);
			enctranInit.defaultEnable[chId] = enableEncoderFlag[chId];
			enctranInit.imgSize[chId] = channelsImgSize[chId];
			enctranInit.inputFPS[chId] = chInf->fps;
			enctranInit.encPrm[chId].fps = chInf->fps;
			enctranInit.encPrm[chId].bitrate = params[0];
			enctranInit.encPrm[chId].minQP = params[1];
			enctranInit.encPrm[chId].maxQP = params[2];
			enctranInit.encPrm[chId].minQI = params[3];
			enctranInit.encPrm[chId].maxQI = params[4];
			enctranInit.encPrm[chId].minQB = params[5];
			enctranInit.encPrm[chId].maxQB = params[6];
			enctranInit.srcType[chId] = APPSRC;
		}
		if(initParam->encStreamIpaddr != NULL){
			enctranInit.bRtp = true;
			strcpy(enctranInit.destIpaddr, initParam->encStreamIpaddr);
		}

		enctran = new CEncTrans;
		OSA_assert(enctran != NULL);

		enctran->create();
		enctran->init(&enctranInit);
		enctran->run();
		for(chId=0; chId<channels; chId++){
			imgQEnc[chId] = enctran->m_bufQue[chId];
			imgQEncSem[chId] = enctran->m_bufSem[chId];
			vOSDs[chId] = new cr_osd::DCOSD(&imgOsd[chId], fontSizeVideo[chId], fileNameFont[chId]);
			cr_osd::vosdFactorys.push_back(vOSDs[chId]);
		}
	}

	if(bRender)
	{
		renderFPS = initParam->renderFPS;
		if(renderFPS<=0)
			renderFPS = 30;

		if(videoWindow == NULL){
			VWIND_Prm dsInit;
			memset(&dsInit, 0, sizeof(dsInit));
			dsInit.disSched = initParam->renderSched;
			if(dsInit.disSched<=0.00001)
				dsInit.disSched = 0.01;
			dsInit.bFullScreen = true;
			dsInit.nChannels = channels;
			dsInit.memType = memtype_glpbo;//memtype_glpbo;//memtype_cuhost;//memtype_cudev;
			dsInit.nQueueSize = 4;
			dsInit.disFPS = initParam->renderFPS;
			dsInit.renderfunc = renderCall;
			//dsInit.winWidth = initParam->renderSize.width;
			//dsInit.winHeight = initParam->renderSize.height;
			for(chId=0; chId<channels; chId++){
				dsInit.channelInfo[chId].w = channelsImgSize[chId].width;
				dsInit.channelInfo[chId].h = channelsImgSize[chId].height;
				dsInit.channelInfo[chId].fps = initParam->chnInfo[chId].fps;
				if(!bEncoder && channelsFormat[chId] == V4L2_PIX_FMT_GREY)
					dsInit.channelInfo[chId].c = 1;
				else
					dsInit.channelInfo[chId].c = 3;
			}
#if 0
			int parentWinId = 0;
			VWIND_Prm parentPrm;
			memset(&parentPrm, 0, sizeof(parentPrm));
			parentPrm.bFullScreen = true;
			static CGluVideoWindow *parent = new CGluVideoWindow(initParam->renderRC, 0, 0);
			parent->Create(parentPrm);
			parentWinId = parent->m_winId;
			//initParam->renderRC.width = initParam->renderRC.width/2;
			//initParam->renderRC.height = initParam->renderRC.height/2;

			gluWindow = new CGluVideoWindow(initParam->renderRC, parentWinId);
			gluWindow->Create(dsInit);
#else
			gluWindow = new CGluVideoWindow(initParam->renderRC);
			gluWindow->Create(dsInit);
#endif
		}else{
			videoWindow->m_initPrm.renderfunc = renderCall;
			gluWindow = videoWindow;
		}

		for(chId=0; chId<channels; chId++){

			imgQRender[chId] = gluWindow->m_bufQue[chId];

			if(channelsRenderMode[chId] == 1 &&
					channelsImgSize[chId].width*channelsRenderRatio[chId].x<gluWindow->m_rc.width &&
					channelsImgSize[chId].height*channelsRenderRatio[chId].y<gluWindow->m_rc.height)
			{
				int toWidth = channelsImgSize[chId].width*channelsRenderRatio[chId].x+0.5f;
				int toHeight = channelsImgSize[chId].height*channelsRenderRatio[chId].y+0.5f;
				mainWinRenderRC[chId] = cv::Rect( (gluWindow->m_rc.width-toWidth)>>1, (gluWindow->m_rc.height - toHeight)>>1, toWidth, toHeight );
			}else{
				mainWinRenderRC[chId] = cv::Rect(0, 0, gluWindow->m_rc.width, gluWindow->m_rc.height);
			}
		}

		inputQ = new InputQueue;
		inputQ->create(channels, 4);

		if(!bEncoder)
			glosdInit();

		subRc = gluWindow->m_vvideoRenders[1]->m_viewPort;//gluWindow->m_renders[1].displayrect;
		subMatric = gluWindow->m_vvideoRenders[1]->m_matrix;//gluWindow->m_renders[1].transform;
	}

#if 1
	scene  = new CSceneProcess();
	blob = new CBlobDetectProcess(scene);
	bkgd = new CBkgdDetectProcess(blob);
	motion = new CMotionDetectProcess(bkgd);
	intell = new CINTELLProcess(motion);
	mmtd = new CMMTDProcess(intell);
	general = new CGeneralProc(notify,mmtd);

	blob->m_bHide = bHideOSD;
	motion->m_bHide = bHideOSD;
	intell->m_bHide = bHideOSD;
	mmtd->m_bHide = bHideOSD;
	general->m_bHide = bHideOSD;

#else
	mmtd = new CMMTDProcess();
	general = new CGeneralProc(notify,mmtd);
	mmtd->m_bHide = bHideOSD;
	general->m_bHide = bHideOSD;
#endif

	for(chId=0; chId<channels; chId++){
		if(!bEncoder && gluWindow!=NULL){
			general->m_dc[chId] = cv::Mat(mainWinRenderRC[chId].height, mainWinRenderRC[chId].width, CV_8UC1, NULL);
		}else
			general->m_dc[chId] = imgOsd[chId];

		general->m_vosds[chId] = vOSDs[chId];
		general->m_imgSize[chId] = channelsImgSize[chId];
	}

	proc = general;
	general->creat();
	general->init();
	general->run();
	
	if(gDeviceMem != NULL){
		cudaFree(gDeviceMem);
		gDeviceMem = NULL;
	}
	if(gDeviceMem == NULL){
		ret = cudaMalloc((void**)&gDeviceMem, 1920*1080*3);
		assert(ret == OSA_SOK);
	}
	
	return ret;
}

static int uninit()
{
	general->stop();
	general->destroy();
	enctran->stop();
	enctran->destroy();
	if(gluWindow != NULL){
		gluWindow->Destroy();
		delete gluWindow;
		gluWindow = NULL;
	}

	if(enctran == NULL){
		for(int i=0; i<cr_osd::vosdFactorys.size(); i++)
			delete cr_osd::vosdFactorys[i];
	}
	cr_osd::vosdFactorys.clear();

	delete enctran;
	delete general;
	delete mmtd;
	delete intell;
	delete motion;
	delete bkgd;
	delete blob;
	delete scene;
	for(int chId=0; chId<nValidChannels; chId++){
		cudaFreeHost(imgOsd[chId].data);
		cudaFreeHost(memsI420[chId]);
		if(vOSDs[chId] != NULL)
			delete vOSDs[chId];
		vOSDs[chId] = NULL;
		if(motionCompensator[chId])
			motionCompensator[chId] = std::move(nullptr);
	}

	cuConvertUinit();
	if(cumutex != NULL){
		OSA_mutexUnlock(cumutex);
		OSA_mutexLock(cumutex);
		OSA_mutexDelete(cumutex);
		delete cumutex;
		cumutex = NULL;
	}
	if(inputQ != NULL)
		delete inputQ;

	if(gDeviceMem != NULL){
		cudaFree(gDeviceMem);
		gDeviceMem = NULL;
	}

}
#define ZeroCpy	(0)
static void encTranFrame(int chId, const Mat& img, const struct v4l2_buffer& bufInfo, bool bEnc)
{
	Mat i420;
	OSA_BufInfo* info = NULL;
	unsigned char *mem = NULL;

	enctran->scheduler(chId);

	if(bEnc){
		if(ZeroCpy){
			if(imgQEnc[chId] != NULL)
				info = image_queue_getEmpty(imgQEnc[chId]);
		}
		if(info != NULL){
			info->chId = chId;
			info->timestampCap = (uint64)bufInfo.timestamp.tv_sec*1000000000ul
					+ (uint64)bufInfo.timestamp.tv_usec*1000ul;
			info->timestamp = (uint64_t)getTickCount();
			mem = (unsigned char *)info->virtAddr;
		}
	}

	if(mem == NULL)
		mem = memsI420[chId];
	i420 = Mat((int)(img.rows+img.rows/2), img.cols, CV_8UC1, mem);
	if(enableOSDFlag){
		if(enableEnhFlag[chId])
			cuConvertEnh_async(chId, img, imgOsd[chId], i420, ezoomxFlag[chId], colorYUVFlag);
		else
			cuConvert_async(chId, img, imgOsd[chId], i420, ezoomxFlag[chId], colorYUVFlag);
	}else{
		if(enableEnhFlag[chId])
			cuConvertEnh_async(chId, img, i420, ezoomxFlag[chId]);
		else
			cuConvert_async(chId, img, i420, ezoomxFlag[chId]);
	}

	if(bEnc){
		if(!ZeroCpy)
			enctran->pushData(i420, chId, V4L2_PIX_FMT_YUV420M);
		if(info != NULL){
			info->channels = i420.channels();
			info->width = i420.cols;
			info->height = i420.rows;
			info->format = V4L2_PIX_FMT_YUV420M;
			image_queue_putFull(imgQEnc[chId], info);
			OSA_semSignal(imgQEncSem[chId]);
		}
	}
}
#undef ZeroCpy

static void renderFrameAfterEnc(int chId, const Mat& img, const struct v4l2_buffer& bufInfo)
{
	Mat bgr;
	OSA_BufInfo* info = image_queue_getEmpty(imgQRender[chId]);
	if(info != NULL)
	{
		bgr = Mat(img.rows,img.cols,CV_8UC3, info->physAddr);
		cuConvertConn_yuv2bgr_i420(chId, bgr, CUT_FLAG_devAlloc);
		info->chId = chId;
		info->channels = bgr.channels();
		info->width = bgr.cols;
		info->height = bgr.rows;
		info->format = V4L2_PIX_FMT_BGR24;
		info->timestampCap = (uint64)bufInfo.timestamp.tv_sec*1000000000ul
				+ (uint64)bufInfo.timestamp.tv_usec*1000ul;
		info->timestamp = (uint64_t)getTickCount();

		image_queue_putFull(imgQRender[chId], info);

#if 1//wo qv
		GLBatchMini lineBatch;
		lineBatch.Reset();
		lineBatch.Begin(GL_LINES, 2);
		lineBatch.Vertex3f(0.1,0.1, 0);
		lineBatch.Vertex3f(0.2,0.2, 0);
		lineBatch.End();
#endif
		if(0)
		//if(chId == 1)
		{
			Mat dis = Mat(img.rows,img.cols,CV_8UC3);
			cudaMemcpy(dis.data, bgr.data, bgr.rows*bgr.cols*bgr.channels(), cudaMemcpyDeviceToHost);
			cv::imshow("test" , dis);
			cv::waitKey(1);
		}
	}else{
		//OSA_printf("%s %d: overflow!", __func__, __LINE__);
		//gluWindow->m_timerRun = false;
	}
}

static unsigned long render_i[CORE_CHN_MAX] = {0, 0, 0, 0};
static unsigned long render_to[CORE_CHN_MAX] = {0, 0, 0, 0};
static void renderFrame(int chId, const Mat& img, const struct v4l2_buffer& bufInfo, int format)
{
	bool bRender = (chId == curChannelFlag || chId == curSubChannelIdFlag || bindBlendFlag[chId] != 0);

	if(render_i[chId]==render_to[chId])
	{
		if(bRender && imgQRender[chId] != NULL)
		{
			Mat frame;
			OSA_BufInfo* info = image_queue_getEmpty(imgQRender[chId]);
			if(info != NULL)
			{
				if(format==V4L2_PIX_FMT_YUYV){
					frame = Mat(img.rows,img.cols,CV_8UC3, info->physAddr);
					if(enableEnhFlag[chId])
						cuConvertEnh_yuv2bgr_yuyv_async(chId, img, frame, CUT_FLAG_devAlloc);
					else{
						cuConvert_yuv2bgr_yuyv_async(chId, img, frame, CUT_FLAG_devAlloc);
					}
					info->format = V4L2_PIX_FMT_BGR24;
				}
				else if(format==V4L2_PIX_FMT_UYVY){
					
					frame = Mat(img.rows,img.cols,CV_8UC3, info->physAddr);
					if(enableEnhFlag[chId])
						cuConvertEnh_yuv2bgr_uyvy_async(chId, img, frame, CUT_FLAG_devAlloc);
					else{
						cuConvert_yuv2bgr_uyvy_async(chId, img, frame, CUT_FLAG_devAlloc);
					}
					info->format = V4L2_PIX_FMT_BGR24;
				}
				else if(format==V4L2_PIX_FMT_BGR24){
					frame = Mat(img.rows,img.cols,CV_8UC3, info->physAddr);
					cudaMemcpy(info->physAddr, img.data, frame.rows*frame.cols*frame.channels(), cudaMemcpyHostToDevice);
					info->format = V4L2_PIX_FMT_BGR24;
					//OSA_printf("%s %d: %d x %d", __func__, __LINE__, frame.cols, frame.rows);
				}
				else//if(format==V4L2_PIX_FMT_GREY)
				{
					frame = Mat(img.rows,img.cols,CV_8UC1, info->physAddr);
					if(enableEnhFlag[chId])
						cuConvertEnh_gray(chId, img, frame, CUT_FLAG_devAlloc);
					else
						cudaMemcpy(info->physAddr, img.data, frame.rows*frame.cols*frame.channels(), cudaMemcpyHostToDevice);
					info->format = format;//V4L2_PIX_FMT_GREY;
				}

				cv::Matx33f transform = cv::Matx33f::eye();

#ifdef USER_VXIO
				if(enableStabFlag[chId]){
					cvGpuMat inFrame = cvGpuMat(frame.rows, frame.cols, frame.type(), (void*)frame.data, frame.step.buf[0]);
					cvGpuMat outFrame = cvGpuMat(frame.rows, frame.cols, frame.type(), (void*)frame.data, frame.step.buf[0]);
					cvGpuMat midFrame = cvGpuMat(frame.rows, frame.cols>>1, frame.type(), (void*)gDeviceMem, frame.step.buf[0]>>1);

					if(enableStabSideBySideFlag[chId])
					{
						cudaMemcpy2D(midFrame.data, midFrame.step, inFrame.data+(inFrame.cols>>1)*inFrame.channels(), inFrame.step,
								midFrame.cols*midFrame.channels(), midFrame.rows, cudaMemcpyDeviceToDevice);
					}


					//cv::Mat mask = cv::Mat(frame.rows, frame.cols, CV_8UC1);
				    //mask.setTo(0);
				    //cv::Rect rec = cv::Rect(50, 100, frame.cols-100, frame.rows-220);
				    //mask(rec).setTo(255);
				    //cvGpuMat gpuMask(mask);
					//OSA_printf("[%p] %d x %d step = %ld ", inFrame.data, inFrame.cols, inFrame.rows, inFrame.step);
					if(nSabCount[chId] == 0){
						MotionComp::InitParams stabInitParams;
						stabInitParams.bBorderTransparent = stabParams[chId].bBorderTransparent;
						stabInitParams.cropMargin = stabParams[chId].cropMargin;
						stabInitParams.bCropMarginScale = stabParams[chId].bCropMarginScale;
						stabInitParams.noise_cov = stabParams[chId].noise_cov;
						stabInitParams.bPreprocess = stabParams[chId].bPreProcess;

						motionCompensator[chId].reset(MotionComp::create((MotionComp::MotionModel)stabParams[chId].mm, !stabParams[chId].bFixedPos));

						motionCompensator[chId]->init(inFrame, stabInitParams);
					}else{
						motionCompensator[chId]->process(inFrame);
					}
					cvGpuMat stabFrame = motionCompensator[chId]->getOut();

					if(enableStabSideBySideFlag[chId])
					{
						cudaMemcpy2D(outFrame.data, outFrame.step, stabFrame.data, stabFrame.step,
							(outFrame.cols>>1)*outFrame.channels(), outFrame.rows, cudaMemcpyDeviceToDevice);
						cudaMemcpy2D(outFrame.data+(outFrame.cols>>1)*outFrame.channels(), outFrame.step, midFrame.data, midFrame.step,
							(outFrame.cols>>1)*outFrame.channels(), outFrame.rows, cudaMemcpyDeviceToDevice);
					}
					else
					{
						cudaMemcpy2D(outFrame.data, outFrame.step, stabFrame.data, stabFrame.step,
								outFrame.cols*outFrame.channels(), outFrame.rows, cudaMemcpyDeviceToDevice);
					}


					
					//cudaDeviceSynchronize();
					//motionCompensator[chId]->printPerfs();
					nSabCount[chId] ++;
					transform = motionCompensator[chId]->getTransform();
				}
#endif
				info->chId = chId;
				info->channels = frame.channels();
				info->width = frame.cols;
				info->height = frame.rows;
				info->timestampCap = (uint64)bufInfo.timestamp.tv_sec*1000000000ul
						+ (uint64)bufInfo.timestamp.tv_usec*1000ul;
				info->timestamp = (uint64_t)getTickCount();
				*(cv::Matx33f *)info->virtAddr = transform;
				image_queue_putFull(imgQRender[chId], info);
				//OSA_printf("%p\n", info->virtAddr);
				//vOSDs[chId].set();
			}
			else{
				static unsigned int nDiscard[CORE_CHN_MAX] = { };
				//OSA_printf("core %s %d: ch%d overflow!", __func__, __LINE__, chId);
				//image_queue_switchEmpty(imgQRender[chId]);
				nDiscard[chId]++;
				if((nDiscard[chId]%50) == 0){
					printf("\n[%d]%s %d: %s ch%d overflow cnt = %d!", OSA_getCurTimeInMsec(),__FILE__, __LINE__, __func__, chId, nDiscard[chId]);
					fflush(stdout);
				}
			}
		}
		int inc = channelsFPS[chId]/renderFPS;
		if(inc<=0)inc = 1;
		render_to[chId]=render_i[chId]+inc;
	}
	render_i[chId]++;
}

static void processFrameAtOnce(int cap_chid, unsigned char *src, const struct v4l2_buffer& capInfo, int format)
{
	if(capInfo.flags & V4L2_BUF_FLAG_ERROR)
		return;

	//uint64_t timestamp = (uint64_t)capInfo.timestamp.tv_sec*100000000ULL + capInfo.timestamp.tv_usec*1000ULL;
	//uint64_t timestamp = getTickCount();
	//struct timespec timestamp;
	//clock_gettime(CLOCK_MONOTONIC, &timestamp);
	uint64_t tm = capInfo.timestamp.tv_sec*1000000000ULL+capInfo.timestamp.tv_usec*1000ULL;

	if(curChannelFlag == cap_chid /*|| curSubChannelIdFlag == cap_chid*/ || bindBlendFlag[cap_chid] != 0){
		Mat img;
		if(format==V4L2_PIX_FMT_YUYV || format==V4L2_PIX_FMT_UYVY)
		{
			img	= Mat(channelsImgSize[cap_chid].height,channelsImgSize[cap_chid].width,CV_8UC2, src);
		}
		else if(format==V4L2_PIX_FMT_GREY)
		{
			img = Mat(channelsImgSize[cap_chid].height,channelsImgSize[cap_chid].width,CV_8UC1,src);
		}
		else if(format==V4L2_PIX_FMT_BGR24){
			img = Mat(channelsImgSize[cap_chid].height,channelsImgSize[cap_chid].width,CV_8UC3,src);
		}
		else{
			OSA_assert(0);
		}
		proc->process(cap_chid, curFovIdFlag[cap_chid], ezoomxFlag[cap_chid], img, Mat(), tm);
		if(general->m_bTrack && general->m_iTrackStat == 1){
			general->m_shTrkTarget.valid = true;
			general->m_shTrkTarget.Box = cv::Rect(general->m_rcTrk.x, general->m_rcTrk.y, general->m_rcTrk.width, general->m_rcTrk.height);
		}else{
			general->m_shTrkTarget.valid = false;
		}
	}

}

static void processFrame(int cap_chid, unsigned char *src, const struct v4l2_buffer& capInfo, int format)
{
	Mat img;
	uint64_t tm = capInfo.timestamp.tv_sec*1000000000ULL+capInfo.timestamp.tv_usec*1000ULL;

	//if(capInfo.flags & V4L2_BUF_FLAG_ERROR)
	//	return;
	//struct timespec ns0;
	//clock_gettime(CLOCK_MONOTONIC_RAW, &ns0);
	if(format==V4L2_PIX_FMT_YUYV || format==V4L2_PIX_FMT_UYVY)
	{
		//OSA_printf("%s ch%d %d", __func__, cap_chid, OSA_getCurTimeInMsec());
		//OSA_printf("%s: %d ch%d %p", __func__, __LINE__, cap_chid, src);
		img	= Mat(channelsImgSize[cap_chid].height,channelsImgSize[cap_chid].width,CV_8UC2, src);
	}
	else if(format==V4L2_PIX_FMT_GREY)
	{
		//OSA_printf("%s ch%d %d", __func__, cap_chid, OSA_getCurTimeInMsec());
		//OSA_printf("%s: %d ch%d %p", __func__, __LINE__, cap_chid, src);
		img = Mat(channelsImgSize[cap_chid].height,channelsImgSize[cap_chid].width,CV_8UC1,src);
	}
	else if(format==V4L2_PIX_FMT_BGR24){
		img = Mat(channelsImgSize[cap_chid].height,channelsImgSize[cap_chid].width,CV_8UC3,src);
		//OSA_printf("%s %d: %d x %d", __func__, __LINE__, img.cols, img.rows);
	}else{
		OSA_assert(0);
	}

	if(bindBlendFlag[cap_chid] != 0 && gluWindow != NULL && bkgd != NULL){
		GLV_BlendPrm prm;
		prm.fAlpha = bkgd->m_alpha;
		prm.thr0Min = bkgd->m_thr0Min;
		prm.thr0Max = bkgd->m_thr0Max;
		prm.thr1Min = bkgd->m_thr1Min;
		prm.thr1Max = bkgd->m_thr1Max;
		for(int chId=0; chId<VWIN_CHAN_MAX; chId++){
			if((bindBlendFlag[cap_chid] & (1<<chId))!=0)
				gluWindow->dynamic_config(CGluVideoWindow::VWIN_CFG_BlendPrm, chId*VWIN_CHAN_MAX+cap_chid, &prm);
		}
	}

	if(imgQEnc[cap_chid] != NULL)
	{
		if(vOSDs[cap_chid] != NULL)
			vOSDs[cap_chid]->begin(colorRGBAFlag, curThicknessFlag);
		if( vOSDs[cap_chid] != NULL && (curChannelFlag == cap_chid /*|| curSubChannelIdFlag == cap_chid*/ || bindBlendFlag[cap_chid] != 0) )
			proc->OnOSD(cap_chid, curFovIdFlag[cap_chid], ezoomxFlag[cap_chid], general->m_dc[cap_chid], general->m_vosds[cap_chid], tm);
		if(vOSDs[cap_chid] != NULL && enableOSDFlag)
			vOSDs[cap_chid]->Draw();

		encTranFrame(cap_chid, img, capInfo, enableEncoderFlag[cap_chid]);

		if(imgQRender[cap_chid] != NULL){
			renderFrameAfterEnc(cap_chid, img, capInfo);
		}

		if(vOSDs[cap_chid] != NULL)
			vOSDs[cap_chid]->end();
	}
	else if(imgQRender[cap_chid] != NULL)
	{
		/*if(!isEqual(ezoomxFlag[cap_chid]*1.0, scaleFlag[cap_chid])){
			scaleFlag[cap_chid] = ezoomxFlag[cap_chid]*1.0;
			GLfloat fscale = scaleFlag[cap_chid];
			GLfloat mat4[16] ={
				fscale, 0.0f, 0.0f, 0.0f,
				0.0f, fscale, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			};
			gluWindow->dynamic_config(CGluVideoWindow::DS_CFG_VideoTransMat, cap_chid, mat4);
		}*/
		if(curChannelFlag == cap_chid && !isEqual(ezoomxFlag[cap_chid]*1.0, scaleFlag[0])){
			scaleFlag[0] = ezoomxFlag[cap_chid]*1.0;
			GLfloat fscale = scaleFlag[0];
			GLfloat mat4[16] ={
				fscale, 0.0f, 0.0f, 0.0f,
				0.0f, fscale, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			};
			gluWindow->dynamic_config(CGluVideoWindow::VWIN_CFG_ViewTransMat, 0, mat4);
		}
		//gluWindow->m_bOsd = enableOSDFlag;
		renderFrame(cap_chid, img, capInfo, format);
		if(vOSDs[cap_chid] != NULL)
			vOSDs[cap_chid]->begin(colorRGBAFlag, curThicknessFlag);
		if(vOSDs[cap_chid] != NULL && (curChannelFlag == cap_chid /*|| curSubChannelIdFlag == cap_chid*/ || bindBlendFlag[cap_chid] != 0)){
			uint64_t tm = gluWindow->m_vvideos[cap_chid]->m_curBufInfo.timestampCap;
			proc->OnOSD(cap_chid, curFovIdFlag[cap_chid], ezoomxFlag[cap_chid], general->m_dc[cap_chid], general->m_vosds[cap_chid], tm);
		}
	}

	//struct timespec ns1;
	//clock_gettime(CLOCK_MONOTONIC_RAW, &ns1);
	//printf("[%ld.%ld] ch%d timestamp %ld.%ld flags %08X\n", ns1.tv_sec, ns1.tv_nsec/1000000,
	//		cap_chid, ns0.tv_sec, ns0.tv_nsec/1000000, info.flags);
}

static void videoInput(int cap_chid, unsigned char *src, const struct v4l2_buffer& capInfo, int format)
{
	if(capInfo.flags & V4L2_BUF_FLAG_ERROR){
		//OSA_printf("%s %d: ch%d V4L2_BUF_FLAG_ERROR(0x%x)", __func__, __LINE__, cap_chid, capInfo.flags);
		return;
	}
	//if(capInfo.flags & V4L2_BUF_FLAG_ERROR)
	//	return;
	processFrameAtOnce(cap_chid,src, capInfo, format);
	if(inputQ != NULL){
		int channels = ( (format == V4L2_PIX_FMT_GREY) ? 1 : ((format == V4L2_PIX_FMT_BGR24 || format == V4L2_PIX_FMT_RGB24) ? 3 : 2) );
		inputQ->input(cap_chid, src, &capInfo, channelsImgSize[cap_chid], format, channels);
		OSA_assert(gluWindow != NULL);
	}else{
		processFrame(cap_chid,src, capInfo, format);
	}
}

static void glosdInit(void)
{
	cr_osd::glShaderManager.InitializeStockShaders();
	for(int chId=0; chId<nValidChannels; chId++){
		if(vOSDs[chId] == NULL)
		{
			vOSDs[chId] = new cr_osd::GLOSD(mainWinRenderRC[chId], fontSizeVideo[chId], fileNameFont[chId]);
		}
	}
	if(enctran == NULL)
	{
		glosdFront = new cr_osd::GLOSD(gluWindow->m_rc.width, gluWindow->m_rc.height, fontSizeRender, fileNameFontRender);
		cr_osd::vosdFactorys.push_back(glosdFront);
	}
}

static void renderCall(int displayId, int stepIdx, int stepSub, int context)
{
	if(renderHook != NULL)
		renderHook(displayId, stepIdx, stepSub, context);

	if(enctran == NULL)
	{
		OSA_assert(imgQEnc[0] == NULL);
		/*if(stepIdx == CGluVideoWindow::RUN_ENTER)
		{
			for(int chId =0; chId<nValidChannels; chId++){
				if(vOSDs[chId] != NULL)
					vOSDs[chId]->begin(colorRGBAFlag, curThicknessFlag);
			}
		}*/
		if(stepIdx == CGluVideoWindow::RUN_WIN)
		{
			if(enableOSDFlag && stepSub == 0 && vOSDs[context] != NULL){
				vOSDs[context]->Draw();
			}
		}
		if(stepIdx == CGluVideoWindow::RUN_SWAP)
		{
			for(int chId =0; chId<nValidChannels; chId++){
				if(vOSDs[chId] != NULL)
					vOSDs[chId]->end();
			}
			if(glosdFront != NULL)
				glosdFront->Draw();
		}
	}

	if(stepIdx == CGluVideoWindow::RUN_ENTER)
	{
		OSA_assert(inputQ != NULL);
	#if 0
		for(int chId = 0; chId < nValidChannels; chId++){
			OSA_BufInfo* info = NULL;
			info = inputQ->getFullQueue(chId);
			if(info == NULL)
				continue;
			OSA_assert(info->chId == chId);
			processFrame(chId, (unsigned char*)info->virtAddr, *(struct v4l2_buffer*)info->resource, info->format);
			inputQ->putEmptyQueue(info);
		}
	#endif
	#if 0
		for(int chId = 0; chId < nValidChannels; chId++){
			OSA_BufInfo* info = NULL;
			int nFull = inputQ->getFullQueueCount(chId);
			for(int i=0; i<nFull-1; i++){
				info = inputQ->getFullQueue(chId);
				inputQ->putEmptyQueue(info);
			}
			info = inputQ->getFullQueue(chId);
			if(info == NULL)
				continue;
			OSA_assert(info->chId == chId);
			processFrame(chId, (unsigned char*)info->virtAddr, *(struct v4l2_buffer*)info->resource, info->format);
			inputQ->putEmptyQueue(info);
		}
	#endif
	#if 1
		for(int chId = 0; chId < nValidChannels; chId++){
			OSA_BufInfo* info = NULL;
			int nFull = inputQ->getFullQueueCount(chId);
			for(int i=0; i<nFull; i++){
				info = inputQ->getFullQueue(chId);
				OSA_assert(info->chId == chId);
				processFrame(chId, (unsigned char*)info->virtAddr, *(struct v4l2_buffer*)info->resource, info->format);
				inputQ->putEmptyQueue(info);
			}
		}
	#endif
	}
}//void renderCall(int displayId, int stepIdx, int stepSub, int context)

};//namespace cr_local

#define UPDATE() do{update();if(m_notifySem != NULL)OSA_semSignal(m_notifySem);}while(0)

class Core_1001 : public ICore_1001
{
	Core_1001():m_notifySem(NULL),m_bRun(false){
		coreId = ID;
		memset(&m_stats, 0, sizeof(m_stats));
	};
	virtual ~Core_1001(){uninit();};
	friend ICore* ICore::Qury(int coreID);
	OSA_SemHndl m_updateSem;
	OSA_SemHndl *m_notifySem;
	bool m_bRun;
	void update();
	static void *thrdhndl_update(void *context)
	{
		Core_1001 *core = (Core_1001 *)context;
		while(core->m_bRun){
			OSA_semWait(&core->m_updateSem, OSA_TIMEOUT_FOREVER);
			if(!core->m_bRun)
				break;
			core->update();
			if(core->m_notifySem != NULL)
				OSA_semSignal(core->m_notifySem);
		}
		return NULL;
	}
public:
	static char m_version[16];
	static unsigned int ID;
	static wchar_t sztest[128];
	virtual int init(void *pParam, int paramSize)
	{
		printf("\r\n crCore(ID:%08X) v%s--------------Build date: %s %s \r\n",
				ID, m_version, __DATE__, __TIME__);

		int ret = OSA_SOK;
		int nChannels = 0;
		OSA_semCreate(&m_updateSem, 1, 0);
		if(sizeof(CORE1001_INIT_PARAM) == paramSize){
			CORE1001_INIT_PARAM *initParam = (CORE1001_INIT_PARAM*)pParam;
			m_notifySem = initParam->notify;
			ret = cr_local::init(initParam, &m_updateSem);
			nChannels = initParam->nChannels;
		}else if(sizeof(CORE1001_INIT_PARAM2) == paramSize){
			CORE1001_INIT_PARAM2 *initParam2 = (CORE1001_INIT_PARAM2*)pParam;
			CORE1001_INIT_PARAM initParam;
			memset(&initParam, 0, sizeof(initParam));
			memcpy(initParam.chnInfo, initParam2->chnInfo, sizeof(initParam.chnInfo));
			initParam.nChannels = initParam2->nChannels;
			initParam.notify = initParam2->notify;
			initParam.bEncoder = initParam2->bEncoder;
			initParam.bHideOSD = initParam2->bHideOSD;
			initParam.encStreamIpaddr = initParam2->encStreamIpaddr;
			memcpy(initParam.encoderParamTab, initParam2->encoderParamTab, sizeof(initParam.encoderParamTab));
			memcpy(initParam.encoderParamTabMulti, initParam2->encoderParamTabMulti, sizeof(initParam.encoderParamTabMulti));
			if(initParam2->videoWindow != NULL){
				initParam.bRender = true;
				initParam.renderRC = initParam2->videoWindow->m_rc;
				initParam.renderFPS = initParam2->videoWindow->m_initPrm.disFPS;
				initParam.renderSched = initParam2->videoWindow->m_initPrm.disSched;
				initParam.renderHook = initParam2->videoWindow->m_initPrm.renderfunc;
			}
			ret = cr_local::init(&initParam, &m_updateSem, initParam2->videoWindow);
			nChannels = initParam.nChannels;
		}else{
			OSA_assert(0);
		}
		memset(&m_stats, 0, sizeof(m_stats));
		update();
		for(int chId=0; chId<nChannels; chId++)
			m_dc[chId] = cr_local::general->m_dc[chId];
		m_bRun = true;
		start_thread(thrdhndl_update, this);
		cr_osd::put(sztest, cv::Point(800, 30), cv::Scalar(80, 80, 80, 10), 128, L"== ID:%x v%s ==", ID, m_version);
		return ret;
	}
	virtual int uninit()
	{
		m_bRun = false;
		OSA_semSignal(&m_updateSem);
		int ret = cr_local::uninit();
		cr_osd::clear();
		memset(&m_stats, 0, sizeof(m_stats));
		OSA_semDelete(&m_updateSem);
		return ret;
	}
	virtual void processFrame(int chId, unsigned char *data, struct v4l2_buffer capInfo, int format)
	{
		static unsigned int cnt = 10;
		if(capInfo.flags & V4L2_BUF_FLAG_ERROR){
			swprintf(sztest, 128, L"== ID:%x v%s ==", ID, m_version);
			cnt = 10;
		}else if(cnt==0){
			swprintf(sztest, 128, L"");
		}
		cnt = (cnt > 0) ? (cnt-1) : 0;
		cr_local::videoInput(chId, data, capInfo, format);
	}

	virtual int setMainChId(int chId, int fovId, int ndrop, const cv::Size& acqSize)
	{
		UTC_SIZE sz;
		sz.width = acqSize.width; sz.height = acqSize.height;
		int ret = cr_local::setMainChId(chId, fovId, ndrop, sz);
		UPDATE();
		return ret;
	}
	virtual int setSubChId(int chId)
	{
		int ret = cr_local::setSubChId(chId);
		UPDATE();
		return ret;
	}
	virtual int enableTrack(bool enable, cv::Size winSize, bool bFixSize)
	{
		UTC_SIZE sz;
		sz.width = winSize.width; sz.height = winSize.height;
		int ret = cr_local::enableTrack(enable, sz, bFixSize);
		UPDATE();
		return ret;
	}
	virtual int enableTrack(bool enable, Rect2f winRect, bool bFixSize)
	{
		UTC_RECT_float rc;
		rc.x = winRect.x; rc.y = winRect.y;
		rc.width = winRect.width; rc.height = winRect.height;
		int ret = cr_local::enableTrack(enable, rc, bFixSize);
		UPDATE();
		return ret;
	}
	//virtual int enableMMTD(bool enable, int nTarget, int nSel = 0)
	virtual int enableMMTD(bool enable, int nTarget, int nSel = 0, const cv::Rect& roi=cv::Rect(0,0,0,0))
	{
		int ret = cr_local::enableMMTD(enable, nTarget, nSel, roi);
		UPDATE();
		return ret;
	}
	virtual int enableIntellDetect(bool enable, int nTarget, int nSel = 0, const cv::Rect& roi=cv::Rect(0,0,0,0))
	{
		int ret = cr_local::enableIntellDetect(enable, nTarget, nSel, roi);
		UPDATE();
		return ret;
	}
	virtual int enableMotionDetect(bool enable)
	{
		int ret = cr_local::enableMotionDetect(enable);
		UPDATE();
		return ret;
	}
	virtual int enableTrackByMMTD(int index, cv::Size *winSize, bool bFixSize)
	{
		int ret = cr_local::enableTrackByMMTD(index, winSize, bFixSize);
		UPDATE();
		return ret;
	}
	virtual int enableEnh(bool enable)
	{
		int ret = cr_local::enableEnh(enable);
		UPDATE();
		return ret;
	}
	virtual int enableEnh(int chId, bool enable)
	{
		int ret = cr_local::enableEnh(chId, enable);
		UPDATE();
		return ret;
	}

	virtual int enableSideBySide(bool enable)
	{
		int ret = cr_local::enableSideBySide(enable);
		UPDATE();
		return ret;	
	}
	
	virtual int enableSideBySide(int chId, bool enable)
	{
		int ret = cr_local::enableSideBySide(chId, enable);
		UPDATE();
		return ret;	
	}

	virtual int enableStab(bool enable, const CORE_STAB_PARAM& params)
	{
		int ret = cr_local::enableStab(enable, params);
		UPDATE();
		return ret;
	}
	virtual int enableStab(int chId, bool enable, const CORE_STAB_PARAM& params)
	{
		int ret = cr_local::enableStab(chId, enable, params);
		UPDATE();
		return ret;
	}
	virtual int enableBlob(bool enable)
	{
		int ret = cr_local::enableBlob(enable);
		UPDATE();
		return ret;
	}
	virtual int bindBlend(int blendchId, const cv::Matx44f& matric)
	{
		int ret = cr_local::bindBlend(blendchId, matric);
		UPDATE();
		return ret;
	}
	virtual int bindBlend(int chId, int blendchId, const cv::Matx44f& matric)
	{
		int ret = cr_local::bindBlend(chId, blendchId, matric);
		UPDATE();
		return ret;
	}
	virtual int enableOSD(bool enable)
	{
		int ret = cr_local::enableOSD(enable);
		UPDATE();
		return ret;
	}
	virtual int enableEncoder(int chId, bool enable)
	{
		int ret = cr_local::enableEncoder(chId, enable);
		UPDATE();
		return ret;
	}
	virtual int setAxisPos(cv::Point pos)
	{
		int ret = cr_local::setAxisPos(pos);
		UPDATE();
		return ret;
	}
	virtual int saveAxisPos()
	{
		int ret = cr_local::saveAxisPos();
		UPDATE();
		return ret;
	}
	virtual int setTrackPosRef(cv::Point2f ref)
	{
		int ret = cr_local::setTrackPosRef(ref);
		UPDATE();
		return ret;
	}
	virtual int setTrackCoast(int nFrames)
	{
		int ret = cr_local::setTrackCoast(nFrames);
		UPDATE();
		return ret;
	}
	virtual int setTrackForce(bool enable)
	{
		int ret = cr_local::setTrackForce(enable);
		UPDATE();
		return ret;
	}
	virtual int setEZoomx(int value)
	{
		int ret = cr_local::setEZoomx(value);
		UPDATE();
		return ret;
	}
	virtual int setEZoomx(int chId, int value)
	{
		int ret = cr_local::setEZoomx(chId, value);
		UPDATE();
		return ret;
	}
	virtual int setWinPos(int winId, const cv::Rect& rc)
	{
		int ret = cr_local::setWinPos(winId, rc);
		UPDATE();
		return ret;
	}
	virtual int setWinMatric(int winId, const cv::Matx44f& matric)
	{
		int ret = cr_local::setWinMatric(winId, matric);
		UPDATE();
		return ret;
	}

	virtual int setOSDColor(int yuv, int thickness)
	{
		int ret = cr_local::setOSDColor(yuv, thickness);
		UPDATE();
		return ret;
	}
	virtual int setOSDColor(cv::Scalar color, int thickness)
	{
		int ret = cr_local::setOSDColor(color, thickness);
		UPDATE();
		return ret;
	}
	virtual int setEncTransLevel(int iLevel)
	{
		int ret = cr_local::setEncTransLevel(iLevel);
		UPDATE();
		return ret;
	}
	virtual int setHideSysOsd(bool bHideOSD)
	{
		int ret = cr_local::setHideSysOsd(bHideOSD);
		UPDATE();
		return ret;
	}
	virtual int setHideSysOsd(unsigned int mask)
	{
		int ret = cr_local::setHideSysOsd(mask);
		UPDATE();
		return ret;
	}
};
unsigned int Core_1001::ID = COREID_1001;
char Core_1001::m_version[16] = CORE_1001_VERSION_;
wchar_t Core_1001::sztest[128] = L"";

void Core_1001::update()
{
	//OSA_printf("%s %d: enter.", __func__, __LINE__);
	m_stats.mainChId = cr_local::curChannelFlag;
	m_stats.subChId = cr_local::curSubChannelIdFlag;
	m_stats.acqWinSize.width = cr_local::general->m_sizeAcqWin.width;
	m_stats.acqWinSize.height = cr_local::general->m_sizeAcqWin.height;
	m_stats.enableTrack = cr_local::enableTrackFlag;
	m_stats.enableMMTD = cr_local::enableMMTDFlag;
	m_stats.enableINTELL = cr_local::enableIntellFlag;
	m_stats.enableMotionDetect = cr_local::enableMotionDetectFlag;
	m_stats.enableBlob = cr_local::enableBlobFlag;
	m_stats.enableOSD = cr_local::enableOSDFlag;
	m_stats.iTrackorStat = cr_local::general->m_iTrackStat;
	UTC_RECT_float curRC = cr_local::general->m_rcTrk;
	m_stats.trackPos.x = curRC.x+curRC.width*0.5f;
	m_stats.trackPos.y = curRC.y+curRC.height*0.5f;
	m_stats.trackWinSize.width = curRC.width;
	m_stats.trackWinSize.height = curRC.height;
	m_stats.trackPosFilter.x = cr_local::general->m_rcTrkFlt.x+cr_local::general->m_rcTrkFlt.width*0.5f;
	m_stats.trackPosFilter.y = cr_local::general->m_rcTrkFlt.y+cr_local::general->m_rcTrkFlt.height*0.5f;
	m_stats.lossCoastFrames = cr_local::general->m_iTrackLostCnt;
	m_stats.lossCoastTelapse = cr_local::general->m_telapseLost;
	m_stats.subRc = cr_local::subRc;
	m_stats.subMatric = cr_local::subMatric;
	m_stats.colorYUV = cr_local::colorYUVFlag;
	m_stats.transLevel = cr_local::curTransLevel;
	m_stats.trackForceFlag = cr_local::trackForceFlag;
	m_stats.renderSched = cr_local::gluWindow->m_initPrm.disSched;
	if(cr_local::mmtd->m_bEnable || (cr_local::intell && cr_local::intell->m_bEnable)){
		int cnt = min(MAX_TGT_NUM, CORE_TGT_NUM_MAX);
		for(int i=0; i<cnt; i++)
		{
			m_stats.tgts[i].valid = cr_local::general->m_shTargets[i].valid;
			m_stats.tgts[i].index = i;
			m_stats.tgts[i].Box = cr_local::general->m_shTargets[i].Box;
			m_stats.tgts[i].pos = tRectCenter(m_stats.tgts[i].Box);
		}
	}else if(cr_local::motion && cr_local::motion->m_bEnable){
		int chId = cr_local::motion->m_curChId;
		int cnt = cr_local::motion->m_targets[chId].size();
		int i;
		cnt = min(cnt, CORE_TGT_NUM_MAX);
		for(i=0; i<cnt; i++)
		{
			m_stats.tgts[i].valid = true;
			m_stats.tgts[i].index = cr_local::motion->m_targets[chId][i].index;
			m_stats.tgts[i].Box = cr_local::motion->m_targets[chId][i].targetRect;
			m_stats.tgts[i].pos = tRectCenter(m_stats.tgts[i].Box);
		}
		for(;i<CORE_TGT_NUM_MAX; i++)
			m_stats.tgts[i].valid = false;
	}

	if(cr_local::blob){
		m_stats.blob.valid = cr_local::blob->m_bValid;
		m_stats.blob.pos = cr_local::blob->m_pos;
		m_stats.blob.Box = cr_local::blob->m_rc;
	}

	for(int chId = 0; chId < cr_local::nValidChannels; chId++){
		CORE1001_CHN_STATS *chn = &m_stats.chn[chId];
		chn->imgSize = cr_local::general->m_imgSize[chId];
		chn->fovId = cr_local::curFovIdFlag[chId];
		chn->axis.x = cr_local::general->m_AxisCalibX[chId][chn->fovId];
		chn->axis.y = cr_local::general->m_AxisCalibY[chId][chn->fovId];
		chn->enableEnh = cr_local::enableEnhFlag[chId];
		chn->iEZoomx = cr_local::ezoomxFlag[chId];
		chn->enableEncoder = cr_local::enableEncoderFlag[chId];//enctran->m_enable[chId];
		chn->frameTimestamp = cr_local::general->m_frameTimestamp[chId];
		chn->blendBindId = cr_local::bindBlendTFlag[chId];
		chn->blendMatric = cr_local::blendMatric[chId];
		chn->enableStabilizer = cr_local::enableStabFlag[chId];
	}

	for(int chId = 0; chId < cr_local::gluWindow->m_vvideos.size(); chId++){
		m_stats.frameTimestampRender[chId] = cr_local::gluWindow->m_vvideos[chId]->m_tmBak;
	}
}

static int rafcnt = 0;
static ICore *coreRaf = NULL;
ICore* ICore::Qury(int coreID)
{
	if(coreRaf != NULL){
		rafcnt ++;
		return coreRaf;
	}
	if(Core_1001::ID == coreID){
		coreRaf = new Core_1001;
		rafcnt =1;
	}

	return coreRaf;
}
void ICore::Release(ICore* core)
{
	if(core != NULL && coreRaf != NULL && core == coreRaf){
		rafcnt --;
		if(rafcnt == 0){
			delete coreRaf;
			coreRaf = NULL;
		}
	}
}

#include "gluVideoWindowSecondScreen.hpp"
extern CGluVideoWindowSecond *gSecondWindow;
void* ICore::QureyObj(int objId)
{
	void *ret = NULL;

	if(objId == 0)
		ret = cr_local::enctran;
	if(objId == 1)
		ret = cr_local::gluWindow;
	if(objId == 2)
		ret = gSecondWindow;
	if(objId == 3)
		ret = cr_local::proc;
	return ret;
}


