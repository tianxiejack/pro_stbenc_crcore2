/*
 * app_core.hpp
 *
 *  Created on: Dec 21, 2018
 *      Author: ubuntu
 */

#ifndef STI_CORE_HPP_
#define STI_CORE_HPP_

#include <crCore.hpp>


__inline__ bool isEqual(const cv::Rect& rc1, const cv::Rect& rc2)
{
	return (rc1.x == rc2.x && rc1.y==rc2.y && rc1.width==rc2.width && rc1.height ==  rc2.height);
}
__inline__ bool isEqual(const cv::Matx44f& m1, const cv::Matx44f& m2)
{
	return (memcmp(m1.val, m2.val, sizeof(float)*16) == 0);
}

struct sti_cmd_stats
{
	bool setAxisPos;
	bool saveAxisPos;
	bool setTrackPosRef;
};

typedef struct _STI_CORE
{
	int chksize;
	//main window video channel
	int mainChId;
	int fovId[CORE_CHN_MAX];
	int ezoomx[CORE_CHN_MAX];
	//sub window video channel
	int subChId;
	cv::Rect subRc;
	cv::Matx44f subMatric;
	//
	cv::Size acqWinSize;
	int enableTrack;
	int enableMMTD;
	int enableMotionDetect;
	int enableEnh[CORE_CHN_MAX];
	int enableBlob;
	int bindBlend[CORE_CHN_MAX];
	cv::Matx44f blendMatric[CORE_CHN_MAX];
	int enableEncoder[CORE_CHN_MAX];
	int enableOSD;
	int colorYUV;
	int transLevel;
	bool bFixSize;
	int iMMTDTrack;
	int nTargets;
	Rect2f *rcTrack;
	int ndrop;

	bool setAxisPos;
	cv::Point posAxis;
	bool saveAxisPos;
	bool setTrackPosRef;
	cv::Point2f posTrackRef;
	struct sti_cmd_stats cmdStats;
}STI_CORE;

static void STI_core_default(STI_CORE *stCore, const cv::Size& screenSize = cv::Size(1920,1080))
{
	OSA_assert(stCore != NULL);
	memset(stCore, 0, sizeof(STI_CORE));
	stCore->chksize = sizeof(STI_CORE);
	stCore->subChId = -1;
	stCore->enableOSD = true;
	stCore->colorYUV = 0x008080FF;
	stCore->transLevel = 1;
	stCore->subMatric = cv::Matx44f::eye();
	stCore->subRc.x = screenSize.width*2/3;
	stCore->subRc.y = screenSize.height*2/3;
	stCore->subRc.width = screenSize.width/3;
	stCore->subRc.height = screenSize.height/3;
	stCore->bFixSize = true;
	stCore->nTargets = 8;
	stCore->iMMTDTrack = -1;
	for(int chId=0; chId<CORE_CHN_MAX; chId++){
		stCore->ezoomx[chId] = 1;
		stCore->bindBlend[chId] = -1;
		stCore->blendMatric[chId] = cv::Matx44f::eye();
	}
}

static int STI_core_load(STI_CORE *stCore)
{
	int chId;
	ICore_1001 *core = (ICore_1001 *)ICore::Qury(COREID_1001);
	if(core == NULL)
		return OSA_EFAIL;
	CORE1001_STATS stats;
	memcpy(&stats, &core->m_stats, sizeof(stats));

	stCore->mainChId = stats.mainChId;
	for(chId=0; chId<CORE_CHN_MAX; chId++){
		stCore->fovId[chId] = stats.chn[chId].fovId;
		stCore->ezoomx[chId] = stats.chn[chId].iEZoomx;
		stCore->enableEnh[chId] = stats.chn[chId].enableEnh;
		stCore->enableEncoder[chId] = stats.chn[chId].enableEncoder;
		stCore->bindBlend[chId] = stats.chn[chId].blendBindId;
		stCore->blendMatric[chId] = stats.chn[chId].blendMatric;
	}
	stCore->acqWinSize = stats.acqWinSize;
	stCore->subChId = stats.subChId;
	stCore->subRc = stats.subRc;
	stCore->subMatric = stats.subMatric;
	stCore->enableTrack = stats.enableTrack;
	stCore->enableMMTD = stats.enableMMTD;
	stCore->enableMotionDetect = stats.enableMotionDetect;
	stCore->enableBlob = stats.enableBlob;
	stCore->enableOSD = stats.enableOSD;
	stCore->colorYUV = stats.colorYUV;
	stCore->transLevel = stats.transLevel;

	ICore::Release(core);

	return OSA_SOK;
}

static int STI_core_update(STI_CORE *stCore)
{

	ICore_1001 *core = (ICore_1001 *)ICore::Qury(COREID_1001);
	if(core == NULL)
		return OSA_EFAIL;
	CORE1001_STATS stats;
	memcpy(&stats, &core->m_stats, sizeof(stats));

	int curChId = stCore->mainChId;
	int curFovId = stCore->fovId[curChId];
	cv::Size winSize = stCore->acqWinSize;
	if(stats.mainChId != curChId || stats.chn[curChId].fovId != curFovId ||
			stats.acqWinSize.width != winSize.width || stats.acqWinSize.height != winSize.height
			|| stCore->ndrop>0)
	{
		OSA_printf("%s %d: setMainChId %d %d %d, (%d,%d)", __func__, __LINE__, curChId, curFovId, stCore->ndrop, winSize.width, winSize.height);
		core->setMainChId(curChId, curFovId, stCore->ndrop, winSize);
		stCore->ndrop = 0;
	}
	int curEZoomx = stCore->ezoomx[curChId];
	if(stats.chn[curChId].iEZoomx != curEZoomx)
	{
		OSA_printf("%s %d: setEZoomx %d, (%d)", __func__, __LINE__, curEZoomx, stats.chn[curChId].iEZoomx);
		core->setEZoomx(curEZoomx);
	}

	int subChId = stCore->subChId;
	if(stats.subChId != subChId){
		core->setSubChId(subChId);
	}
	if(stats.subChId>=0){
		if(!isEqual(stats.subRc,stCore->subRc))
			core->setWinPos(1, stCore->subRc);
		if(!isEqual(stats.subMatric,stCore->subMatric))
			core->setWinMatric(1, stCore->subMatric);
	}

	if(stats.enableTrack != stCore->enableTrack){
		OSA_printf("%s %d: enableTrack (%d,%d)", __func__, __LINE__, stats.enableTrack, stCore->enableTrack);
		if(stCore->enableTrack == false){
			OSA_printf("%s %d: enableTrack %d ,(%d,%d), %d", __func__, __LINE__, false, winSize.width, winSize.height, stCore->bFixSize);
			core->enableTrack(false, winSize, stCore->bFixSize);
		}else{
			if(stCore->rcTrack != NULL){
				OSA_printf("%s %d: enableTrack %d ,(%d,%d), %d", __func__, __LINE__, true, winSize.width, winSize.height, stCore->bFixSize);
				core->enableTrack(true, *stCore->rcTrack, stCore->bFixSize);
			}else if(stCore->iMMTDTrack>=0){
				OSA_printf("%s %d: enableTrackByMMTD %d ,(%d,%d), %d", __func__, __LINE__, stCore->iMMTDTrack, winSize.width, winSize.height, stCore->bFixSize);
				core->enableTrackByMMTD(stCore->iMMTDTrack, &winSize, stCore->bFixSize);
			}else
				core->enableTrack(true, winSize, stCore->bFixSize);
			stCore->iMMTDTrack = -1;
			stCore->rcTrack = NULL;
		}
	}
	else if(stCore->enableTrack && stCore->rcTrack != NULL)
	{
		core->enableTrack(true, *stCore->rcTrack, stCore->bFixSize);
		stCore->rcTrack = NULL;
	}

	if(stats.enableMMTD != stCore->enableMMTD)
		core->enableMMTD(stCore->enableMMTD, stCore->nTargets);

	if(stats.enableMotionDetect != stCore->enableMotionDetect)
		core->enableMotionDetect(stCore->enableMotionDetect);

	if(stats.enableBlob != stCore->enableBlob)
		core->enableBlob(stCore->enableBlob);

	if(stats.enableOSD != stCore->enableOSD)
		core->enableOSD(stCore->enableOSD);

	if(stats.colorYUV != stCore->colorYUV)
		core->setOSDColor(stCore->colorYUV);

#if 1
	for(int chId=0; chId<CORE_CHN_MAX; chId++){
		if(stats.chn[chId].enableEnh != stCore->enableEnh[chId])
			core->enableEnh(chId, stCore->enableEnh[chId]);
		if(stats.chn[chId].enableEncoder != stCore->enableEncoder[chId])
			core->enableEncoder(chId, stCore->enableEncoder[chId]);
		if(stats.chn[chId].blendBindId != stCore->bindBlend[chId])
			core->bindBlend(chId, stCore->bindBlend[chId], stCore->blendMatric[chId]);
	}

	if(stats.transLevel != stCore->transLevel)
		core->setEncTransLevel(stCore->transLevel);

	if(stCore->setAxisPos)
		core->setAxisPos(stCore->posAxis);
	stCore->cmdStats.setAxisPos = stCore->setAxisPos;

	if(stCore->saveAxisPos != stCore->cmdStats.saveAxisPos){
		stCore->cmdStats.saveAxisPos = stCore->saveAxisPos;
		if(stCore->saveAxisPos)
			core->saveAxisPos();
	}

	if(stCore->setTrackPosRef)
		core->setTrackPosRef(stCore->posTrackRef);
	stCore->cmdStats.setTrackPosRef = stCore->setTrackPosRef;

#endif

	ICore::Release(core);

	return OSA_SOK;
}

#endif /* STI_CORE_HPP_ */

