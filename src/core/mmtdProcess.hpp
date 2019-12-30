#if 1
/*
 * mmtdProcess.hpp
 *
 *  Created on: Sep 19, 2018
 *      Author: wzk
 */

#ifndef MMTDPROCESS_HPP_
#define MMTDPROCESS_HPP_

#include "processBase.hpp"
#include "MMTD.h"

typedef struct MMTD_config{
	int DetectGapparm;
	int MinArea;
	int MaxArea;
	int stillPixel;
	int movePixel;
	float lapScaler;
	int lumThred;
	int srModel;
	int meanThred1;
	int stdThred;
	int meanThred2;
	int sortType;
	int bClimitWH;
	int SalientSize;

}MMTD_CFG;

class CMMTDProcess : public CProcessBase
{
	CMMTD *m_mmtd;
	int m_curChId;
	int m_nCount;
	int m_nSelect;
	int m_nDrop;
	int m_fovId;
	int m_ezoomx;
	OSDU_Info m_units[MAX_CHAN][MAX_TGT_NUM];
	cv::Size m_imgSize[MAX_CHAN];
	int m_numMap[MAX_TGT_NUM];
	int m_curNumber;

	MMTD_CFG m_cfg[MAX_CHAN];

	cv::Rect m_roi;
	cv::Rect m_roiSet;
	void initDefaultConfig(MMTD_CFG& cfg);
	int ReadCfgMmtFromFile(MMTD_CFG& cfg, string cfgAvtFile);
	int setConfig(MMTD_CFG& cfg, const cv::Size& roi);
public :
	enum{
		VP_CFG_MMTDEnable=VP_CFG_MMTD_BASE,
		VP_CFG_MMTDTargetCount,
		VP_CFG_MMTDNewData,
		VP_CFG_MMTDQuit,
		VP_CFG_MMTDMax
	};
	CMMTDProcess(IProcess *proc = NULL);
	virtual ~CMMTDProcess();
	virtual int process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp);
	virtual int dynamic_config(int type, int iPrm, void* pPrm = NULL, int prmSize = 0);
	virtual int OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp);

	bool m_bEnable;

protected:

private:
	OSA_ThrHndl m_threadHdl;
	OSA_MsgqHndl m_msgQHdl;
	int thread_process();
	int privateProcess(int chId, int fovId, int ezoomx, Mat& frame, uint64_t timestamp);
	int private_dynamic_config(int type, int iPrm, void* pPrm);
	static void *entryFuncThread(void *context)
	{
		CMMTDProcess *process = (CMMTDProcess*)context;
		process->thread_process();
		return NULL;
	}

	struct privateProcessCtx{
		bool busy;
		int chId;
		int fovId;
		int ezoomx;
		uint64_t timestamp;
	};
	cv::Mat m_threadFrame;
	privateProcessCtx m_threadCtx;
};



#endif /* MMTDPROCESS_HPP_ */

#endif


