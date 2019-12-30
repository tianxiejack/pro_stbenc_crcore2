/*
 * motionDetectProcess.hpp
 *
 *  Created on: Sep 19, 2018
 *      Author: wzk
 */
#if 1
#ifndef MOTIONDETECTPROCESS_HPP_
#define MOTIONDETECTPROCESS_HPP_

#include "processBase.hpp"
#include "mvdectInterface.hpp"

typedef std::vector<TRK_RECT_INFO> vMDTarget;
#define MAX_MOTION_TGT_NUM		(256)
class CMotionDetectProcess : public CProcessBase
{
	CMvDectInterface *m_inter;
	Rect m_roi;
	WARN_MODE m_curMode;
	int m_nCount;
	int m_fovId;
	int m_ezoomx;
	bool m_bOpen;
	OSDU_Info m_units[MAX_CHAN][MAX_MOTION_TGT_NUM];
	cv::Size m_imgSize[MAX_CHAN];
	int m_accuracy;
	int m_inputMinArea;
	int m_inputMaxArea;
	int m_threshold;
	unsigned long m_cnt[MAX_CHAN];
	int ReadCfgFile();
	static void notifyHdl(void *context, int chId){
		CMotionDetectProcess *pThis = (CMotionDetectProcess*)context;
		pThis->update(chId);
	};
	void update(int chId);
public :
	enum{
		VP_CFG_MONTIONEnable=VP_CFG_MONTION_BASE,
		VP_CFG_MONTIONTargetCount,
		VP_CFG_Max
	};
	CMotionDetectProcess(IProcess *proc = NULL);
	virtual ~CMotionDetectProcess();
	virtual int process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp);
	virtual int dynamic_config(int type, int iPrm, void* pPrm = NULL, int prmSize = 0);
	virtual int OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp);

	bool m_bEnable;
	int m_curChId;
	vMDTarget m_targets[MAX_CHAN];
};



#endif /* MOTIONDETECTPROCESS_HPP_ */
#endif
