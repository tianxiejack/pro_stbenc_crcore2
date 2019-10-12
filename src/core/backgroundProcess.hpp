/*
 * bkgdDetectProcess.hpp
 *
 *  Created on: Sep 19, 2018
 *      Author: wzk
 */
#if 1
#ifndef BKGDDETECTPROCESS_HPP_
#define BKGDDETECTPROCESS_HPP_

#include "processBase.hpp"

typedef	struct	 _bkgd_target{
	cv::Rect	targetRect;
}BKGD_TARGET;

class CBkgdDetectProcess : public CProcessBase
{
	int m_curChId;
	cv::Size m_imgSize[MAX_CHAN];
	float m_threshold;
	int m_mode;
	unsigned long m_cnt[MAX_CHAN];
	int ReadCfgFile();
	void detect(Mat frame, int chId);
public :
	enum{
		VP_CFG_BkgdDetectEnable=VP_CFG_BKGD_BASE,
		VP_CFG_Max
	};
	CBkgdDetectProcess(IProcess *proc = NULL);
	virtual ~CBkgdDetectProcess();
	virtual int process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp);
	virtual int dynamic_config(int type, int iPrm, void* pPrm = NULL, int prmSize = 0);
	virtual int OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp);

	bool m_bEnable;
	float m_alpha;
	float m_thr0Min;
	float m_thr0Max;
	float m_thr1Min;
	float m_thr1Max;
};



#endif /* BKGDDETECTPROCESS_HPP_ */
#endif
