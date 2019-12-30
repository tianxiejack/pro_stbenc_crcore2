/*
 * blobDetectProcess.hpp
 *
 *  Created on: Sep 19, 2018
 *      Author: wzk
 */
#if 1
#ifndef BLOBDETECTPROCESS_HPP_
#define BLOBDETECTPROCESS_HPP_

#include "processBase.hpp"

typedef	struct	 _blob_tgt_info{
	cv::RotatedRect						targetRect;
	int									index;
	int 								area;
	float 								ratioHW;
}BLOB_TGT_INFO;

typedef std::vector<BLOB_TGT_INFO> vBlobTarget;
#define MAX_BLOB_TGT_NUM		(16)
class CBlobDetectProcess : public CProcessBase
{
	int m_curChId;
	int m_nCount;
	int m_fovId;
	int m_ezoomx;
	OSDU_Info m_units[MAX_CHAN][MAX_BLOB_TGT_NUM];
	cv::Size m_imgSize[MAX_CHAN];
	float m_threshold;
	unsigned long m_cnt[MAX_CHAN];
	int ReadCfgFile();
	int detect(int chId, Mat frame);
	void update(int chId);
public :
	enum{
		VP_CFG_BLOBEnable=VP_CFG_BLOB_BASE,
		VP_CFG_BLOBTargetCount,
		VP_CFG_Max
	};
	CBlobDetectProcess(IProcess *proc = NULL);
	virtual ~CBlobDetectProcess();
	virtual int process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp);
	virtual int dynamic_config(int type, int iPrm, void* pPrm = NULL, int prmSize = 0);
	virtual int OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp);

	bool m_bEnable;
	vBlobTarget m_targets[MAX_CHAN];
	bool m_bValid;
	cv::Point2f m_pos;
	cv::Rect m_rc;

protected:
	cv::Size m_dSize;
	int m_minArea;
	int m_maxArea;
	float m_minRatioHW;
	float m_maxRatioHW;
};



#endif /* BLOBDETECTPROCESS_HPP_ */
#endif
