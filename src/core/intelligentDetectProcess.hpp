#if 1
/*
 * intelligentDetectProcess.hpp
 *
 *  Created on: Sep 19, 2018
 *      Author: wzk
 */

#ifndef INTELLPROCESS_HPP_
#define INTELLPROCESS_HPP_

#include <stdint.h>
#include "processBase.hpp"
#include "DetecterFactory.hpp"
#include "Detector.hpp"
#define MAX_INTELL_TGT_NUM  16

class CINTELLProcess : public CProcessBase
{
	int m_curChId;
	int m_nCount;
	int m_nSelect;
	int m_nDrop;
	int m_fovId;
	int m_ezoomx;
	OSDU_Info m_units[MAX_CHAN][MAX_INTELL_TGT_NUM];
	cv::Size m_imgSize[MAX_CHAN];
	int m_numMap[MAX_INTELL_TGT_NUM];
	int m_curNumber;

	cv::Rect m_roi;
	cv::Rect m_roiSet;
public :
	enum{
		VP_CFG_INTELLEnable=VP_CFG_INTELL_BASE,
		VP_CFG_INTELLTargetCount,
		VP_CFG_INTELLNewData,
		VP_CFG_INTELLQuit,
		VP_CFG_INTELLMax
	};
	CINTELLProcess(IProcess *proc = NULL);
	virtual ~CINTELLProcess();
	virtual int process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp);
	virtual int dynamic_config(int type, int iPrm, void* pPrm = NULL, int prmSize = 0);
	virtual int OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp);

	bool m_bEnable;
	Detector *m_detector;

protected:
	vector<string> m_classes;

	vector<BoundingBox> detectbox_;
	vector<BoundingBox> trackbox_;

private:
	OSA_ThrHndl m_threadHdl;
	OSA_MsgqHndl m_msgQHdl;
	int thread_process();
	int privateProcess(int chId, int fovId, int ezoomx, Mat& frame, uint64_t timestamp);
	int private_dynamic_config(int type, int iPrm, void* pPrm);

	void process2rgb(Mat &src,Mat &dst);

	static void trackcall(vector<BoundingBox>& trackbox,void *context,int chid );
	static void detectcall(vector<BoundingBox>& trackbox,void *context,int chid);
	static void *entryFuncThread(void *context)
	{
		CINTELLProcess *process = (CINTELLProcess*)context;
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
	 bool bProcessBusy ;
	 int processchid;
};

#endif /* INTELLPROCESS_HPP_ */

#endif


