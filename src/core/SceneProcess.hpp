/*
 * bkgdDetectProcess.hpp
 *
 *  Created on: Sep 19, 2018
 *      Author: wzk
 */
#if 1
#ifndef SCENEPROCESS_HPP_
#define SCENEPROCESS_HPP_

#include "processBase.hpp"
#include "sceneProc.hpp"
#include "crosd.hpp"

class CSceneProcess : public CProcessBase
{
	SceneProc m_obj;
	int m_curChId;
	cv::Size m_imgSize[MAX_CHAN];
	unsigned long m_cnt[MAX_CHAN];
	std::vector<float> vArrayOrg;
	std::vector<float> vArrayFilter;
	cr_osd::IPattern* patOrg;
	cr_osd::IPattern* patFilter;
	int ReadCfgFile();
	void detect(const Mat& frame, int chId);
public :
	enum{
		VP_CFG_SceneEnable=VP_CFG_SCENE_BASE,
		VP_CFG_Max
	};
	CSceneProcess(IProcess *proc = NULL);
	virtual ~CSceneProcess();
	virtual int process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp);
	virtual int dynamic_config(int type, int iPrm, void* pPrm = NULL, int prmSize = 0);
	virtual int OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp);

	bool m_bEnable;
};



#endif /* SCENEPROCESS_HPP_ */
#endif
