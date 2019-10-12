/*
 * processbase.hpp
 *
 *  Created on: May 5, 2017
 *      Author: ubuntu
 */

#ifndef PROCESS_BASE_HPP_
#define PROCESS_BASE_HPP_

#include "trackerProcess.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class CGeneralProc : public CTrackerProc
{
public:
	CGeneralProc(OSA_SemHndl *notifySem, IProcess *proc = NULL);
	virtual ~CGeneralProc();

	virtual void OnCreate();
	virtual void OnDestroy();
	virtual void OnInit();
	virtual void OnConfig(int type, int iPrm, void* pPrm);
	virtual void OnRun();
	virtual void OnStop();
	virtual void Ontimer();
	virtual bool OnPreProcess(int chId, Mat &frame);
	virtual bool OnProcess(int chId, Mat &frame);
	virtual int OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp);

	int WriteCalibAxisToFile();
	int ReadCalibAxisFromFile();
	int ReadCfgAvtFromFile();	

protected:
	void osd_cvdraw_trk(Mat &dc, IDirectOSD *osd, UTC_RECT_float rcTrack, int iStat, bool bShow = true);

	enum{
		U_WIN = 0,
		U_AXIS,
		U_MAX
	};

	int m_trkStatBak;
};

#endif /* PROCESSBASE_HPP_ */
