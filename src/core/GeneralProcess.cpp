/*
 * processbase.cpp
 *
 *  Created on: May 5, 2017
 *      Author: ubuntu
 */
#include "GeneralProcess.hpp"
//#include "vmath.h"

#define CALIB_AXIS_FILE		"CalibAxisFile.yml"
#define CONFIG_AVT_FILE		"ConfigAvtFile.yml"

using namespace TRACKER_ALG;

PROC_TARGETINFO CProcessBase::m_shTargets[MAX_SHARE_TGT_NUM] = {};
PROC_TARGETINFO CProcessBase::m_shTrkTarget = {};

CGeneralProc::CGeneralProc(OSA_SemHndl *notifySem, IProcess *proc)
	:CTrackerProc(notifySem, proc),m_trkStatBak(-1)
{
}

CGeneralProc::~CGeneralProc()
{
}

void CGeneralProc::OnCreate(){}
void CGeneralProc::OnDestroy(){}
void CGeneralProc::OnInit()
{
	if(ReadCalibAxisFromFile() != 0)
	{
		for(int i=0; i<MAX_CHAN; i++){
			for(int j=0; j<MAX_NFOV_PER_CHAN; j++){
				if(m_AxisCalibX[i][j]<=0 || m_AxisCalibX[i][j]>1920)
				m_AxisCalibX[i][j] = 1920/2;
				if(m_AxisCalibY[i][j]<=0 || m_AxisCalibY[i][j]>1080)
				m_AxisCalibY[i][j] = 1080/2;
			}
		}
	}

	//if(ReadCfgAvtFromFile() != 0)
	{
		UTC_DYN_PARAM dynamicParam;
		dynamicParam.occlusion_thred = 0.28;
		dynamicParam.retry_acq_thred = 0.38;
		UtcSetDynParam(m_track, dynamicParam);
		TRK_SECH_RESTRAINT resTraint;
		resTraint.res_distance = 80;
		resTraint.res_area = 5000;
		UtcSetRestraint(m_track, resTraint);
		UtcSetUpFactor(m_track, 0.0175);
		UtcSetIntervalFrame(m_track, 3);
		UtcSetEnhance(m_track, false);
		UtcSetEnhfClip(m_track, 4.5);
		UtcSetBlurFilter(m_track, true);
		UtcSetBigSearch(m_track, true);
		UtcSetSceneMV(m_track, true);
		UtcSetTrajSechType(m_track, SCENE_MOVE_JUDGE);
		UtcSetSimilarDetect(m_track, true);
		UtcSetSechValid(m_track, false);
		UtcSetExtSearch(m_track, false);
		UtcSetPrintTS(m_track, true, true, true);
		UtcSetMultiModalDisp(m_track, false);
		UtcSetRoiMaxWidth(m_track, 200);
		UtcSetPLT_BS(m_track, tPLT_WRK, BoreSight_Mid);
	}
	ReadCfgAvtFromFile();
}
void CGeneralProc::OnConfig(int type, int iPrm, void* pPrm)
{
	switch(type)
	{
	case VP_CFG_SaveAxisToFile:
		WriteCalibAxisToFile();
		break;
	default:
		break;
	}
}
void CGeneralProc::OnRun()
{

}
void CGeneralProc::OnStop()
{

}
void CGeneralProc::Ontimer(){}
bool CGeneralProc::OnPreProcess(int chId, Mat &frame)
{
	return true;
}

__inline__ UTC_RECT_float tRectScale(UTC_RECT_float rc, cv::Size orgSize, cv::Size scaleSize)
{
	UTC_RECT_float ret;
	cv::Point2f fscale((float)scaleSize.width/orgSize.width, (float)scaleSize.height/orgSize.height);
	ret.x = scaleSize.width/2.0f - (orgSize.width/2.0f - rc.x)*fscale.x;
	ret.y = scaleSize.height/2.0f - (orgSize.height/2.0f - rc.y)*fscale.y;
	ret.width = rc.width*fscale.x;
	ret.height = rc.height*fscale.y;
	return ret;
}

int CGeneralProc::OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp)
{
	int ret = CProcessBase::OnOSD(chId, fovId, ezoomx, dc, osd, timestamp);
	float scalex = dc.cols/1920.0;
	float scaley = dc.rows/1080.0;

	int curChId = m_curChId;
	bool curTrack = m_bTrack;
	int curStat = m_iTrackStat;
	Point2f curAxis;
	curAxis.x = m_AxisCalibX[chId][fovId];
	curAxis.y = m_AxisCalibY[chId][fovId];
	Point2f tmpPoint = tPosScale(curAxis, m_imgSize[chId], (float)ezoomx);
	tmpPoint = tPosScale(tmpPoint, m_imgSize[chId], cv::Size(dc.cols, dc.rows));
	cv::Point axis((int)(tmpPoint.x+0.5), (int)(tmpPoint.y+0.5));
	UTC_RECT_float curRC = tRectScale(m_rcTrkFlt, m_imgSize[chId], cv::Size(dc.cols, dc.rows));//m_rcTrk;//tRectScale(m_rcTrack, m_imgSize[chId], (float)m_curEZoomx[chId]);

	if((curChId == chId && !m_bHide)){
		osd->cross(axis, cv::Size2f(scalex, scaley), 0);
	}
	if((curChId == chId && curTrack && !m_bHide)){
		osd_cvdraw_trk(dc, osd, curRC, curStat, true);
	}
#if 0
	if(curChId == chId){
		static unsigned int dbgCnt = 0;
		if(m_iTrackStat>0)
		{
			dbgCnt++;
			if(dbgCnt == 1){
				OSA_printf("%s %d: ch%d fov%d ezoomx%d, img(%d,%d) dc(%d,%d axsis(%f,%f)(%d,%d)",
						__func__, __LINE__, chId, fovId, ezoomx,
						m_imgSize[chId].width, m_imgSize[chId].height,dc.cols, dc.rows,
						curAxis.x, curAxis.y, axis.x, axis.y);
			}
		}else{
			dbgCnt = 0;
		}
	}
#endif

	//if(curChId == chId)
	//std::cout << "proc tm = " << m_frameTimestamp[chId] << " rend tm = " << timestamp << " delt = " << (m_frameTimestamp[chId]-timestamp) << std::endl;

	return ret;
}

void CGeneralProc::osd_cvdraw_trk(Mat &dc, IDirectOSD *osd, UTC_RECT_float rcTrack, int iStat, bool bShow)
{
	//UTC_RECT_float rcResult = rcTrack;

	if(rcTrack.width == 0 || rcTrack.height == 0)
        return ;

	m_trkStatBak = iStat;

	if(iStat == 0 || iStat == 1){
		//if(fabs(rcResult.x-rcTrack.x)<1.5 && fabs(rcResult.y-rcTrack.y)<1.5)
		//	osd->rectangle(cv::Rect(floor(rcResult.x), floor(rcResult.y), floor(rcResult.width), floor(rcResult.height)), !bShow);
		//else
			osd->rectangle(cv::Rect(floor(rcTrack.x), floor(rcTrack.y), floor(rcTrack.width), floor(rcTrack.height)), !bShow);
		//if(m_bForceTrackBak)
		//	osd->rectangle(cv::Rect(rcResult.x-3, rcResult.y-3, rcResult.width+6, rcResult.height+6), !bShow);
	}else{
		std::vector<cv::Point> vPts;
		vPts.resize(16);
		vPts[0].x = rcTrack.x;
		vPts[0].y = rcTrack.y;
		vPts[1].x = rcTrack.x+rcTrack.width/4;
		vPts[1].y = rcTrack.y;
		vPts[2].x = rcTrack.x;
		vPts[2].y = rcTrack.y;
		vPts[3].x = rcTrack.x;
		vPts[3].y = rcTrack.y+rcTrack.height/4;
		vPts[4].x = rcTrack.x+rcTrack.width*3/4;
		vPts[4].y = rcTrack.y;
		vPts[5].x = rcTrack.x+rcTrack.width;
		vPts[5].y = rcTrack.y;
		vPts[6].x = rcTrack.x+rcTrack.width;
		vPts[6].y = rcTrack.y;
		vPts[7].x = rcTrack.x+rcTrack.width;
		vPts[7].y = rcTrack.y+rcTrack.height/4;
		vPts[8].x = rcTrack.x;
		vPts[8].y = rcTrack.y+rcTrack.height*3/4;
		vPts[9].x = rcTrack.x;
		vPts[9].y = rcTrack.y+rcTrack.height;
		vPts[10].x = rcTrack.x;
		vPts[10].y = rcTrack.y+rcTrack.height;
		vPts[11].x = rcTrack.x+rcTrack.width/4;
		vPts[11].y = rcTrack.y+rcTrack.height;
		vPts[12].x = rcTrack.x+rcTrack.width*3/4;
		vPts[12].y = rcTrack.y+rcTrack.height;
		vPts[13].x = rcTrack.x+rcTrack.width;
		vPts[13].y = rcTrack.y+rcTrack.height;
		vPts[14].x = rcTrack.x+rcTrack.width;
		vPts[14].y = rcTrack.y+rcTrack.height*3/4;
		vPts[15].x = rcTrack.x+rcTrack.width;
		vPts[15].y = rcTrack.y+rcTrack.height;
		osd->line(vPts, !bShow);
	}
}

bool CGeneralProc::OnProcess(int chId, Mat &frame)
{
	//if(m_bTrack && m_iTrackStat == 2 && m_iTrackLostCnt > 30*5){
	//	m_bTrack = false;
	//	m_iTrackStat = 0;
	//}
	CTrackerProc::OnProcess(chId, frame);
	return true;
}

int CGeneralProc::WriteCalibAxisToFile()
{
	string CalibFile;
	CalibFile = CALIB_AXIS_FILE;
	char calib_x[64] = "calib_x";
	char calib_y[64] = "calib_y";

	FILE *fp = fopen(CalibFile.c_str(), "wt");
	if(fp !=NULL)
	{
		fseek(fp, 0, SEEK_END);
		int len = ftell(fp);
		fclose(fp);
		if(len == 0)
		{
			FileStorage fw(CalibFile,FileStorage::WRITE);
			if(fw.isOpened())
			{
				for(int i=0; i<MAX_CHAN; i++){
					for(int j=0; j<MAX_NFOV_PER_CHAN; j++){
						sprintf(calib_x, "calib_x_%d-%d", i,j);
						sprintf(calib_y, "calib_y_%d-%d", i,j);
						fw <<calib_x<< (float)m_AxisCalibX[i][j];
						fw <<calib_y<< (float)m_AxisCalibY[i][j];
					}
				}
			}
		}
	}

	FILE *fpp = fopen(CalibFile.c_str(), "rt");
	if(fpp !=NULL)
	{
		fseek(fpp, 0, SEEK_END);
		int len = ftell(fpp);
		fclose(fpp);
		if(len > 10)
			return 0;
	}else
		return -1;
}

int CGeneralProc::ReadCalibAxisFromFile()
{
	string CalibFile;
	CalibFile = CALIB_AXIS_FILE;

	char calib_x[64] = "calib_x";
	char calib_y[64] = "calib_y";

	FILE *fp = fopen(CalibFile.c_str(), "rt");
	if(fp != NULL)
	{
		fseek(fp, 0, SEEK_END);
		int len = ftell(fp);
		fclose(fp);

		if(len < 10)
			return -1;
		else
		{
			FileStorage fr(CalibFile, FileStorage::READ);
			if(fr.isOpened())
			{
				for(int i=0; i<MAX_CHAN; i++){
					for(int j=0; j<MAX_NFOV_PER_CHAN; j++){
						sprintf(calib_x, "calib_x_%d-%d", i,j);
						sprintf(calib_y, "calib_y_%d-%d", i,j);
						m_AxisCalibX[i][j] = (float)fr[calib_x];
						m_AxisCalibY[i][j] = (float)fr[calib_y];
					}
				}
				return 0;
			}else
				return -1;
		}
	}else
		return -1;
}

int CGeneralProc::ReadCfgAvtFromFile()
{
	string cfgAvtFile;
	cfgAvtFile = CONFIG_AVT_FILE;

	char cfg_avt[16] = "cfg_avt_";
    int configId_Max=128;
    float cfg_blk_val[128];

	FILE *fp = fopen(cfgAvtFile.c_str(), "rt");
	if(fp != NULL)
	{
		fseek(fp, 0, SEEK_END);
		int len = ftell(fp);
		fclose(fp);

		if(len < 10)
			return -1;
		else
		{
			FileStorage fr(cfgAvtFile, FileStorage::READ);
			if(fr.isOpened())
			{
				for(int i=0; i<configId_Max; i++){
					sprintf(cfg_avt, "cfg_avt_%d", i);
					FileNode rdStatus = fr[cfg_avt];
					if(!rdStatus.empty())
						cfg_blk_val[i] = (float)rdStatus;
					else
						cfg_blk_val[i] = -1.0;
					//printf(" update cfg [%d] %f \n", i, cfg_blk_val[i]);
				}
			}else
				return -1;
		}

		OSA_assert(m_track != NULL);

		UTC_DYN_PARAM dyPrm;
		if(cfg_blk_val[0] > 0)
			dyPrm.occlusion_thred = cfg_blk_val[0];
		else
			dyPrm.occlusion_thred = 0.32;
		if(cfg_blk_val[1] > 0)
			dyPrm.retry_acq_thred = cfg_blk_val[1];
		else
			dyPrm.retry_acq_thred = 0.38;
		UtcSetDynParam(m_track, dyPrm);

		float up_factor;
		if(cfg_blk_val[2] > 0)
			up_factor = cfg_blk_val[2];
		else
			up_factor = 0.0055;
		UtcSetUpFactor(m_track, up_factor);

		int interval=0;
		if(cfg_blk_val[3] > 0)
			interval = (int)cfg_blk_val[3];
		else
			interval = 10;
		UtcSetIntervalFrame(m_track,interval);

		int pltWork;
		int bsType;
		pltWork=(int)cfg_blk_val[4];
		bsType=(int)cfg_blk_val[5];
		if(pltWork>=0&&pltWork<=1)
			;
		else
			pltWork=1;
		if(bsType>=0&&bsType<=2)
			;
		else
			bsType=1;
		UtcSetPLT_BS(m_track,(tPLT)pltWork,(BS_Type)bsType);

		TRK_SECH_RESTRAINT resTraint;
		if(cfg_blk_val[6] > 0)
			resTraint.res_distance = (int)(cfg_blk_val[6]);
		else
			resTraint.res_distance = 80;
		if(cfg_blk_val[7] > 0)
			resTraint.res_area = (int)(cfg_blk_val[7]);
		else
			resTraint.res_area = 5000;
		UtcSetRestraint(m_track, resTraint);

		unsigned int enhenable=(int)(cfg_blk_val[8]);
		UtcSetEnhance(m_track, enhenable);

		float cliplimit=cfg_blk_val[9];
		if(cliplimit<=0)
			cliplimit=3.0;
		UtcSetEnhfClip(m_track,cliplimit);

		bool bEnable=(int)(cfg_blk_val[10]);
		UtcSetBlurFilter(m_track,bEnable);

		bEnable=(int)(cfg_blk_val[11]);
		UtcSetPredict(m_track,bEnable);

		int moveX,moveY;
		moveX = (int)cfg_blk_val[12];
		moveY = (int)cfg_blk_val[13];
		UtcSetMvPixel(m_track,moveX,moveY);

		int moveX2,moveY2;
		moveX2 = (int)cfg_blk_val[14];
		moveY2 = (int)cfg_blk_val[15];
		UtcSetMvPixel2(m_track,moveX2,moveY2);

		int segPixelX= (int)cfg_blk_val[16];
		int segPixelY= (int)cfg_blk_val[17];
		UtcSetSegPixelThred(m_track,segPixelX,segPixelY);

		int maxValue=(int)cfg_blk_val[18];
		UtcSetRoiMaxWidth(m_track,maxValue);

		bEnable=(int)(cfg_blk_val[19]);
		UtcSetBigSearch(m_track,bEnable);

		bEnable=(int)(cfg_blk_val[20]);
		UtcSetExtSearch(m_track,bEnable);

		bEnable=(int)(cfg_blk_val[21]);
		UtcSetSceneMV(m_track,bEnable);

		float trkAngleThred=cfg_blk_val[22];
		//UtcSetTrkAngleThred(m_track,trkAngleThred);

		bool bSearchAxis=(int)(cfg_blk_val[23]);
		bool bEstimateSearch=(int)(cfg_blk_val[24]);
		UtcSetSechAxis(m_track,bSearchAxis,bEstimateSearch);

		unsigned int trajSechType=(int )(cfg_blk_val[25]);
		if(trajSechType>4)
			trajSechType=1;
		UtcSetTrajSechType(m_track,(TRAJ_SECH_TYPE)trajSechType);

		bEnable=(int)(cfg_blk_val[26]);
		UtcSetSimilarDetect(m_track,bEnable);


		float panAng=cfg_blk_val[27];
		float tiltAng=cfg_blk_val[28];
		float panVel=cfg_blk_val[29];
		float tiltVel=cfg_blk_val[30];
		//UtcSetPlatPanTilt();

		int framefreq=(int)(cfg_blk_val[31]);
		UtcSetFrameFreq(m_track,framefreq);

		UKF_COV_PARAM ukparam;
		ukparam.measure_noise_cov_xy=cfg_blk_val[32];
		ukparam.proc_noise_cov_vxvy=cfg_blk_val[33];
		ukparam.proc_noise_cov_xy=cfg_blk_val[34];
		UtcSetUKFParam(m_track,ukparam);

		bEnable=(int)(cfg_blk_val[35]);
		UtcSetTrajAnalysis(m_track,bEnable);

		bEnable=(int)(cfg_blk_val[36]);
		UtcSetMultiModalDisp(m_track,bEnable);

		bEnable=(int)(cfg_blk_val[37]);
		printf("UtcSetSechValid the benable %d\n",bEnable);
		UtcSetSechValid(m_track,bEnable);

		 bool bTrkPrintf=(int)(cfg_blk_val[38]);
		 bool bScenePrintf=(int)(cfg_blk_val[39]);
		 bool bSimPrintf=(int)(cfg_blk_val[40]);
		UtcSetPrintTS(m_track,bTrkPrintf,bScenePrintf,bSimPrintf);

		bool bSimthread=(int)(cfg_blk_val[41]);

		UtcSetSimilarThredType(m_track,bSimthread);
		int secminarea=(int)(cfg_blk_val[42]);
		int secmaxarea=(int)(cfg_blk_val[43]);
		UtcSetDynamicTargetSize(m_track,secminarea,secmaxarea);
		
		bool bDynTargetDetect=(int)(cfg_blk_val[44]);
		UtcSetDynTargetDetect(m_track,bDynTargetDetect);

		m_lossCoastTelapseMax = (Uint32)cfg_blk_val[45];
		m_reTrkFliterFrameCnt = (Uint32)cfg_blk_val[46];
		OSA_printf("[General]%s %d: lossCoastTelapseMax = %d ms m_reTrkFliterFrameCnt = %d", __func__, __LINE__,
				m_lossCoastTelapseMax, m_reTrkFliterFrameCnt);

		return 0;

	}
	else
		return -1;
}



