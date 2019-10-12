/*
 * CTrackerProc.cpp
 *
 *  Created on: 2017
 *      Author: sh
 */
#include "trackerProcess.hpp"
//#include "vmath.h"
//#include "arm_neon.h"

using namespace TRACKER_ALG;
//using namespace vmath;

//extern void cutColor(cv::Mat src, cv::Mat &dst, int code);

UTCTRACK_HANDLE g_track = NULL;

static int OSA_thrCreate_sched(OSA_ThrHndl *hndl, OSA_ThrEntryFunc entryFunc, Uint32 pri, Uint32 stackSize, void *prm)
{
  int status=OSA_SOK;
  pthread_attr_t thread_attr;
  struct sched_param schedprm;


  // initialize thread attributes structure
  status = pthread_attr_init(&thread_attr);

  if(status!=OSA_SOK) {
    OSA_ERROR("OSA_thrCreate() - Could not initialize thread attributes\n");
    return status;
  }

  if(stackSize!=OSA_THR_STACK_SIZE_DEFAULT)
    pthread_attr_setstacksize(&thread_attr, stackSize);

  status |= pthread_attr_setinheritsched(&thread_attr, PTHREAD_EXPLICIT_SCHED);

  status |= pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);
  //status |= pthread_attr_setschedpolicy(&thread_attr, SCHED_OTHER);

  if(pri>OSA_THR_PRI_MAX)
    pri=OSA_THR_PRI_MAX;
  else
  if(pri<OSA_THR_PRI_MIN)
    pri=OSA_THR_PRI_MIN;

  schedprm.sched_priority = pri;

  status |= pthread_attr_setschedparam(&thread_attr, &schedprm);

  if(status!=OSA_SOK) {
    OSA_ERROR("OSA_thrCreate() - Could not initialize thread attributes\n");
    goto error_exit;
  }

  status = pthread_create(&hndl->hndl, &thread_attr, entryFunc, prm);

  if(status != OSA_SOK) {
    OSA_ERROR("OSA_thrCreate() - Could not create thread [%d] %s\n", status, strerror(status));
    //OSA_assert(status == OSA_SOK);
  }else{
	  OSA_printf("%s %d: priority = %d(%d-%d)\n", __FILE__, __LINE__, pri, OSA_THR_PRI_MIN,OSA_THR_PRI_MAX);
  }

error_exit:
  pthread_attr_destroy(&thread_attr);

  return status;
}

int CTrackerProc::MAIN_threadCreate(void)
{
	int iRet = OSA_SOK;
	//iRet = OSA_semCreate(&mainProcThrObj.procNotifySem ,1,0) ;
	OSA_assert(iRet == OSA_SOK);
	iRet = OSA_msgqCreate(&mainProcThrObj.hMsgQ);
	OSA_assert(iRet == OSA_SOK);
	iRet = OSA_msgqCreate(&mainProcThrObj.hMsgQAck);
	OSA_assert(iRet == OSA_SOK);
	iRet = OSA_mutexCreate(&mainProcThrObj.m_mutex);
	OSA_assert(iRet == OSA_SOK);

	mainProcThrObj.exitProcThread = false;

	mainProcThrObj.initFlag = true;

	mainProcThrObj.pParent = (void*)this;

	m_mainMem[0]= new unsigned char[1080*1920];
	m_mainMem[1]= new unsigned char[1080*1920];

	//iRet = OSA_thrCreate_sched(&mainProcThrObj.thrHandleProc, mainProcTsk, OSA_THR_PRI_DEFAULT, OSA_THR_STACK_SIZE_DEFAULT, &mainProcThrObj);
	//if(iRet != OSA_SOK)
		iRet = OSA_thrCreate(&mainProcThrObj.thrHandleProc, mainProcTsk, OSA_THR_PRI_DEFAULT, OSA_THR_STACK_SIZE_DEFAULT, &mainProcThrObj);

	return iRet;
}

void CTrackerProc::printfInfo(Mat mframe,UTC_RECT_float &result,bool show)
{
	char strFrame[128];
	cv::Scalar RBG = show?cvScalar(255,255,0,255):cvScalar(0,0,0,0);
	
	sprintf(strFrame, "resultXY=(%0.1f,%0.1f) ", result.x, result.y) ;
	putText(mframe,  strFrame, Point(80, 80),FONT_HERSHEY_DUPLEX, 1, RBG,1);

}

static void extractYUYV2Gray(Mat src, Mat dst)
{
	int ImgHeight, ImgWidth;

	ImgWidth = src.cols;
	ImgHeight = src.rows;
	uint8_t  *  pDst8_t;
	uint8_t *  pSrc8_t;

	const int stap = (ImgHeight*ImgWidth);
	pSrc8_t = (uint8_t*)(src.data);
	pDst8_t = (uint8_t*)(dst.data);

	for(int y = 0; y < stap; y++)
	{
		pDst8_t[y] = pSrc8_t[y*2];
	}
}

int CTrackerProc::process(int chId, int fovId, int ezoomx, Mat frame, Mat frameGray, uint64_t timestamp)
{
	static int ppBak = 1;
	if(frame.cols<=0 || frame.rows<=0)
		return 0;

	m_imgSize[chId].width = frame.cols;
	m_imgSize[chId].height = frame.rows;

//	tstart = getTickCount();
	//if(chId == m_curChId)
	{
		if(!OnPreProcess(chId, frame)){
			return 0;
		}

		OSA_mutexLock(&mainProcThrObj.m_mutex);
		int pp;
		pp = mainProcThrObj.pp;
		if(ppBak != pp)
		{
			int channel = frame.channels();
			//OSA_printf("%s: %d ch%d frame %dx%d", __func__, __LINE__, chId, frame.cols, frame.rows);
			Mat frame_gray = Mat(frame.rows, frame.cols, CV_8UC1, m_mainMem[pp]);
			//OSA_printf("%s: %d ", __func__, __LINE__);
			if(channel == 2)
			{
				//OSA_printf("%s %d:[%d] %p", __func__, __LINE__, OSA_getCurTimeInMsec(), frame.data);
				OSA_assert(frame.data != NULL && frame_gray.data != NULL);
				extractYUYV2Gray(frame, frame_gray);
				//OSA_printf("%s %d:[%d] ", __func__, __LINE__, OSA_getCurTimeInMsec());
			}
			else if(channel == 3){
				cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
			}else{
				memcpy(frame_gray.data, frame.data, frame.cols*frame.rows*channel*sizeof(unsigned char));
			}
			mainProcThrObj.cxt[pp].chId = chId;
			mainProcThrObj.cxt[pp].fovId = fovId;
			mainProcThrObj.cxt[pp].ezoomx = ezoomx;
			mainProcThrObj.cxt[pp].timestamp = timestamp;
			mainFrame[pp] = frame;
			mainFramegray[pp] = frame_gray;
			//OSA_printf("%s %d: ch%d(%d) fov%d(%d)", __func__, __LINE__, chId, m_curChId, fovId, m_curFovId[m_curChId]);
			OSA_msgqSendMsg(&mainProcThrObj.hMsgQ, NULL, VP_CFG_NewData, NULL, 0, NULL);
			ppBak = mainProcThrObj.pp;
		}else{
			//OSA_printf("WARN! %s %d timeout", __func__, __LINE__);
		}
		OSA_mutexUnlock(&mainProcThrObj.m_mutex);
	}

	//OnOSD(chId, fovId, ezoomx, m_dc[chId], m_vosds[chId]);
	//	OSA_printf("process_frame: chId = %d, time = %f sec \n",chId,  ( (getTickCount() - tstart)/getTickFrequency()) );

	return 0;
}

void CTrackerProc::main_proc_func()
{
	int ret = OSA_SOK;
	OSA_MsgHndl msgRcv;
	//OSA_printf("%s: Main Proc Tsk Is Entering..[%d].\n",__func__, mainProcThrObj.exitProcThread);
	unsigned int framecount=0;
	int istatBak = -1;

	while(mainProcThrObj.exitProcThread ==  false)
	{
		/******************************************/
		//OSA_semWait(&mainProcThrObj.procNotifySem, OSA_TIMEOUT_FOREVER);
		ret = OSA_msgqRecvMsgEx(&mainProcThrObj.hMsgQ, &msgRcv, OSA_TIMEOUT_FOREVER);
		OSA_assert(ret == OSA_SOK);

		if(msgRcv.cmd == VP_CFG_Quit){
			OSA_printf("%s %d: ... break", __func__, __LINE__);
			break;
		}
		if(msgRcv.cmd != VP_CFG_NewData){
			void *pPrm = (msgRcv.pPrm == NULL) ? NULL : ((unsigned char*)msgRcv.pPrm+sizeof(int));
			int prmSize = (msgRcv.pPrm == NULL) ? 0 : (*(int*)msgRcv.pPrm);
			dynamic_config_(msgRcv.cmd, msgRcv.flags, pPrm, prmSize);
			if(msgRcv.pPrm != NULL)
				OSA_memFree(msgRcv.pPrm);
			if(m_usrNotifySem!=NULL)
				OSA_semSignal(m_usrNotifySem);
			ret = OSA_msgqSendMsg(&mainProcThrObj.hMsgQAck, NULL, 0, NULL, 0, NULL);
			OSA_assert(ret == OSA_SOK);
			continue;
		}

		//OSA_printf("%s:pp = %d ,\n",__func__, mainProcThrObj.pp);
		OSA_mutexLock(&mainProcThrObj.m_mutex);
		int pp = mainProcThrObj.pp;
		mainProcThrObj.pp ^=1;
		OSA_mutexUnlock(&mainProcThrObj.m_mutex);
		Mat frame = mainFrame[pp];
		Mat frame_gray = mainFramegray[pp];
		int chId = mainProcThrObj.cxt[pp].chId;
		int fovId = mainProcThrObj.cxt[pp].fovId;
		int ezoomx = mainProcThrObj.cxt[pp].ezoomx;
		uint64_t timestamp = mainProcThrObj.cxt[pp].timestamp;

		CProcessBase::process(chId, fovId, ezoomx, frame, frame_gray, timestamp);

		int iTrackStat = ReAcqTarget();
		m_curEZoomx[chId] = ezoomx;
		if(!m_bTrack || chId != m_curChId || fovId != m_curFovId[m_curChId]){
			m_frameTimestamp[chId] = timestamp;
			OnProcess(chId, frame);
			continue;
		}

		/******************************************/
		if(iTrackStat >= 0)
		{
			UTC_RECT_float tmpRc;
			if(iTrackStat == 0 || iTrackStat == 3)
				tmpRc = m_rcAcq;
			else
				tmpRc = m_rcTrackSelf;
			m_iTrackStat = process_track(iTrackStat, frame_gray, frame_gray, tmpRc);
			if(m_bFixSize){
				m_rcTrackSelf.x = tmpRc.x+(tmpRc.width-m_rcAcq.width)/2;
				m_rcTrackSelf.y = tmpRc.y+(tmpRc.height-m_rcAcq.height)/2;
				m_rcTrackSelf.width = m_rcAcq.width;
				m_rcTrackSelf.height = m_rcAcq.height;
			}else{
				m_rcTrackSelf = tmpRc;
			}
		}
		else
		{
			m_iTrackStat = 2;
			m_rcTrackSelf = m_rcAcq;
		}
		m_rcTrk = tRectScale(m_rcTrackSelf, m_imgSize[chId], (float)m_curEZoomx[chId]);

		if(m_iTrackStat > 1){
			m_iTrackLostCnt++;
			if(m_lostTimestamp == 0ul)
				m_lostTimestamp = timestamp;
			m_telapseLost = (Uint32)((timestamp - m_lostTimestamp)*0.000001);
			//OSA_printf("m_iTrackLostCnt=%d m_telapseLost=%d m_lossCoastTelapseMax=%d", m_iTrackLostCnt, m_telapseLost, m_lossCoastTelapseMax);
			if(m_lossCoastTelapseMax>0 && m_telapseLost>=m_lossCoastTelapseMax){
				m_iTrackStat = 3;
			}
		}else{
			m_iTrackLostCnt = 0;
			m_lostTimestamp = 0ul;
			m_telapseLost = 0;
		}

		UTC_RECT_float rcFlt = m_rcTrackSelf;
		if(m_filter != NULL){
			double dbM[3];
	        dbM[0] = 0.0f;
	        dbM[1] = m_rcTrackSelf.x+m_rcTrackSelf.width*0.5;
	        dbM[2] = m_rcTrackSelf.y+m_rcTrackSelf.height*0.5;
			if(m_iTrackStat == 0 || (m_iTrackStat != istatBak && m_iTrackStat == 1)){
				m_filter->KalmanOpen(6, 3, 0);
				m_filter->KalmanInitParam(dbM[0], dbM[1], dbM[2], 0.0);
			}else if(m_iTrackStat == 1){
				m_filter->Kalman(dbM, NULL);
				rcFlt.x = (float)m_filter->m_hKalman->state_post[2]-m_rcTrackSelf.width*0.5;
				rcFlt.y = (float)m_filter->m_hKalman->state_post[4]-m_rcTrackSelf.height*0.5;
				if(fabs(rcFlt.x-m_rcTrackSelf.x)>2.5 || fabs(rcFlt.y-m_rcTrackSelf.y)>3.0)
					rcFlt = m_rcTrackSelf;
			}
		}
		m_rcTrkFlt = tRectScale(rcFlt, m_imgSize[chId], (float)m_curEZoomx[chId]);

		if(istatBak != m_iTrackStat){
			istatBak = m_iTrackStat;
			printf("\n[%d] %s %d: stat = %d\n", OSA_getCurTimeInMsec(), __func__, __LINE__, m_iTrackStat);
		}

		m_frameTimestamp[chId] = timestamp;

		OnProcess(chId, frame);
		framecount++;

	/************************* while ********************************/
	}
	OSA_printf("%s: Main Proc Tsk Is Exit.\n",__func__);
}

int CTrackerProc::MAIN_threadDestroy(void)
{
	int iRet = OSA_SOK;

	OSA_mutexLock(&mainProcThrObj.m_mutex);

	mainProcThrObj.exitProcThread = true;
	//OSA_semSignal(&mainProcThrObj.procNotifySem);
	OSA_msgqSendMsg(&mainProcThrObj.hMsgQ, NULL, VP_CFG_Quit, NULL, 0, NULL);

	iRet = OSA_thrDelete(&mainProcThrObj.thrHandleProc);

	mainProcThrObj.initFlag = false;
	//OSA_semDelete(&mainProcThrObj.procNotifySem);
	OSA_msgqDelete(&mainProcThrObj.hMsgQ);
	OSA_msgqDelete(&mainProcThrObj.hMsgQAck);
	OSA_mutexDelete(&mainProcThrObj.m_mutex);
	delete [] m_mainMem[0];
	delete [] m_mainMem[1];

	return iRet;
}

CTrackerProc::CTrackerProc(OSA_SemHndl *notifySem, IProcess *proc)
	:CProcessBase(proc, VP_IDENTIFY_TRK),m_track(NULL),m_curChId(0),m_usrNotifySem(notifySem),
	 m_bFixSize(false),m_lostTimestamp(0ul),m_telapseLost(0),m_lossCoastTelapseMax(0),m_reTrkFliterFrameCnt(0),
	 m_bForceTrack(false),m_bForceTrackBak(false)
{
	memset(&mainProcThrObj, 0, sizeof(MAIN_ProcThrObj));
	memset(&m_rcTrk, 0, sizeof(m_rcTrk));
	memset(&m_rcTrkFlt, 0, sizeof(m_rcTrkFlt));
	memset(m_mainMem, 0, sizeof(m_mainMem));
	memset(m_curFovId, 0, sizeof(m_curFovId));
	memset(m_vosds, 0, sizeof(m_vosds));
	m_bTrack = false;
	m_iTrackStat = 0;
	m_iTrackLostCnt = 0;
	m_sizeAcqWin.width = 80;
	m_sizeAcqWin.height = 60;
	m_axis.x=960;
	m_axis.y=540;
	m_intervalFrame = 0;
	m_reTrackStat = 0;
	rcMoveX = 0;
	rcMoveY = 0;

	algOsdRect = false;

	memset(m_AxisCalibX, 0, sizeof(m_AxisCalibX));
	memset(m_AxisCalibY, 0, sizeof(m_AxisCalibY));

	for(int i=0; i<MAX_CHAN; i++){
		m_curEZoomx[i] = 1;
		m_imgSize[i].width = 1920;
		m_imgSize[i].height = 1080;
		for(int j=0; j<MAX_NFOV_PER_CHAN; j++){
			m_AxisCalibX[i][j] = m_imgSize[i].width/2;
			m_AxisCalibY[i][j] = m_imgSize[i].height/2;
		}
		m_frameTimestamp[i] = 0l;
	}

	m_filter = new crcore::CKalmanFilter();
}

CTrackerProc::~CTrackerProc()
{
	destroy();
}

int CTrackerProc::creat()
{
	int i = 0;

	trackinfo_obj=(Track_InfoObj *)malloc(sizeof(Track_InfoObj));

	MAIN_threadCreate();

	m_track = CreateUtcTrk();
	OSA_assert(m_track);
	g_track = m_track;

	OnCreate();

	return 0;
}

int CTrackerProc::destroy()
{
	stop();

	MAIN_threadDestroy();

	OnDestroy();

	if(m_filter != NULL)
		delete m_filter;
	m_filter = NULL;


	if(m_track != NULL)
		DestroyUtcTrk(m_track);
	m_track = NULL;
	g_track = NULL;

	return 0;
}

int CTrackerProc::init()
{
	OnInit();
	for(int i=0; i<MAX_CHAN; i++){
		for(int j=0; j<MAX_NFOV_PER_CHAN; j++){
			if(m_AxisCalibX[i][j]<=0 || m_AxisCalibX[i][j]>1920)
			m_AxisCalibX[i][j] = 1920/2;
			if(m_AxisCalibY[i][j]<=0 || m_AxisCalibY[i][j]>1080)
			m_AxisCalibY[i][j] = 1080/2;
		}
	}
	m_axis.x = m_AxisCalibX[m_curChId][m_curFovId[m_curChId]];
	m_axis.y = m_AxisCalibY[m_curChId][m_curFovId[m_curChId]];

	return 0;
}

int CTrackerProc::dynamic_config(int type, int iPrm, void* pPrm, int prmSize)
{
	unsigned char *memPrm = NULL;

	//OSA_printf("%s %i: type %d iPrm %d pPrm = %p prmSize = %d", __func__, __LINE__,
	//		type, iPrm, pPrm, prmSize);
	if(prmSize>0 && pPrm!=NULL){
		memPrm = (unsigned char*)OSA_memAlloc(prmSize+sizeof(int));
		OSA_assert(memPrm != NULL);
		memcpy(memPrm+sizeof(int), pPrm, prmSize);
		*(int*)memPrm = prmSize;
	}
	int iRet = OSA_msgqSendMsg(&mainProcThrObj.hMsgQ, NULL, type, memPrm, iPrm, NULL);

	OnConfig(type, iPrm, pPrm);

	OSA_MsgHndl msgAck;
	iRet = OSA_msgqRecvMsgEx(&mainProcThrObj.hMsgQAck, &msgAck, OSA_TIMEOUT_FOREVER);
	OSA_assert(iRet == OSA_SOK);

	return iRet;
}

int CTrackerProc::dynamic_config_(int type, int iPrm, void* pPrm, int prmSize)
{
	int iret = OSA_SOK;
	int bakChId;
	float fwidth, fheight;

	iret = CProcessBase::dynamic_config(type, iPrm, pPrm, prmSize);

	if(type<VP_CFG_BASE || type>VP_CFG_Max)
		return iret;

	switch(type)
	{
	case VP_CFG_MainChId:
		bakChId = m_curChId;
		m_curChId = iPrm;
		//if(m_curChId != bakChId)
		//	OnOSD(bakChId, m_curFovId[bakChId], m_curEZoomx[bakChId], m_dc[bakChId], m_vosds[bakChId]);
		m_iTrackStat = 0;
		m_iTrackLostCnt = 0;
		if(pPrm != NULL){
			VPCFG_MainChPrm *mchPrm = (VPCFG_MainChPrm*)pPrm;
			m_intervalFrame = mchPrm->iIntervalFrames;
			m_reTrackStat = 0;
			if(mchPrm->fovId>=0 && mchPrm->fovId<MAX_NFOV_PER_CHAN)
				m_curFovId[m_curChId] = mchPrm->fovId;
		}else{
			m_intervalFrame = 0;
			m_reTrackStat = 0;
		}
		m_axis.x = m_AxisCalibX[m_curChId][m_curFovId[m_curChId]];
		m_axis.y = m_AxisCalibY[m_curChId][m_curFovId[m_curChId]];
		//fwidth = (float)m_imgSize[m_curChId].width;
		//fheight = (float)m_imgSize[m_curChId].height;
		//m_axis.x = fwidth/2.0f - (fwidth/2.0f - m_axis.x)*(float)m_curEZoomx;
		//m_axis.y = fheight/2.0f - (fheight/2.0f - m_axis.y)*(float)m_curEZoomx;
		update_acqRc();
		OSA_printf("%s VP_CFG_MainChId: curCh%d -%d m_rcAcq(%.1f %.1f %.1f %.1f)",__func__,m_curChId,m_curFovId[m_curChId],m_rcAcq.x, m_rcAcq.y, m_rcAcq.width, m_rcAcq.height);
		break;
	case VP_CFG_MainFov:
		if(iPrm>=0 && iPrm<MAX_NFOV_PER_CHAN){
			m_curFovId[m_curChId] = iPrm;
			m_iTrackStat = 0;
			m_iTrackLostCnt = 0;
			m_intervalFrame = (pPrm == NULL) ? 1: *(int*)pPrm;
			m_reTrackStat = 0;
			m_axis.x = m_AxisCalibX[m_curChId][m_curFovId[m_curChId]];
			m_axis.y = m_AxisCalibY[m_curChId][m_curFovId[m_curChId]];
			//fwidth = (float)m_imgSize[m_curChId].width;
			//fheight = (float)m_imgSize[m_curChId].height;
			//m_axis.x = fwidth/2.0f - (fwidth/2.0f - m_axis.x)*(float)m_curEZoomx;
			//m_axis.y = fheight/2.0f - (fheight/2.0f - m_axis.y)*(float)m_curEZoomx;
			update_acqRc();
			OSA_printf("%s VP_CFG_MainFov: curCh%d -%d m_rcAcq(%.1f %.1f %.1f %.1f)",__func__,m_curChId,m_curFovId[m_curChId],m_rcAcq.x, m_rcAcq.y, m_rcAcq.width, m_rcAcq.height);
		}
		break;
	case VP_CFG_Axis:
		{
			cv::Point2f *axisPrm = (cv::Point2f*)pPrm;
			m_axis.x = axisPrm->x;
			m_axis.y = axisPrm->y;
			update_acqRc();
			OSA_printf("%s VP_CFG_Axis: curCh%d -%d m_rcAcq(%.1f %.1f %.1f %.1f)",__func__,m_curChId,m_curFovId[m_curChId],m_rcAcq.x, m_rcAcq.y, m_rcAcq.width, m_rcAcq.height);
		}
		break;
	case VP_CFG_SaveAxisToArray:
		m_AxisCalibX[m_curChId][m_curFovId[m_curChId]] = m_axis.x;
		m_AxisCalibY[m_curChId][m_curFovId[m_curChId]] = m_axis.y;
		break;
	case VP_CFG_AcqWinSize:
		if(pPrm != NULL){
			memcpy(&m_sizeAcqWin, pPrm, sizeof(m_sizeAcqWin));
			//OSA_printf("%s: m_sizeAcqWin(%d,%d)", __func__, m_sizeAcqWin.width, m_sizeAcqWin.height);
			update_acqRc();
			m_iTrackStat = 0;
			m_iTrackLostCnt = 0;
			m_intervalFrame = 0;
			m_reTrackStat = 0;
		}
		m_bFixSize = iPrm;
		OSA_printf("%s VP_CFG_AcqWinSize: curCh%d -%d m_rcAcq(%.1f %.1f %.1f %.1f)",__func__,m_curChId,m_curFovId[m_curChId],m_rcAcq.x, m_rcAcq.y, m_rcAcq.width, m_rcAcq.height);
		break;
	case VP_CFG_TrkEnable:
		if(pPrm == NULL)
		{
			update_acqRc();
		}
		else
		{
			UTC_RECT_float urTmp;
			memcpy(&urTmp, pPrm, sizeof(urTmp));
			m_rcAcq = tRectScale(urTmp, m_imgSize[m_curChId], 1.0/m_curEZoomx[m_curChId]);
			m_rcTrackSelf = urTmp;
			OSA_printf(" m_rcAcq update by assign %f %f %f %f\n", m_rcAcq.x, m_rcAcq.y, m_rcAcq.width, m_rcAcq.height);
		}

		if(iPrm == 0)		// exit trk
		{
			m_bTrack = false;
			m_iTrackStat = 0;
			m_iTrackLostCnt = 0;
			m_intervalFrame = 0;
			m_reTrackStat = 0;
			m_rcTrackSelf.width = 0;
			m_rcTrackSelf.height = 0;
			m_rcTrackSelf.x = m_axis.x;
			m_rcTrackSelf.y = m_axis.y;
		}
		else				// start or reset trk
		{
			m_iTrackStat = 0;
			m_iTrackLostCnt = 0;
			m_intervalFrame = 0;
			m_reTrackStat = 0;
			m_bTrack = true;
		}
		OSA_printf("%s VP_CFG_TrkEnable: curCh%d -%d m_rcAcq(%.1f %.1f %.1f %.1f)",__func__,m_curChId,m_curFovId[m_curChId],m_rcAcq.x, m_rcAcq.y, m_rcAcq.width, m_rcAcq.height);
		break;
	case VP_CFG_TrkPosRef:
		if(m_bTrack && pPrm != NULL)
		{
			cv::Point2f ref = *(cv::Point2f*)pPrm;
			cv::Rect_<float> rc(m_rcTrackSelf.x, m_rcTrackSelf.y, m_rcTrackSelf.width, m_rcTrackSelf.height);
			if(ref.x>0.1) rc.x = m_rcTrackSelf.x+0.5;
			if(ref.x<-0.1) rc.x = m_rcTrackSelf.x-0.5;
			if(ref.y>0.1) rc.y = m_rcTrackSelf.y+0.5;
			if(ref.y<-0.1) rc.y = m_rcTrackSelf.y-0.5;
			if(rc.width<3) rc.width = 3;
			if(rc.height<3) rc.height = 3;

			m_rcAcq.width = rc.width;m_rcAcq.height = rc.height;
			m_rcAcq.x = rc.x + ref.x;
			m_rcAcq.y = rc.y + ref.y;
			m_iTrackStat = 0;
			m_iTrackLostCnt = 0;
			m_intervalFrame = 0;
			m_reTrackStat = 0;
			//OSA_printf("ref(%f, %f)",ref.x, ref.y);
			//OSA_printf("tk(%f,%f,%f,%f)",m_rcTrackSelf.x, m_rcTrackSelf.y, m_rcTrackSelf.width, m_rcTrackSelf.height);
			//OSA_printf("aq(%f,%f,%f,%f)\n\r",m_rcAcq.x, m_rcAcq.y, m_rcAcq.width, m_rcAcq.height);
		}
		break;
	case VP_CFG_TrkCoast:
		if(m_bTrack)
		{
			m_intervalFrame = iPrm;
			m_reTrackStat = m_iTrackStat;
		}
		break;

	case VP_CFG_TrkForce:
		m_bForceTrack = iPrm;

		printf("\n[%d] %s %d: m_bForceTrack = %d m_iTrackStat = %d\n", OSA_getCurTimeInMsec(), __func__, __LINE__, m_bForceTrack, m_iTrackStat);

		break;

	default:
		break;
	}
	//OSA_printf("%s %i: ", __func__, __LINE__);

	return iret;
}

void CTrackerProc::update_acqRc()
{
	UTC_RECT_float rc;
	rc.width  = m_sizeAcqWin.width/m_curEZoomx[m_curChId];
	rc.height = m_sizeAcqWin.height/m_curEZoomx[m_curChId];
	rc.x = m_axis.x - rc.width/2;
	rc.y = m_axis.y - rc.height/2;
	m_rcAcq = rc;
	m_rcTrk = tRectScale(m_rcAcq, m_imgSize[m_curChId], (float)m_curEZoomx[m_curChId]);
	m_rcTrkFlt = m_rcTrk;
	m_rcTrackSelf = rc;
	//OSA_printf("%s: curCh%d -%d m_rcAcq(%.1f %.1f %.1f %.1f)",__func__,m_curChId,m_curFovId[m_curChId],m_rcAcq.x, m_rcAcq.y, m_rcAcq.width, m_rcAcq.height);
}

int CTrackerProc::run()
{

	OnRun();

	return 0;
}

int CTrackerProc::stop()
{
	OnStop();

	//OSA_printf(" %d:%s enter\n", OSA_getCurTimeInMsec(),__func__);

	return 0;
}

void CTrackerProc::Track_reacq(UTC_RECT_float & rcTrack,int acqinterval)
{
	m_intervalFrame=acqinterval;
	m_rcAcq=rcTrack;
}

int CTrackerProc::ReAcqTarget()
{
	int iRet = m_iTrackStat;

	if(m_intervalFrame > 0){
		m_intervalFrame--;
		if(m_intervalFrame == 0){
			iRet = m_reTrackStat;	// need acq
			m_iTrackLostCnt = 0;
		}
		else
			iRet = -1;	// need pause
	}
	if(m_intervalFrame<0)
		return -1;
	return iRet;
}

int CTrackerProc::process_track(int trackStatus, const Mat& frame_gray, Mat& frame_dis, UTC_RECT_float &rcResult)
{
	int iRet = OSA_SOK;
#if 1
	if(m_bForceTrackBak != m_bForceTrack){
		m_bForceTrackBak = m_bForceTrack;

		if(trackStatus>0){
			if(!m_bForceTrackBak){
				trackStatus = process_track_force(trackStatus, frame_gray, frame_dis, rcResult);
				iRet = process_track_(0, frame_gray, frame_dis, rcResult);
			}else{
				static Mat med;
				frame_gray.copyTo(med);
				trackStatus = process_track_(trackStatus, med, med, rcResult);
				iRet = process_track_force(0, frame_gray, frame_dis, rcResult);
			}
			return iRet;
		}
	}

	if(!m_bForceTrackBak){
		iRet = process_track_(trackStatus, frame_gray, frame_dis, rcResult);
		//cv::imshow("ttt", frame_gray);
		//cv::waitKey(1);
	}else{
		iRet = process_track_force(trackStatus, frame_gray, frame_dis, rcResult);
	}
#else
	iRet = process_track_force(trackStatus, frame_gray, frame_dis, rcResult);
#endif

	return iRet;
}

int CTrackerProc::process_track_(int trackStatus, const Mat& frame_gray, Mat& frame_dis, UTC_RECT_float &rcResult)
{
	IMG_MAT image;
	cv::Size imgSize(frame_gray.cols, frame_gray.rows);

	image.data_u8 	= frame_gray.data;
	image.width 	= frame_gray.cols;
	image.height 	= frame_gray.rows;
	image.channels 	= 1;
	image.step[0] 	= image.width;
	image.dtype 	= 0;
	image.size	= frame_gray.cols*frame_gray.rows;

#if 0
	static Mat Once;
	if(trackStatus == 0)
		frame_gray.copyTo(Once);
	static int ii = 0;
	image.data_u8 = Once.data + (ii>>2)*Once.cols;
	ii ++;
	if(ii == 100) ii = 0;
	//cv::imshow("gray", Once);
	//cv::waitKey(1);
#endif

	if(trackStatus != 0)
	{
		//rcResult = UtcTrkProc(m_track, image, &trackStatus);
		int curStat = trackStatus;
		rcResult = UtcTrkProc(m_track, image, &curStat);
		if(curStat == m_argStatBak){
			m_nArgStatKeep ++;
		}else{
			m_nArgStatKeep = 1;
			m_argStatBak = curStat;
			if(curStat != 1)
				m_bArgStatFilter = true;
		}

		if(curStat == 1 && m_bArgStatFilter){
			if(m_nArgStatKeep>=m_reTrkFliterFrameCnt){
				m_bArgStatFilter = false;
				trackStatus = curStat;
			}
		}else{
			trackStatus = curStat;
		}
	}
	else
	{
		UTC_ACQ_param acq;
		//OSA_printf("ACQ (%f, %f, %f, %f)",
		//		rcResult.x, rcResult.y, rcResult.width, rcResult.height);
		acq.axisX = m_axis.x;// image.width/2;
		acq.axisY = m_axis.y;//image.height/2;

#if(0)
		acq.rcWin.x = (int)(rcResult.x);
		acq.rcWin.y = (int)(rcResult.y);
		acq.rcWin.width = (int)(rcResult.width);
		acq.rcWin.height = (int)(rcResult.height);
#else
		UTC_RECT_float rcTmp = rcResult;
		rcTmp.height = rcTmp.width*3.0/4.0;
		rcTmp.y += (rcResult.height - rcTmp.height)*0.5;
		acq.rcWin.x = (int)floor(rcTmp.x);
		acq.rcWin.y = (int)floor(rcTmp.y);
		acq.rcWin.width = (int)floor(rcTmp.width);
		acq.rcWin.height = (int)floor(rcTmp.height);
#endif
		if(acq.rcWin.width<0)
		{
			acq.rcWin.width=0;
		}
		else if(acq.rcWin.width>= image.width)
		{
			acq.rcWin.width=80;
		}
		if(acq.rcWin.height<0)
		{
			acq.rcWin.height=0;
		}
		else if(acq.rcWin.height>= image.height)
		{
			acq.rcWin.height=60;
		}
		if(acq.rcWin.x<0)
		{
			acq.rcWin.x=0;
		}
		else if(acq.rcWin.x>image.width-acq.rcWin.width)
		{
			acq.rcWin.x=image.width-acq.rcWin.width;
		}
		if(acq.rcWin.y<0)
		{
			acq.rcWin.y=0;
		}
		else if(acq.rcWin.y>image.height-acq.rcWin.height)
		{
			acq.rcWin.y=image.height-acq.rcWin.height;
		}

		UtcTrkAcq(m_track, image, acq);
		trackStatus = 1;

		m_bArgStatFilter = false;
		m_nArgStatKeep = 1;
		m_argStatBak = trackStatus;
	}

	return trackStatus;
}

int CTrackerProc::process_track_force(int trackStatus, const Mat& frame_gray, Mat& frame_dis, UTC_RECT_float &rcResult)
{
	IMG_MAT image;
	cv::Size imgSize(frame_gray.cols, frame_gray.rows);
	image.data_u8 	= frame_gray.data;
	image.width 	= frame_gray.cols;
	image.height 	= frame_gray.rows;
	image.channels 	= 1;
	image.step[0] 	= image.width;
	image.dtype 	= 0;
	image.size	= frame_gray.cols*frame_gray.rows;

	if(trackStatus != 0)
	{
		rcResult = MedTrkProc(m_track, image, &trackStatus);
	}
	else
	{
		UTC_ACQ_param acq;
		acq.axisX = m_axis.x;
		acq.axisY = m_axis.y;
		acq.rcWin.x = (rcResult.x);
		acq.rcWin.y = (rcResult.y);
		acq.rcWin.width = (rcResult.width);
		acq.rcWin.height = (rcResult.height);
		if(acq.rcWin.width<1e-5)
		{
			acq.rcWin.width=0.0f;
		}
		else if(acq.rcWin.width>= frame_gray.cols)
		{
			acq.rcWin.width=80.0f;
		}
		if(acq.rcWin.height<1e-5)
		{
			acq.rcWin.height=0.0f;
		}
		else if(acq.rcWin.height>= frame_gray.rows)
		{
			acq.rcWin.height=60.0f;
		}
		if(acq.rcWin.x<1e-5)
		{
			acq.rcWin.x=0.0f;
		}
		else if(acq.rcWin.x>frame_gray.cols-acq.rcWin.width)
		{
			acq.rcWin.x=frame_gray.cols-acq.rcWin.width;
		}
		if(acq.rcWin.y<1e-5)
		{
			acq.rcWin.y=0.0f;
		}
		else if(acq.rcWin.y>frame_gray.rows-acq.rcWin.height)
		{
			acq.rcWin.y=frame_gray.rows-acq.rcWin.height;
		}
		MedTrkAcq(m_track, image, acq);
		trackStatus = 1;
	}
	return trackStatus;
}


