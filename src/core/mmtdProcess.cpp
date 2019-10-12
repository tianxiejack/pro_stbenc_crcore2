#if 1
#include "mmtdProcess.hpp"

#define CONFIG_MMT_FILE		"ConfigMmtFile.yml"

CMMTD *g_mmtd = NULL;

CMMTDProcess::CMMTDProcess(IProcess *proc)
	:CProcessBase(proc, VP_IDENTIFY_MMTD),m_nCount(MAX_TGT_NUM-1),m_nSelect(MAX_TGT_NUM-1),m_curChId(0), m_bEnable(false),m_nDrop(0),
	 m_fovId(0), m_ezoomx(1), m_roi(cv::Rect(0,0,0,0)),m_curNumber(0),m_roiSet(cv::Rect(0,0,0,0))
{
	memset(m_shTargets, 0, sizeof(m_shTargets));
	memset(m_numMap, -1, sizeof(m_numMap));
	memset(m_units, 0, sizeof(m_units));
	memset(&m_shTrkTarget, 0, sizeof(m_shTrkTarget));
	for(int chId=0; chId<MAX_CHAN; chId++){
		m_imgSize[chId].width = 1920;
		m_imgSize[chId].height = 1080;
	}
	m_mmtd = new CMMTD();
	OSA_assert(m_mmtd != NULL);
	g_mmtd = m_mmtd;
	m_mmtd->SetTargetNum(m_nCount);

	memset(m_cfg, 0, sizeof(m_cfg));
	for(int chId=0; chId<MAX_CHAN; chId++){
		initDefaultConfig(m_cfg[chId]);
		ReadCfgMmtFromFile(m_cfg[chId], CONFIG_MMT_FILE);
		char szFile[256];
		sprintf(szFile, "ConfigMmtFile%02d.yml", chId);
		string cfgFile = szFile;
		ReadCfgMmtFromFile(m_cfg[chId], cfgFile);
		OSA_printf("MMTD ch%d config paramers:", chId);
		OSA_printf("DetectGapparm, MinArea, MaxArea,stillPixel, movePixel,lapScaler,lumThred,SalientSize = %d,%d,%d,%d,%d,%f,%d,%d",
				m_cfg[chId].DetectGapparm, m_cfg[chId].MinArea, m_cfg[chId].MaxArea,m_cfg[chId].stillPixel, m_cfg[chId].movePixel,
				m_cfg[chId].lapScaler,m_cfg[chId].lumThred,m_cfg[chId].SalientSize);
		OSA_printf("lumThred, srModel, meanThred1, stdThred, meanThred2, sortType, bClimitWH = %d, %d, %d, %d, %d, %d, %d\n",
				m_cfg[chId].lumThred, m_cfg[chId].srModel, m_cfg[chId].meanThred1,m_cfg[chId].stdThred,m_cfg[chId].meanThred2,
				m_cfg[chId].sortType, m_cfg[chId].bClimitWH);
	}

	int ret = OSA_SOK;
	memset(&m_threadCtx, 0, sizeof(m_threadCtx));
	ret = OSA_msgqCreate(&m_msgQHdl);
	OSA_assert(ret == OSA_SOK);
	ret = OSA_thrCreate(&m_threadHdl, entryFuncThread, OSA_THR_PRI_MIN, OSA_THR_STACK_SIZE_DEFAULT, this);
	OSA_assert(ret == OSA_SOK);
}

CMMTDProcess::~CMMTDProcess()
{
	OSA_msgqSendMsg(&m_msgQHdl, NULL, VP_CFG_MMTDQuit, NULL, 0, NULL);
	OSA_thrDelete(&m_threadHdl);
	OSA_msgqDelete(&m_msgQHdl);

	if(m_mmtd != NULL)
		delete m_mmtd;
	m_mmtd = NULL;
}

int CMMTDProcess::thread_process()
{
	OSA_MsgHndl msg;
	memset(&m_threadCtx, 0, sizeof(m_threadCtx));
	for(;;)
	{
		int ret = OSA_msgqRecvMsgEx(&m_msgQHdl, &msg, OSA_TIMEOUT_FOREVER);
		OSA_assert(ret == OSA_SOK);
		if(msg.cmd == VP_CFG_MMTDQuit){
			OSA_printf("%s %d: ... Quit", __func__, __LINE__);
			break;
		}
		if(msg.cmd == VP_CFG_MMTDNewData){
			m_threadCtx.busy = true;
			ret = privateProcess(m_threadCtx.chId, m_threadCtx.fovId, m_threadCtx.ezoomx, m_threadFrame, m_threadCtx.timestamp);
			m_threadCtx.busy = false;
		}else{
			void *pPrm = (msg.pPrm == NULL) ? NULL : ((unsigned char*)msg.pPrm+sizeof(int));
			//int prmSize = (msg.pPrm == NULL) ? 0 : (*(int*)msg.pPrm);
			ret = private_dynamic_config(msg.cmd, msg.flags, pPrm);
			if(msg.pPrm != NULL)
				OSA_memFree(msg.pPrm);
		}
	}
	return OSA_SOK;
}

inline Rect tRectCropScale(cv::Size imgSize, float fs)
{
	float fscaled = 1.0/fs;
	cv::Size2f rdsize(imgSize.width/2.0, imgSize.height/2.0);
	cv::Point point((int)(rdsize.width-rdsize.width*fscaled+0.5), (int)(rdsize.height-rdsize.height*fscaled+0.5));
	cv::Size sz((int)(imgSize.width*fscaled+0.5), (int)(imgSize.height*fscaled+0.5));
	return Rect(point, sz);
}

inline int meancr(unsigned char *img, int width)
{
	return ((img[0]+img[1]+img[width]+img[width+1])>>2);
}
static void mResize(Mat src, Mat dst, int zoomx)
{
	int width = src.cols;
	int height = src.rows;
	int zoomxStep = zoomx>>1;
	uint8_t  *  pDst8_t;
	uint8_t *  pSrc8_t;

	pSrc8_t = (uint8_t*)(src.data)+(height/2-(height/(zoomx<<1)))*width+(width/2-(width/(zoomx<<1)));
	pDst8_t = (uint8_t*)(dst.data);

	for(int y = 0; y < height; y++)
	{
		int halfIy = y>>zoomxStep;
		for(int x = 0; x < width; x++){
			pDst8_t[y*width+x] = meancr(pSrc8_t+halfIy*width+(x>>zoomxStep), width);
		}
	}
}

inline void mResizeX2(Mat src, Mat dst)
{
	int width = src.cols;
	int height = src.rows;
	uint8_t *  pDst8_t;
	uint8_t *  pSrc8_t;

	pSrc8_t = (uint8_t*)(src.data)+(height>>2)*width+(width>>2);
	pDst8_t = (uint8_t*)(dst.data);

	for(int y = 0; y < (height>>1); y++)
	{
		for(int x = 0; x < (width>>1); x++){
			pDst8_t[y*2*width+x*2] = pSrc8_t[y*width+x];
			pDst8_t[y*2*width+x*2+1] = (pSrc8_t[y*width+x]+pSrc8_t[y*width+x+1])>>1;
			pDst8_t[(y*2+1)*width+x*2] = (pSrc8_t[y*width+x]+pSrc8_t[(y+1)*width+x])>>1;
			pDst8_t[(y*2+1)*width+x*2+1] = (pSrc8_t[y*width+x]+pSrc8_t[y*width+x+1]+pSrc8_t[(y+1)*width+x]+pSrc8_t[(y+1)*width+x+1])>>2;
		}
	}
}

int CMMTDProcess::privateProcess(int chId, int fovId, int ezoomx, Mat& frame, uint64_t timestamp)
{
	int iRet = OSA_SOK;
	m_imgSize[chId].width = frame.cols;
	m_imgSize[chId].height = frame.rows;
	m_frameTimestamp[chId] = timestamp;

	//OSA_printf("%s %d: ch%d(%d) fov%d(%d)", __func__, __LINE__, chId, m_curChId, fovId, m_fovId);

	if(m_curChId != chId)
		return iRet;

	OSA_mutexLock(&m_mutexlock);

	if(m_fovId!= fovId){
		memset(m_shTargets, 0, sizeof(m_shTargets));
		memset(m_numMap, -1, sizeof(m_numMap));
		m_curNumber = 0;
		m_mmtd->ClearAllMMTD();
		m_fovId= fovId;
		m_nDrop = 3;
		for(int i=0; i<MAX_TGT_NUM; i++)
			m_units[m_curChId][i].bNeedDraw = false;
	}

	if(m_nDrop>0){
		m_nDrop --;
		if(m_nDrop == 0){
			if(m_roi.width == 0 || m_roi.height == 0){
				m_roi = Rect(0, 0, frame.cols, frame.rows);
			}
			setConfig(m_cfg[m_curChId], cv::Size(m_roi.width, m_roi.height));
		}
		OSA_mutexUnlock(&m_mutexlock);
		return iRet;
	}

	if(m_bEnable && m_curChId == chId)
	{
		TARGETBOX targets[MAX_TGT_NUM];
		memset(targets, 0, sizeof(targets));
		if(m_roi.width == 0 || m_roi.height == 0 || m_roi.x+m_roi.width>frame.cols || m_roi.y+m_roi.height>frame.rows){
			m_roi = Rect(0, 0, frame.cols, frame.rows);
		}
		if(ezoomx>1){
			//cv::Size imgSize(frame.cols, frame.rows);
			//roi = tRectCropScale(imgSize, ezoomx);
		}
		if(m_ezoomx!=ezoomx){
			memset(m_shTargets, 0, sizeof(m_shTargets));
			memset(m_numMap, -1, sizeof(m_numMap));
			m_curNumber = 0;
			m_mmtd->ClearAllMMTD();
			m_ezoomx=ezoomx;
			for(int i=0; i<MAX_TGT_NUM; i++)
				m_units[m_curChId][i].bNeedDraw = false;
		}
		OSA_mutexUnlock(&m_mutexlock);
		if(m_ezoomx<=1){
			//OSA_printf("CMMTDProcess::dynamic_config roi(%d,%d,%d,%d)", m_roi.x, m_roi.y,m_roi.width,m_roi.height);
			m_mmtd->MMTDProcessRect(frame, targets, m_roi, frame, 0);
			//cv::imshow("ttt", frame);
			//cv::waitKey(1);
		}else{
			//int64 tks = getTickCount();
			cv::Mat rsMat = cv::Mat(frame.rows, frame.cols, CV_8UC1);
			if(m_ezoomx == 4){
				cv::Mat rsMat0 = cv::Mat(frame.rows, frame.cols, CV_8UC1);
				mResizeX2(frame, rsMat0);
				mResizeX2(rsMat0, rsMat);
			}
			else if(m_ezoomx == 8){
				cv::Mat rsMat0 = cv::Mat(frame.rows, frame.cols, CV_8UC1);
				mResizeX2(frame, rsMat);
				mResizeX2(rsMat, rsMat0);
				mResizeX2(rsMat0, rsMat);
			}else{
				mResizeX2(frame, rsMat);
			}
			//OSA_printf("chId = %d, resize: time = %f sec \n", chId, ( (getTickCount() - tks)/getTickFrequency()) );
			m_mmtd->MMTDProcessRect(rsMat, targets, m_roi, frame, 0);
		}
		OSA_mutexLock(&m_mutexlock);

		int nValid = 0;
		if(m_nSelect < m_nCount)
		{
			bool bOrder[MAX_TGT_NUM];
			memset(bOrder, 0, sizeof(bOrder));
			for(int i=0; i<m_nCount; i++){
				if(m_shTargets[i].valid){
					OSA_assert(m_numMap[i]>=0);
					m_shTargets[i].valid = targets[m_numMap[i]].valid;
					if(m_shTargets[i].valid){
						m_shTargets[i].Box = targets[m_numMap[i]].Box;
						bOrder[m_numMap[i]] = true;
						nValid ++ ;
					}
				}
			}

			for(int i=0; i<m_nCount; i++){
				if(nValid<m_nSelect && targets[i].valid && !bOrder[i]){
					m_shTargets[m_curNumber].valid = targets[i].valid;
					m_shTargets[m_curNumber].Box = targets[i].Box;
					m_numMap[m_curNumber] = i;
					m_curNumber++;
					m_curNumber = (m_curNumber == m_nCount) ? 0 : m_curNumber;
					bOrder[i] = true;
					nValid ++ ;
				}
			}

			for(int i=0; i<m_nCount; i++)
			{
				if( m_shTargets[i].valid && !m_bHide && !(m_shTrkTarget.valid && isRectOverlap(m_shTrkTarget.Box, m_shTargets[i].Box)) ){
					m_units[chId][i].orgPos = Point(m_shTargets[i].Box.x + m_shTargets[i].Box.width/2, m_shTargets[i].Box.y + m_shTargets[i].Box.height/2);
					m_units[chId][i].orgRC = m_shTargets[i].Box;
					m_units[chId][i].orgValue = i;
					m_units[chId][i].bNeedDraw = true;
				}else{
					m_units[chId][i].bNeedDraw = false;
				}
			}
		}
		else
		{
			memset(m_shTargets, 0, sizeof(m_shTargets));
			for(int i=0; i<m_nCount; i++){
				m_shTargets[i].Box = targets[i].Box;
				m_shTargets[i].valid = targets[i].valid;
			}
			for(int i=0; i<m_nCount; i++)
			{
				if(m_shTargets[i].valid && m_units[chId][i].bNeedDraw && !m_bHide){
					m_units[chId][i].orgPos = Point(m_shTargets[i].Box.x + m_shTargets[i].Box.width/2, m_shTargets[i].Box.y + m_shTargets[i].Box.height/2);
					m_units[chId][i].orgRC = m_shTargets[i].Box;
					m_units[chId][i].orgValue = i;
					nValid ++ ;
				}else{
					m_units[chId][i].bNeedDraw = false;
				}
			}
			for(int i=0; i<m_nCount; i++)
			{
				if(!m_units[chId][i].bNeedDraw && m_shTargets[i].valid && nValid<m_nSelect && !m_bHide){
					m_units[chId][i].bNeedDraw = true;
					m_units[chId][i].orgPos = Point(m_shTargets[i].Box.x + m_shTargets[i].Box.width/2, m_shTargets[i].Box.y + m_shTargets[i].Box.height/2);
					m_units[chId][i].orgRC = m_shTargets[i].Box;
					m_units[chId][i].orgValue = i;
					nValid ++ ;
				}
			}
		}

		//OSA_printf("CMMTDProcess::%s %d: ch%d nvalid = %d", __func__, __LINE__, chId, nValid);
	}
	OSA_mutexUnlock(&m_mutexlock);
	return iRet;
}

int CMMTDProcess::process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp)
{
	int iRet = OSA_SOK;
	iRet = CProcessBase::process(chId, fovId, ezoomx, frameOrg, frameGray, timestamp);

	//return iRet;
	//return privateProcess(chId, fovId, ezoomx, frame, timestamp);
	if(!m_threadCtx.busy){
		m_threadCtx.chId = chId;
		m_threadCtx.fovId = fovId;
		m_threadCtx.ezoomx = ezoomx;
		m_threadCtx.timestamp = timestamp;
		m_threadFrame = frameGray;
		iRet = OSA_msgqSendMsg(&m_msgQHdl, NULL, VP_CFG_MMTDNewData, NULL, 0, NULL);
		OSA_assert(iRet == OSA_SOK);
	}
	return iRet;
}

int CMMTDProcess::OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp)
{
	int ret = CProcessBase::OnOSD(chId, fovId, ezoomx, dc, osd, timestamp);
	float scalex = dc.cols/1920.0;
	float scaley = dc.rows/1080.0;
	int winWidth = 40*scalex;
	int winHeight = 40*scaley;
	bool bFixSize = false;

	OSA_mutexLock(&m_mutexlock);
	if(m_curChId == chId)
	{
		if(!m_bHide && m_bDebug && (m_roi.width != m_imgSize[chId].width || m_roi.height != m_imgSize[chId].height))
			osd->rectangle(m_roi, 0);
		for(int i=0; i<MAX_TGT_NUM; i++){
			if(m_units[chId][i].bNeedDraw){
				if(!bFixSize){
					winWidth = m_units[chId][i].orgRC.width;
					winHeight = m_units[chId][i].orgRC.height;
				}
				Rect rc(m_units[chId][i].orgPos.x-winWidth/2, m_units[chId][i].orgPos.y-winHeight/2, winWidth, winHeight);
				rc = tRectScale(rc, m_imgSize[chId], cv::Size(dc.cols, dc.rows));
				m_units[chId][i].drawRC = rc;
				osd->numberedBox(m_units[chId][i].drawRC, i+1, 0);
			}
		}
	}
	OSA_mutexUnlock(&m_mutexlock);
}

int CMMTDProcess::private_dynamic_config(int type, int iPrm, void* pPrm)
{
	int iret = OSA_SOK;

	OSA_mutexLock(&m_mutexlock);
	switch(type)
	{
	case VP_CFG_MMTDTargetCount:
		m_nCount = iPrm;
		m_mmtd->SetTargetNum(m_nCount);
		m_nSelect = m_nCount;
		if(pPrm != NULL)
			m_nSelect = *(int*)pPrm;
		iret = OSA_SOK;
		break;
	case VP_CFG_MMTDEnable:
		memset(m_shTargets, 0, sizeof(m_shTargets));
		memset(m_numMap, -1, sizeof(m_numMap));
		m_curNumber = 0;
		m_bEnable = iPrm;
		m_nDrop = 3;
		m_mmtd->ClearAllMMTD();
		for(int i=0; i<MAX_TGT_NUM; i++)
			m_units[m_curChId][i].bNeedDraw = false;
		m_roi = Rect(0, 0, 0, 0);
		if(m_bEnable && pPrm != NULL){
			m_roi = *(cv::Rect*)pPrm;
			OSA_printf("CMMTDProcess::dynamic_config roi(%d,%d,%d,%d)", m_roi.x, m_roi.y,m_roi.width,m_roi.height);
		}
		m_roiSet = m_roi;
		iret = OSA_SOK;
		break;
	case VP_CFG_MainChId:
		memset(m_shTargets, 0, sizeof(m_shTargets));
		memset(m_numMap, -1, sizeof(m_numMap));
		m_curNumber = 0;
		if(m_curChId>=0){
			for(int i=0; i<MAX_TGT_NUM; i++)
				m_units[m_curChId][i].bNeedDraw = false;
		}
		m_curChId = iPrm;
		m_nDrop = 3;
		m_mmtd->ClearAllMMTD();
		if(m_curChId>=0){
			for(int i=0; i<MAX_TGT_NUM; i++)
				m_units[m_curChId][i].bNeedDraw = false;
		}
		m_roi = m_roiSet;
		break;
	case VP_CFG_GetTargetInfo:
		//OSA_printf("mmtd %s: %d %d", __func__, iPrm, m_shTargets[iPrm].valid);
		if(iPrm>=0 && iPrm<MAX_TGT_NUM && m_shTargets[iPrm].valid && pPrm != NULL){
			PROC_TARGETINFO *tgt = (PROC_TARGETINFO*)pPrm;
			tgt->valid = m_shTargets[iPrm].valid;
			tgt->Box = m_shTargets[iPrm].Box;
			iret = OSA_SOK;
		}
		break;
	default:
		break;
	}
	OSA_mutexUnlock(&m_mutexlock);

	return iret;
}

int CMMTDProcess::dynamic_config(int type, int iPrm, void* pPrm, int prmSize)
{
	int iret = OSA_SOK;

	//cout << "CMMTDProcess::dynamic_config type " << type << " iPrm " << iPrm << endl;
	iret = CProcessBase::dynamic_config(type, iPrm, pPrm, prmSize);

	if(type<VP_CFG_BASE || type>VP_CFG_MMTDMax)
		return iret;

	if(type == VP_CFG_GetTargetInfo)
		return private_dynamic_config(type, iPrm, pPrm);

	unsigned char *memPrm = NULL;
	if(prmSize>0 && pPrm!=NULL){
		memPrm = (unsigned char*)OSA_memAlloc(prmSize+sizeof(int));
		OSA_assert(memPrm != NULL);
		memcpy(memPrm+sizeof(int), pPrm, prmSize);
		*(int*)memPrm = prmSize;
	}
	iret = OSA_msgqSendMsg(&m_msgQHdl, NULL, type, memPrm, iPrm, NULL);
	OSA_assert(iret == OSA_SOK);
	return iret;
}

void CMMTDProcess::initDefaultConfig(MMTD_CFG& cfg)
{
	cfg.DetectGapparm = 10;
	cfg.MinArea = 900;
	cfg.MaxArea = 20000;
	cfg.stillPixel = 6;
	cfg.movePixel = 16;
	cfg.lapScaler = 1.25f;
	cfg.lumThred = 50;
	cfg.srModel = 2;
	cfg.meanThred1 = 35;
	cfg.stdThred = 12;
	cfg.meanThred2 = 65;
	cfg.sortType = 1;
	cfg.bClimitWH = 0;
	cfg.SalientSize = 0;
}

int CMMTDProcess::ReadCfgMmtFromFile(MMTD_CFG& cfg, string cfgFile)
{
	char cfg_avt[16] = "cfg_mmt_";
	int configId_Max=128;
	float cfg_blk_val[128];

	FILE *fp = fopen(cfgFile.c_str(), "rt");

	if(fp != NULL)
	{
		fseek(fp, 0, SEEK_END);
		int len = ftell(fp);
		fclose(fp);

		if(len < 10)
			return -1;
		else
		{
			FileStorage fr(cfgFile, FileStorage::READ);
			if(fr.isOpened())
			{
				for(int i=0; i<configId_Max; i++){
					sprintf(cfg_avt, "cfg_mmt_%d", i);
					cfg_blk_val[i] = (float)fr[cfg_avt];
					//printf(" update cfg [%d] %f \n", i, cfg_blk_val[i]);
				}
			}else
				return -1;
		}


		int DetectGapparm = (int)cfg_blk_val[0];
		if((DetectGapparm<0) || (DetectGapparm>15))
			DetectGapparm = 10;
		cfg.DetectGapparm = DetectGapparm;

		int MinArea = (int)cfg_blk_val[1];
		int MaxArea = (int)cfg_blk_val[2];
		if(MinArea>0)
			cfg.MinArea = MinArea;
		if(MaxArea>0)
			cfg.MaxArea = MaxArea;

		cfg.stillPixel = (int)cfg_blk_val[3];
		cfg.movePixel = (int)cfg_blk_val[4];
		cfg.lapScaler = cfg_blk_val[5];
		cfg.lumThred = (int)cfg_blk_val[6];
		cfg.srModel = (int)cfg_blk_val[7];
		cfg.meanThred1 	= (int)cfg_blk_val[8];
		cfg.stdThred 		= (int)cfg_blk_val[9];
		cfg.meanThred2 	= (int)cfg_blk_val[10];
		cfg.sortType 	= (int)cfg_blk_val[11];
		cfg.bClimitWH 	= (int)cfg_blk_val[12];
		cfg.SalientSize 	= (int)cfg_blk_val[13];

		return 0;
	}
	else
		return -1;
}

int CMMTDProcess::setConfig(MMTD_CFG& cfg, const cv::Size& roi)
{
	m_mmtd->SetSRDetectGap(cfg.DetectGapparm);
	m_mmtd->SetConRegMinMaxArea(cfg.MinArea, cfg.MaxArea);
	m_mmtd->SetMoveThred(cfg.stillPixel, cfg.movePixel);
	m_mmtd->SetLapScaler(cfg.lapScaler);
	m_mmtd->SetSRLumThred(cfg.lumThred);
	m_mmtd->SetSRDetectMode(cfg.srModel);
	m_mmtd->SetSRDetectParam(cfg.meanThred1,cfg.stdThred,cfg.meanThred2);
	m_mmtd->SetTargetSortType(cfg.sortType);
	m_mmtd->SetEnableClimitWH(cfg.bClimitWH);
	if(cfg.SalientSize == 0){
		if(roi.width<=720)
			cfg.SalientSize = 64;
		else if(roi.width<=1280)
			cfg.SalientSize = 128;
		else
			cfg.SalientSize = 160;
	}
	m_mmtd->SetSalientSize(cv::Size(cfg.SalientSize,cfg.SalientSize));
	OSA_printf("DetectGapparm, MinArea, MaxArea,stillPixel, movePixel,lapScaler,lumThred,SalientSize = %d,%d,%d,%d,%d,%f,%d,%d",
			cfg.DetectGapparm, cfg.MinArea, cfg.MaxArea,cfg.stillPixel, cfg.movePixel,cfg.lapScaler,cfg.lumThred,cfg.SalientSize);
	OSA_printf("lumThred, srModel, meanThred1, stdThred, meanThred2, sortType, bClimitWH = %d, %d, %d, %d, %d, %d, %d\n",
			cfg.lumThred, cfg.srModel, cfg.meanThred1,cfg.stdThred,cfg.meanThred2, cfg.sortType, cfg.bClimitWH);
}

#endif

