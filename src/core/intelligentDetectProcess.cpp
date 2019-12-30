#ifdef USER_INTELL
#include "intelligentDetectProcess.hpp"

#include <dlfcn.h>
#include <unistd.h>

typedef DetectorFactory* (*LDFUNC_GETINSTANCE)();
typedef Detector* (*LDFUNC_createDetector)(DetectorFactory::DetectroName name);

static std::string getExecPath()
{
    char buf[1024];
    ssize_t len = ::readlink("/proc/self/exe", buf, sizeof(buf));

    if (len <= 0){
        OSA_printf("Can't determine the executable's location");
        return "";
    }

    std::string execPath = std::string(buf, len);

    size_t pathPos = execPath.rfind("/");

    if (pathPos == std::string::npos){
    	OSA_printf("Can't find the directory");
    	return "";
    }

    return execPath.substr(0, pathPos+1);
}

static void* loadFunc(std::string strFile, std::string strFunc)
{
	if(strFile.size() == 0 || strFunc.size() == 0)
		return NULL;

	static void *obj = NULL;
	if(obj == NULL){
		obj = dlopen(strFile.c_str(), RTLD_NOW);//RTLD_LAZY);
		if(obj == NULL)
			OSA_printf("%s\n", dlerror());
	}

	if(obj != NULL){
		void* func = dlsym(obj, strFunc.c_str());
		if(func == NULL)
			OSA_printf("%s\n", dlerror());
		return func;
	}

	return NULL;
}

__inline void remove(char *str)
{
	if (!str || 0 == strlen(str))
	return;
	char *p = str;
	int len = strlen(p);
	while (p[len - 1] == '\n')
	len --;
	p[len] = '\0';
	strncpy(str, p, strlen(p)+1);
}

__inline int readclassnmaeformtxt(string name, vector<string>& classsname)
{
	char buf[100];
	FILE *fp=fopen(name.c_str(),"r");
	if(fp==NULL)
		return -1;
	while(!feof(fp))
	{
		memset(buf,0,100);
		char *line = fgets(buf,100,fp);
		printf("%s",line);
		remove(buf);
		classsname.push_back(string(buf));
	}

	return 0;
}

CINTELLProcess::CINTELLProcess(IProcess *proc)
	:CProcessBase(proc, VP_IDENTIFY_INTELL),m_nCount(MAX_INTELL_TGT_NUM),m_nSelect(MAX_INTELL_TGT_NUM),m_curChId(0), m_bEnable(false),m_nDrop(0),
	 m_fovId(0), m_ezoomx(1), m_roi(cv::Rect(0,0,0,0)),m_curNumber(0),m_roiSet(cv::Rect(0,0,0,0)),m_detector(NULL),bProcessBusy(false),processchid(0)
{
	memset(m_shTargets, 0, sizeof(m_shTargets));
	memset(m_numMap, -1, sizeof(m_numMap));
	memset(m_units, 0, sizeof(m_units));
	memset(&m_shTrkTarget, 0, sizeof(m_shTrkTarget));
	for(int chId=0; chId<MAX_CHAN; chId++){
		m_imgSize[chId].width = 1920;
		m_imgSize[chId].height = 1080;
	}

	{

		std::string txtLabels = getExecPath()+"../config/labels.txt";
		if(readclassnmaeformtxt(txtLabels, m_classes) != 0){
			OSA_printf("%s %d: Can't read %s !\n", __func__, __LINE__, txtLabels.c_str());
			m_detector = NULL;
			return;
		}

		int ret = OSA_SOK;
		memset(&m_threadCtx, 0, sizeof(m_threadCtx));
		ret = OSA_msgqCreate(&m_msgQHdl);
		OSA_assert(ret == OSA_SOK);
		ret = OSA_thrCreate(&m_threadHdl, entryFuncThread, OSA_THR_PRI_MIN, OSA_THR_STACK_SIZE_DEFAULT, this);
		OSA_assert(ret == OSA_SOK);
	}
}

CINTELLProcess::~CINTELLProcess()
{
	if(m_detector){
		OSA_msgqSendMsg(&m_msgQHdl, NULL, VP_CFG_INTELLQuit, NULL, 0, NULL);
		OSA_thrDelete(&m_threadHdl);
		OSA_msgqDelete(&m_msgQHdl);
		//	delete m_detector;
		//m_detector = NULL;
	}
}

//static bool bProcessBusy = false;
void CINTELLProcess::trackcall(vector<BoundingBox>& trackbox,void *context,int chid )
{
	CINTELLProcess*ptr=(CINTELLProcess*)context;
	if(ptr==NULL)
		return;
	ptr->trackbox_.clear();
	//OSA_mutexLock(&mulocktrack);
	ptr->trackbox_.insert(ptr->trackbox_.begin(),trackbox.begin(),trackbox.end());
	//OSA_mutexUnlock(&mulocktrack);
	//swap(detectbox_,trackbox);
}

void CINTELLProcess::detectcall(vector<BoundingBox>& trackbox,void *context,int chid )
{
	CINTELLProcess*ptr=(CINTELLProcess*)context;
	if(ptr==NULL)
		return;
	ptr->detectbox_.clear();
	//OSA_mutexLock(&mulock);
	ptr->detectbox_.insert(ptr->detectbox_.begin(),trackbox.begin(),trackbox.end());
	//printf("the process size=%d\n",trackbox.size());
	//OSA_mutexUnlock(&mulock);
	//swap(detectbox_,trackbox);
	ptr->bProcessBusy = false;
}

int CINTELLProcess::thread_process()
{
	std::vector<std::string> model;
	std::vector<cv::Size> modelsize;
	std::vector<Detector*> detectors;
	std::vector<int > weights;
	std::string ymlModel = getExecPath()+"../config/yolo_v3_tiny.yml";
	model.push_back(ymlModel);
	modelsize.push_back(cv::Size(1920,1080));
	DetectorFactory *factory = DetectorFactory::getinstance();
	
	m_detector = factory->createDetector(DetectorFactory::DEEPLEARNDETECTOR);
	if(m_detector==NULL)
		return -1;
	//OSA_printf("%s %d:  m_detector = %p\n", __func__, __LINE__, m_detector);
	m_detector->setparam(Detector::MAXTRACKNUM, MAX_INTELL_TGT_NUM);
	m_detector->init(model,modelsize);
	m_detector->dynamicsetparam(Detector::DETECTMOD,0);
	m_detector->dynamicsetparam(Detector::DETECTNOTRACK,0);
	m_detector->dynamicsetparam(Detector::DETECTFREQUENCY,1);
	m_detector->dynamicsetparam(Detector::DETECTSCALE100x,100);
	m_detector->dynamicsetparam(Detector::DETECTTRACKCORRECT,0);
	detectors.push_back(m_detector);
	int a=5;
	weights.push_back(a);

	m_detector=DetectorFactory::getinstance()->createDetector(DetectorFactory::SALIENCYDETECTOR);
	if(m_detector==NULL)
		return -1;
	m_detector->setparam(Detector::MAXTRACKNUM,MAX_INTELL_TGT_NUM);
	m_detector->init(model,modelsize);
	m_detector->dynamicsetparam(Detector::DETECTSCALE100x,100);
	m_detector->dynamicsetparam(Detector::DETECTMOD,0);
	m_detector->dynamicsetparam(Detector::DETECTNOTRACK,0);
	m_detector->dynamicsetparam(Detector::DETECTFREQUENCY,1);
	m_detector->dynamicsetparam(Detector::DETECTTRACKCORRECT,0);
	detectors.push_back(m_detector);
	a=5;
	weights.push_back(a);


	m_detector=DetectorFactory::getinstance()->createDetector(DetectorFactory::COMPOSITEDETECTOR);
	if(m_detector==NULL)
		return -1;
	m_detector->setparam(Detector::MAXTRACKNUM,MAX_INTELL_TGT_NUM);
	m_detector->init(detectors,weights);
	m_detector->dynamicsetparam(Detector::DETECTMOD,1);
	m_detector->dynamicsetparam(Detector::DETECTNOTRACK,1);
	m_detector->dynamicsetparam(Detector::DETECTFREQUENCY,1);
	m_detector->dynamicsetparam(Detector::DETECTSCALE100x,100);
	m_detector->dynamicsetparam(Detector::DETECTTRACKCORRECT,0);

	
	m_detector->getversion();
	m_detector->setasyncdetect(detectcall,trackcall,this);
	m_detector->dynamicsetparam(Detector::DETECTTRACKSTOPTIME,1000);
	OSA_MsgHndl msg;
	memset(&m_threadCtx, 0, sizeof(m_threadCtx));

	for(;;)
	{
		int ret = OSA_msgqRecvMsgEx(&m_msgQHdl, &msg, OSA_TIMEOUT_FOREVER);
		OSA_assert(ret == OSA_SOK);
		if(msg.cmd == VP_CFG_INTELLQuit){
			OSA_printf("%s %d: ... Quit", __func__, __LINE__);
			break;
		}
		if(msg.cmd == VP_CFG_INTELLNewData){
			m_threadCtx.busy = true;
			//OSA_printf("detect process %s %d: ... Quit", __func__, __LINE__);
			//Mat processframe;
			double exec_time = (double)getTickCount();
			//process2rgb(m_threadFrame,processframe);
			//exec_time = ((double)getTickCount() - exec_time)*1000./getTickFrequency();
			//OSA_printf("the %s exec_time=%f MS\n",__func__,exec_time);
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
void CINTELLProcess::process2rgb(Mat& src,Mat& dst)
{
	cv::Mat temp;
	//temp.create(Size(src.cols,src.rows))
	
	temp.create(src.rows,src.cols,src.type());
	double exec_time = (double)getTickCount();
	int width = src.cols;
	int height = src.rows;
	uint8_t *  pDst8_t;
	uint8_t *  pSrc8_t;

	pSrc8_t = (uint8_t*)(src.data);
	pDst8_t = (uint8_t*)(temp.data);
	int channels=src.channels();
	for(int y = 0; y < (height); y++)
	{
		for(int x = 0; x < (width*channels); x++)
			{
				pDst8_t[y*width*channels+x]=pSrc8_t[y*width*channels+x];
			}

	}
	//memcpy(temp.data,src.data,src.rows*src.cols*src.channels());
	exec_time = ((double)getTickCount() - exec_time)*1000./getTickFrequency();
	//OSA_printf("the %s exec_time=%f MS\n",__func__,exec_time);
	if(src.channels()==2)
		cv::cvtColor(temp, dst, CV_YUV2RGB_YUYV);
	else if(src.channels()==1)
		cv::cvtColor(temp, dst, CV_GRAY2RGB);
	//for(int i=0;i<src.col)


}
int CINTELLProcess::privateProcess(int chId, int fovId, int ezoomx, Mat& frame, uint64_t timestamp)
{
	int iRet = OSA_SOK;
	m_imgSize[chId].width = frame.cols;
	m_imgSize[chId].height = frame.rows;
	m_frameTimestamp[chId] = timestamp;

	//OSA_printf("%s %d: ch%d(%d) fov%d(%d)", __func__, __LINE__, chId, m_curChId, fovId, m_fovId);

	if(m_curChId != chId)
		return iRet;

	if(bProcessBusy)
		return iRet;

	OSA_mutexLock(&m_mutexlock);

	if(m_fovId!= fovId){
		memset(m_shTargets, 0, sizeof(m_shTargets));
		memset(m_numMap, -1, sizeof(m_numMap));
		m_curNumber = 0;
		if(m_detector!=NULL)
				m_detector->dynamicsetparam(Detector::DETECTDETECTCLOSE,1);
		//m_mmtd->ClearAllMMTD();
		m_fovId= fovId;
		m_nDrop = 3;
		for(int i=0; i<MAX_INTELL_TGT_NUM; i++)
			m_units[m_curChId][i].bNeedDraw = false;
	}

	if(m_nDrop>0){
		m_nDrop --;
		if(m_nDrop == 0){
			if(m_roi.width == 0 || m_roi.height == 0){
				m_roi = Rect(0, 0, frame.cols, frame.rows);
			}
			//setConfig(m_cfg[m_curChId], cv::Size(m_roi.width, m_roi.height));
		}
		OSA_mutexUnlock(&m_mutexlock);
		return iRet;
	}
	cv::Rect m_roizoom;
	if(m_bEnable && m_curChId == chId)
	{
		if(m_roi.width == 0 || m_roi.height == 0 || m_roi.x+m_roi.width>frame.cols || m_roi.y+m_roi.height>frame.rows){
			m_roi = Rect(0, 0, frame.cols, frame.rows);
		}
		if(m_ezoomx!=ezoomx){
			memset(m_shTargets, 0, sizeof(m_shTargets));
			memset(m_numMap, -1, sizeof(m_numMap));
			m_curNumber = 0;
			
			if(m_detector!=NULL)
				m_detector->dynamicsetparam(Detector::DETECTDETECTCLOSE,1);
			//m_mmtd->ClearAllMMTD();
			m_ezoomx=ezoomx;
			for(int i=0; i<MAX_INTELL_TGT_NUM; i++)
				m_units[m_curChId][i].bNeedDraw = false;
		}
		if(m_ezoomx<=1){
			//OSA_printf("CINTELLProcess::dynamic_config roi(%d,%d,%d,%d)", m_roi.x, m_roi.y,m_roi.width,m_roi.height);
			//m_roi=Rect(frame.cols/2-frame.cols/(2*2),frame.rows/2-frame.rows/(2*2), frame.cols/2, frame.rows/2);
			Mat processframe;
			m_roi=Rect(0,0, frame.cols, frame.rows);
			process2rgb(frame,processframe);
			m_detector->detectasync(processframe,processchid,m_roi,true);
			//cv::imshow("ttt", frame);
			//cv::waitKey(1);
		}else{
			//int64 tks = getTickCount();
			cv::Mat rsMat = cv::Mat(frame.rows, frame.cols, CV_8UC1);
			if(m_ezoomx == 4){
				m_roizoom=Rect(frame.cols/2-frame.cols/(2*m_ezoomx),frame.rows/2-frame.rows/(2*m_ezoomx), frame.cols/m_ezoomx, frame.rows/m_ezoomx);
				/*
				cv::Mat rsMat0 = cv::Mat(frame.rows, frame.cols, CV_8UC1);
				mResizeX2(frame, rsMat0);
				mResizeX2(rsMat0, rsMat);
				*/
			}
			else if(m_ezoomx == 8){
				m_roizoom=Rect(frame.cols/2-frame.cols/(2*m_ezoomx),frame.rows/2-frame.rows/(2*m_ezoomx), frame.cols/m_ezoomx, frame.rows/m_ezoomx);
				/*
				cv::Mat rsMat0 = cv::Mat(frame.rows, frame.cols, CV_8UC1);
				mResizeX2(frame, rsMat);
				mResizeX2(rsMat, rsMat0);
				mResizeX2(rsMat0, rsMat);
				*/
			}else{
				m_roizoom=Rect(frame.cols/2-frame.cols/(2*2),frame.rows/2-frame.rows/(2*2), frame.cols/2, frame.rows/2);
				//mResizeX2(frame, rsMat);
			}
			//OSA_printf("chId = %d, resize: time = %f sec \n", chId, ( (getTickCount() - tks)/getTickFrequency()) );
			//m_mmtd->MMTDProcessRect(rsMat, targets, m_roi, frame, 0);
			Mat processframe;
			process2rgb(frame,processframe);
			m_detector->detectasync(processframe,processchid,m_roizoom,true);
		}

		int nValid = 0;
#if 0
		bool bOrder[MAX_INTELL_TGT_NUM];
		memset(bOrder, 0, sizeof(bOrder));
		for(int i=0; i<m_nCount; i++){
			if(m_shTargets[i].valid){
				OSA_assert(m_numMap[i]>=0);
				//m_shTargets[i].valid = targets[m_numMap[i]].valid;
				if(m_shTargets[i].valid){
					//m_shTargets[i].Box = targets[m_numMap[i]].Box;
					bOrder[m_numMap[i]] = true;
					nValid ++ ;
				}
			}
		}

		for(int i=0; i<m_nCount; i++){
			//if(nValid<m_nSelect && targets[i].valid && !bOrder[i])
			{
				//m_shTargets[m_curNumber].valid = targets[i].valid;
				//m_shTargets[m_curNumber].Box = targets[i].Box;
				m_numMap[m_curNumber] = i;
				m_curNumber++;
				m_curNumber = (m_curNumber == m_nCount) ? 0 : m_curNumber;
				bOrder[i] = true;
				nValid ++ ;
			}
		}

		for(int i=0; i<m_nCount; i++)
		{
			if( m_shTargets[i].valid && !m_bHide && !(m_trkTarget.valid && isRectOverlap(m_trkTarget.Box, m_shTargets[i].Box)) ){
				m_units[chId][i].orgPos = Point(m_shTargets[i].Box.x + m_shTargets[i].Box.width/2, m_shTargets[i].Box.y + m_shTargets[i].Box.height/2);
				m_units[chId][i].orgRC = m_shTargets[i].Box;
				m_units[chId][i].orgValue = i;
				m_units[chId][i].bNeedDraw = true;
			}else{
				m_units[chId][i].bNeedDraw = false;
			}
		}

#else
		int nTrack = min((int)trackbox_.size(), m_nCount);
		memset(m_shTargets, 0, sizeof(m_shTargets));
		//printf("the ntrack=%d\n",nTrack);
		for(int i=0; i<nTrack; i++){
			m_shTargets[i].Box = trackbox_[i];
			m_shTargets[i].valid = trackbox_[i].trackstatus;
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
#endif
	}
	OSA_mutexUnlock(&m_mutexlock);
	//OSA_printf("%s %d: ch%d(%d) nvalid = %d", __func__, __LINE__, chId, m_bEnable, nValid);
	return iRet;
}

int CINTELLProcess::process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp)
{
	int iRet = OSA_SOK;
	iRet = CProcessBase::process(chId, fovId, ezoomx, frameOrg, frameGray, timestamp);

	//return iRet;
	//return privateProcess(chId, fovId, ezoomx, frame, timestamp);
	if(m_detector && !m_threadCtx.busy){
		m_threadCtx.chId = chId;
		m_threadCtx.fovId = fovId;
		m_threadCtx.ezoomx = ezoomx;
		m_threadCtx.timestamp = timestamp;
		m_threadFrame = frameOrg;
		iRet = OSA_msgqSendMsg(&m_msgQHdl, NULL, VP_CFG_INTELLNewData, NULL, 0, NULL);
		OSA_assert(iRet == OSA_SOK);
	}
	return iRet;
}

int CINTELLProcess::OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp)
{
	int ret = CProcessBase::OnOSD(chId, fovId, ezoomx, dc, osd, timestamp);
	float scalex = dc.cols/1920.0;
	float scaley = dc.rows/1080.0;
	int winWidth = 40*scalex;
	int winHeight = 40*scaley;
	bool bFixSize = false;

	//OSA_mutexLock(&m_mutexlock);
	if(m_curChId == chId)
	{
		if(!m_bHide && m_bDebug && (m_roi.width != m_imgSize[chId].width || m_roi.height != m_imgSize[chId].height))
			osd->rectangle(m_roi, 0);
		for(int i=0; i<MAX_INTELL_TGT_NUM; i++){
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
	//OSA_mutexUnlock(&m_mutexlock);
}

int CINTELLProcess::private_dynamic_config(int type, int iPrm, void* pPrm)
{
	int iret = OSA_SOK;

	OSA_mutexLock(&m_mutexlock);
	switch(type)
	{
	case VP_CFG_INTELLTargetCount:
		m_nCount = iPrm;
		//m_mmtd->SetTargetNum(m_nCount);
		m_nSelect = m_nCount;
		if(pPrm != NULL)
			m_nSelect = *(int*)pPrm;
		iret = OSA_SOK;
		break;
	case VP_CFG_INTELLEnable:
		memset(m_shTargets, 0, sizeof(m_shTargets));
		memset(m_numMap, -1, sizeof(m_numMap));
		m_curNumber = 0;
		m_bEnable = iPrm;
		m_nDrop = 3;
		if(m_detector!=NULL)
				m_detector->dynamicsetparam(Detector::DETECTDETECTCLOSE,1);
		//m_mmtd->ClearAllMMTD();
		for(int i=0; i<MAX_INTELL_TGT_NUM; i++)
			m_units[m_curChId][i].bNeedDraw = false;
		m_roi = Rect(0, 0, 0, 0);
		if(m_bEnable && pPrm != NULL){
			m_roi = *(cv::Rect*)pPrm;
			OSA_printf("CINTELLProcess::dynamic_config roi(%d,%d,%d,%d)", m_roi.x, m_roi.y,m_roi.width,m_roi.height);
		}
		m_roiSet = m_roi;
		iret = OSA_SOK;
		break;
	case VP_CFG_MainChId:
		memset(m_shTargets, 0, sizeof(m_shTargets));
		memset(m_numMap, -1, sizeof(m_numMap));
		m_curNumber = 0;
		if(m_curChId>=0){
			for(int i=0; i<MAX_INTELL_TGT_NUM; i++)
				m_units[m_curChId][i].bNeedDraw = false;
		}
		m_curChId = iPrm;
		m_nDrop = 3;
		if(m_detector!=NULL)
				m_detector->dynamicsetparam(Detector::DETECTDETECTCLOSE,1);
		//m_mmtd->ClearAllMMTD();
		if(m_curChId>=0){
			for(int i=0; i<MAX_INTELL_TGT_NUM; i++)
				m_units[m_curChId][i].bNeedDraw = false;
		}
		m_roi = m_roiSet;
		break;
	case VP_CFG_GetTargetInfo:
		//OSA_printf("mmtd %s: %d %d", __func__, iPrm, m_shTargets[iPrm].valid);
		if(iPrm>=0 && iPrm<MAX_INTELL_TGT_NUM && m_shTargets[iPrm].valid && pPrm != NULL){
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

int CINTELLProcess::dynamic_config(int type, int iPrm, void* pPrm, int prmSize)
{
	int iret = OSA_SOK;


	//cout << "CINTELLProcess::dynamic_config type " << type << " iPrm " << iPrm << endl;
	iret = CProcessBase::dynamic_config(type, iPrm, pPrm, prmSize);

	if(type<VP_CFG_BASE || type>VP_CFG_INTELLMax)
		return iret;

	if(m_detector){
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
	}
	return iret;
}

#else

#include "intelligentDetectProcess.hpp"

CINTELLProcess::CINTELLProcess(IProcess *proc)
	:CProcessBase(proc, VP_IDENTIFY_INTELL),m_nCount(MAX_INTELL_TGT_NUM),m_nSelect(MAX_INTELL_TGT_NUM),m_curChId(0), m_bEnable(false),m_nDrop(0),
	 m_fovId(0), m_ezoomx(1), m_roi(cv::Rect(0,0,0,0)),m_curNumber(0),m_roiSet(cv::Rect(0,0,0,0)),m_detector(NULL),bProcessBusy(false),processchid(0)
{
	memset(m_shTargets, 0, sizeof(m_shTargets));
}

CINTELLProcess::~CINTELLProcess()
{
}

int CINTELLProcess::process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp)
{
	return CProcessBase::process(chId, fovId, ezoomx, frameOrg, frameGray, timestamp);
}

int CINTELLProcess::dynamic_config(int type, int iPrm, void* pPrm, int prmSize)
{
	return CProcessBase::dynamic_config(type, iPrm, pPrm, prmSize);
}

int CINTELLProcess::OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp)
{
	return CProcessBase::OnOSD(chId, fovId, ezoomx, dc, osd, timestamp);
}

#endif

