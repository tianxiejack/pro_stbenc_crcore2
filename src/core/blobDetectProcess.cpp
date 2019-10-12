#if 1
#include "blobDetectProcess.hpp"

#define CONFIG_BLOB_FILE		"ConfigBlobDetectFile.yml"

CBlobDetectProcess::CBlobDetectProcess(IProcess *proc)
	:CProcessBase(proc, VP_IDENTIFY_BLOB),m_nCount(1),m_curChId(0), m_bEnable(false),
	 m_fovId(0), m_ezoomx(1), m_bValid(false)
{
	memset(m_units, 0, sizeof(m_units));
	memset(m_cnt, 0, sizeof(m_cnt));
	for(int chId=0; chId<MAX_CHAN; chId++){
		m_imgSize[chId].width = 1920;
		m_imgSize[chId].height = 1080;
	}
	m_threshold = 0.1;
	m_dSize.width = 1920/2;
	m_dSize.height = 1080/2;
	m_minArea = 80*80;
	m_maxArea = m_dSize.area();
	m_minRatioHW = 0.5;
	m_maxRatioHW = 2.0;
	ReadCfgFile();
}

CBlobDetectProcess::~CBlobDetectProcess()
{
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

int CBlobDetectProcess::process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp)
{
	int iRet = CProcessBase::process(chId, fovId, ezoomx, frameOrg, frameGray, timestamp);

	if(m_curChId != chId || !m_bEnable)
		return iRet;

	m_imgSize[chId].width = frameGray.cols;
	m_imgSize[chId].height = frameGray.rows;

	if(m_fovId!= fovId || m_ezoomx!=ezoomx){
		m_fovId= fovId;
		m_ezoomx=ezoomx;
		OSA_mutexLock(&m_mutexlock);
		m_targets[m_curChId].clear();
		for(int i=0; i<MAX_BLOB_TGT_NUM; i++)
			m_units[m_curChId][i].bNeedDraw = false;
		OSA_mutexUnlock(&m_mutexlock);
		m_cnt[chId] = 0;
	}

	if(m_bEnable && m_curChId == chId)
	{
		m_cnt[chId]++;

		if(m_ezoomx<=1){
			detect(chId, frameGray);
		}else{
			//int64 tks = getTickCount();
			cv::Mat rsMat = cv::Mat(frameGray.rows, frameGray.cols, CV_8UC1);
			if(m_ezoomx == 4){
				cv::Mat rsMat0 = cv::Mat(frameGray.rows, frameGray.cols, CV_8UC1);
				mResizeX2(frameGray, rsMat0);
				mResizeX2(rsMat0, rsMat);
			}
			else if(m_ezoomx == 8){
				cv::Mat rsMat0 = cv::Mat(frameGray.rows, frameGray.cols, CV_8UC1);
				mResizeX2(frameGray, rsMat);
				mResizeX2(rsMat, rsMat0);
				mResizeX2(rsMat0, rsMat);
			}else{
				mResizeX2(frameGray, rsMat);
			}
			detect(chId, rsMat);
		}
		update(chId);
	}

	return iRet;
}

int CBlobDetectProcess::detect(int chId, Mat frame)
{
	cv::Size dSize(frame.cols/2, frame.rows/2);
	int minArea = 40*40;
	int maxArea = 400*400;
	float minRatioHW = 0.2;
	float maxRatioHW = 5.0;
	dSize = m_dSize;
	minArea = m_minArea;
	maxArea = m_maxArea;
	minRatioHW = m_minRatioHW;
	maxRatioHW = m_maxRatioHW;
	if(dSize.width>frame.cols)
		dSize.width=frame.cols;
	if(dSize.height>frame.rows)
		dSize.height=frame.rows;
	cv::Point dPos(frame.cols/2-(dSize.width>>1), frame.rows/2-(dSize.height>>1));
	cv::Rect dRect(dPos, dSize);
	Mat dysrc = frame(dRect);
	Mat hist;
	int histSize[1];
	int channels[1];
	const float *ranges[1];
	float hranges[2];
	histSize[0] = 256;
	hranges[0] = 0.0;
	hranges[1] = 256.0;
	ranges[0] = hranges;
	channels[0] = 0;
	cv::calcHist(&dysrc, 1, channels, cv::Mat(), hist,1, histSize,  ranges);
	//imshow("gray", dysrc);
	//waitKey(1);

	int cout =0, ptthred = (int)(dysrc.rows*dysrc.cols*m_threshold);
	int i;
	for(i=255; i>=0; i--){
		cout+=hist.at<float>(i);
		if(cout>ptthred)
			break;
	}
	//if(cout>ptthred*1.1&&i<255)
	//	i++;

	Mat thresh;
	threshold(dysrc, thresh, i, 255, CV_THRESH_BINARY/*CV_THRESH_OTSU*/);
	//imshow("thresh", thresh);
	//waitKey(1);

	vector< vector<Point> > contours;
	findContours(thresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	vBlobTarget vtgts;
	vtgts.clear();
	int index = 0;
	int memArea[1024];
	int ncnt = contours.size();
	for (int i = 0; i < min(ncnt, 1024); i++){
		cv::Rect rc = boundingRect(contours[i]);
		int curArea = rc.area();
		//float ratioHW = rc.
		if(curArea>=minArea && curArea<=maxArea)
		{
			Mat pointsf;
			Mat(contours[i]).convertTo(pointsf, CV_32F);
			RotatedRect box = fitEllipse(pointsf);
			if(box.size.width<1)
				continue;
			float ratioHW = box.size.height/box.size.width;
			if(ratioHW>minRatioHW && ratioHW<maxRatioHW){
				BLOB_TGT_INFO tgt;
				tgt.index = index;
				tgt.area = box.size.width*box.size.height;
				tgt.ratioHW = ratioHW;
				box.center.x += dPos.x;
				box.center.y += dPos.y;
				tgt.targetRect = box;
				memArea[index] = tgt.area;
				vtgts.push_back(tgt);
				index++;
			}
		}
	}

	if(vtgts.size()>1){
		cv::Mat mArea(1, vtgts.size(), CV_32SC1, memArea);
		cv::Mat mIdx(1, vtgts.size(), CV_32SC1, 0);
		cv::sortIdx(mArea, mIdx, CV_SORT_EVERY_ROW+CV_SORT_DESCENDING);
		m_targets[chId].clear();
		for(int i=0; i<vtgts.size(); i++){
			int idx = mIdx.at<int>(i);
			vtgts[idx].index = i;
			m_targets[chId].push_back(vtgts[idx]);
		}
	}else{
		m_targets[chId].clear();
		for(int i=0; i<vtgts.size(); i++){
			vtgts[i].index = i;
			m_targets[chId].push_back(vtgts[i]);
		}
	}
}

void CBlobDetectProcess::update(int chId)
{
	//OSA_printf("%s %d: ch%d size = %ld", __func__, __LINE__, chId, m_targets[chId].size());
	//OSA_mutexLock(&m_mutexlock);
	int cnt = 0;
	{
		int i=0;
		if(m_bEnable&& m_curChId == chId){
			cnt = m_targets[chId].size();
			cnt = m_nCount < cnt ? m_nCount : cnt;
			if(!m_bHide){
				for(i=0; i<cnt; i++)
				{
					m_units[chId][i].bNeedDraw = true;
					m_units[chId][i].orgPos = Point(m_targets[chId][i].targetRect.center.x+0.5,
							m_targets[chId][i].targetRect.center.y+0.5);
					m_units[chId][i].orgRRC = m_targets[chId][i].targetRect;
					m_units[chId][i].orgValue = m_targets[chId][i].index;
				}
			}
		}
		for(;i<MAX_BLOB_TGT_NUM; i++)
			m_units[chId][i].bNeedDraw = false;
	}
	if(cnt > 0){
		m_bValid = true;
		m_rc = m_targets[chId][0].targetRect.boundingRect();
		m_pos = m_targets[chId][0].targetRect.center;
	}else{
		m_bValid = false;
	}
	//OSA_mutexUnlock(&m_mutexlock);
}

__inline__ void draw_center(Mat dc, Point center, int length, CvScalar color, int thickness)
{
	Point pt1,pt2;
	pt1.x=center.x-length;pt1.y=center.y-length;
	pt2.x=center.x+length;pt2.y=center.y+length;
	line(dc, pt1, pt2, color, thickness, CV_AA, 0 );
	pt1.x=center.x-length;pt1.y=center.y+length;
	pt2.x=center.x+length;pt2.y=center.y-length;
	line(dc, pt1, pt2, color, thickness, CV_AA, 0 );
}

int CBlobDetectProcess::OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp)
{
	int ret = CProcessBase::OnOSD(chId, fovId, ezoomx, dc, osd, timestamp);

	float scalex = dc.cols/1920.0;
	float scaley = dc.rows/1080.0;

	//OSA_mutexLock(&m_mutexlock);
	if(m_curChId == chId){
		for(int i=0; i<MAX_BLOB_TGT_NUM; i++){
			if(m_units[chId][i].bNeedDraw){
				RotatedRect rc = m_units[chId][i].orgRRC;
				if(m_imgSize[chId].width != dc.cols && m_imgSize[chId].height != dc.rows)
					rc = tRectScale(rc, m_imgSize[chId], cv::Size(dc.cols, dc.rows));
				m_units[chId][i].drawRRC = rc;
				osd->ellipse(m_units[chId][i].drawRRC, m_units[chId][i].orgValue+1, (m_nCount>1) ? 6 : 2);
			}
		}
	}
	//OSA_mutexUnlock(&m_mutexlock);
}

int CBlobDetectProcess::dynamic_config(int type, int iPrm, void* pPrm, int prmSize)
{
	int iret = OSA_SOK;

	iret = CProcessBase::dynamic_config(type, iPrm, pPrm, prmSize);

	if(type<VP_CFG_BASE || type>VP_CFG_Max)
		return iret;

	//cout << "CBlobDetectProcess::dynamic_config type " << type << " iPrm " << iPrm << endl;
	OSA_mutexLock(&m_mutexlock);
	switch(type)
	{
	case VP_CFG_BLOBTargetCount:
		m_nCount = iPrm;
		iret = OSA_SOK;
		break;
	case VP_CFG_BLOBEnable:
		m_bEnable = iPrm;
		m_targets[m_curChId].clear();
		for(int i=0; i<MAX_BLOB_TGT_NUM; i++)
			m_units[m_curChId][i].bNeedDraw = false;
		m_cnt[m_curChId] = 0;
		iret = OSA_SOK;
		break;
	case VP_CFG_MainChId:
		if(m_bEnable){
			m_targets[m_curChId].clear();
			for(int i=0; i<MAX_BLOB_TGT_NUM; i++)
				m_units[m_curChId][i].bNeedDraw = false;
			m_curChId = iPrm;
			m_targets[m_curChId].clear();
			for(int i=0; i<MAX_BLOB_TGT_NUM; i++)
				m_units[m_curChId][i].bNeedDraw = false;
		}else{
			m_curChId = iPrm;
		}
		m_cnt[m_curChId] = 0;
		break;
	default:
		break;
	}
	OSA_mutexUnlock(&m_mutexlock);
	return iret;
}

int CBlobDetectProcess::ReadCfgFile()
{
	int iret = -1;
	string cfgFile;
	cfgFile = CONFIG_BLOB_FILE;
	FILE *fp = fopen(cfgFile.c_str(), "rt");
	if(fp != NULL)
	{
		fseek(fp, 0, SEEK_END);
		int len = ftell(fp);
		fclose(fp);

		if(len > 10)
		{
			FileStorage fr(cfgFile, FileStorage::READ);
			if(fr.isOpened())
			{
				float fvalue;
				int ivalue;
				fvalue = (float)fr["cfg_blob_threshold"];
				if(fvalue>0.001 && fvalue<1.001)
					m_threshold = fvalue;

				ivalue = (int)fr["cfg_blob_roi_width"];
				if(ivalue > 0)
					m_dSize.width = ivalue;
				ivalue = (int)fr["cfg_blob_roi_height"];
				if(ivalue > 0)
					m_dSize.height = ivalue;

				ivalue = (int)fr["cfg_blob_area_min"];
				if(ivalue > 0)
					m_minArea = ivalue;
				ivalue = (int)fr["cfg_blob_area_max"];
				if(ivalue > 0)
					m_maxArea = ivalue;

				fvalue = (float)fr["cfg_blob_ratio_min"];
				if(fvalue>0.001 && fvalue<1.001)
					m_minRatioHW = fvalue;
				fvalue = (float)fr["cfg_blob_ratio_max"];
				if(fvalue>0.001 && fvalue<10000.001)
					m_maxRatioHW = fvalue;

				iret = 0;
			}
		}
	}
	OSA_printf("BlobDetect %s: thr %f roi(%dx%d) area(%d,%d) ratio(%f,%f)\n", __func__,
			m_threshold, m_dSize.width, m_dSize.height, m_minArea, m_maxArea, m_minRatioHW, m_maxRatioHW);
	return iret;
}
#endif
