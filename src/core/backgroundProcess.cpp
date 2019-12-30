#if 1
#include "backgroundProcess.hpp"

#define CONFIG_BKGD_FILE		"ConfigBkgdDetectFile.yml"

CBkgdDetectProcess::CBkgdDetectProcess(IProcess *proc)
	:CProcessBase(proc, VP_IDENTIFY_BKGD),m_curChId(0), m_bEnable(false)
{
	memset(m_cnt, 0, sizeof(m_cnt));
	for(int chId=0; chId<MAX_CHAN; chId++){
		m_imgSize[chId].width = 1920;
		m_imgSize[chId].height = 1080;
	}

	m_alpha = 0.5;
	m_thr0Min = 0;
	m_thr0Max = 0;
	m_thr1Min = 0;
	m_thr1Max = 0;

	m_mode = 0x01;
	m_threshold = 0.05f;
	ReadCfgFile();
}

CBkgdDetectProcess::~CBkgdDetectProcess()
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

int CBkgdDetectProcess::process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp)
{
	int iRet = CProcessBase::process(chId, fovId, ezoomx, frameOrg, frameGray, timestamp);

	m_imgSize[chId].width = frameGray.cols;
	m_imgSize[chId].height = frameGray.rows;

	if(m_curChId != chId)
		return iRet;

	if(m_bEnable)
	{
		m_cnt[chId]++;
		detect(frameGray, chId);
	}

	return iRet;
}
void CBkgdDetectProcess::detect(Mat frame, int chId)
{
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
	cv::calcHist(&frame, 1, channels, cv::Mat(), hist,1, histSize,  ranges);

	float thr0Min = 0.0;
	float thr0Max = 0.0;
	float thr1Min = 0.0;
	float thr1Max = 0.0;
	int mode = m_mode;
	int whitethr = 256;
	int blackthr = 0;
	int ptthred = (int)(frame.rows*frame.cols*m_threshold);
	int whiteout = 0;
	int blackout = 0;
	if((mode&0x01) == 0x01){
		int cout =0;
		int i;
		for(i=255; i>=0; --i){
			cout+=hist.at<float>(i);
			if(cout>ptthred)
				break;
		}
		if(i<128) i=128;
		thr1Min = i/255.0;
		thr1Max = 1.0;
		whitethr = i;
		whiteout = cout;
	}
	if((mode&0x02) == 0x02){
		int cout =0;
		int i;
		for(i=0; i<256; ++i){
			cout+=hist.at<float>(i);
			if(cout>ptthred)
				break;
		}
		if(i>128) i=128;
		thr0Min = 0;
		thr0Max = i/255.0;
		blackthr = i;
		blackout = cout;
	}
	if(mode == 0x08){

	}

	m_thr0Min = thr0Min;
	m_thr0Max = thr0Max;
	m_thr1Min = thr1Min;
	m_thr1Max = thr1Max;
	//OSA_printf("(%d) ||%d %d %f, %f, || %d %d %f, %f",
	//		ptthred, blackthr, blackout, m_thr0Min, m_thr0Max,
	//		whitethr, whiteout, m_thr1Min, m_thr1Max);
}

int CBkgdDetectProcess::OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp)
{
	int ret = CProcessBase::OnOSD(chId, fovId, ezoomx, dc, osd, timestamp);
}

int CBkgdDetectProcess::dynamic_config(int type, int iPrm, void* pPrm, int prmSize)
{
	int iret = OSA_SOK;

	iret = CProcessBase::dynamic_config(type, iPrm, pPrm, prmSize);

	if(type<VP_CFG_BASE || type>VP_CFG_Max)
		return iret;

	//cout << "CBkgdDetectProcess::dynamic_config type " << type << " iPrm " << iPrm << endl;
	OSA_mutexLock(&m_mutexlock);
	switch(type)
	{
	case VP_CFG_BkgdDetectEnable:
		m_bEnable = iPrm;
		if(pPrm != NULL)
			m_curChId = *(int*)pPrm;
		m_cnt[m_curChId] = 0;
		iret = OSA_SOK;
		break;
	default:
		break;
	}
	OSA_mutexUnlock(&m_mutexlock);
	return iret;
}

int CBkgdDetectProcess::ReadCfgFile()
{
	int iret = -1;
	string cfgFile;
	cfgFile = CONFIG_BKGD_FILE;
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
				m_mode = (int)fr["cfg_mode"];

				float fvalue;
				fvalue = (float)fr["cfg_threshold"];
				if(fvalue>0.0000001 && fvalue<1.0001)
					m_threshold = fvalue;
				fvalue = (float)fr["cfg_alpha"];
				if(fvalue>0.0000001 && fvalue<1.0001)
					m_alpha = fvalue;

				iret = 0;
			}
		}
	}
	OSA_printf("BkgdDetect %s: mode %d thr %f alpha %f\n",__func__, m_mode, m_threshold, m_alpha);
	return iret;
}
#endif
