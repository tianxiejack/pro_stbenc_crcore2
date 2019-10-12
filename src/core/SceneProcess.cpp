#if 1
#include "SceneProcess.hpp"
#include <sys/time.h>


#define CONFIG_BKGD_FILE		"ConfigSceneFile.yml"
using namespace cr_osd;

//#define DBG_WAVE

CSceneProcess::CSceneProcess(IProcess *proc)
	:CProcessBase(proc, VP_IDENTIFY_SCENE),m_curChId(0), m_bEnable(false)
{
	memset(m_cnt, 0, sizeof(m_cnt));
	for(int chId=0; chId<MAX_CHAN; chId++){
		m_imgSize[chId].width = 1920;
		m_imgSize[chId].height = 1080;
	}

	ReadCfgFile();
#ifdef DBG_WAVE
	vArrayOrg.clear();
	vArrayFilter.clear();
	vArrayOrg.resize(300);
	vArrayFilter.resize(300);
	patOrg = IPattern::Create(&vArrayOrg, cv::Rect(0, 0, 600, 200), cv::Scalar(0, 0, 255, 255));
	patFilter = IPattern::Create(&vArrayFilter, cv::Rect(0, 200, 600, 200), cv::Scalar(0, 255, 0, 255));
#endif
}

CSceneProcess::~CSceneProcess()
{
#ifdef DBG_WAVE
	IPattern::Destroy(patOrg);
	IPattern::Destroy(patFilter);
#endif
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

int CSceneProcess::process(int chId, int fovId, int ezoomx, Mat frameOrg, Mat frameGray, uint64_t timestamp)
{
	int iRet = CProcessBase::process(chId, fovId, ezoomx, frameOrg, frameGray, timestamp);

	m_imgSize[chId].width = frameGray.cols;
	m_imgSize[chId].height = frameGray.rows;

	if(m_curChId != chId)
		return iRet;

	if(m_bEnable)
	{
		detect(frameGray, chId);
		m_cnt[chId]++;
	}

	return iRet;
}
#define M3D_PI_DIV_180 (0.017453292519943296)
void CSceneProcess::detect(const Mat& frame, int chId)
{
	//Rect roi = Rect(100-100*sin((m_cnt[chId]%60)*6*M3D_PI_DIV_180), 100, frame.cols-200, frame.rows-200);
	Rect roi = Rect(100, 100, frame.cols-200, frame.rows-200);
	Mat frameROI(frame, roi);

	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	uint64_t us = now.tv_sec*1000000+now.tv_nsec/1000;
	*(uint64_t*)frameROI.data = us;

	if(m_cnt[chId] == 0){
		m_obj.SetStandardSize(cv::Size(frameROI.cols, frameROI.rows));
		m_obj.InitSceneLock(frameROI);
	}else
		m_obj.CalSceneLock(frameROI);
#ifdef DBG_WAVE
	vArrayOrg.erase(vArrayOrg.begin());
	vArrayOrg.push_back(m_obj.m_instanVel.x/15.f);
	//vArrayOrg.push_back(sin((m_cnt[chId]%60)*6*M3D_PI_DIV_180));
	vArrayFilter.erase(vArrayFilter.begin());
	vArrayFilter.push_back(m_obj.m_filterVel.x/15.0f);
#endif
}

int CSceneProcess::OnOSD(int chId, int fovId, int ezoomx, Mat& dc, IDirectOSD *osd, uint64_t timestamp)
{

	int ret = CProcessBase::OnOSD(chId, fovId, ezoomx, dc, osd, timestamp);



}

int CSceneProcess::dynamic_config(int type, int iPrm, void* pPrm, int prmSize)
{
	int iret = OSA_SOK;

	iret = CProcessBase::dynamic_config(type, iPrm, pPrm, prmSize);

	if(type<VP_CFG_BASE || type>VP_CFG_Max)
		return iret;

	//cout << "CSceneProcess::dynamic_config type " << type << " iPrm " << iPrm << endl;
	OSA_mutexLock(&m_mutexlock);
	switch(type)
	{
	case VP_CFG_SceneEnable:
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

int CSceneProcess::ReadCfgFile()
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
			}
		}
	}
	//OSA_printf("BkgdDetect %s: mode %d thr %f alpha %f\n",__func__, m_mode, m_threshold, m_alpha);
	return iret;
}
#endif
