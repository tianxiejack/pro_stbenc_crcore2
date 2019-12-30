#ifndef		_MMTD_HEAD_
#define		_MMTD_HEAD_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

typedef struct{
	int valid;
	cv::Rect Box;
}TARGETBOX;

#define MAX_TGT_NUM	12

typedef enum{
	SR_DETECT_MODEL_1 = 1,
	SR_DETECT_MODEL_2,
	SR_DETECT_MODEL_3,
}SR_DETECT_ENUM;

class CMMTD
{
public:
	CMMTD();
	~CMMTD();
	/*
	 * MAX_TGT_NUM in "mmthead.h"
	 * tgtNum < MAX_TGT_NUM=12
	 */
	void 	SetTargetNum(int tgtNum);
	/*
	 * default
	 * minArea = 80
	 * maxArea = 3600
	 */
	void 	SetConRegMinMaxArea(int minArea = 80, int maxArea = 3600);
	/*
	 * default
	 * srSize = cv::Size(128, 128)
	 */
	void 	SetSalientSize(cv::Size srSize=cv::Size(128, 128));
	/*
	 * default
	 * gapFrames = 3
	 */
	void 	SetSRDetectGap(int gapFrames = 3);
	/*
	 * default
	 * lumThred = 50
	 */
	void 	SetSRLumThred(int lumThred = 50);
	/*设置静止目标和运动目标的阈值像素
	 * default
	 * stillPixel = 6
	 * movePixel = 16
	 */
	void SetMoveThred(int stillPixel = 6, int movePixel = 16);
	/*
	 * default
	 * lapScaler = 1.25
	 *
	 */
	void SetLapScaler(float lapScaler = 1.25);
	/*
	 * default
	 * srModel = 1 //1: sub block process SR detect; 2: full block process SR detect; 3: sub block + full block process SR detect
	 */
	void SetSRDetectMode(int srModel);
	/*
	 * default
	 * meanThred1 = 35;
	 * stdThred = 12;
	 * meanThred2 = 65;
	 */
	void SetSRDetectParam(int meanThred1, int stdThred, int meanThred2);
	/*
	 *default
	 *sortType = 1 //1: dist sort; 2: area sort; 3: dist+area sort
	 */
	void SetTargetSortType(int sortType);
	/*
	 * default
	 * bClimit = 1
	 * 设置是否使能限制输出宽高比例
	 */
	void SetEnableClimitWH(int bClimit);
	/*
	 * default
	 * bKalman = false;
	 */
	void SetEnableKalman(int bKalman);
	/*
	 * default
	 * bSeaDetect = false;
	 * 设置海面目标检测使能
	 */
	void SetSeaDetect(int bSeaDetect);
	/*
	 * default
	 * segThreshold = 0.04;
	 * 设置分割亮度百分数
	 * clip(segThreshold, 0.01, 0.9)
	 */
	void SetSegThreshold(float segThreshold);

	void	ClearAllMMTD();
	void	MMTDProcess(cv::Mat &image, TARGETBOX *tgBox, cv::Mat &osdFrame, int bDisp = 0);
	void	MMTDProcessRect(cv::Mat &image, TARGETBOX *tgBox, cv::Rect roi, cv::Mat &osdFrame, int bDisp = 0);
	void 	SRDetector(cv::Mat &image, cv::Mat &osdFrame);
	void 	SRDetectorRect(cv::Mat &image, cv::Rect roi, cv::Mat &osdFrame);
	void 	SRDetector2(cv::Mat &image, cv::Mat &osdFrame);
	void 	SRDetectorRect2(cv::Mat &image, cv::Rect roi, cv::Mat &osdFrame);
	void 	SRDetector3(cv::Mat &image, cv::Mat &osdFrame);
	void 	SRDetectorRect3(cv::Mat &image, cv::Rect roi, cv::Mat &osdFrame);
	void 	SRDetectorPre(cv::Mat &image, cv::Mat &osdFrame, cv::Rect *ConRegRect);
	void 	SRDetectorPre2(cv::Mat &image, cv::Mat &osdFrame);
	void 	SRDetectorRectPre(cv::Mat &image, cv::Rect roi, cv::Mat &osdFrame, cv::Rect *ConRegRect);
	void 	SRDetectorRectPre2(cv::Mat &image, cv::Rect roi, cv::Mat &osdFrame);

private:
	int  m_tgtNum;
	int	m_bInterval;

};

#endif
