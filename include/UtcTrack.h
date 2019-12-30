#ifndef _UTC_TRK_
#define _UTC_TRK_

#include "PCTracker.h"

namespace TRACKER_ALG{

UTCTRACK_HANDLE CreateUtcTrk();

void DestroyUtcTrk(UTCTRACK_HANDLE handle);

UTC_RECT_float UtcTrkAcq(UTCTRACK_OBJ* pUtcTrkObj, IMG_MAT &frame, UTC_ACQ_param inputParam);

UTC_RECT_float UtcTrkProc(UTCTRACK_OBJ *pUtcTrkObj, IMG_MAT &frame, int *pRtnStat);

UTC_RECT_float MedTrkAcq(UTCTRACK_OBJ* pUtcTrkObj, IMG_MAT &frame, UTC_ACQ_param inputParam);

UTC_RECT_float MedTrkProc(UTCTRACK_OBJ *pUtcTrkObj, IMG_MAT &frame, int *pRtnStat);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void UtcSetDynParam(UTCTRACK_OBJ *pUtcTrkObj, UTC_DYN_PARAM dynamicParam);
void UtcSetUpFactor(UTCTRACK_OBJ *pUtcTrkObj, float up_factor);

void UtcGetOptValue(UTCTRACK_OBJ *pUtcTrkObj, float *optValue);
void UtcSetIntervalFrame(UTCTRACK_OBJ *pUtcTrkObj, int gapFrame);

void UtcSetPLT_BS(UTCTRACK_OBJ *pUtcTrkObj, tPLT pltWork, BS_Type bsType);
/*set track search target size
 * default
 * resTrackObj={
	.res_distance = 80,
	.res_area = 5000,
};
 */
void UtcSetRestraint(UTCTRACK_OBJ *pUtcTrkObj,TRK_SECH_RESTRAINT resTraint);
/*
 * default disable enhance
 * bEnable = false;
 */
void UtcSetEnhance(UTCTRACK_OBJ *pUtcTrkObj, bool	bEnable);

/*if enable enhance, ifCliplimit will be efficient
 * fCliplimit = 4.0;
 */
void UtcSetEnhfClip(UTCTRACK_OBJ *pUtcTrkObj, float fCliplimit);
/*
 * default disable gaussfilter
 * bEnable = false;
 */
void UtcSetBlurFilter(UTCTRACK_OBJ *pUtcTrkObj, bool	bEnable);
/*
 * default
 * bPredict:false
 */
void UtcSetPredict(UTCTRACK_OBJ *pUtcTrkObj, bool bPredict);
/*
 * default
 * mvPixelX:20
 * mvPixelY:10
 */
void UtcSetMvPixel(UTCTRACK_OBJ *pUtcTrkObj, int mvPixelX, int mvPixelY);
/*
 * default
 * mvPixelX2:30
 * mvPixelY2:20
 */
void UtcSetMvPixel2(UTCTRACK_OBJ *pUtcTrkObj,  int mvPixelX2, int mvPixelY2);
/*
 * default
 * segPixelX:600
 * segPixelY:450
 */
void UtcSetSegPixelThred(UTCTRACK_OBJ *pUtcTrkObj,  int segPixelX, int segPixelY);
/*
 *default
 *maxValue = 0xFFFFFFF;
 **/
void UtcSetRoiMaxWidth(UTCTRACK_OBJ *pUtcTrkObj, int maxValue);
/*
 *default
 *bBigSearch = false
 *设置是否使能大范围搜索
 **/
void UtcSetBigSearch(UTCTRACK_OBJ *pUtcTrkObj, bool  bBigSearch);
/*
 * default
 *  bExtSearch = false
 *  使能设置扩大上下搜索至三个窗口高度
 */
void UtcSetExtSearch(UTCTRACK_OBJ *pUtcTrkObj, bool  bExtSearch);
/*
 *default
 *bSceneMV = false
 *设置场景运动使能
 **/
void UtcSetSceneMV(UTCTRACK_OBJ *pUtcTrkObj, bool  bSceneMV);
/*
 * 得到场景滤波运动速度
 */
void UtcGetSceneMV(UTCTRACK_OBJ *pUtcTrkObj, float *speedx, float *speedy);
/*
 * 得到场景瞬时运动速度和可信度
 */
void UtcGetSceneinstanMV(UTCTRACK_OBJ *pUtcTrkObj, float *speedx, float *speedy, float *confidence);
/*
 * 得到场景滤波后，每秒运动像素速度
 */
void UtcGetScenePixPerSec(UTCTRACK_OBJ *pUtcTrkObj, float *speedx, float *speedy);
/*
 * 得到场景运动滤波后的角度
 */
void UtcGetSceneFilterAngular(UTCTRACK_OBJ *pUtcTrkObj, float *angular);
/*
 *default
 *	trkAngleThred = 80.0;
 *	clip(30, 180, trkAngleThred)
 *设置跟踪搜索角度范围，<180 or >30
 */
void UtcSetTrkAngleThred(UTCTRACK_OBJ *pUtcTrkObj, float trkAngleThred);
/*
 * default
 * bSearchAxis = true
 * bEstimateSearch = false
 * 使能搜索中心为靶心，默认使能
 * 场景运动使能(bSceneMV)打开状态下，可使能预测位置搜索，默认为关闭
 * 如果bSearchAxis=true, bEstimateSearch=true，状态下，优先使用bSearchAxis=true靶心搜索
 */
void UtcSetSechAxis(UTCTRACK_OBJ *pUtcTrkObj, bool bSearchAxis, bool bEstimateSearch);
/*
 * default
 * trajSechType = DO_NOT_JUDGE;
 * 设置轨迹线上搜索类型，不设置时，轨迹线角度为0度
 */
void UtcSetTrajSechType(UTCTRACK_OBJ *pUtcTrkObj, TRAJ_SECH_TYPE trajSechType);
/*
 * default
 * bSimilarDetect = false
 * 使能轨迹线上相似目标检测
 */
void UtcSetSimilarDetect(UTCTRACK_OBJ *pUtcTrkObj, bool bSimilarDetect);
/*
 * UTC_RECT_float 	searchRect[25]
 * 测试轨迹线搜索区域
 */
void UtcGetSearchRect(UTCTRACK_OBJ *pUtcTrkObj, UTC_RECT_float *secRect);
/*
 * UTC_RECT_float		similarRc
 * 测试轨迹线上相似目标提取
 */
void UtcGetSimilarRect(UTCTRACK_OBJ *pUtcTrkObj, UTC_RECT_float *similarRc);
/*
 * panAng: deg
 * tiltAng: deg
 * panVel: deg/s
 * tiltVel: deg/s
 * 反馈平台方位角，俯仰角，以及方位角速度，俯仰角速度==>ALG
 */
void UtcSetPlatPanTilt(UTCTRACK_OBJ *pUtcTrkObj, float panAng, float tiltAng, float panVel, float tiltVel);
/*
 * 得到滤波后的平台运动速度
 */
void UtcGetPLTMV(UTCTRACK_OBJ *pUtcTrkObj, float *speedx, float *speedy);
/*
 * 为场景运动计算，设置处理视频通道原始帧频
 * 默认为25帧
 */
void UtcSetFrameFreq(UTCTRACK_OBJ *pUtcTrkObj, int framefreq);
/*
 * 设置Kalman滤波参数
 * 默认
 * proc_noise_cov_xy = 0.1
 * proc_noise_cov_vxvy = 0.01
 * measure_noise_cov_xy = 0.5
 */
void UtcSetUKFParam(UTCTRACK_OBJ *pUtcTrkObj, UKF_COV_PARAM param);
/*
 * 设置轨迹UKF预测分析
 * 默认
 * bTrajAnalysis = false;
 */
void UtcSetTrajAnalysis(UTCTRACK_OBJ *pUtcTrkObj, bool bTrajAnalysis);
/*
 * 设置是否显示多峰查看
 * 默认
 * bMultiModalDisp = false;
 */
void UtcSetMultiModalDisp(UTCTRACK_OBJ *pUtcTrkObj, bool bMultiModalDisp);
/*
 *  是否使能，对搜索成功的区域进行后续的验证
 *  默认
 *  bSechValid = false;
 */
void UtcSetSechValid(UTCTRACK_OBJ *pUtcTrkObj, bool bSechValid);
/*
 * 返回没有筛选之前搜索成功区域
 *
 */
void UtcGetSechOKRect(UTCTRACK_OBJ *pUtcTrkObj, UTC_RECT_float *sechOKRc);
/*
 * 返回搜索过程中估算的目标位置
 *
 */
void UtcGetSechEstimateRect(UTCTRACK_OBJ *pUtcTrkObj, UTC_RECT_float *sechEstimateROI);
/*
 *设置使能算法各模块耗费时间打印
 *默认
 *[Track ALG] bTrkPrintf = false
 *[SceneCal ALG] bScenePrintf = false
 *[Similar Detect ALG] bSimPrintf = false;
 */
void UtcSetPrintTS(UTCTRACK_OBJ *pUtcTrkObj, bool bTrkPrintf, bool bScenePrintf, bool bSimPrintf);
/*
 * UTC_RECT_float 	histRect[150]
 * float *angular
 * 测试跟踪目标的历史轨迹
 */
void UtcGetHistTrajectory(UTCTRACK_OBJ *pUtcTrkObj, UTC_RECT_float *histRect,  float *angular);
/*
 *
 * bAveTrkPos = false;
 * 设置使能短时间(<40帧)搜索平均跟踪位置约束
 */
void UtcSetAveTrkPos(UTCTRACK_OBJ *pUtcTrkObj, bool  bAveTrkPos);
/*
 *
 * bSimilarDetectThred :false-->默认固定阈值；true-->动态阈值
 * default:true
 * 设置相似目标检测阈值类型
 */
void UtcSetSimilarThredType(UTCTRACK_OBJ *pUtcTrkObj, bool bSimilarDetectThred);
/*
 * default:
 * minArea:100
 * maxArea:40000
 * 设置动态目标最小面积和最大面积
 */
void UtcSetDynamicTargetSize(UTCTRACK_OBJ *pUtcTrkObj, int minArea, int maxArea);
/*
 *
 * bDynTargetDetect :false
 * default:false
 * 设置使能二次搜索动态目标检测
 */
void UtcSetDynTargetDetect(UTCTRACK_OBJ *pUtcTrkObj, bool bDynTargetDetect);
}

#endif
