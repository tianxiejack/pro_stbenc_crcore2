#ifndef 		_SCENE_PROCE_HPP_
#define		_SCENE_PROCE_HPP_

#include "SceneOptFlow.hpp"
#include "kalman_filters.hpp"


using namespace cv;
using namespace std;
using namespace OptFlowTrk;

namespace OptFlowTrk{

typedef unsigned int UInt32;

#define		MAX_SCENE_FRAMES		200

typedef struct{
	uint64_t ts; //us
	uint64_t deltaT;
	cv::Point2f	mv;
	float confidence;
}SceneState;

typedef enum{
	INIT_SCENE = 0x0,
	RUN_SCENE,
}SCENE_STATUS;

typedef std::vector<SceneState>::iterator		StateVectorIterator;

class	SceneProc{
public:
	SceneProc();
	virtual ~SceneProc();

	void InitSceneLock(const cv::Mat image);
	void CalSceneLock(const cv::Mat image);
	void  AnalyseSceneLock();

	void SetStandardSize(cv::Size sz);
	void InitUKFilter(float x, float y);

	SCENE_STATUS GetSceneStatus(){return sceneStatus;};
	void SetSceneStatus(SCENE_STATUS status){sceneStatus = status;};


public:

	cv::Point2f						m_filterVel;//ukf filter result
	cv::Point2f						m_instanVel;
	float							m_instanConfd;

protected:

	std::vector<SceneState>	m_sceneState;


	Ptr<SceneOptFlow>		m_ScenePtr;
	Ptr<UnscentedKalmanFilter> uncsentedKalmanFilter;
	Ptr<UkfSystemModel> model;

	RNG m_rng;
	double	m_accNoise;
	uint64_t m_bakTS;

private:
	cv::Size	standardSize;
	SCENE_STATUS	sceneStatus;


	cv::Point2f							m_sumDelta;
	cv::Point2f							m_bakDetlta;//pre filter result

};

}

#endif

