/*
 * gluVideoWindow.hpp
 *
 *  Created on: Feb 14, 2019
 *      Author: ubuntu
 */

#ifndef GLUVIDEOWINDOW_HPP_
#define GLUVIDEOWINDOW_HPP_

#include "gluWindow.hpp"
#include "glvideoRender.hpp"

#define VWIN_RENDER_MAX		(9)
#define VWIN_CHAN_MAX		(4)

typedef struct _gluVideoWindow_chninfo{
	int w;
	int h;
	int c;
	int fps;
}VWIND_ChnInfo;

typedef struct _gluVideoWindow_param{
	bool bFullScreen;
	int disFPS;
	float disSched;
	char szScriptFile[256];
	VWIND_ChnInfo channelInfo[VWIN_CHAN_MAX];
	int nChannels;
	int nQueueSize;
	int memType;
	void (*renderfunc)(int winId, int stepIdx, int stepSub, int context);
}VWIND_Prm;


class CGluVideoWindow : public CGluWindow
{
public:
	enum{
		RUN_ENTER = 0,
		RUN_WIN,
		RUN_SWAP,
		RUN_LEAVE
	};
	typedef enum{
		VWIN_CFG_ChId = 0,
		VWIN_CFG_BlendChId,
		VWIN_CFG_MaskChId,
		VWIN_CFG_CropEnable,
		VWIN_CFG_CropRect,
		VWIN_CFG_VideoTransMat,
		VWIN_CFG_ViewTransMat,
		VWIN_CFG_BlendTransMat,
		VWIN_CFG_BlendPrm,
		VWIN_CFG_ViewPos,
		VWIN_CFG_Max
	}VWIN_CFG;
	CGluVideoWindow(const cv::Rect& rc, int parent = 0, int nRender = 2);
	virtual ~CGluVideoWindow();
	int Create(const VWIND_Prm& param);
	virtual void Destroy();
	virtual void Display();
	virtual void Display2();
	virtual void Reshape(int width, int height);
	virtual CGLVideoBlendRender* CreateVedioRender(int index);
	virtual int dynamic_config(VWIN_CFG type, int iPrm, void* pPrm);

	std::vector<CGLVideo *> m_vvideos;
	std::vector<CGLVideoBlendRender *> m_vvideoRenders;
	std::vector<cr_core::RenderBase *> vRenders;
	cr_osa::OSA_BufHndl *m_bufQue[VWIN_CHAN_MAX];
	cv::Matx33f m_warpMat;
	VWIND_Prm m_initPrm;
	int m_nRender;
public:
	int m_blendMap[VWIN_CHAN_MAX];
	int m_maskMap[VWIN_CHAN_MAX];
	GLMatx44f m_glmat44fBlend[VWIN_CHAN_MAX*VWIN_CHAN_MAX];
	GLV_BlendPrm m_glBlendPrm[VWIN_CHAN_MAX*VWIN_CHAN_MAX];
protected:
	int setFPS(int fps);
	OSA_MutexHndl m_mutex;
//private:
protected:
	uint64  m_interval;
	double m_telapse;
	uint64  m_tmBak[VWIN_CHAN_MAX];
	int64   m_tmRender;
    int m_nSwapTimeOut;
	pthread_mutex_t render_lock;    /**< Used for synchronization. */
	pthread_cond_t render_cond;     /**< Used for synchronization. */
    uint64_t render_time_sec;       /**< Seconds component of the time for which a frame should be displayed. */
    uint64_t render_time_nsec;      /**< Nanoseconds component of the time for which a frame should be displayed. */
    struct timespec last_render_time;   /**< Rendering time for the last buffer. */
    unsigned long rCount;

};


#endif /* GLUVIDEOWINDOW_HPP_ */
