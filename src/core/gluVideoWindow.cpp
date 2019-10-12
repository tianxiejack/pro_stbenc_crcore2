/*
 * gluVideoWindow.cpp
 *
 *  Created on: Feb 14, 2019
 *      Author: ubuntu
 */

#include <linux/videodev2.h>
#include "osa_image_queue.h"
#include "gluVideoWindow.hpp"
#include "glvideo.hpp"
#include <cuda_runtime_api.h>
#include <GLBatchMini.h>

using namespace cv;
using namespace cr_osa;

static std::vector<CGLVideo *> glvvideos;
static int raf = 0;

CGluVideoWindow::CGluVideoWindow(const cv::Rect& rc, int parent, int nRender):CGluWindow(rc, parent),m_nRender(nRender),
		m_tmRender(0ul),m_telapse(5.0),m_nSwapTimeOut(0),rCount(0l)
{
	memset(m_bufQue, 0, sizeof(m_bufQue));
	memset(&m_initPrm, 0, sizeof(m_initPrm));
	memset(m_tmBak, 0, sizeof(m_tmBak));
	m_initPrm.disSched = 3.5;
	for(int chId = 0; chId<VWIN_CHAN_MAX; chId++){
		m_blendMap[chId] = -1;
		m_maskMap[chId] = -1;
	}

	for(int i=0; i<VWIN_CHAN_MAX*VWIN_CHAN_MAX; i++){
		m_glmat44fBlend[i] = cv::Matx44f::eye();
		m_glBlendPrm[i].fAlpha = 0.5f;
		m_glBlendPrm[i].thr0Min = 0;
		m_glBlendPrm[i].thr0Max = 0;
		m_glBlendPrm[i].thr1Min = 0;
		m_glBlendPrm[i].thr1Max = 0;
	}
	OSA_mutexCreate(&m_mutex);
	m_warpMat = cv::Matx33f::eye();
}

CGluVideoWindow::~CGluVideoWindow()
{
	OSA_mutexDelete(&m_mutex);
}

int CGluVideoWindow::setFPS(int fps)
{
    if (fps == 0)
    {
        OSA_printf("[Render]Fps 0 is not allowed. Not changing fps");
        return -1;
    }
    pthread_mutex_lock(&render_lock);
    m_initPrm.disFPS = fps;
    m_interval = (1000000000ul)/(uint64)m_initPrm.disFPS;
    render_time_sec = m_interval / 1000000000ul;
    render_time_nsec = (m_interval % 1000000000ul);
    memset(&last_render_time, 0, sizeof(last_render_time));
    pthread_mutex_unlock(&render_lock);
    return 0;
}

#include <X11/Xlib.h>
static int getDisplayResolution(const char* display_name,uint32_t &width, uint32_t &height)
{
    int screen_num;
    Display * x_display = XOpenDisplay(display_name);
    if (NULL == x_display)
    {
        return  -1;
    }

    screen_num = DefaultScreen(x_display);
    width = DisplayWidth(x_display, screen_num);
    height = DisplayHeight(x_display, screen_num);

    XCloseDisplay(x_display);
    x_display = NULL;

    return 0;
}

int CGluVideoWindow::Create(const VWIND_Prm& param)
{
	static bool bInitGLUT = false;
	if(!bInitGLUT)
	{
		char strParams[][32] = {"DS_RENDER", "-display", ":0"};
		char *argv[3];
		int argc = 1;
		for(int i=0; i<argc; i++)
			argv[i] = strParams[i];
		uint32_t screenWidth = 0, screenHeight = 0;
		if(getDisplayResolution(NULL, screenWidth, screenHeight) == 0)
		{
			//m_winWidth = screenWidth;
			//m_winHeight = screenHeight;
		}
		OSA_printf("screen resolution: %d x %d", screenWidth, screenHeight);
	    glutInit(&argc, argv);
	    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
	    bInitGLUT = true;
	}
	int iRet = CGluWindow::Create(param.bFullScreen);

	memcpy(&m_initPrm, &param, sizeof(VWIND_Prm));

	int status=OSA_SOK;
    pthread_mutexattr_t mutex_attr;
    pthread_condattr_t cond_attr;
    status |= pthread_mutexattr_init(&mutex_attr);
    status |= pthread_condattr_init(&cond_attr);
    status |= pthread_condattr_setclock(&cond_attr, CLOCK_MONOTONIC);
    status |= pthread_mutex_init(&render_lock, &mutex_attr);
    status |= pthread_cond_init(&render_cond, &cond_attr);
    pthread_condattr_destroy(&cond_attr);
    pthread_mutexattr_destroy(&mutex_attr);

    setFPS(m_initPrm.disFPS);

	OSA_mutexLock(&m_mutex);
	if(glvvideos.size()==0){
		for(int chId=0; chId<param.nChannels; chId++){
			int nDrop = (param.channelInfo[chId].fps == 0) ? 0 : param.disFPS/param.channelInfo[chId].fps;
			CGLVideo *pVideo = new CGLVideo(chId, cv::Size(param.channelInfo[chId].w, param.channelInfo[chId].h),
					(param.channelInfo[chId].c == 1) ? V4L2_PIX_FMT_GREY : V4L2_PIX_FMT_RGB24,
					param.channelInfo[chId].fps, nDrop, param.memType);
			glvvideos.push_back(pVideo);
			m_vvideos.push_back(pVideo);
			m_bufQue[chId] = &pVideo->m_bufQue;
		}
	}else{
		for(int chId=0; chId<glvvideos.size(); chId++){
			m_vvideos.push_back(glvvideos[chId]);
			m_bufQue[chId] = &glvvideos[chId]->m_bufQue;
		}
	}
	raf ++;
	OSA_mutexUnlock(&m_mutex);

	for(int i=0; i<m_nRender; i++){
		CGLVideoBlendRender *render = CreateVedioRender(i);
		OSA_assert(render != NULL);
		m_vvideoRenders.push_back(render);
	}

	OSA_assert(iRet == OSA_SOK);

	return iRet;
}

void CGluVideoWindow::Destroy()
{
	OSA_printf("[%d] %s %d: GLU%d enter", OSA_getCurTimeInMsec(), __func__, __LINE__, m_winId);
	CGluWindow::Destroy();
	OSA_mutexLock(&m_mutex);
	raf --;
	if(raf == 0)
	{
		int nChannels = glvvideos.size();
		for(int chId=0; chId<nChannels; chId++){
			CGLVideo *pVideo = glvvideos[chId];
			delete pVideo;
		}
		glvvideos.clear();
	}
	m_vvideos.clear();
	for(int i=0; i<m_nRender; i++){
		CGLVideoBlendRender *render = m_vvideoRenders[i];
		delete render;
	}
	m_vvideoRenders.clear();
	OSA_mutexUnlock(&m_mutex);
    pthread_mutex_lock(&render_lock);
    pthread_cond_broadcast(&render_cond);
    pthread_mutex_unlock(&render_lock);
    pthread_mutex_destroy(&render_lock);
    pthread_cond_destroy(&render_cond);
}

void CGluVideoWindow::Display()
{
	if(m_nRender == 0)
		return;
	//OSA_printf("[%d] %s %d: GLU%d enter", OSA_getCurTimeInMsec(), __func__, __LINE__, m_winId);
	OSA_mutexLock(&m_mutex);
	int chId, winId;
	for(chId = 0; chId<VWIN_CHAN_MAX; chId++){
		if(m_bufQue[chId] != NULL && m_bufQue[chId]->bMap){
			OSA_BufInfo* info = NULL;
			cv::Mat img;
			bool bDevMem = false;
			for(int i=0; i<m_bufQue[chId]->numBuf; i++){
				cuMap(&m_bufQue[chId]->bufInfo[i]);

				info = &m_bufQue[chId]->bufInfo[i];
				bDevMem = (info->memtype == memtype_glpbo || info->memtype == memtype_cudev || info->memtype == memtype_cumap);
				void *data = (bDevMem) ? info->physAddr : info->virtAddr;
				if(info->channels == 1){
					img = cv::Mat(info->height, info->width, CV_8UC1, data);
				}else{
					img = cv::Mat(info->height, info->width, CV_8UC3, data);
				}

				cv::Mat full = cv::Mat(img.rows,img.cols,img.type(), cv::Scalar(255, 0, 0, 0));
				if(bDevMem){
					cudaMemcpy(img.data, full.data, img.rows*img.cols*img.channels(), cudaMemcpyHostToDevice);
				}else{
					full.copyTo(img);
				}

				image_queue_putEmpty(m_bufQue[chId], &m_bufQue[chId]->bufInfo[i]);
			}
			m_bufQue[chId]->bMap = false;
		}
	}
	OSA_mutexUnlock(&m_mutex);

	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_MULTISAMPLE);
	glDisable(GL_COLOR_MATERIAL);

	int64 tStamp[10];
	tStamp[0] = getTickCount();
	static int64 tstart = 0ul, tend = 0ul;
	tstart = tStamp[0];
	if(tend == 0ul)
		tend = tstart;

	if(1)
	{
		uint64_t telapse_ns = (uint64_t)(m_telapse*1000000.f);
		uint64_t sleep_ns = render_time_nsec - telapse_ns;
		if (last_render_time.tv_sec != 0 && last_render_time.tv_nsec != 0 && render_time_nsec>telapse_ns)
		{
			pthread_mutex_lock(&render_lock);
			struct timespec now;
			clock_gettime(CLOCK_MONOTONIC, &now);
			struct timespec waittime = last_render_time;
			waittime.tv_nsec += sleep_ns;
			waittime.tv_sec += waittime.tv_nsec / 1000000000UL;
			waittime.tv_nsec %= 1000000000UL;
	        __suseconds_t cur_us, to_us;
	        cur_us = (now.tv_sec * 1000000.0 + now.tv_nsec/1000.0);
	        to_us = (waittime.tv_sec * 1000000.0 + waittime.tv_nsec / 1000.0);
	        if(to_us > cur_us)
	        	pthread_cond_timedwait(&render_cond, &render_lock,&waittime);
			pthread_mutex_unlock(&render_lock);
		}
		else{
			//OSA_printf("%s %d: last_render_time.tv_sec=%ld render_time_nsec=%ld telapse_ns=%ld sleep_ns=%ld",
			//		__func__, __LINE__, last_render_time.tv_sec, render_time_nsec, telapse_ns, sleep_ns);
		}
	}

	tStamp[1] = getTickCount();

	if(m_initPrm.renderfunc != NULL)
		m_initPrm.renderfunc(0, RUN_ENTER, 0, 0);
	tStamp[2] = getTickCount();

	OSA_mutexLock(&m_mutex);
	int nChannels = m_vvideos.size();
	//std::cout << __FILE__ << " " << __LINE__ << ": nChannels = " << nChannels << std::endl;
	for(chId=0; chId<nChannels; chId++){
		CGLVideo *pVideo = m_vvideos[chId];
		pVideo->update();
	}
	OSA_mutexUnlock(&m_mutex);
	m_warpMat = m_vvideos[0]->m_warpMat;
	tStamp[3] = getTickCount();

	tStamp[4] = getTickCount();

	OSA_mutexLock(&m_mutex);
	OSA_assert(m_nRender == m_vvideoRenders.size());
	for(winId=0; winId<m_nRender; winId++){
		CGLVideoBlendRender *render = m_vvideoRenders[winId];
		OSA_assert(render != NULL);
		if(render->m_video == NULL)
			continue;
		chId = render->m_video->m_idx;
		if(chId < 0 || chId >= VWIN_CHAN_MAX)
			continue;
		int blend_chId = m_blendMap[chId];
		if(blend_chId>=0){
			render->blend(m_vvideos[blend_chId]);
			render->matrix(m_glmat44fBlend[chId*VWIN_CHAN_MAX+blend_chId]);
			render->params(m_glBlendPrm[chId*VWIN_CHAN_MAX+blend_chId]);
		}else{
			render->blend(NULL);
		}
		//OSA_printf("%s %d: %s m_video = %p win%d ch%d", __FILE__, __LINE__, __func__, render->m_video, winId, chId);
		render->render();
		if(m_initPrm.renderfunc != NULL)
			m_initPrm.renderfunc(0, RUN_WIN, winId, chId);
	}

	for(int i=0; i<vRenders.size();i++)
		vRenders[i]->Draw();

	OSA_mutexUnlock(&m_mutex);

	if(m_initPrm.renderfunc != NULL)
		m_initPrm.renderfunc(0, RUN_SWAP, 0, 0);

	tStamp[5] = getTickCount();
	int64 tcur = tStamp[5];
	m_telapse = (tStamp[5] - tStamp[1])*0.000001f + m_initPrm.disSched;

	if (last_render_time.tv_sec != 0 && last_render_time.tv_nsec != 0)
	{
		pthread_mutex_lock(&render_lock);
		struct timespec waittime = last_render_time;
		last_render_time.tv_sec += render_time_sec;
		last_render_time.tv_nsec += render_time_nsec;
		last_render_time.tv_sec += last_render_time.tv_nsec / 1000000000UL;
		last_render_time.tv_nsec %= 1000000000UL;
    	struct timespec now;
    	clock_gettime(CLOCK_MONOTONIC, &now);
        __suseconds_t cur_us, rd_us;
        cur_us = (now.tv_sec * 1000000.0 + now.tv_nsec/1000.0);
        rd_us = (last_render_time.tv_sec * 1000000.0 + last_render_time.tv_nsec / 1000.0);
        if (rd_us>cur_us)
        {
    		waittime.tv_sec += render_time_sec;
    		waittime.tv_nsec += render_time_nsec-500000UL;
    		waittime.tv_sec += waittime.tv_nsec / 1000000000UL;
    		waittime.tv_nsec %= 1000000000UL;
    		pthread_cond_timedwait(&render_cond, &render_lock,&waittime);
        }
        else if(rd_us+10000UL<cur_us)
        {
            //OSA_printf("%s %d: win%d frame_is_late(%ld us)", __func__, __LINE__, glutGetWindow(), cur_us - rd_us);
           	memset(&last_render_time, 0, sizeof(last_render_time));
        }
		pthread_mutex_unlock(&render_lock);
	}

	int64 tSwap = getTickCount();
	CGluWindow::Display();
	tStamp[6] = getTickCount();

	if(tStamp[6]-tSwap>5000000UL)
		m_nSwapTimeOut++;
	else
		m_nSwapTimeOut = 0;
	if ((last_render_time.tv_sec == 0 && last_render_time.tv_nsec == 0) || m_nSwapTimeOut>=3)
	{
    	struct timespec now;
    	clock_gettime(CLOCK_MONOTONIC, &now);
		last_render_time.tv_sec = now.tv_sec;
		last_render_time.tv_nsec = now.tv_nsec;
		//printf("\r\nReset render timer. fps(%d) swp(%ld ns)", m_initPrm.disFPS, tStamp[6]-tSwap);
		//fflush(stdout);
	}

	if(m_initPrm.renderfunc != NULL)
		m_initPrm.renderfunc(0, RUN_LEAVE, 0, 0);
	tend = tStamp[6];
	float renderIntv = (tend - m_tmRender)/getTickFrequency();

#if 1
	if(rCount%(m_initPrm.disFPS*100) == 0)
	{
		printf("\r\n[%d] %.4f (ws%.4f,cu%.4f,tv%.4f,to%.4f,rd%.4f,wp%.4f) %.4f(%.4f)",
			OSA_getCurTimeInMsec(),renderIntv,
			(tStamp[1]-tStamp[0])/getTickFrequency(),
			(tStamp[2]-tStamp[1])/getTickFrequency(),
			(tStamp[3]-tStamp[2])/getTickFrequency(),
			(tStamp[4]-tStamp[3])/getTickFrequency(),
			(tStamp[5]-tStamp[4])/getTickFrequency(),
			(tStamp[6]-tStamp[5])/getTickFrequency(),
			m_telapse, m_initPrm.disSched
			);
		fflush(stdout);
	}
	rCount ++;
#endif
	m_tmRender = tend;
}

void CGluVideoWindow::Display2()
{
	if(m_nRender == 0)
		return;
	//OSA_printf("[%d] %s %d: GLU%d enter", OSA_getCurTimeInMsec(), __func__, __LINE__, m_winId);
	OSA_mutexLock(&m_mutex);
	int chId, winId;
	for(chId = 0; chId<VWIN_CHAN_MAX; chId++){
		if(m_bufQue[chId] != NULL && m_bufQue[chId]->bMap){
			OSA_BufInfo* info = NULL;
			cv::Mat img;
			bool bDevMem = false;
			for(int i=0; i<m_bufQue[chId]->numBuf; i++){
				cuMap(&m_bufQue[chId]->bufInfo[i]);

				info = &m_bufQue[chId]->bufInfo[i];
				bDevMem = (info->memtype == memtype_glpbo || info->memtype == memtype_cudev || info->memtype == memtype_cumap);
				void *data = (bDevMem) ? info->physAddr : info->virtAddr;
				if(info->channels == 1){
					img = cv::Mat(info->height, info->width, CV_8UC1, data);
				}else{
					img = cv::Mat(info->height, info->width, CV_8UC3, data);
				}

				cv::Mat full = cv::Mat(img.rows,img.cols,img.type(), cv::Scalar(255, 0, 0, 0));
				if(bDevMem){
					cudaMemcpy(img.data, full.data, img.rows*img.cols*img.channels(), cudaMemcpyHostToDevice);
				}else{
					full.copyTo(img);
				}

				image_queue_putEmpty(m_bufQue[chId], &m_bufQue[chId]->bufInfo[i]);
			}
			m_bufQue[chId]->bMap = false;
		}
	}
	OSA_mutexUnlock(&m_mutex);

	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_MULTISAMPLE);
	glDisable(GL_COLOR_MATERIAL);

	int64 tStamp[10];
	tStamp[0] = getTickCount();
	static int64 tstart = 0ul, tend = 0ul;
	tstart = tStamp[0];
	if(tend == 0ul)
		tend = tstart;

	if(1)
	{
		uint64_t sched_ns = (uint64_t)(m_initPrm.disSched*1000000.f);
		if (sched_ns > 100000 && sched_ns < render_time_nsec-5000000)
		{
			pthread_mutex_lock(&render_lock);
			struct timespec waittime;
			clock_gettime(CLOCK_MONOTONIC, &waittime);
			waittime.tv_nsec += sched_ns;
			waittime.tv_sec += waittime.tv_nsec / 1000000000UL;
			waittime.tv_nsec %= 1000000000UL;
        	pthread_cond_timedwait(&render_cond, &render_lock,&waittime);
			pthread_mutex_unlock(&render_lock);
		}
	}

	tStamp[1] = getTickCount();

	if(m_initPrm.renderfunc != NULL)
		m_initPrm.renderfunc(0, RUN_ENTER, 0, 0);
	tStamp[2] = getTickCount();

	OSA_mutexLock(&m_mutex);
	int nChannels = m_vvideos.size();
	for(chId=0; chId<nChannels; chId++){
		CGLVideo *pVideo = m_vvideos[chId];
		pVideo->update();
	}
	OSA_mutexUnlock(&m_mutex);
	m_warpMat = m_vvideos[0]->m_warpMat;
	tStamp[3] = getTickCount();

	tStamp[4] = getTickCount();

	OSA_mutexLock(&m_mutex);
	OSA_assert(m_nRender == m_vvideoRenders.size());
	for(winId=0; winId<m_nRender; winId++){
		CGLVideoBlendRender *render = m_vvideoRenders[winId];
		OSA_assert(render != NULL);
		if(render->m_video == NULL)
			continue;
		chId = render->m_video->m_idx;
		if(chId < 0 || chId >= VWIN_CHAN_MAX)
			continue;
		int blend_chId = m_blendMap[chId];
		if(blend_chId>=0){
			render->blend(m_vvideos[blend_chId]);
			render->matrix(m_glmat44fBlend[chId*VWIN_CHAN_MAX+blend_chId]);
			render->params(m_glBlendPrm[chId*VWIN_CHAN_MAX+blend_chId]);
		}else{
			render->blend(NULL);
		}
		render->render();
		if(m_initPrm.renderfunc != NULL)
			m_initPrm.renderfunc(0, RUN_WIN, winId, chId);
	}

	for(int i=0; i<vRenders.size();i++)
		vRenders[i]->Draw();

	OSA_mutexUnlock(&m_mutex);

	if(m_initPrm.renderfunc != NULL)
		m_initPrm.renderfunc(0, RUN_SWAP, 0, 0);

	tStamp[5] = getTickCount();
	int64 tcur = tStamp[5];
	m_telapse = (tStamp[5] - tStamp[0])*0.000001f;

	if (last_render_time.tv_sec != 0 && last_render_time.tv_nsec != 0)
	{
		pthread_mutex_lock(&render_lock);
		struct timespec waittime = last_render_time;
		last_render_time.tv_sec += render_time_sec*2;
		last_render_time.tv_nsec += render_time_nsec*2;
		last_render_time.tv_sec += last_render_time.tv_nsec / 1000000000UL;
		last_render_time.tv_nsec %= 1000000000UL;
    	struct timespec now;
    	clock_gettime(CLOCK_MONOTONIC, &now);
        __suseconds_t cur_us, rd_us;
        cur_us = (now.tv_sec * 1000000.0 + now.tv_nsec/1000.0);
        rd_us = (last_render_time.tv_sec * 1000000.0 + last_render_time.tv_nsec / 1000.0);
        if (rd_us>cur_us)
        {
    		waittime.tv_sec += render_time_sec;
    		waittime.tv_nsec += render_time_nsec-500000UL;
    		waittime.tv_sec += waittime.tv_nsec / 1000000000UL;
    		waittime.tv_nsec %= 1000000000UL;
    		pthread_cond_timedwait(&render_cond, &render_lock,&waittime);
        }
        else if(rd_us+10000UL<cur_us)
        {
            //OSA_printf("%s %d: win%d frame_is_late(%ld us)", __func__, __LINE__, glutGetWindow(), cur_us - rd_us);
           	memset(&last_render_time, 0, sizeof(last_render_time));
        }
		pthread_mutex_unlock(&render_lock);
	}

	int64 tSwap = getTickCount();
	CGluWindow::Display2();
	tStamp[6] = getTickCount();

	if(tStamp[6]-tSwap>5000000UL)
		m_nSwapTimeOut++;
	else
		m_nSwapTimeOut = 0;
	if ((last_render_time.tv_sec == 0 && last_render_time.tv_nsec == 0) || m_nSwapTimeOut>=3)
	{
    	struct timespec now;
    	clock_gettime(CLOCK_MONOTONIC, &now);
		last_render_time.tv_sec = now.tv_sec;
		last_render_time.tv_nsec = now.tv_nsec;
		//printf("\r\nReset render timer. fps(%d) swp(%ld ns)", m_initPrm.disFPS, tStamp[6]-tSwap);
		//fflush(stdout);
	}

	if(m_initPrm.renderfunc != NULL)
		m_initPrm.renderfunc(0, RUN_LEAVE, 0, 0);
	tend = tStamp[6];
	float renderIntv = (tend - m_tmRender)/getTickFrequency();

#if 1
	if(rCount%(m_initPrm.disFPS*100) == 0)
	{
		printf("\r\n[%d]win%d %.4f (ws%.4f,cu%.4f,tv%.4f,to%.4f,rd%.4f,wp%.4f) %.4f(%.4f)",
			OSA_getCurTimeInMsec(),m_winId,renderIntv,
			(tStamp[1]-tStamp[0])/getTickFrequency(),
			(tStamp[2]-tStamp[1])/getTickFrequency(),
			(tStamp[3]-tStamp[2])/getTickFrequency(),
			(tStamp[4]-tStamp[3])/getTickFrequency(),
			(tStamp[5]-tStamp[4])/getTickFrequency(),
			(tStamp[6]-tStamp[5])/getTickFrequency(),
			m_telapse, m_initPrm.disSched
			);
		fflush(stdout);
	}
	rCount ++;
#endif
	m_tmRender = tend;
}

void CGluVideoWindow::Reshape(int width, int height)
{
	//OSA_printf("[%d] %s %d: GLU%d %d x %d", OSA_getCurTimeInMsec(), __func__, __LINE__, m_winId, width, height);
	CGluWindow::Reshape(width, height);
	for(int i=0; i<m_vvideoRenders.size(); i++){
		CGLVideoBlendRender *render = m_vvideoRenders[i];
		if(render != NULL){
			cv::Rect viewPort = cv::Rect(0, 0, m_rcReal.width, m_rcReal.height);
			if(i!=0)
				viewPort = cv::Rect(m_rcReal.width*2/3, m_rcReal.height*2/3, m_rcReal.width/3, m_rcReal.height/3);
			render->set(viewPort);
		}
	}
}

CGLVideoBlendRender* CGluVideoWindow::CreateVedioRender(int index)
{
	CGLVideo *video = (m_vvideos.size() > index) ? m_vvideos[index] : NULL;
	GLMatx44f matrix = cv::Matx44f::eye();
	cv::Rect viewPort = cv::Rect(0, 0, m_rcReal.width, m_rcReal.height);
	CGLVideo *blend = NULL;
	GLMatx44f blendMatrix = cv::Matx44f::eye();
	GLV_BlendPrm prm;
	memset(&prm, 0, sizeof(prm));
	prm.fAlpha = 0.5f;
	if(index != 0){
		video = NULL;
		viewPort = cv::Rect(m_rcReal.width*2/3, m_rcReal.height*2/3, m_rcReal.width/3, m_rcReal.height/3);
	}
	CGLVideoBlendRender *render = new CGLVideoBlendRender(video, matrix, viewPort, blend, blendMatrix, prm);
	return render;
}

int CGluVideoWindow::dynamic_config(VWIN_CFG type, int iPrm, void* pPrm)
{
	int iRet = OSA_SOK;
	int chId, renderId;
	bool bEnable;
	cv::Rect *rc;
	int renderCount = m_vvideoRenders.size();
	int videoCount = m_vvideos.size();
	CGLVideoBlendRender *render;
	CGLVideo *pVideo;

	if(type == VWIN_CFG_ChId){
		renderId = iPrm;
		if(renderId >= renderCount || renderId < 0 || m_vvideoRenders[renderId] == NULL)
			return -1;
		if(pPrm == NULL)
			return -2;
		chId = *(int*)pPrm;

		OSA_mutexLock(&m_mutex);
		render = m_vvideoRenders[renderId];
		int curId = (render->m_video != NULL) ? render->m_video->m_idx : -1;
		if(curId >= 0){
			int count = OSA_bufGetFullCount(m_bufQue[curId]);
			while(count>0){
				image_queue_switchEmpty(m_bufQue[curId]);
				count = OSA_bufGetFullCount(m_bufQue[curId]);
			}
		}
		pVideo = (chId>=0 && chId<videoCount) ? m_vvideos[chId] : NULL;
		render->set(pVideo);

		if(chId>=0 && chId<videoCount){
			int count = OSA_bufGetFullCount(m_bufQue[chId]);
			for(int i=0; i<count; i++){
				image_queue_switchEmpty(m_bufQue[chId]);
			}
			OSA_printf("%s %d: switchEmpty %d frames", __func__, __LINE__, count);
		}

		m_telapse = 5.0f;
		OSA_mutexUnlock(&m_mutex);
	}

	if(type == VWIN_CFG_VideoTransMat){
		if(iPrm >= videoCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		OSA_mutexLock(&m_mutex);
		pVideo = m_vvideos[chId];
		memcpy(pVideo->m_matrix.val, pPrm, sizeof(float)*16);
		OSA_mutexUnlock(&m_mutex);
	}

	if(type == VWIN_CFG_ViewTransMat){
		if(iPrm >= renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		OSA_mutexLock(&m_mutex);
		render = m_vvideoRenders[iPrm];
		GLMatx44f matrix;
		memcpy(matrix.val , pPrm, sizeof(float)*16);
		render->set(matrix);
		OSA_mutexUnlock(&m_mutex);
	}

	if(type == VWIN_CFG_ViewPos){
		if(iPrm >= renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		OSA_mutexLock(&m_mutex);
		render = m_vvideoRenders[iPrm];
		rc = (cv::Rect*)pPrm;
		render->set(*rc);
		OSA_mutexUnlock(&m_mutex);
	}

	if(type == VWIN_CFG_BlendChId){
		if(iPrm >= VWIN_CHAN_MAX || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		OSA_mutexLock(&m_mutex);
		m_blendMap[iPrm] = *(int*)pPrm;
		OSA_mutexUnlock(&m_mutex);
	}

	if(type == VWIN_CFG_MaskChId){
		if(iPrm >= VWIN_CHAN_MAX || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		OSA_mutexLock(&m_mutex);
		m_maskMap[iPrm] = *(int*)pPrm;
		OSA_mutexUnlock(&m_mutex);
	}

	if(type == VWIN_CFG_BlendTransMat){
		if(iPrm >= VWIN_CHAN_MAX*VWIN_CHAN_MAX || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		OSA_mutexLock(&m_mutex);
		memcpy(m_glmat44fBlend[iPrm].val, pPrm, sizeof(float)*16);
		OSA_mutexUnlock(&m_mutex);
	}

	if(type == VWIN_CFG_BlendPrm){
		if(iPrm >= VWIN_CHAN_MAX*VWIN_CHAN_MAX || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		OSA_mutexLock(&m_mutex);
		memcpy(&m_glBlendPrm[iPrm], pPrm, sizeof(GLV_BlendPrm));
		OSA_mutexUnlock(&m_mutex);
	}

	return iRet;
}

