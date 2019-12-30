
#include "Displayer.hpp"
#include <cuda.h>
#include <cuda_gl_interop.h>
#include <cuda_runtime_api.h>
#include <X11/Xlib.h>
#include <stdarg.h>
#include "osa_image_queue.h"
#include "cuda_mem.hpp"
#include "cuda_convert.cuh"

#define TAG_VALUE   (0x10001000)

using namespace cv;
using namespace cr_osa;

static CRender *gThis = NULL;

static const GLfloat defaultVertices[8] = {
    -1.f, -1.f,
    1.f, -1.f,
    -1.f, 1.f,
    1.f, 1.f
};
static const GLfloat defaultTextureCoords[8] = {
    0.0f, 1.0f,
    1.0f, 1.0f,
    0.0f, 0.0f,
    1.0f, 0.0f
};

CRender* CRender::createObject()
{
	if(gThis == NULL)
		gThis = new CRender();
	return gThis;
}
void CRender::destroyObject(CRender* obj)
{
	if(gThis != NULL)
		delete gThis;
	gThis = NULL;
}

CRender::CRender()
:m_winId(0), m_winWidth(1920),m_winHeight(1080),m_renderCount(0),m_bFullScreen(false),
 m_bUpdateVertex(false), m_tmRender(0ul),
 m_telapse(5.0), m_nSwapTimeOut(0)
{
	tag = TAG_VALUE;
	gThis = this;
	memset(&m_initPrm, 0, sizeof(m_initPrm));
	memset(m_bufQue, 0, sizeof(m_bufQue));
	memset(m_tmBak, 0, sizeof(m_tmBak));
	memset(m_glProgram, 0, sizeof(m_glProgram));
	m_initPrm.disSched = 3.5;
	for(int chId = 0; chId<DS_CHAN_MAX; chId++){
		m_blendMap[chId] = -1;
		m_maskMap[chId] = -1;
	}
#ifdef __EGL__
    stop_thread = false;
    render_thread = 0;
    x_window = 0;
    x_display = NULL;
    gc = NULL;
    fontinfo = NULL;
    egl_surface = EGL_NO_SURFACE;
    egl_context = EGL_NO_CONTEXT;
    egl_display = EGL_NO_DISPLAY;
    egl_config = NULL;
#endif
}

CRender::~CRender()
{
	//destroy();
	gThis = NULL;
}

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

int CRender::create(DS_InitPrm *pPrm)
{
	memset(m_renders, 0, sizeof(m_renders));
	memset(m_curMap, 0, sizeof(m_curMap));
	for(int i=0; i<DS_RENDER_MAX; i++){
		m_renders[i].transform = cv::Matx44f::eye();
	}
	memset(m_videoInfo, 0, sizeof(m_videoInfo));

	char strParams[][32] = {"DS_RENDER", "-display", ":0"};
	char *argv[3];
	int argc = 1;
	for(int i=0; i<argc; i++)
		argv[i] = strParams[i];
	uint32_t screenWidth = 0, screenHeight = 0;
	if(getDisplayResolution(NULL, screenWidth, screenHeight) == 0)
	{
		m_winWidth = screenWidth;
		m_winHeight = screenHeight;
	}
	OSA_printf("screen resolution: %d x %d", screenWidth, screenHeight);

	OSA_mutexCreate(&m_mutex);

	if(pPrm != NULL)
		memcpy(&m_initPrm, pPrm, sizeof(DS_InitPrm));

	if(m_initPrm.winWidth > 0)
		m_winWidth = m_initPrm.winWidth;
	if(m_initPrm.winHeight > 0)
		m_winHeight = m_initPrm.winHeight;
	if(m_initPrm.disFPS<=0)
		m_initPrm.disFPS = 25;
	if(m_initPrm.nQueueSize < 2)
		m_initPrm.nQueueSize = 2;
	//m_interval = (1000000000ul)/(uint64)m_initPrm.disFPS;
	//memcpy(m_videoSize, m_initPrm.channelsSize, sizeof(m_videoSize));

	initRender();
	gl_updateVertex();

	for(int i=0; i<DS_CHAN_MAX; i++){
		m_glmat44fTrans[i] = cv::Matx44f::eye();
	}
	for(int i=0; i<DS_CHAN_MAX*DS_CHAN_MAX; i++){
		m_glmat44fBlend[i] = cv::Matx44f::eye();
		m_glBlendPrm[i].fAlpha = 0.5f;
		m_glBlendPrm[i].thr0Min = 0;
		m_glBlendPrm[i].thr0Max = 0;
		m_glBlendPrm[i].thr1Min = 0;
		m_glBlendPrm[i].thr1Max = 0;
	}

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

#ifdef __EGL__
    int depth;
    int screen_num;
    uint32_t width = m_mainWinWidth, height = m_mainWinHeight, x_offset = 0, y_offset = 0;
    XSetWindowAttributes window_attributes;
    memset(&window_attributes, 0, sizeof(window_attributes));

    x_display = XOpenDisplay(NULL);
    if (NULL == x_display)
    {
        OSA_printf("[Render] %s %d: Error in opening display", __func__, __LINE__);
        return OSA_EFAIL;
    }
    screen_num = DefaultScreen(x_display);
    depth = DefaultDepth(x_display, screen_num);

    window_attributes.background_pixel =
        BlackPixel(x_display, DefaultScreen(x_display));

    window_attributes.override_redirect = 1;

    x_window = XCreateWindow(x_display,
                             DefaultRootWindow(x_display), x_offset,
                             y_offset, width, height,
                             0,
                             depth, CopyFromParent,
                             CopyFromParent,
                             (CWBackPixel | CWOverrideRedirect),
                             &window_attributes);
    OSA_assert(x_window != 0);

    XSelectInput(x_display, (int32_t) x_window, ExposureMask);
    XMapWindow(x_display, (int32_t) x_window);
    gc = XCreateGC(x_display, x_window, 0, NULL);

    XSetForeground(x_display, gc,
                WhitePixel(x_display, DefaultScreen(x_display)) );
    fontinfo = XLoadQueryFont(x_display, "9x15bold");

    pthread_mutex_lock(&render_lock);
    pthread_create(&render_thread, NULL, renderThread, this);
    pthread_cond_wait(&render_cond, &render_lock);
    pthread_mutex_unlock(&render_lock);

#else
    // GLUT init
    glutInit(&argc, argv);
    int glut_screen_width = glutGet(GLUT_SCREEN_WIDTH);
    int glut_screen_height = glutGet(GLUT_SCREEN_HEIGHT);
    //OSA_printf("%s %d: glutGet %d x %d", __func__, __LINE__, glut_screen_width, glut_screen_height);
	//Double, Use glutSwapBuffers() to show
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
    //glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE);
	//Single, Use glFlush() to show
	//glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB );

	if(0)
	{
		glutInitWindowPosition(1920,0);
		glutInitWindowSize(1440, 900);
	    glutSetOption(GLUT_RENDERING_CONTEXT,GLUT_USE_CURRENT_CONTEXT);
	    int winId2 = glutCreateWindow("DSS2");
	    OSA_assert(winId2 > 0);
	    glutSetWindow(winId2);
		if(m_initPrm.bFullScreen){
			glutFullScreen();
		}
		//int subWin = glutCreateSubWindow(winId2,0,0,1440,900);
		//OSA_assert(subWin > 0);
		//glutSetWindow(subWin);
		glutDisplayFunc(_display2);
		if(m_initPrm.keyboardfunc != NULL)
			glutKeyboardFunc(m_initPrm.keyboardfunc);
		if(m_initPrm.keySpecialfunc != NULL)
			glutSpecialFunc(m_initPrm.keySpecialfunc);
		//mouse event:
		if(m_initPrm.mousefunc != NULL)
			glutMouseFunc(m_initPrm.mousefunc);//GLUT_LEFT_BUTTON GLUT_MIDDLE_BUTTON GLUT_RIGHT_BUTTON; GLUT_DOWN GLUT_UP

		GLenum err = glewInit();
		if (GLEW_OK != err) {
			fprintf(stderr, "\n[Render] %s %d: Error in glewInit. %s\n", __func__, __LINE__, glewGetErrorString(err));
			return -1;
		}
		OSA_printf("[Render] %s %d: glewInit success", __func__, __LINE__);
		glClearColor(1.0f, 0.0f, 0.01f, 0.0f );
		glClear(GL_COLOR_BUFFER_BIT);
	}

    glutInitWindowPosition(m_initPrm.winPosX, m_initPrm.winPosY);
    glutInitWindowSize(m_winWidth, m_winHeight);
    OSA_printf("%s %d: window(%d,%d,%d,%d)",__func__, __LINE__, m_initPrm.winPosX, m_initPrm.winPosY, m_winWidth, m_winHeight);

    glutSetOption(GLUT_RENDERING_CONTEXT,GLUT_USE_CURRENT_CONTEXT);
    m_winId = glutCreateWindow("DSS1");
    OSA_assert(m_winId > 0);
    glutSetWindow(m_winId);
	glutDisplayFunc(_display);
	glutReshapeFunc(_reshape);
	if(m_initPrm.keyboardfunc != NULL)
		glutKeyboardFunc(m_initPrm.keyboardfunc);
	if(m_initPrm.keySpecialfunc != NULL)
		glutSpecialFunc(m_initPrm.keySpecialfunc);
	//mouse event:
	if(m_initPrm.mousefunc != NULL)
		glutMouseFunc(m_initPrm.mousefunc);//GLUT_LEFT_BUTTON GLUT_MIDDLE_BUTTON GLUT_RIGHT_BUTTON; GLUT_DOWN GLUT_UP
	//glutMotionFunc();//button down
	//glutPassiveMotionFunc();//button up
	//glutEntryFunc();//state GLUT_LEFT, GLUT_ENTERED
	if(m_initPrm.visibilityfunc != NULL)
		glutVisibilityFunc(m_initPrm.visibilityfunc);
	glutCloseFunc(_close);

	GLenum err = glewInit();
	if (GLEW_OK != err) {
		fprintf(stderr, "\n[Render] %s %d: Error in glewInit. %s\n", __func__, __LINE__, glewGetErrorString(err));
		return -1;
	}

	gl_init();

    glutSetWindow(m_winId);
	if(m_initPrm.bFullScreen){
		glutFullScreen();
		m_bFullScreen = true;
	}

#endif

	return 0;
}

#ifdef __EGL__
int CRender::renderHandle(void)
{
	{
		EGLBoolean egl_status;
		static EGLint rgba8888[] = {
			EGL_RED_SIZE, 8,
			EGL_GREEN_SIZE, 8,
			EGL_BLUE_SIZE, 8,
			EGL_ALPHA_SIZE, 8,
			EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
			EGL_NONE,
		};
		int num_configs = 0;
		EGLint context_attribs[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
		egl_display = eglGetDisplay(x_display);
		if (EGL_NO_DISPLAY == egl_display)
		{
			OSA_printf("[Render] %s %d: Unable to get egl display", __func__, __LINE__);
			return OSA_EFAIL;
		}
		OSA_printf("[Render] %s %d: Egl Got display %ld", __func__, __LINE__, (size_t)egl_display);

		egl_status = eglInitialize(egl_display, 0, 0);
		if (!egl_status)
		{
			OSA_printf("[Render] %s %d: Unable to initialize egl library", __func__, __LINE__);
			return OSA_EFAIL;
		}

		egl_status = eglChooseConfig(egl_display, rgba8888, &egl_config, 1, &num_configs);
		if (!egl_status)
		{
			OSA_printf("[Render] %s %d: Error at eglChooseConfig", __func__, __LINE__);
			return OSA_EFAIL;
		}
		OSA_printf("[Render] %s %d: Got numconfigs as %d", __func__, __LINE__, num_configs);

		egl_context = eglCreateContext(egl_display, egl_config, EGL_NO_CONTEXT, context_attribs);
		if (eglGetError() != EGL_SUCCESS)
		{
			OSA_printf("[Render] %s %d: Error in eglCreateContext %d", __func__, __LINE__, eglGetError());
			return OSA_EFAIL;
		}
		OSA_printf("[Render] %s %d: eglCreateContext success", __func__, __LINE__);
		egl_surface = eglCreateWindowSurface(egl_display, egl_config,(EGLNativeWindowType)x_window, NULL);
		if (egl_surface == EGL_NO_SURFACE)
		{
			OSA_printf("[Render] %s %d: Error in creating egl surface %d", __func__, __LINE__, eglGetError());
			return OSA_EFAIL;
		}
		OSA_printf("[Render] %s %d: eglCreateWindowSurface success", __func__, __LINE__);

		eglMakeCurrent(egl_display, egl_surface, egl_surface, egl_context);
		if (eglGetError() != EGL_SUCCESS)
		{
			OSA_printf("[Render] %s %d: Error in eglMakeCurrent %d", __func__, __LINE__, eglGetError());
			return OSA_EFAIL;
		}
		OSA_printf("[Render] %s %d: eglMakeCurrent success", __func__, __LINE__);

		if(gl_init() != OSA_SOK)
			return OSA_EFAIL;
	}

	return OSA_SOK;
}
#endif

int CRender::destroy()
{
    pthread_mutex_lock(&render_lock);
    pthread_cond_broadcast(&render_cond);
    pthread_mutex_unlock(&render_lock);

#ifdef __EGL__
	{
		EGLBoolean egl_status;
	    if (egl_display != EGL_NO_DISPLAY)
	    {
	        eglMakeCurrent(egl_display, EGL_NO_SURFACE,EGL_NO_SURFACE, EGL_NO_CONTEXT);
	    }

	    if (egl_surface != EGL_NO_SURFACE)
	    {
	        egl_status = eglDestroySurface(egl_display, egl_surface);
	        if (egl_status == EGL_FALSE)
	        {
	            OSA_printf("[Render]%s %d: EGL surface destruction failed", __func__, __LINE__);
	        }
	    }

	    if (egl_context != EGL_NO_CONTEXT)
	    {
	        egl_status = eglDestroyContext(egl_display, egl_context);
	        if (egl_status == EGL_FALSE)
	        {
	            OSA_printf("[Render]%s %d: EGL context destruction failed", __func__, __LINE__);
	        }
	    }

	    if (egl_display != EGL_NO_DISPLAY)
	    {
	        eglReleaseThread();
	        eglTerminate(egl_display);
	    }
	}
	if (fontinfo)
	{
		XFreeFont(x_display, fontinfo);
	}

	if (gc)
	{
		XFreeGC(x_display, gc);
	}

	if (x_window)
	{
		XUnmapWindow(x_display, (int32_t) x_window);
		XFlush(x_display);
		XDestroyWindow(x_display, (int32_t) x_window);
	}
	if (x_display)
	{
		XCloseDisplay(x_display);
	}
#else
	glutLeaveMainLoop();
#endif

	gl_uninit();
	OSA_mutexDelete(&m_mutex);

    pthread_mutex_destroy(&render_lock);
    pthread_cond_destroy(&render_cond);

	return 0;
}

int CRender::setFPS(float fps)
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

int CRender::initRender(bool updateMap)
{
	int i=0;

	//OSA_printf("Render::%s %d: win%d (%d x %d)", __func__, __LINE__, m_winId, m_winWidth, m_winHeight);

	if(updateMap){
		m_renders[i].video_chId    = 0;
	}
	m_renders[i].displayrect.x = 0;
	m_renders[i].displayrect.y = 0;
	m_renders[i].displayrect.width = m_winWidth;
	m_renders[i].displayrect.height = m_winHeight;
	i++;

	if(updateMap){
		m_renders[i].video_chId    = -1;
	}
	m_renders[i].displayrect.x = m_winWidth*2/3;
	m_renders[i].displayrect.y = m_winHeight*2/3;
	m_renders[i].displayrect.width = m_winWidth/3;
	m_renders[i].displayrect.height = m_winHeight/3;
	i++;

	if(updateMap){
		m_renders[i].video_chId    = -1;
	}
	m_renders[i].displayrect.x = m_winWidth*2/3;
	m_renders[i].displayrect.y = m_winHeight*2/3;
	m_renders[i].displayrect.width = m_winWidth/3;
	m_renders[i].displayrect.height = m_winHeight/3;
	i++;

	if(updateMap){
		m_renders[i].video_chId    = -1;
	}
	m_renders[i].displayrect.x = m_winWidth*2/3;
	m_renders[i].displayrect.y = m_winHeight*2/3;
	m_renders[i].displayrect.width = m_winWidth/3;
	m_renders[i].displayrect.height = m_winHeight/3;
	i++;

	m_renderCount = i;

	for(i=0; i<m_renderCount; i++){
		m_curMap[i] = m_renders[i].video_chId;
	}

	return 0;
}

static unsigned long ndisCnt = 0;
void CRender::_display(void)
{
	OSA_assert(gThis->tag == TAG_VALUE);
	//OSA_printf("[%ld-%d]%s %d: winId = %d", ndisCnt++, OSA_getCurTimeInMsec(), __func__, __LINE__, glutGetWindow());
	gThis->gl_display();
#ifndef __EGL__
	glutPostRedisplay();
#endif
}

void CRender::_display2(void)
{
	OSA_assert(gThis->tag == TAG_VALUE);
	//OSA_printf("[%ld-%d]%s %d: winId = %d", ndisCnt++, OSA_getCurTimeInMsec(), __func__, __LINE__, glutGetWindow());
	gThis->gl_display2();
#ifndef __EGL__
	glutPostRedisplay();
#endif
}

void CRender::_reshape(int width, int height)
{
	assert(gThis != NULL);
	gThis->m_winWidth = width;
	gThis->m_winHeight = height;
	OSA_printf("CRender::%s %d: win(%dx%d)", __func__, __LINE__, width, height);
	gThis->initRender(false);
	gThis->gl_updateVertex();
}

void CRender::_close(void)
{
	if(gThis->m_initPrm.closefunc != NULL)
		gThis->m_initPrm.closefunc();
}

int CRender::dynamic_config(DS_CFG type, int iPrm, void* pPrm)
{
	int iRet = 0;
	int chId;
	bool bEnable;
	DS_Rect *rc;

	if(type == DS_CFG_ChId){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		chId = *(int*)pPrm;

		OSA_mutexLock(&m_mutex);
		int curId = m_renders[iPrm].video_chId;
		if(curId >= 0){
			int count = OSA_bufGetFullCount(&m_bufQue[curId]);
			while(count>0){
				image_queue_switchEmpty(&m_bufQue[curId]);
				count = OSA_bufGetFullCount(&m_bufQue[curId]);
			}
		}
		m_renders[iPrm].video_chId = chId;
		//if(winId == 0)
		{
			int toId = chId;
			if(toId >= 0 && toId<DS_CHAN_MAX){
				int count = OSA_bufGetFullCount(&m_bufQue[toId]);
				for(int i=0; i<count; i++){
					image_queue_switchEmpty(&m_bufQue[toId]);
				}
				OSA_printf("%s %d: switchEmpty %d frames", __func__, __LINE__, count);
			}
		}
		m_telapse = 5.0f;
		OSA_mutexUnlock(&m_mutex);
	}

	if(type == DS_CFG_BlendChId){
		if(iPrm >= DS_CHAN_MAX || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		OSA_mutexLock(&m_mutex);
		m_blendMap[iPrm] = *(int*)pPrm;
		OSA_mutexUnlock(&m_mutex);
	}

	if(type == DS_CFG_MaskChId){
		if(iPrm >= DS_CHAN_MAX || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		OSA_mutexLock(&m_mutex);
		m_maskMap[iPrm] = *(int*)pPrm;
		OSA_mutexUnlock(&m_mutex);
	}

	if(type == DS_CFG_CropEnable){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		bEnable = *(bool*)pPrm;
		m_renders[iPrm].bCrop = bEnable;
	}

	if(type == DS_CFG_CropRect){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		rc = (DS_Rect*)pPrm;
		m_renders[iPrm].croprect = *rc;
		gl_updateVertex();
	}

	if(type == DS_CFG_VideoTransMat){
		if(iPrm >= DS_CHAN_MAX || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		memcpy(m_glmat44fTrans[iPrm].val, pPrm, sizeof(float)*16);
	}

	if(type == DS_CFG_ViewTransMat){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		memcpy(m_renders[iPrm].transform.val , pPrm, sizeof(float)*16);
		gl_updateVertex();
	}


	if(type == DS_CFG_ViewPos){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		rc = (DS_Rect*)pPrm;
		m_renders[iPrm].displayrect = *rc;
	}

	if(type == DS_CFG_BlendTransMat){
		if(iPrm >= DS_CHAN_MAX*DS_CHAN_MAX || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		memcpy(m_glmat44fBlend[iPrm].val, pPrm, sizeof(float)*16);
	}

	if(type == DS_CFG_BlendPrm){
		if(iPrm >= DS_CHAN_MAX*DS_CHAN_MAX || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		memcpy(&m_glBlendPrm[iPrm], pPrm, sizeof(DS_BlendPrm));
	}

	return iRet;
}

GLuint CRender::gl_PBO(int chId, int width, int height, int channels)
{
	assert(chId>=0 && chId<DS_CHAN_MAX);

	if(m_videoInfo[chId].w  == width  && m_videoInfo[chId].h == height && m_videoInfo[chId].c == channels )
		return buffId_input[chId];

	//OSA_printf("%s: w = %d h = %d (%dx%d) cur %d\n", __FUNCTION__, width, height, m_videoSize[chId].w, m_videoSize[chId].h, buffId_input[chId]);

	if(m_videoInfo[chId].w != 0){
		glDeleteBuffers(1, &buffId_input[chId]);
		glGenBuffers(1, &buffId_input[chId]);
	}

	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, buffId_input[chId]);
	glBufferData(GL_PIXEL_UNPACK_BUFFER, width*height*channels, NULL, GL_DYNAMIC_COPY);//GL_STATIC_DRAW);//GL_DYNAMIC_DRAW);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

	m_videoInfo[chId].w = width;
	m_videoInfo[chId].h = height;
	m_videoInfo[chId].c = channels;

	//OSA_printf("%s: w = %d h = %d (%dx%d) out %d\n", __FUNCTION__, width, height, m_videoSize[chId].w, m_videoSize[chId].h, buffId_input[chId]);
	return buffId_input[chId];
}

/***********************************************************************/

#define TEXTURE_ROTATE (0)
#define ATTRIB_VERTEX 3
#define ATTRIB_TEXTURE 4

int CRender::gl_init(void)
{
	// Blue background
	glClearColor(1.0f, 0.0f, 0.01f, 0.0f );
	gl_loadProgram();
	//OSA_printf("[Render] %s %d: gl_loadProgram success", __func__, __LINE__);

	glGenBuffers(DS_CHAN_MAX, buffId_input);
	glGenTextures(DS_CHAN_MAX, textureId_input);
	for(int i=0; i<DS_CHAN_MAX; i++)
	{
		glBindTexture(GL_TEXTURE_2D, textureId_input[i]);
		assert(glIsTexture(textureId_input[i]));
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);//GL_NEAREST);//GL_NEAREST_MIPMAP_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);//GL_NEAREST);//GL_NEAREST_MIPMAP_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);//GL_CLAMP);//GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);//GL_CLAMP);//GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D, 0, 3, 1920, 1080, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, NULL);
	}

	//glEnable(GL_LINE_SMOOTH);
	//glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
	glClear(GL_COLOR_BUFFER_BIT);

	for(int chId=0; chId<m_initPrm.nChannels; chId++){
		image_queue_create(&m_bufQue[chId], m_initPrm.nQueueSize,
				m_initPrm.channelInfo[chId].w*m_initPrm.channelInfo[chId].h*m_initPrm.channelInfo[chId].c,
				m_initPrm.memType);
		for(int i=0; i<m_bufQue[chId].numBuf; i++){
			m_bufQue[chId].bufInfo[i].width = m_initPrm.channelInfo[chId].w;
			m_bufQue[chId].bufInfo[i].height = m_initPrm.channelInfo[chId].h;
			m_bufQue[chId].bufInfo[i].channels = m_initPrm.channelInfo[chId].c;
		}
	}

	return OSA_SOK;
}

void CRender::gl_uninit()
{
	gl_unloadProgram();
	glDeleteTextures(DS_CHAN_MAX, textureId_input);
	glDeleteBuffers(DS_CHAN_MAX, buffId_input);
	for(int chId=0; chId<DS_CHAN_MAX; chId++)
		cudaResource_UnregisterBuffer(chId);
	for(int chId=0; chId<m_initPrm.nChannels; chId++)
		image_queue_delete(&m_bufQue[chId]);
}

int CRender::gl_updateVertex(void)
{
	int iRet = 0;
	int winId, chId, i;
	DS_Rect rc;
	//GLfloat ftmp;

	for(winId=0; winId<m_renderCount; winId++)
	{
		m_glvVerts[winId][0] = -1.0f; m_glvVerts[winId][1] 	= 1.0f;
		m_glvVerts[winId][2] = 1.0f;  m_glvVerts[winId][3] 	= 1.0f;
		m_glvVerts[winId][4] = -1.0f; m_glvVerts[winId][5] 	= -1.0f;
		m_glvVerts[winId][6] = 1.0f;  m_glvVerts[winId][7] 	= -1.0f;

		m_glvTexCoords[winId][0] = 0.0; m_glvTexCoords[winId][1] = 0.0;
		m_glvTexCoords[winId][2] = 1.0; m_glvTexCoords[winId][3] = 0.0;
		m_glvTexCoords[winId][4] = 0.0; m_glvTexCoords[winId][5] = 1.0;
		m_glvTexCoords[winId][6] = 1.0; m_glvTexCoords[winId][7] = 1.0;

		memcpy(m_glvVerts[winId], defaultVertices, sizeof(defaultVertices));
		memcpy(m_glvTexCoords[winId], defaultTextureCoords, sizeof(defaultTextureCoords));
	}

	for(winId=0; winId<m_renderCount; winId++)
	{
		chId = m_renders[winId].video_chId;
		if(chId < 0 || chId >= DS_CHAN_MAX)
			continue;
		rc = m_renders[winId].croprect;
		if(m_videoInfo[chId].w<=0 || m_videoInfo[chId].h<=0){
			iRet ++;
			continue;
		}
		if(rc.width == 0 || rc.height == 0){
			continue;
		}
		m_glvTexCoords[winId][0] = (GLfloat)rc.x/m_videoInfo[chId].w;
		m_glvTexCoords[winId][1] = (GLfloat)rc.y/m_videoInfo[chId].h;

		m_glvTexCoords[winId][2] = (GLfloat)(rc.x+rc.width)/m_videoInfo[chId].w;
		m_glvTexCoords[winId][3] = (GLfloat)rc.y/m_videoInfo[chId].h;

		m_glvTexCoords[winId][4] = (GLfloat)rc.x/m_videoInfo[chId].w;
		m_glvTexCoords[winId][5] = (GLfloat)(rc.y+rc.height)/m_videoInfo[chId].h;

		m_glvTexCoords[winId][6] = (GLfloat)(rc.x+rc.width)/m_videoInfo[chId].w;
		m_glvTexCoords[winId][7] = (GLfloat)(rc.y+rc.height)/m_videoInfo[chId].h;
	}

	return iRet;
}

void CRender::gl_updateTexVideo()
{
	static int bCreate[DS_CHAN_MAX] = {0,0,0,0};
	static unsigned long nCnt[DS_CHAN_MAX] = {0,0,0,0};
	for(int chId = 0; chId < DS_CHAN_MAX; chId++)
	{
		bool bDevMem = false;
		OSA_BufInfo* info = NULL;
		Mat img;
		nCnt[chId] ++;
		if(nCnt[chId] > 300){
			int count = OSA_bufGetFullCount(&m_bufQue[chId]);
			nCnt[chId] = 1;
			if(count>1){
				OSA_printf("[%d]%s: ch%d queue count = %d, sync",
										OSA_getCurTimeInMsec(), __func__, chId, count);
				while(count>1){
					//image_queue_switchEmpty(&m_bufQue[chId]);
					info = image_queue_getFull(&m_bufQue[chId]);
					OSA_assert(info != NULL);
					image_queue_putEmpty(&m_bufQue[chId], info);
					count = OSA_bufGetFullCount(&m_bufQue[chId]);
				}
			}
		}

		int nDrop = m_initPrm.disFPS/m_initPrm.channelInfo[chId].fps;
		if(nDrop>1 && (nCnt[chId]%nDrop)!=1)
			continue;

		info = image_queue_getFull(&m_bufQue[chId]);
		if(info != NULL)
		{
			GLuint pbo = 0;
			bDevMem = (info->memtype == memtype_cudev || info->memtype == memtype_cumap);
			void *data = (bDevMem) ? info->physAddr : info->virtAddr;
			if(info->channels == 1){
				img = cv::Mat(info->height, info->width, CV_8UC1, data);
			}else{
				img = cv::Mat(info->height, info->width, CV_8UC3, data);
			}
			m_tmBak[chId] = info->timestampCap;

			OSA_assert(img.cols > 0 && img.rows > 0);
			if(bDevMem)
			{
				unsigned int byteCount = img.cols*img.rows*img.channels();
				unsigned char *dev_pbo = NULL;
				size_t tmpSize;
				pbo = gl_PBO(chId, img.cols, img.rows, img.channels());
				OSA_assert(pbo == buffId_input[chId]);
				glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
				cudaResource_RegisterBuffer(chId, pbo, byteCount);
				cudaResource_mapBuffer(chId, (void **)&dev_pbo, &tmpSize);
				assert(tmpSize == byteCount);
				cudaMemcpy(dev_pbo, img.data, byteCount, cudaMemcpyDeviceToDevice);
				//cudaDeviceSynchronize();
				cudaResource_unmapBuffer(chId);
				cudaResource_UnregisterBuffer(chId);
				img.data = NULL;
			}else if(info->memtype == memtype_glpbo){
				cuUnmap(info);
				pbo = info->pbo;
				glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
			}else{
				unsigned int byteCount = img.cols*img.rows*img.channels();
				unsigned char *dev_pbo = NULL;
				size_t tmpSize;
				pbo = gl_PBO(chId, img.cols, img.rows, img.channels());
				OSA_assert(pbo == buffId_input[chId]);
				OSA_assert(img.data != NULL);
				glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
				cudaResource_RegisterBuffer(chId, pbo, byteCount);
				cudaResource_mapBuffer(chId, (void **)&dev_pbo, &tmpSize);
				assert(tmpSize == byteCount);
				cudaMemcpy(dev_pbo, img.data, byteCount, cudaMemcpyHostToDevice);
				//cudaDeviceSynchronize();
				cudaResource_unmapBuffer(chId);
				cudaResource_UnregisterBuffer(chId);
			}
			glBindTexture(GL_TEXTURE_2D, textureId_input[chId]);
			if(!bCreate[chId])
			{
				if(img.channels() == 1)
					glTexImage2D(GL_TEXTURE_2D, 0, img.channels(), img.cols, img.rows, 0, GL_RED, GL_UNSIGNED_BYTE, NULL);
				else if(img.channels() == 3)
					glTexImage2D(GL_TEXTURE_2D, 0, img.channels(), img.cols, img.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, NULL);
				else if(img.channels() == 4)
					glTexImage2D(GL_TEXTURE_2D, 0, img.channels(), img.cols, img.rows, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, NULL);
				m_videoInfo[chId].w = img.cols;
				m_videoInfo[chId].h = img.rows;
				m_videoInfo[chId].c = img.channels();
				bCreate[chId] = true;
			}
			else
			{
				if(img.channels() == 1)
					glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, img.cols, img.rows, GL_RED, GL_UNSIGNED_BYTE, NULL);
				else if(img.channels() == 3)
					glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, img.cols, img.rows, GL_BGR_EXT, GL_UNSIGNED_BYTE, NULL);
				else if(img.channels() == 4)
					glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, img.cols, img.rows, GL_BGRA_EXT, GL_UNSIGNED_BYTE, NULL);
			}
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
			if(info->memtype == memtype_glpbo){
				cuMap(info);
			}
			image_queue_putEmpty(&m_bufQue[chId], info);
		}
	}
}

void CRender::UpdateOSD(void)
{
	//cv::line(m_imgOsd[0], cv::Point(300, 300), cv::Point(800, 300), cv::Scalar(255), 2, CV_AA);
#if(0)
	{
		glViewport(0, 0, m_mainWinWidth, m_mainWinHeight);
		static GLOSD  glosd;
		GLOSDNumberedBox box(&glosd);
		box.numbox(128, Rect(1920/2-30, 1080/2-20, 60, 40), Scalar(255, 255, 0, 255), 1);
		glosd.Draw();
	}
#endif
#if(0)
	{
		using namespace cr_osd;
		static GLOSD  glosd;
		//static DCOSD  glosd(&m_imgOsd[0]);
		GLOSDLine line1(&glosd);
		GLOSDLine line2(&glosd);
		GLOSDRectangle rectangle1(&glosd);
		GLOSDPolygon ploygon0(&glosd, 3);
		GLOSDRect rect0(&glosd);
		static Point ct(0, 0);
		static int incx = 1;
		static int incy = 1;
		if(ct.x<-(1920/2-100) || ct.x>1920/2-100)
			incx *= -1;
		if(ct.y<-(1080/2-100) || ct.y>1080/2-100)
			incy *= -1;
		ct.x += incx;
		ct.y += incy;
		Point center(1920/2+ct.x, 1080/2+ct.y);
		line1.line(Point(center.x-100, center.y), Point(center.x+100, center.y), Scalar(0, 255, 0, 255), 2);
		line2.line(Point(center.x, center.y-100), Point(center.x, center.y+100), Scalar(255, 255, 0, 255), 2);
		rectangle1.rectangle(Rect(center.x-50, center.y-50, 100, 100), Scalar(255, 0, 0, 255), 1);
		cv::Point pts[] = {cv::Point(center.x, center.y-80),cv::Point(center.x-75, center.y+38),cv::Point(center.x+75, center.y+38)};
		ploygon0.polygon(pts, Scalar(0, 0, 255, 255), 3);
		rect0.rect(Rect(center.x-50, center.y-50, 100, 100), Scalar(28, 28, 28, 255), 6);
		//GLOSDLine line3(&glosd);
		//GLOSDLine line4(&glosd);
		//line3.line(Point(1920/2-50, 1080/2), Point(1920/2+50, 1080/2), Scalar(0, 255, 0, 255), 2);
		//line4.line(Point(1920/2, 1080/2-50), Point(1920/2, 1080/2+50), Scalar(255, 255, 0, 255), 2);
		GLOSDCross cross(&glosd);
		cross.cross(Point(1920/2, 1080/2), Size(50, 50), Scalar(255, 255, 0, 255), 1);
		GLOSDNumberedBox box(&glosd);
		box.numbox(128, Rect(1920/2-30, 1080/2-20, 60, 40), Scalar(255, 255, 0, 255), 1);
		GLOSDTxt txt1(&glosd);
		GLOSDTxt txt2(&glosd);
		static wchar_t strTxt1[128] = L"0";
		txt1.txt(Point(center.x-5, center.y-txt1.m_fontSize+10), strTxt1, Scalar(255, 0, 255, 128));
		static wchar_t strTxt2[128];
		swprintf(strTxt2, 128, L"%d, %d", center.x, center.y);
		txt2.txt(Point(center.x+10, center.y-txt1.m_fontSize-10), strTxt2, Scalar(255, 255, 255, 200));
		glosd.Draw();
	};
#endif
#if(0)
	{
		using namespace cr_osd;
		glViewport(0, 0, m_mainWinWidth, m_mainWinHeight);
		glShaderManager.UseStockShader(GLT_SHADER_TEXTURE_SHADED, 0);
		cv::Size viewSize(m_mainWinWidth, m_mainWinHeight);
		static GLTXT txt2;
		wchar_t strTxt[128] = L"常用命令 hello world ! 1234567890";
		//wchar_t strTxt[256] = L"(hello world ! 1234567890 `!@#$%^&_*__+=-~[]{}|:;',./<>?)";
		//wchar_t strTxt[256] = L"=_";
		swprintf(strTxt, 128, L"常用命令 %6.3f hello world !", OSA_getCurTimeInMsec()*0.001f);
		txt2.putText(viewSize, strTxt, cv::Point(20, 300), cv::Scalar(255, 255, 0, 255));
		txt2.putText(viewSize, strTxt, cv::Point(20, 350), cv::Scalar(0, 255, 255, 255));
		txt2.putText(viewSize, strTxt, cv::Point(20, 400), cv::Scalar(255, 0, 255, 255));
		txt2.putText(viewSize, strTxt, cv::Point(20, 450), cv::Scalar(255, 255, 255, 255));
		txt2.putText(viewSize, strTxt, cv::Point(20, 500), cv::Scalar(0, 0, 0, 255));
	}
#endif
#if(0)
	{
		using namespace cr_osd;
		static DCTXT txt2;
		//wchar_t strTxt[128] = L"常用命令 hello world ! 1234567890";
		wchar_t strTxt[256] = L"(常用命令 hello world ! 1234567890 `!@#$%^&_*__+=-~[]{}|:;',./<>?)";
		//wchar_t strTxt[256] = L"=_";
		//swprintf(strTxt, 128, L"常用命令 hello world !%6.3f", OSA_getCurTimeInMsec()*0.001f);
		txt2.putText(m_imgDC[0], strTxt, cv::Point(20, 300), cv::Scalar(255, 255, 0, 255));
		txt2.putText(m_imgDC[0], strTxt, cv::Point(20, 350), cv::Scalar(0, 255, 255, 255));
		txt2.putText(m_imgDC[0], strTxt, cv::Point(20, 400), cv::Scalar(255, 0, 255, 255));
		txt2.putText(m_imgDC[0], strTxt, cv::Point(20, 450), cv::Scalar(255, 255, 255, 255));
		txt2.putText(m_imgDC[0], strTxt, cv::Point(20, 500), cv::Scalar(0, 0, 0, 255));
	}
#endif

#if 0
	{
		using namespace cr_osd;
		static cv::Mat wave(60, 60, CV_32FC1);
		static cv::Rect rc(1500, 20, 400, 400);
		static IPattern* pattern = NULL;
		//static int cnt = 0;
		//cnt ^=1;
		//wave.setTo(Scalar::all((double)cnt));
		if(pattern == NULL){

			cv::RNG rng = cv::RNG(OSA_getCurTimeInMsec());
			for(int i=0; i<wave.rows; i++){
				for(int j=0; j<wave.cols; j++)
				{
//					wave.at<float>(i, j) = sin(i*2*CV_PI/180.0);
					wave.at<float>(i, j)= std::exp(-1.0*((i-wave.rows/2)*(i-wave.rows/2)+(j-wave.cols/2)*(j-wave.cols/2))/(2.0*10*10)) - 1.0;///(CV_PI*2.0*3.0*3.0);

				}
			}

			pattern = IPattern::Create(wave, rc);
		}
		pattern->draw();
	}
#endif
}

void CRender::gl_display(void)
{
	int winId, chId;
	GLint glProg = 0;
	int iret;
	__suseconds_t cur_us, rd_us;

	for(chId = 0; chId<m_initPrm.nChannels; chId++){
		if(m_bufQue[chId].bMap){
			for(int i=0; i<m_bufQue[chId].numBuf; i++){
				cuMap(&m_bufQue[chId].bufInfo[i]);
				image_queue_putEmpty(&m_bufQue[chId], &m_bufQue[chId].bufInfo[i]);
			}
			m_bufQue[chId].bMap = false;
		}
	}

	int64 tStamp[10];
	tStamp[0] = getTickCount();
	static int64 tstart = 0ul, tend = 0ul;
	tstart = tStamp[0];
	if(tend == 0ul)
		tend = tstart;

	//if(m_initPrm.renderfunc == NULL)
	if(1)
	{
		uint64_t telapse_ns = (uint64_t)(m_telapse*1000000.f);
		uint64_t sleep_ns = render_time_nsec - telapse_ns;
		if (last_render_time.tv_sec != 0 && render_time_nsec>telapse_ns)
		{
			pthread_mutex_lock(&render_lock);
			struct timespec waittime = last_render_time;
			waittime.tv_nsec += sleep_ns;
			waittime.tv_sec += waittime.tv_nsec / 1000000000UL;
			waittime.tv_nsec %= 1000000000UL;
	        /*
	        struct timespec now;
    		clock_gettime(CLOCK_MONOTONIC, &now);
	        cur_us = (now.tv_sec * 1000000.0 + now.tv_nsec/1000.0);
	        rd_us = (waittime.tv_sec * 1000000.0 + waittime.tv_nsec / 1000.0);
	        if (rd_us<cur_us)
	        {
				OSA_printf("%s %d: last(%ld.%ld) now(%ld.%ld)sleep_ns(%ld) wait(%ld.%ld)",
						__func__, __LINE__, last_render_time.tv_sec, last_render_time.tv_nsec/1000UL,
						now.tv_sec, now.tv_usec, sleep_ns, waittime.tv_sec, waittime.tv_nsec/1000UL);
	        }else*/
	        {
	    		pthread_cond_timedwait(&render_cond, &render_lock,&waittime);
	        }
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

	gl_updateTexVideo();
	tStamp[3] = getTickCount();

	tStamp[4] = getTickCount();

    //int viewport[4];
    //glGetIntegerv(GL_VIEWPORT, viewport);
    //glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    //glScissor(viewport[0], viewport[1], viewport[2], viewport[3]);

	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_MULTISAMPLE);
	if(1)
	{
		OSA_mutexLock(&m_mutex);
		for(winId=0; winId<m_renderCount; winId++)
		{
			chId = m_curMap[winId];
			if(m_curMap[winId]!= m_renders[winId].video_chId){
				m_curMap[winId] = m_renders[winId].video_chId;
			}
			if(chId < 0 || chId >= DS_CHAN_MAX)
				continue;

			glUseProgram(m_glProgram[1]);
			GLint Uniform_tex_in = glGetUniformLocation(m_glProgram[1], "tex_in");
			GLint Uniform_mvp = glGetUniformLocation(m_glProgram[1], "mvpMatrix");
			GLMatx44f mTrans = m_glmat44fTrans[chId]*m_renders[winId].transform;
			glUniformMatrix4fv(Uniform_mvp, 1, GL_FALSE, mTrans.val);
			glUniform1i(Uniform_tex_in, 0);
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, textureId_input[chId]);
			glVertexAttribPointer(ATTRIB_VERTEX, 2, GL_FLOAT, GL_FALSE, 0, m_glvVerts[winId]);
			glVertexAttribPointer(ATTRIB_TEXTURE, 2, GL_FLOAT, GL_FALSE, 0, m_glvTexCoords[winId]);
			glEnableVertexAttribArray(ATTRIB_VERTEX);
			glEnableVertexAttribArray(ATTRIB_TEXTURE);
			//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glViewport(m_renders[winId].displayrect.x,
					m_renders[winId].displayrect.y,
					m_renders[winId].displayrect.width, m_renders[winId].displayrect.height);
			glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
			glUseProgram(0);

			int blend_chId = m_blendMap[chId];
			if(blend_chId>=0 && blend_chId<DS_CHAN_MAX){
				mTrans = m_glmat44fBlend[chId*DS_CHAN_MAX+blend_chId]*m_glmat44fTrans[chId]*m_renders[winId].transform;
				int maskId = m_maskMap[blend_chId];
				if(maskId < 0 || maskId >= DS_CHAN_MAX){
					if(m_videoInfo[blend_chId].c == 1){
						glProg = m_glProgram[2];
					}else{
						glProg = m_glProgram[3];
					}
					glUseProgram(glProg);
					GLint Uniform_tex_in = glGetUniformLocation(glProg, "tex_in");
					GLint uniform_fAlpha = glGetUniformLocation(glProg, "fAlpha");
					GLint uniform_thr0Min = glGetUniformLocation(glProg, "thr0Min");
					GLint uniform_thr0Max = glGetUniformLocation(glProg, "thr0Max");
					GLint uniform_thr1Min = glGetUniformLocation(glProg, "thr1Min");
					GLint uniform_thr1Max = glGetUniformLocation(glProg, "thr1Max");
					GLint Uniform_mvp = glGetUniformLocation(glProg, "mvpMatrix");
					glUniformMatrix4fv(Uniform_mvp, 1, GL_FALSE, mTrans.val);
					glUniform1f(uniform_fAlpha, m_glBlendPrm[chId*DS_CHAN_MAX+blend_chId].fAlpha);
					glUniform1f(uniform_thr0Min, m_glBlendPrm[chId*DS_CHAN_MAX+blend_chId].thr0Min);
					glUniform1f(uniform_thr0Max, m_glBlendPrm[chId*DS_CHAN_MAX+blend_chId].thr0Max);
					glUniform1f(uniform_thr1Min, m_glBlendPrm[chId*DS_CHAN_MAX+blend_chId].thr1Min);
					glUniform1f(uniform_thr1Max, m_glBlendPrm[chId*DS_CHAN_MAX+blend_chId].thr1Max);
					glUniform1i(Uniform_tex_in, 0);
					glActiveTexture(GL_TEXTURE0);
					glBindTexture(GL_TEXTURE_2D, textureId_input[blend_chId]);
				}else{
					glUseProgram(m_glProgram[4]);
					GLint Uniform_tex_in = glGetUniformLocation(m_glProgram[4], "tex_in");
					GLint Uniform_tex_mask = glGetUniformLocation(m_glProgram[4], "tex_mask");
					GLint Uniform_mvp = glGetUniformLocation(m_glProgram[4], "mvpMatrix");
					glUniformMatrix4fv(Uniform_mvp, 1, GL_FALSE, mTrans.val);
					glUniform1i(Uniform_tex_in, 0);
					glActiveTexture(GL_TEXTURE0);
					glBindTexture(GL_TEXTURE_2D, textureId_input[blend_chId]);
					glUniform1i(Uniform_tex_mask, 1);
					glActiveTexture(GL_TEXTURE1);
					glBindTexture(GL_TEXTURE_2D, textureId_input[maskId]);
				}
				glVertexAttribPointer(ATTRIB_VERTEX, 2, GL_FLOAT, GL_FALSE, 0, m_glvVerts[winId]);
				glVertexAttribPointer(ATTRIB_TEXTURE, 2, GL_FLOAT, GL_FALSE, 0, m_glvTexCoords[winId]);
				glEnableVertexAttribArray(ATTRIB_VERTEX);
				glEnableVertexAttribArray(ATTRIB_TEXTURE);
				glEnable(GL_MULTISAMPLE);
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				//glViewport(m_renders[winId].displayrect.x,
				//		m_renders[winId].displayrect.y,
				//		m_renders[winId].displayrect.width, m_renders[winId].displayrect.height);
				glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
				glDisable(GL_MULTISAMPLE);
				glDisable(GL_BLEND);
				glUseProgram(0);
			}

			if(m_initPrm.renderfunc != NULL)
				m_initPrm.renderfunc(0, RUN_WIN, winId, chId);
		}
		OSA_mutexUnlock(&m_mutex);
	}
	//glValidateProgram(m_glProgram);

	UpdateOSD();

	if(m_initPrm.renderfunc != NULL)
		m_initPrm.renderfunc(0, RUN_SWAP, 0, 0);

	tStamp[5] = getTickCount();

	int64 tcur = tStamp[5];
	m_telapse = (tStamp[5] - tStamp[1])*0.000001f + m_initPrm.disSched;

	if (last_render_time.tv_sec != 0)
	{
		pthread_mutex_lock(&render_lock);
		struct timespec waittime = last_render_time;
		last_render_time.tv_sec += render_time_sec;
		last_render_time.tv_nsec += render_time_nsec;
		last_render_time.tv_sec += last_render_time.tv_nsec / 1000000000UL;
		last_render_time.tv_nsec %= 1000000000UL;
    	struct timespec now;
    	clock_gettime(CLOCK_MONOTONIC, &now);
        cur_us = (now.tv_sec * 1000000.0 + now.tv_nsec / 1000.0);
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
            OSA_printf("%s %d: win%d frame_is_late(%ld us)", __func__, __LINE__, glutGetWindow(), cur_us - rd_us);
            if(cur_us - rd_us > 10000UL)
            	memset(&last_render_time, 0, sizeof(last_render_time));
        }
		pthread_mutex_unlock(&render_lock);
	}

	//glFinish();

	int64 tSwap = getTickCount();
#ifdef __EGL__
	eglSwapBuffers(egl_display, egl_surface);
#else
	glutSwapBuffers();
#endif
	tStamp[6] = getTickCount();

#if 1
	if(tStamp[6]-tSwap>3000000UL)
		m_nSwapTimeOut++;
	else
		m_nSwapTimeOut = 0;
	if (last_render_time.tv_sec == 0 || m_nSwapTimeOut>=3)
	{
    	struct timespec now;
    	clock_gettime(CLOCK_MONOTONIC, &now);
		last_render_time.tv_sec = now.tv_sec;
		last_render_time.tv_nsec = now.tv_nsec;
		printf("\r\nReset render timer. fps(%d) swp(%ld ns)", m_initPrm.disFPS, tStamp[6]-tSwap);
		fflush(stdout);
	}
#endif
	if(m_initPrm.renderfunc != NULL)
		m_initPrm.renderfunc(0, RUN_LEAVE, 0, 0);
	tend = tStamp[6];
	float renderIntv = (tend - m_tmRender)/getTickFrequency();

#if 1
	static unsigned long rCount = 0;
	if(rCount%(m_initPrm.disFPS*100) == 0){
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
	//if(!(mask&1)){
	//	OSA_printf("%s %d: null", __func__, __LINE__);
	//}
	m_tmRender = tend;
}

void CRender::gl_display2(void)
{
	int winId, chId;
	GLint glProg = 0;
	int iret;
	glClear(GL_COLOR_BUFFER_BIT);
	if(m_initPrm.renderfunc != NULL)
		m_initPrm.renderfunc(1, RUN_ENTER, 0, 0);
	if(1)
	{
		//OSA_mutexLock(&m_mutex);
		for(winId=0; winId<m_renderCount; winId++)
		{
			chId = m_curMap[winId];
			if(m_curMap[winId]!= m_renders[winId].video_chId){
				m_curMap[winId] = m_renders[winId].video_chId;
			}
			if(chId < 0 || chId >= DS_CHAN_MAX)
				continue;

			glUseProgram(m_glProgram[1]);
			GLint Uniform_tex_in = glGetUniformLocation(m_glProgram[1], "tex_in");
			GLint Uniform_mvp = glGetUniformLocation(m_glProgram[1], "mvpMatrix");
			GLMatx44f mTrans = m_glmat44fTrans[chId]*m_renders[winId].transform;
			glUniformMatrix4fv(Uniform_mvp, 1, GL_FALSE, mTrans.val);
			glUniform1i(Uniform_tex_in, 0);
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, textureId_input[chId]);
			glVertexAttribPointer(ATTRIB_VERTEX, 2, GL_FLOAT, GL_FALSE, 0, m_glvVerts[winId]);
			glVertexAttribPointer(ATTRIB_TEXTURE, 2, GL_FLOAT, GL_FALSE, 0, m_glvTexCoords[winId]);
			glEnableVertexAttribArray(ATTRIB_VERTEX);
			glEnableVertexAttribArray(ATTRIB_TEXTURE);
			//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glViewport(m_renders[winId].displayrect.x,
					m_renders[winId].displayrect.y,
					m_renders[winId].displayrect.width, m_renders[winId].displayrect.height);
			glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
			glUseProgram(0);

			int blend_chId = m_blendMap[chId];
			if(blend_chId>=0 && blend_chId<DS_CHAN_MAX){
				mTrans = m_glmat44fBlend[chId*DS_CHAN_MAX+blend_chId]*m_glmat44fTrans[chId]*m_renders[winId].transform;
				int maskId = m_maskMap[blend_chId];
				if(maskId < 0 || maskId >= DS_CHAN_MAX){
					if(m_videoInfo[blend_chId].c == 1){
						glProg = m_glProgram[2];
					}else{
						glProg = m_glProgram[3];
					}
					glUseProgram(glProg);
					GLint Uniform_tex_in = glGetUniformLocation(glProg, "tex_in");
					GLint uniform_fAlpha = glGetUniformLocation(glProg, "fAlpha");
					GLint uniform_thr0Min = glGetUniformLocation(glProg, "thr0Min");
					GLint uniform_thr0Max = glGetUniformLocation(glProg, "thr0Max");
					GLint uniform_thr1Min = glGetUniformLocation(glProg, "thr1Min");
					GLint uniform_thr1Max = glGetUniformLocation(glProg, "thr1Max");
					GLint Uniform_mvp = glGetUniformLocation(glProg, "mvpMatrix");
					glUniformMatrix4fv(Uniform_mvp, 1, GL_FALSE, mTrans.val);
					glUniform1f(uniform_fAlpha, m_glBlendPrm[chId*DS_CHAN_MAX+blend_chId].fAlpha);
					glUniform1f(uniform_thr0Min, m_glBlendPrm[chId*DS_CHAN_MAX+blend_chId].thr0Min);
					glUniform1f(uniform_thr0Max, m_glBlendPrm[chId*DS_CHAN_MAX+blend_chId].thr0Max);
					glUniform1f(uniform_thr1Min, m_glBlendPrm[chId*DS_CHAN_MAX+blend_chId].thr1Min);
					glUniform1f(uniform_thr1Max, m_glBlendPrm[chId*DS_CHAN_MAX+blend_chId].thr1Max);
					glUniform1i(Uniform_tex_in, 0);
					glActiveTexture(GL_TEXTURE0);
					glBindTexture(GL_TEXTURE_2D, textureId_input[blend_chId]);
				}else{
					glUseProgram(m_glProgram[4]);
					GLint Uniform_tex_in = glGetUniformLocation(m_glProgram[4], "tex_in");
					GLint Uniform_tex_mask = glGetUniformLocation(m_glProgram[4], "tex_mask");
					GLint Uniform_mvp = glGetUniformLocation(m_glProgram[4], "mvpMatrix");
					glUniformMatrix4fv(Uniform_mvp, 1, GL_FALSE, mTrans.val);
					glUniform1i(Uniform_tex_in, 0);
					glActiveTexture(GL_TEXTURE0);
					glBindTexture(GL_TEXTURE_2D, textureId_input[blend_chId]);
					glUniform1i(Uniform_tex_mask, 1);
					glActiveTexture(GL_TEXTURE1);
					glBindTexture(GL_TEXTURE_2D, textureId_input[maskId]);
				}
				glVertexAttribPointer(ATTRIB_VERTEX, 2, GL_FLOAT, GL_FALSE, 0, m_glvVerts[winId]);
				glVertexAttribPointer(ATTRIB_TEXTURE, 2, GL_FLOAT, GL_FALSE, 0, m_glvTexCoords[winId]);
				glEnableVertexAttribArray(ATTRIB_VERTEX);
				glEnableVertexAttribArray(ATTRIB_TEXTURE);
				glEnable(GL_MULTISAMPLE);
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				//glViewport(m_renders[winId].displayrect.x,
				//		m_renders[winId].displayrect.y,
				//		m_renders[winId].displayrect.width, m_renders[winId].displayrect.height);
				glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
				glDisable(GL_MULTISAMPLE);
				glDisable(GL_BLEND);
				glUseProgram(0);
			}
			if(m_initPrm.renderfunc != NULL)
				m_initPrm.renderfunc(1, RUN_WIN, winId, chId);
		}
		//OSA_mutexUnlock(&m_mutex);
	}

	if(m_initPrm.renderfunc != NULL)
		m_initPrm.renderfunc(1, RUN_SWAP, 0, 0);
	//glFinish();
#ifdef __EGL__
	eglSwapBuffers(egl_display, egl_surface);
#else
	glutSwapBuffers();
#endif
	if(m_initPrm.renderfunc != NULL)
		m_initPrm.renderfunc(1, RUN_LEAVE, 0, 0);
}

//////////////////////////////////////////////////////
//
static const char *szDefaultShaderVP = ""
		"attribute vec4 vVertex;"
		"attribute vec2 vTexCoords;"
		"varying vec2 vVaryingTexCoords;"
		"void main(void)"
		"{"
		"    gl_Position = vVertex;"
		"    vVaryingTexCoords = vTexCoords;"
		"}";
static const char *szFlatShaderVP = ""
		"attribute vec4 vVertex;"
		"attribute vec2 vTexCoords;"
		"varying vec2 vVaryingTexCoords;"
		"uniform mat4 mvpMatrix;"
		"void main(void)"
		"{"
		"    gl_Position = mvpMatrix*vVertex;"
		"    vVaryingTexCoords = vTexCoords;"
		"}";

static const char *szDefaultShaderFP = ""
		"varying vec2 vVaryingTexCoords;"
		"uniform sampler2D tex_in;"
		"void main(void)"
		"{"
		"	gl_FragColor = texture(tex_in, vVaryingTexCoords);"
		"}";

static const char *szFilterShaderFP = ""
		"varying vec2 vVaryingTexCoords;"
		"uniform sampler2D tex_in;"
		"void main(void)"
		"{"
		"	vec4  vColor;"
		"	float k;"
		"	vColor = texture(tex_in, vVaryingTexCoords);"
		"	k = step(vColor.r, 0.003)*step(vColor.g, 0.003)*step(vColor.b, 0.003);"
		"	gl_FragColor.r = vColor.r*(1-k)+0.00392157*k;"
		"	gl_FragColor.g = vColor.g*(1-k)+0.00392157*k;"
		"	gl_FragColor.b = vColor.b*(1-k)+0.00392157*k;"
		"	gl_FragColor.a = 1.0;"
		"}";

static const char *szAlphaTextureShaderFP = ""
		"varying vec2 vVaryingTexCoords;"
		"uniform sampler2D tex_in;"
		"uniform vec4 vColor;"
		"void main(void)"
		"{"
		"	vec4  vAlpha;"
		"	vAlpha = texture(tex_in, vVaryingTexCoords);"
		"	vColor.a = vAlpha.a;"
		"	gl_FragColor = vColor;"
		"}";
static const char *szBlendMaskTextureShaderFP = ""
		"varying vec2 vVaryingTexCoords;"
		"uniform sampler2D tex_in;"
		"uniform sampler2D tex_mask;"
		"void main(void)"
		"{"
		"	vec4  vColor;"
		"	vec4  vMask;"
		"	vColor = texture(tex_in, vVaryingTexCoords);"
		"	vMask = texture(tex_mask, vVaryingTexCoords);"
		"	vColor.a = vMask.r;"
		"	gl_FragColor = vColor;"
		"}";
static const char *szPolarityShaderFP = ""
		"varying vec2 vVaryingTexCoords;"
		"uniform sampler2D tex_in;"
		"uniform float fAlpha;"
		"uniform float thr0Min;"
		"uniform float thr0Max;"
		"uniform float thr1Min;"
		"uniform float thr1Max;"
		"void main(void)"
		"{"
		"	vec4  vColor;"
		"	vec4  vAlpha;"
		"	float ra0, ra1;"
		"	vColor = texture(tex_in, vVaryingTexCoords);"
		"	ra0 = (1- step(thr0Min,vColor.r)*step(vColor.r,thr0Max) );"
		"	ra1 = (1- step(thr1Min,vColor.r)*step(vColor.r,thr1Max) );"
		"	vColor.a = (1-ra0*ra1)*fAlpha;"
		"	gl_FragColor = vColor;"
		"}";

static const char *szPolarityShaderRGBFP = ""
		"varying vec2 vVaryingTexCoords;"
		"uniform sampler2D tex_in;"
		"uniform float fAlpha;"
		"uniform float thr0Min;"
		"uniform float thr0Max;"
		"uniform float thr1Min;"
		"uniform float thr1Max;"
		"void main(void)"
		"{"
		"	vec4  vColor;"
		"	vec4  vAlpha;"
		"	float ra0,ra1,ga0,ga1,ba0,ba1;"
		"	vColor = texture(tex_in, vVaryingTexCoords);"
#if 0
		"	//vColor.a = fAlpha;"
		"	//vColor.a = step(thrMin, vColor.r)*step(vColor.r, thrMax)*fAlpha;"
		"	//vColor.a = smoothstep(thrMin, thrMax, vColor.r)*step(vColor.r, thrMax)*fAlpha;"
		"	ra = (1-step(thr0Min, vColor.r)*step(vColor.r, thr0Max))*(1-step(thr1Min, vColor.r)*step(vColor.r, thr1Max));"
		"	ga = (1-step(thr0Min, vColor.g)*step(vColor.g, thr0Max))*(1-step(thr1Min, vColor.g)*step(vColor.g, thr1Max));"
		"	ba = (1-step(thr0Min, vColor.b)*step(vColor.b, thr0Max))*(1-step(thr1Min, vColor.b)*step(vColor.b, thr1Max));"
		"	ra = (1-(1-step(vColor.r,thr0Min))*(1-step(thr0Max, vColor.r)))*(1-(1-step(vColor.r,thr1Min))*step(vColor.r, thr1Max));"
		"	ga = (1-(1-step(vColor.g,thr0Min))*(1-step(thr0Max, vColor.g)))*(1-(1-step(vColor.g,thr1Min))*step(vColor.g, thr1Max));"
		"	ba = (1-(1-step(vColor.b,thr0Min))*(1-step(thr0Max, vColor.b)))*(1-(1-step(vColor.b,thr1Min))*step(vColor.b, thr1Max));"
		"	gray = vColor.r*0.299+vColor.g*0.587+vColor.b*0.114;"
#endif
		"	ra0 = (1- step(thr0Min,vColor.r)*step(vColor.r,thr0Max) );"
		"	ra1 = (1- step(thr1Min,vColor.r)*step(vColor.r,thr1Max) );"
		"	ga0 = (1- step(thr0Min,vColor.g)*step(vColor.g,thr0Max) );"
		"	ga1 = (1- step(thr1Min,vColor.g)*step(vColor.g,thr1Max) );"
		"	ba0 = (1- step(thr0Min,vColor.b)*step(vColor.b,thr0Max) );"
		"	ba1 = (1- step(thr1Min,vColor.b)*step(vColor.b,thr1Max) );"
		"	vColor.a = (1-ra0*ra1*ga0*ga1*ba0*ba1)*fAlpha;"
		"	gl_FragColor = vColor;"
		"}";
int CRender::gl_loadProgram()
{
	int iRet = OSA_SOK;
	m_glProgram[0] = gltLoadShaderPairWithAttributes(szDefaultShaderVP, szDefaultShaderFP, 2, ATTRIB_VERTEX, "vVertex", ATTRIB_TEXTURE, "vTexCoords");
	m_glProgram[1] = gltLoadShaderPairWithAttributes(szFlatShaderVP, szDefaultShaderFP, 2, ATTRIB_VERTEX, "vVertex", ATTRIB_TEXTURE, "vTexCoords");
	m_glProgram[2] = gltLoadShaderPairWithAttributes(szFlatShaderVP, szPolarityShaderFP, 2, ATTRIB_VERTEX, "vVertex", ATTRIB_TEXTURE, "vTexCoords");
	m_glProgram[3] = gltLoadShaderPairWithAttributes(szFlatShaderVP, szPolarityShaderRGBFP, 2, ATTRIB_VERTEX, "vVertex", ATTRIB_TEXTURE, "vTexCoords");
	m_glProgram[4] = gltLoadShaderPairWithAttributes(szFlatShaderVP, szBlendMaskTextureShaderFP, 2, ATTRIB_VERTEX, "vVertex", ATTRIB_TEXTURE, "vTexCoords");
	m_glProgram[5] = gltLoadShaderPairWithAttributes(szDefaultShaderVP, szAlphaTextureShaderFP, 2, ATTRIB_VERTEX, "vVertex", ATTRIB_TEXTURE, "vTexCoords");
	return iRet;
}

int CRender::gl_unloadProgram()
{
	int iRet = OSA_SOK;
	int i;
	for(i=0; i<8; i++){
		if(m_glProgram[i] != 0)
			glDeleteProgram(m_glProgram[i]);
		m_glProgram[i] = 0;
	}
	return iRet;
}

//////////////////////////////////////////////////////////////////////////
// Load the shader from the source text
bool CRender::gltLoadShaderSrc(const char *szShaderSrc, GLuint shader)
{
	if(szShaderSrc == NULL)
		return false;
	GLchar *fsStringPtr[1];

	fsStringPtr[0] = (GLchar *)szShaderSrc;
	glShaderSource(shader, 1, (const GLchar **)fsStringPtr, NULL);

	return true;
}

#define MAX_SHADER_LENGTH   8192
static GLubyte shaderText[MAX_SHADER_LENGTH];
////////////////////////////////////////////////////////////////
// Load the shader from the specified file. Returns false if the
// shader could not be loaded
bool CRender::gltLoadShaderFile(const char *szFile, GLuint shader)
{
	GLint shaderLength = 0;
	FILE *fp;

	// Open the shader file
	fp = fopen(szFile, "r");
	if(fp != NULL)
	{
		// See how long the file is
		while (fgetc(fp) != EOF)
			shaderLength++;

		// Allocate a block of memory to send in the shader
		assert(shaderLength < MAX_SHADER_LENGTH);   // make me bigger!
		if(shaderLength > MAX_SHADER_LENGTH)
		{
			fclose(fp);
			return false;
		}

		// Go back to beginning of file
		rewind(fp);

		// Read the whole file in
		if (shaderText != NULL){
			size_t ret = fread(shaderText, 1, (size_t)shaderLength, fp);
			OSA_assert(ret == shaderLength);
		}

		// Make sure it is null terminated and close the file
		shaderText[shaderLength] = '\0';
		fclose(fp);
	}
	else
		return false;    

	// Load the string
	gltLoadShaderSrc((const char *)shaderText, shader);

	return true;
}   

/////////////////////////////////////////////////////////////////
// Load a pair of shaders, compile, and link together. Specify the complete
// source text for each shader. After the shader names, specify the number
// of attributes, followed by the index and attribute name of each attribute
GLuint CRender::gltLoadShaderPairWithAttributes(const char *szVertexProg, const char *szFragmentProg, ...)
{
	// Temporary Shader objects
	GLuint hVertexShader;
	GLuint hFragmentShader; 
	GLuint hReturn = 0;   
	GLint testVal;

	// Create shader objects
	hVertexShader = glCreateShader(GL_VERTEX_SHADER);
	hFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	// Load them. If fail clean up and return null
	// Vertex Program
	//if(gltLoadShaderFile(szVertexProg, hVertexShader) == false)
	if(gltLoadShaderSrc(szVertexProg, hVertexShader) == false)
	{
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		fprintf(stderr, "The shader at %s could ot be found.\n", szVertexProg);
		return (GLuint)NULL;
	}

	// Fragment Program
	//if(gltLoadShaderFile(szFragmentProg, hFragmentShader) == false)
	if(gltLoadShaderSrc(szFragmentProg, hFragmentShader) == false)
	{
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		fprintf(stderr,"The shader at %s  could not be found.\n", szFragmentProg);
		return (GLuint)NULL;
	}

	// Compile them both
	glCompileShader(hVertexShader);
	glCompileShader(hFragmentShader);

	// Check for errors in vertex shader
	glGetShaderiv(hVertexShader, GL_COMPILE_STATUS, &testVal);
	if(testVal == GL_FALSE)
	{
		char infoLog[1024];
		glGetShaderInfoLog(hVertexShader, 1024, NULL, infoLog);
		fprintf(stderr, "The shader at %s failed to compile with the following error:\n%s\n", szVertexProg, infoLog);
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		return (GLuint)NULL;
	}

	// Check for errors in fragment shader
	glGetShaderiv(hFragmentShader, GL_COMPILE_STATUS, &testVal);
	if(testVal == GL_FALSE)
	{
		char infoLog[1024];
		glGetShaderInfoLog(hFragmentShader, 1024, NULL, infoLog);
		fprintf(stderr, "The shader at %s failed to compile with the following error:\n%s\n", szFragmentProg, infoLog);
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		return (GLuint)NULL;
	}

	// Create the final program object, and attach the shaders
	hReturn = glCreateProgram();
	glAttachShader(hReturn, hVertexShader);
	glAttachShader(hReturn, hFragmentShader);


	// Now, we need to bind the attribute names to their specific locations
	// List of attributes
	va_list attributeList;
	va_start(attributeList, szFragmentProg);

	// Iterate over this argument list
	char *szNextArg;
	int iArgCount = va_arg(attributeList, int);	// Number of attributes
	for(int i = 0; i < iArgCount; i++)
	{
		int index = va_arg(attributeList, int);
		szNextArg = va_arg(attributeList, char*);
		glBindAttribLocation(hReturn, index, szNextArg);
	}
	va_end(attributeList);

	// Attempt to link    
	glLinkProgram(hReturn);

	// These are no longer needed
	glDeleteShader(hVertexShader);
	glDeleteShader(hFragmentShader);  

	// Make sure link worked too
	glGetProgramiv(hReturn, GL_LINK_STATUS, &testVal);
	if(testVal == GL_FALSE)
	{
		char infoLog[1024];
		glGetProgramInfoLog(hReturn, 1024, NULL, infoLog);
		fprintf(stderr,"The programs %s and %s failed to link with the following errors:\n%s\n",
			szVertexProg, szFragmentProg, infoLog);
		glDeleteProgram(hReturn);
		return (GLuint)NULL;
	}

	// All done, return our ready to use shader program
	return hReturn;  
}   
