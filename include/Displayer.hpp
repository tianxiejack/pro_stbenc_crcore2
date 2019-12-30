
#ifndef DISPLAYER_HPP_
#define DISPLAYER_HPP_

//#define __EGL__
#define DS_RENDER_MAX		(9)
#define DS_CHAN_MAX         (4)

#include <opencv2/opencv.hpp>
#include <osa_buf.h>

#ifdef __EGL__
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#else
#include <glew.h>
#include <glut.h>
#include <freeglut_ext.h>
#endif

using namespace std;

typedef cv::Matx<GLfloat, 4, 4> GLMatx44f;

typedef struct _ds_chninfo{
	int w;
	int h;
	int c;
	int fps;
}DS_ChnInfo;

typedef cv::Rect DS_Rect;

typedef struct _ds_render
{
	int video_chId;
	DS_Rect displayrect;
	bool bCrop;
	DS_Rect croprect;
	GLMatx44f transform;
}DS_Render;

typedef struct _ds_blend_param{
	GLfloat fAlpha;
	GLfloat thr0Min;
	GLfloat thr0Max;
	GLfloat thr1Min;
	GLfloat thr1Max;
}DS_BlendPrm;

typedef struct _ds_init_param{
	bool bFullScreen;
	int winPosX;
	int winPosY;
	int winWidth;
	int winHeight;
	int disFPS;
	float disSched;
	bool bScript;
	char szScriptFile[256];
	DS_ChnInfo channelInfo[DS_CHAN_MAX];
	int nChannels;
	int nQueueSize;
	int memType;
	//void (*displayfunc)(void);
	void (*mousefunc)(int button, int state, int x, int y);
	//void (*reshapefunc)(int width, int height);
	void (*keyboardfunc)(unsigned char key, int x, int y);
	void (*keySpecialfunc)( int, int, int );
	void (*visibilityfunc)(int state);
	void (*closefunc)(void);
	void (*renderfunc)(int displayId, int stepIdx, int stepSub, int context);
}DS_InitPrm;

class CRender
{
	int tag;
	CRender();
	virtual ~CRender();
public:
	static CRender* createObject();
	static void destroyObject(CRender* obj);
	int create(DS_InitPrm *pPrm);
	int destroy();
	int setFPS(float fps);

	typedef enum{
		DS_CFG_ChId = 0,
		DS_CFG_BlendChId,
		DS_CFG_MaskChId,
		DS_CFG_CropEnable,
		DS_CFG_CropRect,
		DS_CFG_VideoTransMat,
		DS_CFG_ViewTransMat,
		DS_CFG_BlendTransMat,
		DS_CFG_BlendPrm,
		DS_CFG_ViewPos,
		DS_CFG_Max
	}DS_CFG;

	enum{
		RUN_ENTER = 0,
		RUN_WIN,
		RUN_SWAP,
		RUN_LEAVE
	};

	int dynamic_config(DS_CFG type, int iPrm, void* pPrm);
	int m_winId;
	int m_winWidth;
	int m_winHeight;
	bool m_bFullScreen;
	//int m_thickness;
	DS_ChnInfo m_videoInfo[DS_CHAN_MAX];
	GLuint buffId_input[DS_CHAN_MAX];
	cr_osa::OSA_BufHndl m_bufQue[DS_CHAN_MAX];
	//OSA_MutexHndl *m_cumutex;
public:
	DS_InitPrm m_initPrm;
	DS_Render m_renders[DS_RENDER_MAX];
	int m_curMap[DS_RENDER_MAX];
	int m_blendMap[DS_CHAN_MAX];
	int m_maskMap[DS_CHAN_MAX];
	int m_renderCount;
	int initRender(bool updateMap = true);

protected:
	static void _display(void);
	static void _reshape(int width, int height);
	static void _close(void);

	static void _display2(void);
	void gl_display2();

protected:
	GLint	m_glProgram[8];
	GLfloat m_glvVerts[DS_RENDER_MAX][8];
	GLfloat m_glvTexCoords[DS_RENDER_MAX][8];
	bool m_bUpdateVertex;
	GLMatx44f m_glmat44fTrans[DS_CHAN_MAX];
	GLMatx44f m_glmat44fBlend[DS_CHAN_MAX*DS_CHAN_MAX];
	DS_BlendPrm m_glBlendPrm[DS_CHAN_MAX*DS_CHAN_MAX];
	GLuint textureId_input[DS_CHAN_MAX];

	int gl_init();
	void gl_uninit();
	void gl_display();
	GLuint gl_PBO(int chId, int width, int height, int channels);
	void gl_updateTexVideo();
	int gl_updateVertex();
	int gl_loadProgram();
	int gl_unloadProgram();
	bool gltLoadShaderSrc(const char *szShaderSrc, GLuint shader);
	bool gltLoadShaderFile(const char *szFile, GLuint shader);
	GLuint gltLoadShaderPairWithAttributes(const char *szVertexProg, const char *szFragmentProg, ...);

	void UpdateOSD(void);

private:
	OSA_MutexHndl m_mutex;
	uint64  m_interval;
	double m_telapse;
	uint64  m_tmBak[DS_CHAN_MAX];
	int64   m_tmRender;

	pthread_mutex_t render_lock;    /**< Used for synchronization. */
	pthread_cond_t render_cond;     /**< Used for synchronization. */
    uint64_t render_time_sec;       /**< Seconds component of the time for which a
                                         frame should be displayed. */
    uint64_t render_time_nsec;      /**< Nanoseconds component of the time for which
                                         a frame should be displayed. */
    struct timespec last_render_time;   /**< Rendering time for the last buffer. */
    int m_nSwapTimeOut;

#ifdef __EGL__
private:
    Display * x_display;    /**< Connection to the X server created using
                                  XOpenDisplay(). */
    Window x_window;        /**< Holds the window to be used for rendering
                                  created using XCreateWindow(). */

    EGLDisplay egl_display;     /**< Holds the EGL Display connection. */
    EGLContext egl_context;     /**< Holds the EGL rendering context. */
    EGLSurface egl_surface;     /**< Holds the EGL Window render surface. */
    EGLConfig egl_config;       /**< Holds the EGL frame buffer configuration
                                     to be usedfor rendering. */

    bool stop_thread;   /**< Boolean variable used to signal rendering thread
                             to stop. */
    pthread_t render_thread;        /**< The pthread id of the rendering thread. */

    //uint32_t texture_id;        /**< Holds the GL Texture ID used for rendering. */
    GC gc;                      /**< Graphic Context */
    XFontStruct *fontinfo;      /**< Brush's font info */
    char overlay_str[512];       /**< Overlay's text */

    static void * renderThread(void *arg){
    	CRender *renderer = (CRender *) arg;
    	renderer->renderHandle();
    }
    int renderHandle(void);
#endif
};

#endif /* DISPLAYER_HPP_ */
