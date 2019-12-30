/*
 * secondScreenBase.cpp
 *
 *  Created on: Jul 5, 2019
 *      Author: ubuntu
 */

/*********************************************************************
 *
 *
 */
#include <opencv2/opencv.hpp>
#include "gluVideoWindowSecondScreen.hpp"
#include "crCore.hpp"
#include "glosd.hpp"
#include "crcoreSecondScreen.hpp"

namespace cr_osd
{
extern GLShaderManagerMini		glShaderManager;
extern std::vector<cr_osd::GLOSDFactory *> vosdFactorys;
};

typedef struct _secondScreen_ct{
	unsigned int teg;
	CGluVideoWindowSecond *videoWin;
	ICore_1001 *core;
	cr_osd::GLOSD *glosdFront;
	cv::Rect rc;
	int fps;
	int parent;
	bool bFull;
	int fontSize;
	char fontFile[256];
}SECONDSCREEN_CT;
#define SSCT_TEG	0x55ff00AA

static CSecondScreenBase *gSecondScreen = NULL;
static void SecondScreen_renderCall(int winId, int stepIdx, int stepSub, int context)
{
	OSA_assert(gSecondScreen != NULL);
	gSecondScreen->renderCall(stepIdx, stepSub, context);
}

CSecondScreenBase::CSecondScreenBase(const cv::Rect& rc, int fps, bool bFull, int fontSize, const char* fontFile)
{
	OSA_assert(gSecondScreen == NULL);
	gSecondScreen = this;
	SECONDSCREEN_CT *ctx = (SECONDSCREEN_CT*)malloc(sizeof(SECONDSCREEN_CT));
	memset(ctx, 0, sizeof(SECONDSCREEN_CT));
	ctx->teg = SSCT_TEG;
	ctx->rc = rc;
	ctx->fps = fps;
	ctx->parent = 0;
	ctx->bFull = bFull;
	ctx->videoWin = new CGluVideoWindowSecond(ctx->rc, ctx->parent);
	ctx->core = (ICore_1001 *)ICore::Qury(COREID_1001);
	ctx->glosdFront = NULL;
	ctx->fontSize = fontSize;
	if(fontFile != NULL)
		strcpy(ctx->fontFile, fontFile);
	else
		memset(ctx->fontFile, 0, sizeof(ctx->fontFile));

	OSA_printf("%s %d: %s rc(%d,%d,%d,%d)", __FILE__, __LINE__, __func__, rc.x, rc.y, rc.width, rc.height);

	CGluVideoWindowSecond *videoWin = ctx->videoWin;
    VWIND_Prm vwinPrm;
    memset(&vwinPrm, 0, sizeof(vwinPrm));
    vwinPrm.disFPS = ctx->fps;
    vwinPrm.bFullScreen = ctx->bFull;
	vwinPrm.renderfunc = SecondScreen_renderCall;
	vwinPrm.disSched = ctx->core->m_stats.renderSched;//cr_local::render->m_initPrm.disSched;
    int iRet = ctx->videoWin->Create(vwinPrm);
    OSA_assert(iRet == OSA_SOK);
	ctx->glosdFront = new cr_osd::GLOSD(videoWin->m_rc.width, videoWin->m_rc.height, fontSize, fontFile);
	OSA_assert(ctx->glosdFront != NULL);
	cr_osd::vosdFactorys.push_back(ctx->glosdFront);

	m_context = ctx;
	OSA_printf("[%d]%s %d %s", OSA_getCurTimeInMsec(), __FILE__, __LINE__, __func__);
}

CSecondScreenBase::~CSecondScreenBase()
{
	SECONDSCREEN_CT *ctx = (SECONDSCREEN_CT*)m_context;
	if(ctx!=NULL){
		if(ctx->glosdFront != NULL)
			delete ctx->glosdFront;
		delete ctx->videoWin;
		ICore::Release(ctx->core);
		free(m_context);
	}
	m_context = NULL;
}

void CSecondScreenBase::renderCall(int stepIdx, int stepSub, int context)
{
	SECONDSCREEN_CT *ctx = (SECONDSCREEN_CT*)m_context;
	OSA_assert(ctx->teg == SSCT_TEG);
	CGluVideoWindowSecond *videoWin = ctx->videoWin;

	OnRender(stepIdx, stepSub, context);
	if(stepIdx == CGluVideoWindow::RUN_WIN)
	{
		//if(cr_local::enableOSDFlag && stepSub == 0 && context > 0 && cr_local::vOSDs[context] != NULL){
		//	cr_local::vOSDs[context]->Draw();
		//}
	}
	if(stepIdx == CGluVideoWindow::RUN_SWAP)
	{
#if(0)
		using namespace cr_osd;
		OSA_assert(ctx->glosdFront != NULL);
		int width = ctx->glosdFront->m_viewport.width;
		int height = ctx->glosdFront->m_viewport.height;
		GLOSDLine line1(ctx->glosdFront);
		GLOSDLine line2(ctx->glosdFront);
		GLOSDRectangle rectangle1(ctx->glosdFront);
		GLOSDPolygon ploygon0(ctx->glosdFront, 3);
		GLOSDRect rect0(ctx->glosdFront);
		static Point ct(0, 0);
		static int incx = 1;
		static int incy = 1;
		if(ct.x<-(width/2-100) || ct.x>width/2-100)
			incx *= -1;
		if(ct.y<-(height/2-100) || ct.y>height/2-100)
			incy *= -1;
		ct.x += incx;
		ct.y += incy;
		Point center(width/2+ct.x, height/2+ct.y);
		line1.line(Point(center.x-100, center.y), Point(center.x+100, center.y), Scalar(0, 255, 0, 255), 2);
		line2.line(Point(center.x, center.y-100), Point(center.x, center.y+100), Scalar(255, 255, 0, 255), 2);
		rectangle1.rectangle(Rect(center.x-50, center.y-50, 100, 100), Scalar(255, 0, 0, 255), 1);
		cv::Point pts[] = {cv::Point(center.x, center.y-80),cv::Point(center.x-75, center.y+38),cv::Point(center.x+75, center.y+38)};
		ploygon0.polygon(pts, Scalar(0, 0, 255, 255), 3);
		rect0.rect(Rect(center.x-50, center.y-50, 100, 100), Scalar(28, 28, 28, 255), 6);
		//GLOSDLine line3(&glosd);
		//GLOSDLine line4(&glosd);
		//line3.line(Point(width/2-50, height/2), Point(width/2+50, height/2), Scalar(0, 255, 0, 255), 2);
		//line4.line(Point(width/2, height/2-50), Point(width/2, height/2+50), Scalar(255, 255, 0, 255), 2);
		GLOSDCross cross(ctx->glosdFront);
		cross.cross(Point(width/2, height/2), Size(50, 50), Scalar(255, 255, 0, 255), 1);
		GLOSDNumberedBox box(ctx->glosdFront);
		box.numbox(128, Rect(width/2-30, height/2-20, 60, 40), Scalar(255, 255, 0, 255), 1);
		GLOSDTxt txt1(ctx->glosdFront);
		GLOSDTxt txt2(ctx->glosdFront);
		static wchar_t strTxt1[128] = L"0";
		txt1.txt(Point(center.x-5, center.y-txt1.m_fontSize+10), strTxt1, Scalar(255, 0, 255, 128));
		static wchar_t strTxt2[128];
		swprintf(strTxt2, 128, L"%d, %d", center.x, center.y);
		txt2.txt(Point(center.x+10, center.y-txt1.m_fontSize-10), strTxt2, Scalar(255, 255, 255, 200));
#endif
		OSA_assert(ctx->glosdFront != NULL);
		ctx->glosdFront->Draw();
	}
}

int CSecondScreenBase::set(int winId, int chId, const cv::Rect& rc, const cv::Matx44f& matric)
{
	SECONDSCREEN_CT *ctx = (SECONDSCREEN_CT*)m_context;
	OSA_assert(ctx->teg == SSCT_TEG);
	CGluVideoWindowSecond *videoWin = ctx->videoWin;
	int iRet = videoWin->dynamic_config(CGluVideoWindow::VWIN_CFG_ChId, winId, &chId);
	if(iRet != OSA_SOK)
		return iRet;
	cv::Rect rcNew = rc;
	iRet = videoWin->dynamic_config(CGluVideoWindow::VWIN_CFG_ViewPos, winId, &rcNew);
	if(iRet != OSA_SOK)
		return iRet;
	cv::Matx44f matricNew = matric;
	return videoWin->dynamic_config(CGluVideoWindow::VWIN_CFG_ViewTransMat, winId, matricNew.val);
}

int CSecondScreenBase::set(int winId, int chId)
{
	SECONDSCREEN_CT *ctx = (SECONDSCREEN_CT*)m_context;
	OSA_assert(ctx->teg == SSCT_TEG);
	CGluVideoWindowSecond *videoWin = ctx->videoWin;
	return videoWin->dynamic_config(CGluVideoWindow::VWIN_CFG_ChId, winId, &chId);;
}

int CSecondScreenBase::set(int winId, const cv::Rect& rc)
{
	int iRet = OSA_SOK;
	cv::Rect rcNew = rc;
	SECONDSCREEN_CT *ctx = (SECONDSCREEN_CT*)m_context;
	OSA_assert(ctx->teg == SSCT_TEG);
	CGluVideoWindowSecond *videoWin = ctx->videoWin;
	iRet = videoWin->dynamic_config(CGluVideoWindow::VWIN_CFG_ViewPos, winId, &rcNew);
	return iRet;
}

int CSecondScreenBase::set(int winId, const cv::Matx44f& matric)
{
	int iRet = OSA_SOK;
	cv::Matx44f matricNew = matric;
	SECONDSCREEN_CT *ctx = (SECONDSCREEN_CT*)m_context;
	OSA_assert(ctx->teg == SSCT_TEG);
	CGluVideoWindowSecond *videoWin = ctx->videoWin;
	iRet = videoWin->dynamic_config(CGluVideoWindow::VWIN_CFG_ViewTransMat, winId, matricNew.val);
	return iRet;
}



