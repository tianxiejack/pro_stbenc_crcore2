/*
 * renderBase.cpp
 *
 *  Created on: Jul 5, 2019
 *      Author: ubuntu
 */

#include <opencv2/opencv.hpp>
#include "crCore.hpp"
#include "renderBase.hpp"
#include "crcoreRender.hpp"
#include "gluVideoWindow.hpp"
#include "osa_image_queue.h"
#include "glvideo.hpp"
#include "glvideoRender.hpp"

namespace cr_core
{
	class CRenderBaseImpl : public RenderBase
	{
	public:
		CRenderBaseImpl(int WinId, CCoreRenderBase *render, const cv::Scalar& clearColor = cv::Scalar(255, 0, 0, 0));
		virtual ~CRenderBaseImpl();
		void Draw();
		int Draw(const cv::Rect& pos);
		int Draw(const cv::Rect& pos, const cv::Mat& img);
		int Draw(const cv::Rect& pos, const cv::Rect& roi);
		int Draw(const cv::Rect& pos, int videoId, const cv::Rect& roi);

		CGluVideoWindow *m_window;
		CCoreRenderBase *m_render;

	//private:
	protected:
		ICore_1001 *m_core;
		cv::Size m_size;
		int m_channels;
		GLuint textureId;
		GLMatx44f m_matrix;
		cv::Scalar m_defaultColor;
	};

	CRenderBaseImpl::CRenderBaseImpl(int winId, CCoreRenderBase *render, const cv::Scalar& clearColor) :
		m_size(cv::Size(0, 0)),m_channels(0),textureId(0), m_defaultColor(clearColor)
	{
		m_core = (ICore_1001*)ICore::Qury(COREID_1001);
		OSA_assert(m_core);
		if(winId <= 1)
			m_window = (CGluVideoWindow*)ICore::QureyObj(1);
		else
			m_window = (CGluVideoWindow*)ICore::QureyObj(2);

		OSA_assert(m_window != NULL);
		m_render = render;

		glGenTextures(1, &textureId);
		glBindTexture(GL_TEXTURE_2D, textureId);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);//GL_NEAREST);//GL_NEAREST_MIPMAP_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);//GL_NEAREST);//GL_NEAREST_MIPMAP_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);//GL_CLAMP);//GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);//GL_CLAMP);//GL_CLAMP_TO_EDGE);
		glBindTexture(GL_TEXTURE_2D, 0);

		m_matrix = GLMatx44f::eye();

		m_window->vRenders.push_back(this);
	}

	CRenderBaseImpl::~CRenderBaseImpl()
	{
		std::vector<RenderBase*>::iterator it;
		for(it=m_window->vRenders.begin();it!=m_window->vRenders.end();++it){
			if(this == (*it)){
				m_window->vRenders.erase(it);
				break;
			}
		}
		ICore::Release(m_core);

		glDeleteTextures(1, &textureId);
	}

	void CRenderBaseImpl::Draw()
	{
		m_render->OnRender();
	}

	int CRenderBaseImpl::Draw(const cv::Rect& pos)
	{
		if(pos.width <=0 || pos.height <= 0)
			return 0;

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

		GLint glProg = CGLVideoRender::m_glProgram[1];
		glUseProgram(glProg);
		GLint Uniform_tex_in = glGetUniformLocation(glProg, "tex_in");
		glUniform1i(Uniform_tex_in, 0);
		GLint Uniform_mvp = glGetUniformLocation(glProg, "mvpMatrix");
		GLMatx44f mTrans = m_matrix;
		glUniformMatrix4fv(Uniform_mvp, 1, GL_FALSE, mTrans.val);
		glEnable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textureId);
		glVertexAttribPointer(ATTRIB_VERTEX, 2, GL_FLOAT, GL_FALSE, 0, defaultVertices);
		glVertexAttribPointer(ATTRIB_TEXTURE, 2, GL_FLOAT, GL_FALSE, 0, defaultTextureCoords);
		glEnableVertexAttribArray(ATTRIB_VERTEX);
		glEnableVertexAttribArray(ATTRIB_TEXTURE);
		glViewport(pos.x, pos.y, pos.width, pos.height);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		glBindTexture(GL_TEXTURE_2D, 0);
		glDisable(GL_TEXTURE_2D);
		glUseProgram(0);
	}

	int CRenderBaseImpl::Draw(const cv::Rect& pos, const cv::Mat& img)
	{
		int channels = img.channels();
		int width = img.cols;
		int height = img.rows;
		glBindTexture(GL_TEXTURE_2D, textureId);
		if(channels == 1)
			glTexImage2D(GL_TEXTURE_2D, 0, channels, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, img.data);
		else
			glTexImage2D(GL_TEXTURE_2D, 0, channels, width, height, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, img.data);
		glBindTexture(GL_TEXTURE_2D, 0);
		return Draw(pos);
	}

	int CRenderBaseImpl::Draw(const cv::Rect& pos, int videoId, const cv::Rect& roi)
	{
		bool bClear = false;
		cv::Size curSize = cv::Size(roi.width, roi.height);

		if(curSize.width == 0 || curSize.height == 0){
			curSize = cv::Size(pos.width, pos.height);
			bClear = true;
		}

		int nVideo = m_window->m_vvideos.size();
		int channels = m_window->m_vvideos[videoId]->m_channels;
		GLuint srcBufferId = m_window->m_vvideos[videoId]->m_curBufInfo.pbo;
		cv::Size srcSize = m_window->m_vvideos[videoId]->m_size;
		if(srcBufferId == 0)
			m_window->m_vvideos[videoId]->m_pbo;

		if(bClear)
		{
			m_size = curSize;
			m_channels = channels;
			//std::cout << __func__ << " Clear " << " pos = " << pos << " videoId = " << videoId << " roi " << roi << "m_size = " << m_size << std::endl;

			glBindTexture(GL_TEXTURE_2D, textureId);
			cv::Mat mFull = cv::Mat(m_size.height, m_size.width, CV_8UC3, m_defaultColor);
			glTexImage2D(GL_TEXTURE_2D, 0, channels, m_size.width, m_size.height, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, mFull.data);
			glBindTexture(GL_TEXTURE_2D, 0);
			m_matrix = GLMatx44f::eye();
		}
		else if(m_size.width != curSize.width || m_size.height != curSize.height || m_channels != channels)
		{
			m_size = curSize;
			m_channels = channels;
			//std::cout << __func__ << " Init " << " pos = " << pos << " videoId = " << videoId << " roi " << roi << "m_size = " << m_size << std::endl;
		}

		if(!bClear){
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, srcBufferId);
			glBindTexture(GL_TEXTURE_2D, textureId);
			if(channels == 1)
				glTexImage2D(GL_TEXTURE_2D, 0, channels, srcSize.width, srcSize.height, 0, GL_RED, GL_UNSIGNED_BYTE, NULL);
			else
				glTexImage2D(GL_TEXTURE_2D, 0, channels, srcSize.width, srcSize.height, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, NULL);
			glBindTexture(GL_TEXTURE_2D, 0);
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

			cv::Matx44f matricScale;
			cv::Matx44f matricTranslate;
			matricScale = cv::Matx44f::eye();
			matricTranslate = cv::Matx44f::eye();

			cv::Point2f center((float)(roi.x+(roi.width>>1))/(srcSize.width>>1)-1.0f, (float)(roi.y+(roi.height>>1))/(srcSize.height>>1)-1.0f);
			matricTranslate.val[3] = -1.0f * center.x;
			matricTranslate.val[7] = center.y;
			matricScale.val[0] = (float)srcSize.width/roi.width;
			matricScale.val[5] = (float)srcSize.height/roi.height;

			m_matrix = (matricScale * matricTranslate).t();
		}
		return Draw(pos);
	}

	int CRenderBaseImpl::Draw(const cv::Rect& pos, const cv::Rect& roi)
	{
		return Draw(pos, m_core->m_stats.mainChId, roi);
	}

}

CCoreRenderBase::CCoreRenderBase(int winId, const cv::Scalar& bgColor)
{
	cr_core::CRenderBaseImpl *base = new cr_core::CRenderBaseImpl(winId, this, bgColor);
	m_render  = base;
	OSA_assert(m_render);
}

CCoreRenderBase::~CCoreRenderBase()
{
	if(m_render != NULL){
		cr_core::CRenderBaseImpl *base = (cr_core::CRenderBaseImpl *)m_render;
		delete base;
		m_render = NULL;
	}
}

int CCoreRenderBase::Draw(const cv::Rect& pos)
{
	OSA_assert(m_render);
	cr_core::CRenderBaseImpl *base = (cr_core::CRenderBaseImpl *)m_render;
	return base->Draw(pos);
}

int CCoreRenderBase::Draw(const cv::Rect& pos, const cv::Mat& img)
{
	OSA_assert(m_render);
	cr_core::CRenderBaseImpl *base = (cr_core::CRenderBaseImpl *)m_render;
	return base->Draw(pos, img);
}

int CCoreRenderBase::Draw(const cv::Rect& pos, const cv::Rect& roi)
{
	OSA_assert(m_render);
	cr_core::CRenderBaseImpl *base = (cr_core::CRenderBaseImpl *)m_render;
	return base->Draw(pos, roi);
}

int CCoreRenderBase::Draw(const cv::Rect& pos, int videoId, const cv::Rect& roi)
{
	OSA_assert(m_render);
	cr_core::CRenderBaseImpl *base = (cr_core::CRenderBaseImpl *)m_render;
	return base->Draw(pos, videoId, roi);
}

