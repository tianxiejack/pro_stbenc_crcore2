/*
 * glvideoRender.hpp
 *
 *  Created on: Feb 13, 2019
 *      Author: ubuntu
 */

#ifndef GLVIDEORENDER_HPP_
#define GLVIDEORENDER_HPP_

#define ATTRIB_VERTEX 1
#define ATTRIB_TEXTURE 2

typedef cv::Matx<GLfloat, 4, 4> GLMatx44f;
typedef struct _glv_blend_param{
	GLfloat fAlpha;
	GLfloat thr0Min;
	GLfloat thr0Max;
	GLfloat thr1Min;
	GLfloat thr1Max;
}GLV_BlendPrm;

class CGLVideo;

class CGLVideoRender
{
public:
	GLMatx44f m_matrix;
	cv::Rect m_viewPort;
	GLfloat m_vVerts[8];
	GLfloat m_vTexCoords[8];
	OSA_MutexHndl m_mutex;
public:
	CGLVideoRender(CGLVideo *video, const GLMatx44f& matrix, const cv::Rect& viewPort);
	virtual ~CGLVideoRender();
	virtual void render();
	void set(const GLMatx44f& matrix);
	void set(const CGLVideo* video);
	void set(const cv::Rect& viewPort);
	const CGLVideo* m_video;

public:
	static GLint	m_glProgram[8];
	static bool m_bLoadProgram;
	static int gl_loadProgram();
	static int gl_unloadProgram();
	static bool gltLoadShaderSrc(const char *szShaderSrc, GLuint shader);
	static bool gltLoadShaderFile(const char *szFile, GLuint shader);
	static GLuint gltLoadShaderPairWithAttributes(const char *szVertexProg, const char *szFragmentProg, ...);
};

class CGLVideoBlendRender : public CGLVideoRender
{
public:
	const CGLVideo* m_blend;
	GLMatx44f m_matrixBlend;
	GLV_BlendPrm m_blendPrm;
public:
	CGLVideoBlendRender(CGLVideo *video, const GLMatx44f& matrix, const cv::Rect& viewPort, CGLVideo *blend, const GLMatx44f& blendMatrix, const GLV_BlendPrm& prm);
	virtual ~CGLVideoBlendRender();
	virtual void render();
	void matrix(const GLMatx44f& matrix);
	void blend(const CGLVideo* video);
	void params(const GLV_BlendPrm& prm);
};

class CGLVideoMaskBlendRender : public CGLVideoRender
{
public:
	const CGLVideo* m_blend;
	const CGLVideo* m_mask;
	GLMatx44f m_matrixBlend;
public:
	CGLVideoMaskBlendRender(CGLVideo *video, const GLMatx44f& matrix, const cv::Rect& viewPort, CGLVideo *blend, const GLMatx44f& blendMatrix, CGLVideo *mask);
	virtual ~CGLVideoMaskBlendRender();
	virtual void render();
	void matrix(const GLMatx44f& matrix);
	void blend(const CGLVideo* video);
	void mask(const CGLVideo* mask);
};


#endif /* GLVIDEORENDER_HPP_ */
