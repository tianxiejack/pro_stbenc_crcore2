/*
 * glvideoRender.cpp
 *
 *  Created on: Feb 13, 2019
 *      Author: ubuntu
 */

#include <opencv2/opencv.hpp>
#include <osa_buf.h>
#include <glew.h>
#include <glut.h>
#include <freeglut_ext.h>
#include "osa_image_queue.h"
#include <cuda.h>
#include <cuda_gl_interop.h>
#include <cuda_runtime_api.h>
#include <linux/videodev2.h>
#include "glvideo.hpp"
#include "glvideoRender.hpp"

#include <GLBatchMini.h>
#include <GLFrustum.h>
#include <GLFrame.h>
#include <GLMatrixStack.h>
#include <GLGeometryTransform.h>
#include <GLShaderManagerMini.h>

GLint	CGLVideoRender::m_glProgram[8] = {0,};
bool CGLVideoRender::m_bLoadProgram = false;
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
int CGLVideoRender::gl_loadProgram()
{
	int iRet = OSA_SOK;
	m_glProgram[0] = gltLoadShaderPairWithAttributes(szDefaultShaderVP, szDefaultShaderFP, 2, ATTRIB_VERTEX, "vVertex", ATTRIB_TEXTURE, "vTexCoords");
	m_glProgram[1] = gltLoadShaderPairWithAttributes(szFlatShaderVP, szDefaultShaderFP, 2, ATTRIB_VERTEX, "vVertex", ATTRIB_TEXTURE, "vTexCoords");
	m_glProgram[2] = gltLoadShaderPairWithAttributes(szFlatShaderVP, szPolarityShaderFP, 2, ATTRIB_VERTEX, "vVertex", ATTRIB_TEXTURE, "vTexCoords");
	m_glProgram[3] = gltLoadShaderPairWithAttributes(szFlatShaderVP, szPolarityShaderRGBFP, 2, ATTRIB_VERTEX, "vVertex", ATTRIB_TEXTURE, "vTexCoords");
	m_glProgram[4] = gltLoadShaderPairWithAttributes(szFlatShaderVP, szBlendMaskTextureShaderFP, 2, ATTRIB_VERTEX, "vVertex", ATTRIB_TEXTURE, "vTexCoords");
	m_glProgram[5] = gltLoadShaderPairWithAttributes(szDefaultShaderVP, szAlphaTextureShaderFP, 2, ATTRIB_VERTEX, "vVertex", ATTRIB_TEXTURE, "vTexCoords");
	m_bLoadProgram = true;
	return iRet;
}

int CGLVideoRender::gl_unloadProgram()
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
bool CGLVideoRender::gltLoadShaderSrc(const char *szShaderSrc, GLuint shader)
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
bool CGLVideoRender::gltLoadShaderFile(const char *szFile, GLuint shader)
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
GLuint CGLVideoRender::gltLoadShaderPairWithAttributes(const char *szVertexProg, const char *szFragmentProg, ...)
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

static bool gltCheckErrors(GLuint progName)
{
    bool bFoundError = false;
	GLenum error = glGetError();

	if (error != GL_NO_ERROR)
	{
	    fprintf(stderr, "A GL Error has occured\n");
        bFoundError = true;
	}
#ifndef OPENGL_ES
	GLenum fboStatus = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);

	if(fboStatus != GL_FRAMEBUFFER_COMPLETE)
	{
        bFoundError = true;
		fprintf(stderr,"The framebuffer is not complete - ");
		switch (fboStatus)
		{
		case GL_FRAMEBUFFER_UNDEFINED:
			// Oops, no window exists?
            fprintf(stderr, "GL_FRAMEBUFFER_UNDEFINED\n");
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
			// Check the status of each attachment
            fprintf(stderr, "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT\n");
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
			// Attach at least one buffer to the FBO
            fprintf(stderr, "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT\n");
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
			// Check that all attachments enabled via
			// glDrawBuffers exist in FBO
            fprintf(stderr, "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER\n");
            break;
		case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
			// Check that the buffer specified via
			// glReadBuffer exists in FBO
            fprintf(stderr, "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER\n");
			break;
		case GL_FRAMEBUFFER_UNSUPPORTED:
			// Reconsider formats used for attached buffers
            fprintf(stderr, "GL_FRAMEBUFFER_UNSUPPORTED\n");
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
			// Make sure the number of samples for each
			// attachment is the same
            fprintf(stderr, "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE\n");
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
			// Make sure the number of layers for each
			// attachment is the same
            fprintf(stderr, "GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS\n");
			break;
		}
	}

#endif

	if (progName != 0)
	{
		glValidateProgram(progName);
		int iIsProgValid = 0;
		glGetProgramiv(progName, GL_VALIDATE_STATUS, &iIsProgValid);
		if(iIsProgValid == 0)
		{
            bFoundError = true;
			fprintf(stderr, "The current program(%d) is not valid\n", progName);
		}
	}
    return bFoundError;
}

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

CGLVideoRender::CGLVideoRender(CGLVideo *video, const GLMatx44f& matrix, const cv::Rect& viewPort):m_video(video),m_matrix(matrix),m_viewPort(viewPort){
	memcpy(m_vVerts, defaultVertices, sizeof(defaultVertices));
	memcpy(m_vTexCoords, defaultTextureCoords, sizeof(defaultTextureCoords));
	OSA_mutexCreate(&m_mutex);
}
CGLVideoRender::~CGLVideoRender(){
	OSA_mutexDelete(&m_mutex);
}
void CGLVideoRender::render()
{
	if(!CGLVideoRender::m_bLoadProgram)
		int iRet = CGLVideoRender::gl_loadProgram();
	OSA_assert(m_bLoadProgram);
	OSA_mutexLock(&m_mutex);
	if(m_video == NULL){
		OSA_mutexUnlock(&m_mutex);
		return;
	}

	GLint glProg = m_glProgram[1];
	//gltCheckErrors(glProg);
	glUseProgram(glProg);
	GLint Uniform_tex_in = glGetUniformLocation(glProg, "tex_in");
	GLint Uniform_mvp = glGetUniformLocation(glProg, "mvpMatrix");
	GLMatx44f mTrans = m_video->m_matrix*m_matrix;
	glUniformMatrix4fv(Uniform_mvp, 1, GL_FALSE, mTrans.val);
	glUniform1i(Uniform_tex_in, 0);
	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_video->textureId);
	glVertexAttribPointer(ATTRIB_VERTEX, 2, GL_FLOAT, GL_FALSE, 0, m_vVerts);
	glVertexAttribPointer(ATTRIB_TEXTURE, 2, GL_FLOAT, GL_FALSE, 0, m_vTexCoords);
	glEnableVertexAttribArray(ATTRIB_VERTEX);
	glEnableVertexAttribArray(ATTRIB_TEXTURE);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glViewport(m_viewPort.x, m_viewPort.y, m_viewPort.width, m_viewPort.height);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
	glUseProgram(0);
	OSA_mutexUnlock(&m_mutex);
}
void CGLVideoRender::set(const GLMatx44f& matrix)
{
	OSA_mutexLock(&m_mutex);
	m_matrix = matrix;
	OSA_mutexUnlock(&m_mutex);
}
void CGLVideoRender::set(const CGLVideo* video)
{
	OSA_mutexLock(&m_mutex);
	m_video = video;
	OSA_mutexUnlock(&m_mutex);
}
void CGLVideoRender::set(const cv::Rect& viewPort)
{
	OSA_mutexLock(&m_mutex);
	m_viewPort = viewPort;
	OSA_mutexUnlock(&m_mutex);
}


CGLVideoBlendRender::CGLVideoBlendRender(CGLVideo *video, const GLMatx44f& matrix, const cv::Rect& viewPort, CGLVideo *blend, const GLMatx44f& blendMatrix, const GLV_BlendPrm& prm)
		:CGLVideoRender(video, matrix, viewPort),m_blend(blend),m_matrixBlend(blendMatrix)
{
	memcpy(&m_blendPrm, &prm, sizeof(m_blendPrm));
}

CGLVideoBlendRender::~CGLVideoBlendRender()
{
}

void CGLVideoBlendRender::render()
{
	CGLVideoRender::render();

	OSA_mutexLock(&m_mutex);
	if(m_video == NULL || m_blend == NULL){
		OSA_mutexUnlock(&m_mutex);
		return;
	}

	//OSA_printf("%s %d: %d-%d", __FILE__, __LINE__, m_video->m_idx, m_blend->m_idx);
	GLint glProg;
	GLMatx44f mTrans = m_matrixBlend*m_video->m_matrix*m_matrix;

	if(m_blend->m_channels == 1){
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
	glUniform1f(uniform_fAlpha, m_blendPrm.fAlpha);
	glUniform1f(uniform_thr0Min, m_blendPrm.thr0Min);
	glUniform1f(uniform_thr0Max, m_blendPrm.thr0Max);
	glUniform1f(uniform_thr1Min, m_blendPrm.thr1Min);
	glUniform1f(uniform_thr1Max, m_blendPrm.thr1Max);
	glUniform1i(Uniform_tex_in, 0);
	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_blend->textureId);
	glVertexAttribPointer(ATTRIB_VERTEX, 2, GL_FLOAT, GL_FALSE, 0, m_vVerts);
	glVertexAttribPointer(ATTRIB_TEXTURE, 2, GL_FLOAT, GL_FALSE, 0, m_vTexCoords);
	glEnableVertexAttribArray(ATTRIB_VERTEX);
	glEnableVertexAttribArray(ATTRIB_TEXTURE);
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_MULTISAMPLE);
	glDisable(GL_BLEND);
	glDisable(GL_TEXTURE_2D);
	glUseProgram(0);
	OSA_mutexUnlock(&m_mutex);
}
void CGLVideoBlendRender::matrix(const GLMatx44f& matrix)
{
	OSA_mutexLock(&m_mutex);
	m_matrixBlend = matrix;
	OSA_mutexUnlock(&m_mutex);
}
void CGLVideoBlendRender::blend(const CGLVideo* video)
{
	OSA_mutexLock(&m_mutex);
	m_blend = video;
	OSA_mutexUnlock(&m_mutex);
}
void CGLVideoBlendRender::params(const GLV_BlendPrm& prm)
{
	OSA_mutexLock(&m_mutex);
	memcpy(&m_blendPrm, &prm, sizeof(m_blendPrm));
	OSA_mutexUnlock(&m_mutex);
}


CGLVideoMaskBlendRender::CGLVideoMaskBlendRender(CGLVideo *video, const GLMatx44f& matrix, const cv::Rect& viewPort, CGLVideo *blend, const GLMatx44f& blendMatrix, CGLVideo *mask)
		:CGLVideoRender(video, matrix, viewPort),m_blend(blend),m_matrixBlend(blendMatrix),m_mask(mask)
{}

CGLVideoMaskBlendRender::~CGLVideoMaskBlendRender(){}
void CGLVideoMaskBlendRender::render()
{
	CGLVideoRender::render();
	OSA_mutexLock(&m_mutex);
	if(m_video == NULL || m_blend == NULL || m_mask == NULL){
		OSA_mutexUnlock(&m_mutex);
		return;
	}
	GLint glProg;
	GLMatx44f mTrans = m_matrixBlend*m_video->m_matrix*m_matrix;

	glProg = m_glProgram[4];
	glUseProgram(glProg);
	GLint Uniform_tex_in = glGetUniformLocation(glProg, "tex_in");
	GLint Uniform_tex_mask = glGetUniformLocation(glProg, "tex_mask");
	GLint Uniform_mvp = glGetUniformLocation(glProg, "mvpMatrix");
	glUniformMatrix4fv(Uniform_mvp, 1, GL_FALSE, mTrans.val);
	glUniform1i(Uniform_tex_in, 0);
	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_blend->textureId);
	glUniform1i(Uniform_tex_mask, 1);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, m_mask->textureId);
	glVertexAttribPointer(ATTRIB_VERTEX, 2, GL_FLOAT, GL_FALSE, 0, m_vVerts);
	glVertexAttribPointer(ATTRIB_TEXTURE, 2, GL_FLOAT, GL_FALSE, 0, m_vTexCoords);
	glEnableVertexAttribArray(ATTRIB_VERTEX);
	glEnableVertexAttribArray(ATTRIB_TEXTURE);
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_MULTISAMPLE);
	glDisable(GL_BLEND);
	glDisable(GL_TEXTURE_2D);
	glUseProgram(0);
	OSA_mutexUnlock(&m_mutex);
}
void CGLVideoMaskBlendRender::matrix(const GLMatx44f& matrix)
{
	OSA_mutexLock(&m_mutex);
	m_matrixBlend = matrix;
	OSA_mutexUnlock(&m_mutex);
}
void CGLVideoMaskBlendRender::blend(const CGLVideo* video)
{
	OSA_mutexLock(&m_mutex);
	m_blend = video;
	OSA_mutexUnlock(&m_mutex);
}
void CGLVideoMaskBlendRender::mask(const CGLVideo* mask)
{
	OSA_mutexLock(&m_mutex);
	m_mask = mask;
	OSA_mutexUnlock(&m_mutex);
}


