/*
 * glvideo.hpp
 *
 *  Created on: Feb 13, 2019
 *      Author: ubuntu
 */

#ifndef GLVIDEO_HPP_
#define GLVIDEO_HPP_

class CGLVideo
{
	unsigned long nCnt;
public:
	int m_idx;
	cv::Size m_size;
	int m_format;
	int m_channels;
	int m_fps;
	int m_nDrop;
	cr_osa::OSA_BufHndl m_bufQue;
	int m_memType;
	GLuint textureId;
	cv::Matx<GLfloat, 4, 4> m_matrix;
	uint64  m_tmBak;
	cv::Matx33f m_warpMat;
	OSA_BufInfo m_curBufInfo;
	GLuint m_pbo;

	CGLVideo(int idx, const cv::Size& imgSize, int format, int fps = 30, int nDrop = 0, int memType = memtype_glpbo);

	virtual ~CGLVideo();

	virtual void update();
};


#endif /* GLVIDEO_HPP_ */
