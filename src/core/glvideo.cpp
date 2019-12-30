/*
 * glvideo.cpp
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
#include "cuda_mem.hpp"
#include <linux/videodev2.h>
#include "glvideo.hpp"

__inline__ int GetChannels(int format)
{
	return (format==V4L2_PIX_FMT_GREY) ? 1 : 3;
}

CGLVideo::CGLVideo(int idx, const cv::Size& imgSize, int format, int fps, int nDrop, int memType)
:m_idx(idx),m_nDrop(nDrop), m_size(imgSize),m_format(format),m_fps(fps),m_memType(memType),textureId(0),nCnt(0ul),m_pbo(-1)
{
	memset(&m_bufQue, 0, sizeof(m_bufQue));
	m_matrix = cv::Matx44f::eye();
	//m_nDrop = m_disFPS/m_fps;
	int channels = GetChannels(format);
	int iRet = image_queue_create(&m_bufQue, 3, m_size.width*m_size.height*channels,m_memType);
	OSA_assert(iRet == OSA_SOK);
	for(int i=0; i<m_bufQue.numBuf; i++){
		m_bufQue.bufInfo[i].width = m_size.width;
		m_bufQue.bufInfo[i].height = m_size.height;
		m_bufQue.bufInfo[i].channels = channels;
		m_bufQue.bufInfo[i].format = m_format;
		cv::Matx33f* warpMat = new cv::Matx33f(1.0, 0.0, 0.0,
											   0.0, 1.0, 0.0,
											   0.0, 0.0, 1.0);
		m_bufQue.bufInfo[i].virtAddr = warpMat;
	}
	//OSA_printf("%s %d: textureId = %d", __func__, __LINE__, textureId);
	glGenTextures(1, &textureId);
	//OSA_printf("%s %d: textureId = %d", __func__, __LINE__, textureId);
	//assert(glIsTexture(textureId));
	glBindTexture(GL_TEXTURE_2D, textureId);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);//GL_NEAREST);//GL_NEAREST_MIPMAP_NEAREST);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);//GL_NEAREST);//GL_NEAREST_MIPMAP_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);//GL_CLAMP);//GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);//GL_CLAMP);//GL_CLAMP_TO_EDGE);
	if(channels == 1)
		glTexImage2D(GL_TEXTURE_2D, 0, channels, m_size.width, m_size.height, 0, GL_RED, GL_UNSIGNED_BYTE, NULL);
	else
		glTexImage2D(GL_TEXTURE_2D, 0, channels, m_size.width, m_size.height, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, NULL);
	glBindTexture(GL_TEXTURE_2D, 0);

	if(memtype_glpbo != m_memType)
	{
		glGenBuffers(1, &m_pbo);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, m_pbo);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, m_size.width*m_size.height*channels, NULL, GL_DYNAMIC_COPY);//GL_DYNAMIC_COPY);//GL_STATIC_DRAW);//GL_DYNAMIC_DRAW);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	}
	m_channels = channels;
	m_warpMat = cv::Matx33f::eye();
	memset(&m_curBufInfo, 0, sizeof(m_curBufInfo));
}

CGLVideo::~CGLVideo()
{
	for(int i=0; i<m_bufQue.numBuf; i++){
		cv::Matx33f* warpMat = (cv::Matx33f*)m_bufQue.bufInfo[i].virtAddr;
		if(warpMat != NULL)
			delete warpMat;
	}
	image_queue_delete(&m_bufQue);
	glDeleteTextures(1, &textureId);
	if(m_pbo>0)
		glDeleteBuffers(1, &m_pbo);
}

void CGLVideo::update()
{
	OSA_BufInfo* info = NULL;
	cv::Mat img;
	bool bDevMem = false;

	if(m_bufQue.bMap){
		for(int i=0; i<m_bufQue.numBuf; i++){
			cuMap(&m_bufQue.bufInfo[i]);
			image_queue_putEmpty(&m_bufQue, &m_bufQue.bufInfo[i]);
		}
		m_bufQue.bMap = false;
	}
	int nDrop = 0;

	nCnt ++;
	if(nCnt > 300)
	{
		int count = OSA_bufGetFullCount(&m_bufQue);
		if(count>1){
			while(count>1){
				//image_queue_switchEmpty(&m_bufQue[chId]);
				info = image_queue_getFull(&m_bufQue);
				OSA_assert(info != NULL);
				image_queue_putEmpty(&m_bufQue, info);
				count = OSA_bufGetFullCount(&m_bufQue);
				nDrop ++;
			}
			//OSA_printf("[%d-%ld]%s: ch%d queue drop = %d[%d], sync", OSA_getCurTimeInMsec(), nCnt, __func__, m_idx, nDrop, m_nDrop);
		}
		nCnt = 1;
	}else if(1){
		int count = OSA_bufGetFullCount(&m_bufQue);
		while(count>2){
			//image_queue_switchEmpty(&m_bufQue[chId]);
			info = image_queue_getFull(&m_bufQue);
			OSA_assert(info != NULL);
			image_queue_putEmpty(&m_bufQue, info);
			count = OSA_bufGetFullCount(&m_bufQue);
			nDrop ++;
		}
		//if(nDrop > 0)
		//	OSA_printf("[%d-%ld]%s: ch%d queue drop = %d [%d]", OSA_getCurTimeInMsec(), nCnt, __func__, m_idx, nDrop, m_nDrop);
	}

	if(m_nDrop>1 && (nCnt%m_nDrop)!=1)
		return;

	info = image_queue_getFull(&m_bufQue);
	if(info != NULL)
	{
		//OSA_printf("%s %d: %s chId = %d", __FILE__, __LINE__, __func__, m_idx);
		GLuint pbo = 0;
		bDevMem = (info->memtype == memtype_cudev || info->memtype == memtype_cumap);
		void *data = (info->memtype == memtype_glpbo || info->memtype == memtype_cudev || info->memtype == memtype_cumap) ? info->physAddr : info->virtAddr;
		if(info->channels == 1){
			img = cv::Mat(info->height, info->width, CV_8UC1, data);
		}else{
			img = cv::Mat(info->height, info->width, CV_8UC3, data);
		}
		m_tmBak = info->timestampCap;

		OSA_assert(img.cols > 0 && img.rows > 0);
		if(bDevMem)
		{
			unsigned int byteCount = img.cols*img.rows*img.channels();
			unsigned char *dev_pbo = NULL;
			size_t tmpSize;
			pbo = m_pbo;
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
			cudaResource_RegisterBuffer(m_idx, pbo, byteCount);
			cudaResource_mapBuffer(m_idx, (void **)&dev_pbo, &tmpSize);
			assert(tmpSize == byteCount);
			cudaMemcpy(dev_pbo, img.data, byteCount, cudaMemcpyDeviceToDevice);
			//cudaDeviceSynchronize();
			cudaResource_unmapBuffer(m_idx);
			cudaResource_UnregisterBuffer(m_idx);
			img.data = NULL;
		}else if(info->memtype == memtype_glpbo){
			if(0)
			{
				//std::cout << __FILE__ << " " << __LINE__ << ": ch" << m_idx << " img " << (void*)img.data << " " << img.cols << "x" << img.rows << std::endl;
				cv::Mat test = cv::Mat(img.rows,img.cols,img.type(), cv::Scalar(255, 0,0,0));
				cudaMemcpy(img.data, test.data, img.rows*img.cols*img.channels(), cudaMemcpyHostToDevice);
				//cv::imshow("test" , test);
				//cv::waitKey(1);
			}
			if(0)
			//if(m_idx == 1)
			{
				//std::cout << __FILE__ << " " << __LINE__ << ": ch" << m_idx << " img " << (void*)img.data << " " << img.cols << "x" << img.rows << std::endl;
				cv::Mat dis = cv::Mat(img.rows,img.cols,img.type());
				cudaMemcpy(dis.data, img.data, img.rows*img.cols*img.channels(), cudaMemcpyDeviceToHost);
				cv::imshow("test" , dis);
				cv::waitKey(1);
			}
			cuUnmap(info);
			pbo = info->pbo;
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
		}else{
			unsigned int byteCount = img.cols*img.rows*img.channels();
			unsigned char *dev_pbo = NULL;
			size_t tmpSize;
			pbo = m_pbo;
			OSA_assert(img.data != NULL);
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
			cudaResource_RegisterBuffer(m_idx, pbo, byteCount);
			cudaResource_mapBuffer(m_idx, (void **)&dev_pbo, &tmpSize);
			assert(tmpSize == byteCount);
			cudaMemcpy(dev_pbo, img.data, byteCount, cudaMemcpyHostToDevice);
			//cudaDeviceSynchronize();
			cudaResource_unmapBuffer(m_idx);
			cudaResource_UnregisterBuffer(m_idx);
		}
		glBindTexture(GL_TEXTURE_2D, textureId);
		if(img.channels() == 1)
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, img.cols, img.rows, GL_RED, GL_UNSIGNED_BYTE, NULL);
		else //if(img.channels() == 3)
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, img.cols, img.rows, GL_BGR_EXT, GL_UNSIGNED_BYTE, NULL);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		if(info->memtype == memtype_glpbo){
			cuMap(info);
		}
		m_warpMat = *(cv::Matx33f*)info->virtAddr;
		m_curBufInfo = *info;
		image_queue_putEmpty(&m_bufQue, info);
	}
}
