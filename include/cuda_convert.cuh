/*
 * Copyright (c) 2015, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef __YUV2RGB_CUH__
#define __YUV2RGB_CUH__

//#include <opencv2/core/core.hpp>
//#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>

//#define WHITECOLOR 		0x008080BE
#define WHITECOLOR 		0x008080FF
#define YELLOWCOLOR 	0x009110D2
#define CRAYCOLOR		0x0010A6AA
#define GREENCOLOR		0x00223691
#define MAGENTACOLOR	0x00DECA6A
#define REDCOLOR		0x00F05A51
#define BLUECOLOR		0x006EF029
#define BLACKCOLOR		0x00808010
#define BLANKCOLOR		0x00000000

void gpuConvertYUYVtoRGB(unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height);

void gpuConvertYUYVtoBGR(unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height);

void gpuConvertYUYVtoGray(unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height);

void gpuConvertYUYVtoI420(unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height);

void gpuConvertYUYVtoI420AndOsd(unsigned char  *osd,unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height, int colorYUV);

void gpuConvertYUYVtoI420AndZoomxAndOsd(unsigned char  *osd,unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height, int zoomx, int colorYUV);

void gpuConvertGRAYtoI420AndOsd(unsigned char  *osd,unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height, int colorYUV);

void gpuConvertGRAYtoI420AndZoomxAndOsd(unsigned char  *osd,unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height, int zoomx, int colorYUV);

#define CUT_FLAG_hostAlloc	(1)
#define CUT_FLAG_devAlloc	(2)

cudaError_t cuConvertInit(int nChannels, OSA_MutexHndl *mutexLock = NULL);
cudaError_t cuConvertUinit();
cudaError_t cuConvert_async(int chId, cv::Mat src, cv::Mat osd, cv::Mat &dst, int zoomx, int colorYUV);
cudaError_t cuConvertEnh_async(int chId, cv::Mat src, cv::Mat osd, cv::Mat &dst, int zoomx, int colorYUV);
cudaError_t cuConvert_async(int chId, cv::Mat src, cv::Mat &dst, int zoomx);
cudaError_t cuConvertEnh_async(int chId, cv::Mat src, cv::Mat &dst, int zoomx);
cudaError_t cuConvert_yuv2bgr_yuyv_async(int chId, cv::Mat src, cv::Mat &dst, int flag);
cudaError_t cuConvertEnh_yuv2bgr_yuyv_async(int chId, cv::Mat src, cv::Mat &dst, int flag);
cudaError_t cuConvertEnh_gray(int chId, cv::Mat src, cv::Mat &dst, int flag);
cudaError_t cuConvertConn_yuv2bgr_i420(int chId, cv::Mat &dst, int flag);

#endif
