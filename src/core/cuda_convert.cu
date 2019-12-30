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
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "osa_mutex.h"
#include "cuda_convert.cuh"
#include "enh.hpp"
//#include <ImagesCPU.h>
//#include <ImagesNPP.h>
//#include <ImageIO.h>
//#include <Exceptions.h>
#include <npp.h>
#include <helper_cuda.h>

using namespace std;
using namespace cv;

#define GETYVAL(yuv)	(yuv & 0xff)
#define GETUVAL(yuv)	(((yuv)>>8) & 0xff)
#define GETVVAL(yuv)	(((yuv)>>16) & 0xff)

__device__ inline float clamp(float val, float mn, float mx)
{
	return (val >= mn)? ((val <= mx)? val : mx) : mn;
}

__global__ void gpuConvertYUYVtoRGB_kernel(unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	for (int i = 0; i < height; ++i) {
		int y0 = src[i*width*2+idx*4+0];
		int cb = src[i*width*2+idx*4+1];
		int y1 = src[i*width*2+idx*4+2];
		int cr = src[i*width*2+idx*4+3];

		dst[i*width*3+idx*6+0] = clamp(1.164f * (y0 - 16) + 1.596f * (cr - 128)                      , 0.0f, 255.0f);
		dst[i*width*3+idx*6+1] = clamp(1.164f * (y0 - 16) - 0.813f * (cr - 128) - 0.391f * (cb - 128), 0.0f, 255.0f);
		dst[i*width*3+idx*6+2] = clamp(1.164f * (y0 - 16)                       + 2.018f * (cb - 128), 0.0f, 255.0f);

		dst[i*width*3+idx*6+3] = clamp(1.164f * (y1 - 16) + 1.596f * (cr - 128)                      , 0.0f, 255.0f);
		dst[i*width*3+idx*6+4] = clamp(1.164f * (y1 - 16) - 0.813f * (cr - 128) - 0.391f * (cb - 128), 0.0f, 255.0f);
		dst[i*width*3+idx*6+5] = clamp(1.164f * (y1 - 16)                       + 2.018f * (cb - 128), 0.0f, 255.0f);
	}
}

void gpuConvertYUYVtoRGB(unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height)
{
	unsigned char *d_src = NULL;
	unsigned char *d_dst = NULL;
	size_t planeSize = width * height * sizeof(unsigned char);

	unsigned int flags;
	bool srcIsMapped = (cudaHostGetFlags(&flags, src) == cudaSuccess) && (flags & cudaHostAllocMapped);
	bool dstIsMapped = (cudaHostGetFlags(&flags, dst) == cudaSuccess) && (flags & cudaHostAllocMapped);

	if (srcIsMapped) {
		d_src = src;
		cudaStreamAttachMemAsync(NULL, src, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_src, planeSize * 2);
		cudaMemcpy(d_src, src, planeSize * 2, cudaMemcpyHostToDevice);
	}
	if (dstIsMapped) {
		d_dst = dst;
		cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_dst, planeSize * 3);
	}

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	gpuConvertYUYVtoRGB_kernel<<<numBlocks, blockSize>>>(d_src, d_dst, width, height);
	cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachHost);
	cudaStreamSynchronize(NULL);

	if (!srcIsMapped) {
		cudaMemcpy(dst, d_dst, planeSize * 3, cudaMemcpyDeviceToHost);
		cudaFree(d_src);
	}
	if (!dstIsMapped) {
		cudaFree(d_dst);
	}
}

#if 1
#define DESCALE(x, n)    (((x) + (1 << ((n)-1)))>>(n))
#define COEFFS_0 		(22987)
#define COEFFS_1 		(-11698)
#define COEFFS_2 		(-5636)
#define COEFFS_3 		(29049)
#define clip(minv, maxv, value)  ((value)<minv) ? minv : (((value)>maxv) ? maxv : (value))

__global__ void gpuConvertYUYVtoBGR_kernel(unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	for (int i = 0; i < height; ++i) {
		int y0 = src[i*width*2+idx*4+0];
		int cb = src[i*width*2+idx*4+1];
		int y1 = src[i*width*2+idx*4+2];
		int cr = src[i*width*2+idx*4+3];
		int b = DESCALE((cb - 128)*COEFFS_3, 14);
		int g = DESCALE((cb - 128)*COEFFS_2 + (cr - 128)*COEFFS_1, 14);
		int r = DESCALE((cr - 128)*COEFFS_0, 14);

		dst[i*width*3+idx*6+0] = clip(0, 255, y0 + b);//B
		dst[i*width*3+idx*6+1] = clip(0, 255, y0 + g);//G
		dst[i*width*3+idx*6+2] = clip(0, 255, y0 + r);//R

		dst[i*width*3+idx*6+3] = clip(0, 255, y1 + b);//B
		dst[i*width*3+idx*6+4] = clip(0, 255, y1 + g);//G
		dst[i*width*3+idx*6+5] = clip(0, 255, y1 + r);//R
	}
}

__global__ void gpuConvertUYVYtoBGR_kernel(unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	for (int i = 0; i < height; ++i) {

		int cb = src[i*width*2+idx*4+0];
		int y0 = src[i*width*2+idx*4+1];
		int cr = src[i*width*2+idx*4+2];
		int y1 = src[i*width*2+idx*4+3];

		int b = DESCALE((cb - 128)*COEFFS_3, 14);
		int g = DESCALE((cb - 128)*COEFFS_2 + (cr - 128)*COEFFS_1, 14);
		int r = DESCALE((cr - 128)*COEFFS_0, 14);

		dst[i*width*3+idx*6+0] = clip(0, 255, y0 + b);//B
		dst[i*width*3+idx*6+1] = clip(0, 255, y0 + g);//G
		dst[i*width*3+idx*6+2] = clip(0, 255, y0 + r);//R

		dst[i*width*3+idx*6+3] = clip(0, 255, y1 + b);//B
		dst[i*width*3+idx*6+4] = clip(0, 255, y1 + g);//G
		dst[i*width*3+idx*6+5] = clip(0, 255, y1 + r);//R
	}
}

#else
__global__ void gpuConvertYUYVtoBGR_kernel(unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	for (int i = 0; i < height; ++i) {
		int y0 = src[i*width*2+idx*4+0];
		int cb = src[i*width*2+idx*4+1];
		int y1 = src[i*width*2+idx*4+2];
		int cr = src[i*width*2+idx*4+3];

		dst[i*width*3+idx*6+0] = clamp(1.164f * (y0 - 16)                       + 2.018f * (cb - 128), 0.0f, 255.0f);
		dst[i*width*3+idx*6+1] = clamp(1.164f * (y0 - 16) - 0.813f * (cr - 128) - 0.391f * (cb - 128), 0.0f, 255.0f);
		dst[i*width*3+idx*6+2] = clamp(1.164f * (y0 - 16) + 1.596f * (cr - 128)                      , 0.0f, 255.0f);

		dst[i*width*3+idx*6+3] = clamp(1.164f * (y1 - 16)                       + 2.018f * (cb - 128), 0.0f, 255.0f);
		dst[i*width*3+idx*6+4] = clamp(1.164f * (y1 - 16) - 0.813f * (cr - 128) - 0.391f * (cb - 128), 0.0f, 255.0f);
		dst[i*width*3+idx*6+5] = clamp(1.164f * (y1 - 16) + 1.596f * (cr - 128)                      , 0.0f, 255.0f);
	}
}
#endif

void gpuConvertYUYVtoBGR(unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height)
{
	unsigned char *d_src = NULL;
	unsigned char *d_dst = NULL;
	size_t planeSize = width * height * sizeof(unsigned char);

	unsigned int flags;
	bool srcIsMapped = (cudaHostGetFlags(&flags, src) == cudaSuccess) && (flags & cudaHostAllocMapped);
	bool dstIsMapped = (cudaHostGetFlags(&flags, dst) == cudaSuccess) && (flags & cudaHostAllocMapped);

	if (srcIsMapped) {
		d_src = src;
		cudaStreamAttachMemAsync(NULL, src, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_src, planeSize * 2);
		cudaMemcpy(d_src, src, planeSize * 2, cudaMemcpyHostToDevice);
	}
	if (dstIsMapped) {
		d_dst = dst;
		cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_dst, planeSize * 3);
	}

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	gpuConvertYUYVtoBGR_kernel<<<numBlocks, blockSize>>>(d_src, d_dst, width, height);
	cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachHost);
	cudaStreamSynchronize(NULL);

	if (!srcIsMapped) {
		cudaFree(d_src);
	}

	if (!dstIsMapped) {
		cudaMemcpy(dst, d_dst, planeSize * 3, cudaMemcpyDeviceToHost);
		cudaFree(d_dst);
	}
}

void gpuConvertYUYVtoBGR_dev(unsigned char *src, unsigned char *d_dst,
		unsigned int width, unsigned int height)
{
	unsigned char *d_src = NULL;
	size_t planeSize = width * height * sizeof(unsigned char);

	unsigned int flags;
	bool srcIsMapped = (cudaHostGetFlags(&flags, src) == cudaSuccess) && (flags & cudaHostAllocMapped);

	if (srcIsMapped) {
		d_src = src;
		cudaStreamAttachMemAsync(NULL, src, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_src, planeSize * 2);
		cudaMemcpy(d_src, src, planeSize * 2, cudaMemcpyHostToDevice);
	}

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	gpuConvertYUYVtoBGR_kernel<<<numBlocks, blockSize>>>(d_src, d_dst, width, height);
	cudaStreamSynchronize(NULL);

	if (!srcIsMapped) {
		cudaFree(d_src);
	}
}

//////////////////////////////////////////////////////////////////////
//
//
__global__ void gpuConvertYUYVtoGray_kernel(unsigned char *src,
		unsigned char *dY,
		unsigned int width, unsigned int height)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int halfH = height>>1;
	for (int i = 0; i < halfH; ++i) {
		int y00 = src[i*2*width*2+idx*4+0];
		int y01 = src[i*2*width*2+idx*4+2];
		int y10 = src[(i*2+1)*width*2+idx*4+0];
		int y11 = src[(i*2+1)*width*2+idx*4+2];

		dY[i*2*width+idx*2+0] = y00;
		dY[i*2*width+idx*2+1] = y01;
		dY[(i*2+1)*width+idx*2+0] = y10;
		dY[(i*2+1)*width+idx*2+1] = y11;
	}
}

void gpuConvertYUYVtoGray(unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height)
{
	unsigned char *d_src = NULL;
	unsigned char *d_dst = NULL;
	size_t planeSize = width * height * sizeof(unsigned char);

	unsigned int flags;
	bool srcIsMapped = (cudaHostGetFlags(&flags, src) == cudaSuccess) && (flags & cudaHostAllocMapped);
	bool dstIsMapped = (cudaHostGetFlags(&flags, dst) == cudaSuccess) && (flags & cudaHostAllocMapped);

	if (srcIsMapped) {
		d_src = src;
		cudaStreamAttachMemAsync(NULL, src, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_src, planeSize * 2);
		cudaMemcpy(d_src, src, planeSize * 2, cudaMemcpyHostToDevice);
	}
	if (dstIsMapped) {
		d_dst = dst;
		cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_dst, planeSize);
	}

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	gpuConvertYUYVtoGray_kernel<<<numBlocks, blockSize>>>(d_src,d_dst, width, height);
	cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachHost);
	cudaStreamSynchronize(NULL);

	if (!srcIsMapped) {
		cudaFree(d_src);
	}
	if (!dstIsMapped) {
		cudaMemcpy(dst, d_dst, planeSize, cudaMemcpyDeviceToHost);
		cudaFree(d_dst);
	}
}


//////////////////////////////////////////////////////////////////////
//
//
__global__ void gpuConvertYUYVtoI420_kernel(unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int halfH = height>>1;
	for (int i = 0; i < halfH; ++i) {
		int y00 = src[i*2*width*2+idx*4+0];
		int cb0 = src[i*2*width*2+idx*4+1];
		int y01 = src[i*2*width*2+idx*4+2];
		int cr0 = src[i*2*width*2+idx*4+3];
		int y10 = src[(i*2+1)*width*2+idx*4+0];
		int y11 = src[(i*2+1)*width*2+idx*4+2];

		dY[i*2*width+idx*2+0] = y00;
		dY[i*2*width+idx*2+1] = y01;
		dY[(i*2+1)*width+idx*2+0] = y10;
		dY[(i*2+1)*width+idx*2+1] = y11;
		dU[i*(width>>1)+idx] = cb0;
		dV[i*(width>>1)+idx] = cr0;
	}
}

void gpuConvertYUYVtoI420(unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height)
{
	unsigned char *d_src = NULL;
	unsigned char *d_dst = NULL;
	size_t planeSize = width * height * sizeof(unsigned char);

	unsigned int flags;
	bool srcIsMapped = (cudaHostGetFlags(&flags, src) == cudaSuccess) && (flags & cudaHostAllocMapped);
	bool dstIsMapped = (cudaHostGetFlags(&flags, dst) == cudaSuccess) && (flags & cudaHostAllocMapped);

	if (srcIsMapped) {
		d_src = src;
		cudaStreamAttachMemAsync(NULL, src, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_src, planeSize * 2);
		cudaMemcpy(d_src, src, planeSize * 2, cudaMemcpyHostToDevice);
	}
	if (dstIsMapped) {
		d_dst = dst;
		cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_dst, planeSize * 3/2);
	}

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	gpuConvertYUYVtoI420_kernel<<<numBlocks, blockSize>>>(d_src,
			d_dst, d_dst+planeSize, d_dst+planeSize+(planeSize>>2), width, height);
	cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachHost);
	cudaStreamSynchronize(NULL);

	if (!srcIsMapped) {
		cudaFree(d_src);
	}
	if (!dstIsMapped) {
		cudaMemcpy(dst, d_dst, planeSize * 3/2, cudaMemcpyDeviceToHost);
		cudaFree(d_dst);
	}
}


//////////////////////////////////////////////////////////////////////
//
//
__global__ void gpuConvertYUYVtoI420AndOsd_kernel(unsigned char *osd,unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height,
		int yColor, int uColor, int vColor)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int halfH = height>>1;
	for (int i = 0; i < halfH; ++i) {
		int y00 = src[i*2*width*2+idx*4+0];
		int cb0 = src[i*2*width*2+idx*4+1];
		int y01 = src[i*2*width*2+idx*4+2];
		int cr0 = src[i*2*width*2+idx*4+3];
		int y10 = src[(i*2+1)*width*2+idx*4+0];
		int y11 = src[(i*2+1)*width*2+idx*4+2];

		int a00 = (osd[i*2*width+idx*2+0]+1)>>8;
		int a01 = (osd[i*2*width+idx*2+1]+1)>>8;
		int a10 = (osd[(i*2+1)*width+idx*2+0]+1)>>8;
		int a11 = (osd[(i*2+1)*width+idx*2+1]+1)>>8;
		int auv = (a00 | a01 | a10 | a11);

		dY[i*2*width+idx*2+0] = (1-a00)*y00 + a00*yColor;
		dY[i*2*width+idx*2+1] = (1-a01)*y01 + a01*yColor;
		dY[(i*2+1)*width+idx*2+0] = (1-a10)*y10 + a10*yColor;
		dY[(i*2+1)*width+idx*2+1] = (1-a11)*y11 + a11*yColor;
		dU[i*(width>>1)+idx] = (1-auv)*cb0 + auv*uColor;
		dV[i*(width>>1)+idx] = (1-auv)*cr0 + auv*vColor;
	}
}

void gpuConvertYUYVtoI420AndOsd(unsigned char  *osd,unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height, int colorYUV)
{
	unsigned char *d_src = NULL;
	unsigned char *d_dst = NULL;
	unsigned char *d_osd = NULL;
	size_t planeSize = width * height * sizeof(unsigned char);

	unsigned int flags;
	bool srcIsMapped = (cudaHostGetFlags(&flags, src) == cudaSuccess) && (flags & cudaHostAllocMapped);
	bool dstIsMapped = (cudaHostGetFlags(&flags, dst) == cudaSuccess) && (flags & cudaHostAllocMapped);
	bool osdIsMapped = (cudaHostGetFlags(&flags, dst) == cudaSuccess) && (flags & cudaHostAllocMapped);

	if (srcIsMapped) {
		d_src = src;
		cudaStreamAttachMemAsync(NULL, src, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_src, planeSize * 2);
		cudaMemcpy(d_src, src, planeSize * 2, cudaMemcpyHostToDevice);
	}
	if (osdIsMapped) {
		d_osd = osd;
		cudaStreamAttachMemAsync(NULL, osd, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_osd, planeSize * 1);
		cudaMemcpy(d_osd, osd, planeSize * 1, cudaMemcpyHostToDevice);
	}
	if (dstIsMapped) {
		d_dst = dst;
		cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_dst, planeSize * 3/2);
	}

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	int yColor = GETYVAL(colorYUV);
	int uColor = GETUVAL(colorYUV);
	int vColor = GETVVAL(colorYUV);
	gpuConvertYUYVtoI420AndOsd_kernel<<<numBlocks, blockSize>>>(d_osd,d_src,
			d_dst, d_dst+planeSize, d_dst+planeSize+(planeSize>>2), width, height,
			yColor, uColor, vColor);
	cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachHost);
	cudaStreamSynchronize(NULL);

	if (!srcIsMapped) {
		cudaFree(d_src);
	}
	if (!osdIsMapped) {
		cudaFree(d_osd);
	}
	if (!dstIsMapped) {
		cudaMemcpy(dst, d_dst, planeSize * 3/2, cudaMemcpyDeviceToHost);
		cudaFree(d_dst);
	}
}


//////////////////////////////////
//
//
__global__ void gpuConvertGRAYtoI420_kernel(unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int halfH = height>>1;
	for (int i = 0; i < halfH; ++i) {
		int y00 = src[i*2*width+idx*2+0];
		int y01 = src[i*2*width+idx*2+1];
		int y10 = src[(i*2+1)*width+idx*2+0];
		int y11 = src[(i*2+1)*width+idx*2+1];

		dY[i*2*width+idx*2+0] = y00;
		dY[i*2*width+idx*2+1] = y01;
		dY[(i*2+1)*width+idx*2+0] = y10;
		dY[(i*2+1)*width+idx*2+1] = y11;
		dU[i*(width>>1)+idx] = 0x80;
		dV[i*(width>>1)+idx] = 0x80;
	}
}
__global__ void gpuConvertGRAYtoI420AndOsd_kernel(unsigned char *osd,unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height,
		int yColor, int uColor, int vColor)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int halfH = height>>1;
	for (int i = 0; i < halfH; ++i) {
		int y00 = src[i*2*width+idx*2+0];
		int y01 = src[i*2*width+idx*2+1];
		int y10 = src[(i*2+1)*width+idx*2+0];
		int y11 = src[(i*2+1)*width+idx*2+1];

		int a00 = (osd[i*2*width+idx*2+0]+1)>>8;
		int a01 = (osd[i*2*width+idx*2+1]+1)>>8;
		int a10 = (osd[(i*2+1)*width+idx*2+0]+1)>>8;
		int a11 = (osd[(i*2+1)*width+idx*2+1]+1)>>8;
		int auv = (a00 | a01 | a10 | a11);

		dY[i*2*width+idx*2+0] = (1-a00)*y00 + a00*yColor;
		dY[i*2*width+idx*2+1] = (1-a01)*y01 + a01*yColor;
		dY[(i*2+1)*width+idx*2+0] = (1-a10)*y10 + a10*yColor;
		dY[(i*2+1)*width+idx*2+1] = (1-a11)*y11 + a11*yColor;
		dU[i*(width>>1)+idx] = (1-auv)*0x80 + auv*uColor;
		dV[i*(width>>1)+idx] = (1-auv)*0x80 + auv*vColor;
	}
}

void gpuConvertGRAYtoI420AndOsd(unsigned char  *osd,unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height, int colorYUV)
{
	unsigned char *d_src = NULL;
	unsigned char *d_dst = NULL;
	unsigned char *d_osd = NULL;
	size_t planeSize = width * height * sizeof(unsigned char);

	unsigned int flags;
	bool srcIsMapped = (cudaHostGetFlags(&flags, src) == cudaSuccess) && (flags & cudaHostAllocMapped);
	bool dstIsMapped = (cudaHostGetFlags(&flags, dst) == cudaSuccess) && (flags & cudaHostAllocMapped);
	bool osdIsMapped = (cudaHostGetFlags(&flags, dst) == cudaSuccess) && (flags & cudaHostAllocMapped);

	if (srcIsMapped) {
		d_src = src;
		cudaStreamAttachMemAsync(NULL, src, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_src, planeSize * 1);
		cudaMemcpy(d_src, src, planeSize * 1, cudaMemcpyHostToDevice);
	}
	if (osdIsMapped) {
		d_osd = osd;
		cudaStreamAttachMemAsync(NULL, osd, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_osd, planeSize * 1);
		cudaMemcpy(d_osd, osd, planeSize * 1, cudaMemcpyHostToDevice);
	}
	if (dstIsMapped) {
		d_dst = dst;
		cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_dst, planeSize * 3/2);
	}

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	int yColor = GETYVAL(colorYUV);
	int uColor = GETUVAL(colorYUV);
	int vColor = GETVVAL(colorYUV);
	gpuConvertGRAYtoI420AndOsd_kernel<<<numBlocks, blockSize>>>(d_osd,d_src,
			d_dst, d_dst+planeSize, d_dst+planeSize+(planeSize>>2), width, height,
			yColor, uColor, vColor);
	cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachHost);
	cudaStreamSynchronize(NULL);

	if (!srcIsMapped) {
		cudaFree(d_src);
	}
	if (!osdIsMapped) {
		cudaFree(d_osd);
	}
	if (!dstIsMapped) {
		cudaMemcpy(dst, d_dst, planeSize * 3/2, cudaMemcpyDeviceToHost);
		cudaFree(d_dst);
	}
}

//////////////////////////////////
//         GRAYtoI420AndOsdAndZoomx2
//
__device__ inline int meanValuePix2_gray(unsigned char *img)
{
	return ((img[0]+img[1])>>1);
}
__device__ inline int meanValuePix2_gray(unsigned char *img, int width)
{
	return ((img[0]+img[width])>>1);
}
__device__ inline int meanValuePix4_gray(unsigned char *img, int width)
{
	return ((img[0]+img[1]+img[width]+img[width+1])>>2);
}
__global__ void gpuConvertGRAYtoI420AndZoomx_kernel(unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height, int zoomxStep)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int halfH = height>>1;
	int halfIx = idx>>zoomxStep;
	for (int i = 0; i < halfH; ++i) {
		int halfIy = i>>zoomxStep;
		int y00 = src[halfIy*2*width+halfIx*2+0];
		int y01 = meanValuePix2_gray(src+halfIy*2*width+halfIx*2+0);
		int y10 = meanValuePix2_gray(src+(halfIy*2+1)*width+halfIx*2+0, width);
		int y11 = meanValuePix4_gray(src+(halfIy*2+1)*width+halfIx*2+0, width);

		dY[i*2*width+idx*2+0] = y00;
		dY[i*2*width+idx*2+1] = y01;
		dY[(i*2+1)*width+idx*2+0] = y10;
		dY[(i*2+1)*width+idx*2+1] = y11;
		dU[i*(width>>1)+idx] = 0x80;
		dV[i*(width>>1)+idx] = 0x80;
	}
}
__global__ void gpuConvertGRAYtoI420AndZoomxAndOsd_kernel(unsigned char *osd,unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height, int zoomxStep,
		int yColor, int uColor, int vColor)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int halfH = height>>1;
	int halfIx = idx>>zoomxStep;
	for (int i = 0; i < halfH; ++i) {
		int halfIy = i>>zoomxStep;
		int y00 = src[halfIy*2*width+halfIx*2+0];
		int y01 = meanValuePix2_gray(src+halfIy*2*width+halfIx*2+0);
		int y10 = meanValuePix2_gray(src+(halfIy*2+1)*width+halfIx*2+0, width);
		int y11 = meanValuePix4_gray(src+(halfIy*2+1)*width+halfIx*2+0, width);

		int a00 = (osd[i*2*width+idx*2+0]+1)>>8;
		int a01 = (osd[i*2*width+idx*2+1]+1)>>8;
		int a10 = (osd[(i*2+1)*width+idx*2+0]+1)>>8;
		int a11 = (osd[(i*2+1)*width+idx*2+1]+1)>>8;
		int auv = (a00 | a01 | a10 | a11);

		dY[i*2*width+idx*2+0] = (1-a00)*y00 + a00*yColor;
		dY[i*2*width+idx*2+1] = (1-a01)*y01 + a01*yColor;
		dY[(i*2+1)*width+idx*2+0] = (1-a10)*y10 + a10*yColor;
		dY[(i*2+1)*width+idx*2+1] = (1-a11)*y11 + a11*yColor;
		dU[i*(width>>1)+idx] = (1-auv)*0x80 + auv*uColor;
		dV[i*(width>>1)+idx] = (1-auv)*0x80 + auv*vColor;
	}
}
__global__ void gpuConvertGRAYtoI420AndZoomx_async_kernel(unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height, unsigned int heightBegin, int zoomxStep)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int halfIx = idx>>zoomxStep;
	int halfH = height>>1;
	int halfBegin = heightBegin>>1;
	for (int i = 0; i < halfH; ++i) {
		int halfIy = (i+halfBegin)>>zoomxStep;
		int y00 = src[halfIy*2*width+halfIx*2+0];
		int y01 = meanValuePix2_gray(src+halfIy*2*width+halfIx*2+0);
		int y10 = meanValuePix2_gray(src+(halfIy*2+1)*width+halfIx*2+0, width);
		int y11 = meanValuePix4_gray(src+(halfIy*2+1)*width+halfIx*2+0, width);

		dY[i*2*width+idx*2+0] = y00;
		dY[i*2*width+idx*2+1] = y01;
		dY[(i*2+1)*width+idx*2+0] = y10;
		dY[(i*2+1)*width+idx*2+1] = y11;
		dU[i*(width>>1)+idx] = 0x80;
		dV[i*(width>>1)+idx] = 0x80;
	}
}
__global__ void gpuConvertGRAYtoI420AndZoomxAndOsd_async_kernel(unsigned char *osd,unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height, unsigned int heightBegin, int zoomxStep,
		int yColor, int uColor, int vColor)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int halfIx = idx>>zoomxStep;
	int halfH = height>>1;
	int halfBegin = heightBegin>>1;
	for (int i = 0; i < halfH; ++i) {
		int halfIy = (i+halfBegin)>>zoomxStep;
		int y00 = src[halfIy*2*width+halfIx*2+0];
		int y01 = meanValuePix2_gray(src+halfIy*2*width+halfIx*2+0);
		int y10 = meanValuePix2_gray(src+(halfIy*2+1)*width+halfIx*2+0, width);
		int y11 = meanValuePix4_gray(src+(halfIy*2+1)*width+halfIx*2+0, width);

		int a00 = (osd[i*2*width+idx*2+0]+1)>>8;
		int a01 = (osd[i*2*width+idx*2+1]+1)>>8;
		int a10 = (osd[(i*2+1)*width+idx*2+0]+1)>>8;
		int a11 = (osd[(i*2+1)*width+idx*2+1]+1)>>8;
		int auv = (a00 | a01 | a10 | a11);

		dY[i*2*width+idx*2+0] = (1-a00)*y00 + a00*yColor;
		dY[i*2*width+idx*2+1] = (1-a01)*y01 + a01*yColor;
		dY[(i*2+1)*width+idx*2+0] = (1-a10)*y10 + a10*yColor;
		dY[(i*2+1)*width+idx*2+1] = (1-a11)*y11 + a11*yColor;
		dU[i*(width>>1)+idx] = (1-auv)*0x80 + auv*uColor;
		dV[i*(width>>1)+idx] = (1-auv)*0x80 + auv*vColor;
	}
}

void gpuConvertGRAYtoI420AndZoomxAndOsd(unsigned char  *osd,unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height, int zoomx, int colorYUV)
{
	unsigned char *d_src = NULL;
	unsigned char *d_dst = NULL;
	unsigned char *d_osd = NULL;
	size_t planeSize = width * height * sizeof(unsigned char);

	unsigned int flags;
	bool srcIsMapped = (cudaHostGetFlags(&flags, src) == cudaSuccess) && (flags & cudaHostAllocMapped);
	bool dstIsMapped = (cudaHostGetFlags(&flags, dst) == cudaSuccess) && (flags & cudaHostAllocMapped);
	bool osdIsMapped = (cudaHostGetFlags(&flags, dst) == cudaSuccess) && (flags & cudaHostAllocMapped);

	if (srcIsMapped) {
		d_src = src;
		cudaStreamAttachMemAsync(NULL, src, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_src, planeSize * 1);
		cudaMemcpy(d_src, src, planeSize * 1, cudaMemcpyHostToDevice);
	}
	if (osdIsMapped) {
		d_osd = osd;
		cudaStreamAttachMemAsync(NULL, osd, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_osd, planeSize * 1);
		cudaMemcpy(d_osd, osd, planeSize * 1, cudaMemcpyHostToDevice);
	}
	if (dstIsMapped) {
		d_dst = dst;
		cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_dst, planeSize * 3/2);
	}

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	int yColor = GETYVAL(colorYUV);
	int uColor = GETUVAL(colorYUV);
	int vColor = GETVVAL(colorYUV);
	gpuConvertGRAYtoI420AndZoomxAndOsd_kernel<<<numBlocks, blockSize>>>(d_osd,
			d_src+(height/2-(height/(zoomx<<1)))*width+(width/2-(width/(zoomx<<1))),
			d_dst, d_dst+planeSize, d_dst+planeSize+(planeSize>>2), width, height,zoomx>>1,
			yColor, uColor, vColor);
	cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachHost);
	cudaStreamSynchronize(NULL);

	if (!srcIsMapped) {
		cudaFree(d_src);
	}
	if (!osdIsMapped) {
		cudaFree(d_osd);
	}
	if (!dstIsMapped) {
		cudaMemcpy(dst, d_dst, planeSize * 3/2, cudaMemcpyDeviceToHost);
		cudaFree(d_dst);
	}
}

//////////////////////////////////////////////////////////////////////
//        YUYVtoI420AndZoomxAndOsd
//
__device__ inline int meanValuePix2_yuyv(unsigned char *img)
{
	return (((int)img[0]+img[2])>>1);
}
__device__ inline int meanValuePix2_yuyv(unsigned char *img, int width)
{
	return (((int)img[0]+img[width<<1])>>1);
}
__device__ inline int meanValuePix4_yuyv(unsigned char *img, int width)
{
	return (((int)img[0]+img[2]+img[width<<1]+img[width<<1+2])>>2);
}
/*
__global__ void gpuConvertYUYVtoI420AndZoomxAndOsd_kernel(unsigned char *osd,unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height, int zoomxStep,
		int yColor, int uColor, int vColor)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int zoomIx = idx>>zoomxStep;
	int halfH = height>>1;
	for (int i = 0; i < halfH; ++i) {
		int zoomIy = i>>zoomxStep;
		int y00 = meanValuePix4_yuyv(src+zoomIy*2*width*2+zoomIx*4+0, width);
		int cb0 = src[zoomIy*2*width*2+zoomIx*4+1];
		int y01 = meanValuePix4_yuyv(src+zoomIy*2*width*2+zoomIx*4+2, width);
		int cr0 = src[zoomIy*2*width*2+zoomIx*4+3];
		int y10 = meanValuePix4_yuyv(src+(zoomIy*2+1)*width*2+zoomIx*4+0, width);
		int y11 = meanValuePix4_yuyv(src+(zoomIy*2+1)*width*2+zoomIx*4+2, width);

		int a00 = (osd[i*2*width+idx*2+0]+1)>>8;
		int a01 = (osd[i*2*width+idx*2+1]+1)>>8;
		int a10 = (osd[(i*2+1)*width+idx*2+0]+1)>>8;
		int a11 = (osd[(i*2+1)*width+idx*2+1]+1)>>8;
		int auv = (a00 | a01 | a10 | a11);

		dY[i*2*width+idx*2+0] = (1-a00)*y00 + a00*yColor;
		dY[i*2*width+idx*2+1] = (1-a01)*y01 + a01*yColor;
		dY[(i*2+1)*width+idx*2+0] = (1-a10)*y10 + a10*yColor;
		dY[(i*2+1)*width+idx*2+1] = (1-a11)*y11 + a11*yColor;
		dU[i*(width>>1)+idx] = (1-auv)*cb0 + auv*uColor;
		dV[i*(width>>1)+idx] = (1-auv)*cr0 + auv*vColor;
	}
}

__global__ void gpuConvertYUYVtoI420AndZoomxAndOsd_async_kernel(unsigned char *osd,unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height, unsigned int heightBegin, int zoomxStep,
		int yColor, int uColor, int vColor)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int zoomIx = idx>>zoomxStep;
	int halfH = height>>1;
	int halfB = (heightBegin>>1);
	for (int i = 0; i < halfH; ++i) {
		int zoomIy = (i+halfB)>>zoomxStep;
		int y00 = meanValuePix4_yuyv(src+zoomIy*2*width*2+zoomIx*4+0, width);
		int cb0 = src[zoomIy*2*width*2+zoomIx*4+1];
		int y01 = meanValuePix4_yuyv(src+zoomIy*2*width*2+zoomIx*4+2, width);
		int cr0 = src[zoomIy*2*width*2+zoomIx*4+3];
		int y10 = meanValuePix4_yuyv(src+(zoomIy*2+1)*width*2+zoomIx*4+0, width);
		int y11 = meanValuePix4_yuyv(src+(zoomIy*2+1)*width*2+zoomIx*4+2, width);

		int a00 = (osd[i*2*width+idx*2+0]+1)>>8;
		int a01 = (osd[i*2*width+idx*2+1]+1)>>8;
		int a10 = (osd[(i*2+1)*width+idx*2+0]+1)>>8;
		int a11 = (osd[(i*2+1)*width+idx*2+1]+1)>>8;
		int auv = (a00 | a01 | a10 | a11);

		dY[i*2*width+idx*2+0] = (1-a00)*y00 + a00*yColor;
		dY[i*2*width+idx*2+1] = (1-a01)*y01 + a01*yColor;
		dY[(i*2+1)*width+idx*2+0] = (1-a10)*y10 + a10*yColor;
		dY[(i*2+1)*width+idx*2+1] = (1-a11)*y11 + a11*yColor;
		dU[i*(width>>1)+idx] = (1-auv)*cb0 + auv*uColor;
		dV[i*(width>>1)+idx] = (1-auv)*cr0 + auv*vColor;
	}
}
*/
__global__ void gpuConvertYUYVtoI420AndZoomx_kernel(unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height, int zoomxStep)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int zoomIx = idx>>zoomxStep;
	int halfH = height>>1;
	for (int i = 0; i < halfH; ++i) {
		int zoomIy = i>>zoomxStep;
		int y00 = src[zoomIy*2*width*2+zoomIx*4+0];
		int cb0 = src[zoomIy*2*width*2+zoomIx*4+1];
		int y01 = meanValuePix2_yuyv(src+zoomIy*2*width*2+zoomIx*4+0);
		int cr0 = src[zoomIy*2*width*2+zoomIx*4+3];
		int y10 = meanValuePix2_yuyv(src+zoomIy*2*width*2+zoomIx*4+0, width);
		int y11 = meanValuePix4_yuyv(src+zoomIy*2*width*2+zoomIx*4+0, width);

		dY[i*2*width+idx*2+0] = y00;
		dY[i*2*width+idx*2+1] = y01;
		dY[(i*2+1)*width+idx*2+0] = y10;
		dY[(i*2+1)*width+idx*2+1] = y11;
		dU[i*(width>>1)+idx] = cb0;
		dV[i*(width>>1)+idx] = cr0;
	}
}

__global__ void gpuConvertYUYVtoI420AndZoomxAndOsd_kernel(unsigned char *osd,unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height, int zoomxStep,
		int yColor, int uColor, int vColor)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int zoomIx = idx>>zoomxStep;
	int halfH = height>>1;
	for (int i = 0; i < halfH; ++i) {
		int zoomIy = i>>zoomxStep;
		int y00 = src[zoomIy*2*width*2+zoomIx*4+0];
		int cb0 = src[zoomIy*2*width*2+zoomIx*4+1];
		int y01 = meanValuePix2_yuyv(src+zoomIy*2*width*2+zoomIx*4+0);
		int cr0 = src[zoomIy*2*width*2+zoomIx*4+3];
		int y10 = meanValuePix2_yuyv(src+zoomIy*2*width*2+zoomIx*4+0, width);
		int y11 = meanValuePix4_yuyv(src+zoomIy*2*width*2+zoomIx*4+0, width);

		int a00 = (osd[i*2*width+idx*2+0]+1)>>8;
		int a01 = (osd[i*2*width+idx*2+1]+1)>>8;
		int a10 = (osd[(i*2+1)*width+idx*2+0]+1)>>8;
		int a11 = (osd[(i*2+1)*width+idx*2+1]+1)>>8;
		int auv = (a00 | a01 | a10 | a11);

		dY[i*2*width+idx*2+0] = (1-a00)*y00 + a00*yColor;
		dY[i*2*width+idx*2+1] = (1-a01)*y01 + a01*yColor;
		dY[(i*2+1)*width+idx*2+0] = (1-a10)*y10 + a10*yColor;
		dY[(i*2+1)*width+idx*2+1] = (1-a11)*y11 + a11*yColor;
		dU[i*(width>>1)+idx] = (1-auv)*cb0 + auv*uColor;
		dV[i*(width>>1)+idx] = (1-auv)*cr0 + auv*vColor;
	}
}

__global__ void gpuConvertYUYVtoI420AndZoomx_async_kernel(unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height, unsigned int heightBegin, int zoomxStep)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	int zoomIx = idx>>zoomxStep;
	int halfH = height>>1;
	int halfB = (heightBegin>>1);
	for (int i = 0; i < halfH; ++i) {
		int zoomIy = (i+halfB)>>zoomxStep;
		int y00 = src[zoomIy*2*width*2+zoomIx*4+0];
		int cb0 = src[zoomIy*2*width*2+zoomIx*4+1];
		int y01 = meanValuePix2_yuyv(src+zoomIy*2*width*2+zoomIx*4+0);
		int cr0 = src[zoomIy*2*width*2+zoomIx*4+3];
		int y10 = meanValuePix2_yuyv(src+zoomIy*2*width*2+zoomIx*4+0, width);
		int y11 = meanValuePix4_yuyv(src+zoomIy*2*width*2+zoomIx*4+0, width);

		dY[i*2*width+idx*2+0] = y00;
		dY[i*2*width+idx*2+1] = y01;
		dY[(i*2+1)*width+idx*2+0] = y10;
		dY[(i*2+1)*width+idx*2+1] = y11;
		dU[i*(width>>1)+idx] = cb0;
		dV[i*(width>>1)+idx] = cr0;
	}
}
__global__ void gpuConvertYUYVtoI420AndZoomxAndOsd_async_kernel(unsigned char *osd,unsigned char *src,
		unsigned char *dY,unsigned char *dU,unsigned char *dV,
		unsigned int width, unsigned int height, unsigned int heightBegin, int zoomxStep,
		int yColor, int uColor, int vColor)
{
	const int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}

	const int zoomIx = idx>>zoomxStep;
	const int halfH = height>>1;
	const int halfB = (heightBegin>>1);
	for (int i = 0; i < halfH; ++i) {
		int zoomIy = (i+halfB)>>zoomxStep;

		int y00 = src[zoomIy*2*width*2+zoomIx*4+0];
		int cb0 = src[zoomIy*2*width*2+zoomIx*4+1];
		int y01 = meanValuePix2_yuyv(src+zoomIy*2*width*2+zoomIx*4+0);
		int cr0 = src[zoomIy*2*width*2+zoomIx*4+3];
		int y10 = meanValuePix2_yuyv(src+zoomIy*2*width*2+zoomIx*4+0, width);
		int y11 = meanValuePix4_yuyv(src+zoomIy*2*width*2+zoomIx*4+0, width);

		int a00 = (osd[i*2*width+idx*2+0]+1)>>8;
		int a01 = (osd[i*2*width+idx*2+1]+1)>>8;
		int a10 = (osd[(i*2+1)*width+idx*2+0]+1)>>8;
		int a11 = (osd[(i*2+1)*width+idx*2+1]+1)>>8;
		int auv = (a00 | a01 | a10 | a11);

		dY[i*2*width+idx*2+0] = (1-a00)*y00 + a00*yColor;
		dY[i*2*width+idx*2+1] = (1-a01)*y01 + a01*yColor;
		dY[(i*2+1)*width+idx*2+0] = (1-a10)*y10 + a10*yColor;
		dY[(i*2+1)*width+idx*2+1] = (1-a11)*y11 + a11*yColor;
		dU[i*(width>>1)+idx] = (1-auv)*cb0 + auv*uColor;
		dV[i*(width>>1)+idx] = (1-auv)*cr0 + auv*vColor;
	}
}
void gpuConvertYUYVtoI420AndZoomxAndOsd(unsigned char  *osd,unsigned char *src, unsigned char *dst,
		unsigned int width, unsigned int height, int zoomx, int colorYUV)
{
	unsigned char *d_src = NULL;
	unsigned char *d_dst = NULL;
	unsigned char *d_osd = NULL;
	size_t planeSize = width * height * sizeof(unsigned char);

	unsigned int flags;
	bool srcIsMapped = (cudaHostGetFlags(&flags, src) == cudaSuccess) && (flags & cudaHostAllocMapped);
	bool dstIsMapped = (cudaHostGetFlags(&flags, dst) == cudaSuccess) && (flags & cudaHostAllocMapped);
	bool osdIsMapped = (cudaHostGetFlags(&flags, dst) == cudaSuccess) && (flags & cudaHostAllocMapped);

	if (srcIsMapped) {
		d_src = src;
		cudaStreamAttachMemAsync(NULL, src, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_src, planeSize * 2);
		cudaMemcpy(d_src, src, planeSize * 2, cudaMemcpyHostToDevice);
	}
	if (osdIsMapped) {
		d_osd = osd;
		cudaStreamAttachMemAsync(NULL, osd, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_osd, planeSize * 1);
		cudaMemcpy(d_osd, osd, planeSize * 1, cudaMemcpyHostToDevice);
	}
	if (dstIsMapped) {
		d_dst = dst;
		cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachGlobal);
	} else {
		cudaMalloc(&d_dst, planeSize * 3/2);
	}

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	int yColor = GETYVAL(colorYUV);
	int uColor = GETUVAL(colorYUV);
	int vColor = GETVVAL(colorYUV);
	gpuConvertYUYVtoI420AndZoomxAndOsd_kernel<<<numBlocks, blockSize>>>(d_osd,
			d_src+(height/2-(height/(zoomx<<1)))*width*2+(width/2-(width/(zoomx<<1)))*2,
			d_dst, d_dst+planeSize, d_dst+planeSize+(planeSize>>2), width, height,zoomx>>1,
			yColor, uColor, vColor);
	cudaStreamAttachMemAsync(NULL, dst, 0, cudaMemAttachHost);
	cudaStreamSynchronize(NULL);

	if (!srcIsMapped) {
		cudaFree(d_src);
	}
	if (!osdIsMapped) {
		cudaFree(d_osd);
	}
	if (!dstIsMapped) {
		cudaMemcpy(dst, d_dst, planeSize * 3/2, cudaMemcpyDeviceToHost);
		cudaFree(d_dst);
	}
}

////////////////////////////////////////////////////////////////////
//
//   040_app
//

#define STREAM_CNT	(4)
#define CHANNELS_MAX (4)
#define WIDTH_MAX	(1920)
#define HEIGHT_MAX  (1080)

#define DMEM_CNT	(3)
typedef struct _CUOBJ{
	int index;
	bool bAlloc;
	cudaEvent_t start, stop;
	cudaStream_t streams[STREAM_CNT];
	unsigned char *d_src;
	unsigned char *d_osd;
	unsigned char *d_mem[DMEM_CNT];
	float elapsedTimeMax, elapsedTimeMin, elapsedTimeSum;
	unsigned int count;
	int imem;
	int64 stampBegin;
	int64 stampEnd;
}CUOBJ, *PCUOBJ;
static CUOBJ gObjs[CHANNELS_MAX];
static OSA_MutexHndl *mutex = NULL;
static bool bCreateMutex = false;
#define LOCK     OSA_mutexLock(mutex);
#define UNLOCK   OSA_mutexUnlock(mutex);
static cudaError_t cuConvert_yuyvToI420andOsd(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int colorYUV);
static cudaError_t cuConvert_yuyvToI420AndZoomxAndOsd(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int zoomx, int colorYUV);
static cudaError_t cuConvert_yuyvToI420andOsd_async(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int colorYUV);
static cudaError_t cuConvert_yuyvToI420AndZoomxAndOsd_async(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int zoomx, int colorYUV);
static cudaError_t cuConvert_yuyvToI420AndZoomx_async(PCUOBJ pObj, Mat src, Mat &dst, int zoomx);
static cudaError_t cuConvert_grayToI420andOsd(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int colorYUV);
static cudaError_t cuConvert_grayToI420AndZoomxAndOsd(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int zoomx, int colorYUV);
static cudaError_t cuConvert_grayToI420andOsd_async(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int colorYUV);
static cudaError_t cuConvert_grayToI420AndZoomxAndOsd_async(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int zoomx, int colorYUV);
static cudaError_t cuConvert_grayToI420AndZoomx_async(PCUOBJ pObj, Mat src, Mat &dst, int zoomx);

cudaError_t cuConvertInit(int nChannels, OSA_MutexHndl *mutexLock)
{
	cudaError_t ret = cudaSuccess;
	int count, i;
	cudaDeviceProp prop;
	size_t planeSize = WIDTH_MAX * HEIGHT_MAX;

	if(nChannels<=0 || nChannels >CHANNELS_MAX)
		nChannels = CHANNELS_MAX;

	memset(&gObjs, 0, sizeof(gObjs));

	ret = cudaGetDeviceCount(&count);

	ret = cudaGetDeviceProperties(&prop, 0);

	printf("   --- General Information for GPU device ---\n");
	printf("Name:                  %s\n", prop.name);
	printf("Capability:            %d.%d\n", prop.major, prop.minor);
	printf("Clock rate:            %d\n", prop.clockRate);
	printf("Copy overlap:          %d\n", prop.deviceOverlap);
	printf("Total global mem:      %ld\n", prop.totalGlobalMem);
	printf("Total constant mem:    %ld\n", prop.totalConstMem);
	printf("Multiprocessor count:  %d\n", prop.multiProcessorCount);
	printf("Max threads per block: %d\n", prop.maxThreadsPerBlock);
	printf("Max thread dimensions: (%d, %d, %d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
	printf("Max grid dimensions:   (%d, %d, %d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);

    const NppLibraryVersion *libVer   = nppGetLibVersion();
    printf("NPP Library Version %d.%d.%d\n", libVer->major, libVer->minor, libVer->build);

	int driverVersion, runtimeVersion;
    cudaDriverGetVersion(&driverVersion);
    cudaRuntimeGetVersion(&runtimeVersion);
	printf("CUDA Driver  Version: %d.%d\n", driverVersion/1000, (driverVersion%100)/10);
	printf("CUDA Runtime Version: %d.%d\n", runtimeVersion/1000, (runtimeVersion%100)/10);

	bool bVal = checkCudaCapabilities(1, 0);

	for(int chId = 0; chId<nChannels; chId++)
	{
		gObjs[chId].index = chId;
		ret = cudaEventCreate(&gObjs[chId].start);
		ret = cudaEventCreate(&gObjs[chId].stop);

		for(i=0; i<STREAM_CNT; i++)
			ret = cudaStreamCreate(&gObjs[chId].streams[i]);

		ret = cudaMalloc(&gObjs[chId].d_src, planeSize * 3);
		ret = cudaMalloc(&gObjs[chId].d_osd, planeSize * 4);
		for(i=0; i<DMEM_CNT; i++)
			ret = cudaMalloc(&gObjs[chId].d_mem[i], planeSize * 3);
		gObjs[chId].bAlloc = true;
		gObjs[chId].elapsedTimeMin = 1000.0f;
	}

	if(mutexLock != NULL)
		mutex = mutexLock;
	if(mutex == NULL){
		mutex = new OSA_MutexHndl;
		OSA_mutexCreate(mutex);
		bCreateMutex = true;
	}

	return ret;
}

cudaError_t cuConvertUinit()
{
	int i;

	OSA_mutexUnlock(mutex);
	OSA_mutexLock(mutex);

	for(int chId=0; chId<CHANNELS_MAX; chId++)
	{
		if(!gObjs[chId].bAlloc)
			break;
		for(i=0; i<STREAM_CNT; i++){
			cudaStreamDestroy(gObjs[chId].streams[i]);
		}
		cudaEventDestroy(gObjs[chId].start);
		cudaEventDestroy(gObjs[chId].stop);

		cudaFree(gObjs[chId].d_src);
		cudaFree(gObjs[chId].d_osd);
		for(i=0; i<DMEM_CNT; i++)
			cudaFree(gObjs[chId].d_mem[i]);

		gObjs[chId].bAlloc = false;
	}
	if(bCreateMutex){
		OSA_mutexDelete(mutex);
		delete mutex;
		bCreateMutex = false;
	}
	mutex = NULL;

	return cudaSuccess;
}

static cudaError_t cuConvert_yuyvToI420andOsd(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int colorYUV)
{
	cudaError_t ret = cudaSuccess;
	int width = src.cols, height = src.rows;
	size_t planeSize = width * height;
	int yColor = GETYVAL(colorYUV);
	int uColor = GETUVAL(colorYUV);
	int vColor = GETVVAL(colorYUV);

	pObj->imem = 0;
	unsigned char *d_src = pObj->d_src;
	unsigned char *d_dst = pObj->d_mem[pObj->imem];
	unsigned char *d_osd = pObj->d_osd;

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;

	ret = cudaMemcpy(d_src, src.data, planeSize*2, cudaMemcpyHostToDevice);
	ret = cudaMemcpy(d_osd, osd.data, planeSize, cudaMemcpyHostToDevice);
	gpuConvertYUYVtoI420AndOsd_kernel<<<numBlocks, blockSize>>>(
			d_osd, d_src, d_dst,
			d_dst+planeSize,
			d_dst+planeSize+(planeSize>>2),
			width, height,yColor, uColor, vColor);
	cudaStreamSynchronize(NULL);
	ret = cudaMemcpy(dst.data,d_dst, planeSize*3/2, cudaMemcpyDeviceToHost);

	return ret;
}

static cudaError_t cuConvert_grayToI420andOsd(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int colorYUV)
{
	cudaError_t ret = cudaSuccess;
	int width = src.cols, height = src.rows;
	size_t planeSize = width * height;
	int yColor = GETYVAL(colorYUV);
	int uColor = GETUVAL(colorYUV);
	int vColor = GETVVAL(colorYUV);

	pObj->imem = 0;
	unsigned char *d_src = pObj->d_src;
	unsigned char *d_dst = pObj->d_mem[pObj->imem];
	unsigned char *d_osd = pObj->d_osd;

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;

	ret = cudaMemcpy(d_src, src.data, planeSize, cudaMemcpyHostToDevice);
	ret = cudaMemcpy(d_osd, osd.data, planeSize, cudaMemcpyHostToDevice);
	gpuConvertGRAYtoI420AndOsd_kernel<<<numBlocks, blockSize>>>(
			d_osd, d_src, d_dst,
			d_dst+planeSize,
			d_dst+planeSize+(planeSize>>2),
			width, height, yColor, uColor, vColor);
	cudaStreamSynchronize(NULL);
	ret = cudaMemcpy(dst.data,d_dst, planeSize*3/2, cudaMemcpyDeviceToHost);

	return ret;
}

static cudaError_t cuConvert_yuyvToI420AndZoomxAndOsd(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int zoomx, int colorYUV)
{
	cudaError_t ret = cudaSuccess;
	int width = src.cols, height = src.rows;
	size_t planeSize = width * height;
	int yColor = GETYVAL(colorYUV);
	int uColor = GETUVAL(colorYUV);
	int vColor = GETVVAL(colorYUV);

	pObj->imem = 0;
	unsigned char *d_src = pObj->d_src;
	unsigned char *d_dst = pObj->d_mem[pObj->imem];
	unsigned char *d_osd = pObj->d_osd;

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;

	ret = cudaMemcpy(d_src, src.data, planeSize*2, cudaMemcpyHostToDevice);
	ret = cudaMemcpy(d_osd, osd.data, planeSize, cudaMemcpyHostToDevice);
	if(zoomx<=1){
		gpuConvertYUYVtoI420AndOsd_kernel<<<numBlocks, blockSize>>>(
				d_osd, d_src, d_dst, d_dst+planeSize, d_dst+planeSize+(planeSize>>2),
				width, height,yColor, uColor, vColor);
	}else{
		gpuConvertYUYVtoI420AndZoomxAndOsd_kernel<<<numBlocks, blockSize>>>(
				d_osd, d_src+(int)((float)height/2-((float)height/(zoomx<<1))+0.5f)*width*2+(width/2-(width/(zoomx<<1)))*2,
				d_dst, d_dst+planeSize, d_dst+planeSize+(planeSize>>2),
				width, height, zoomx>>1, yColor, uColor, vColor);
	}
	cudaStreamSynchronize(NULL);
	ret = cudaMemcpy(dst.data,d_dst, planeSize*3/2, cudaMemcpyDeviceToHost);

	return ret;
}

static cudaError_t cuConvert_grayToI420AndZoomxAndOsd(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int zoomx, int colorYUV)
{
	cudaError_t ret = cudaSuccess;
	int width = src.cols, height = src.rows;
	size_t planeSize = width * height;
	int yColor = GETYVAL(colorYUV);
	int uColor = GETUVAL(colorYUV);
	int vColor = GETVVAL(colorYUV);

	pObj->imem = 0;
	unsigned char *d_src = pObj->d_src;
	unsigned char *d_dst = pObj->d_mem[pObj->imem];
	unsigned char *d_osd = pObj->d_osd;

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;

	ret = cudaMemcpy(d_src, src.data, planeSize, cudaMemcpyHostToDevice);
	ret = cudaMemcpy(d_osd, osd.data, planeSize, cudaMemcpyHostToDevice);
	if(zoomx<=1){
		gpuConvertGRAYtoI420AndOsd_kernel<<<numBlocks, blockSize>>>(
				d_osd, d_src, d_dst,
				d_dst+planeSize,
				d_dst+planeSize+(planeSize>>2),
				width, height, yColor, uColor, vColor);
	}else{
		gpuConvertGRAYtoI420AndZoomxAndOsd_kernel<<<numBlocks, blockSize>>>(
				d_osd, d_src+(height/2-(height/(zoomx<<1)))*width+(width/2-(width/(zoomx<<1))),
				d_dst, d_dst+planeSize, d_dst+planeSize+(planeSize>>2),
				width, height, zoomx>>1, yColor, uColor, vColor);
	}
	cudaStreamSynchronize(NULL);
	ret = cudaMemcpy(dst.data,d_dst, planeSize*3/2, cudaMemcpyDeviceToHost);

	return ret;
}

static cudaError_t cuConvert_yuyvToI420AndZoomxAndOsd_async(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int zoomx, int colorYUV)
{
	cudaError_t ret = cudaSuccess;
	int i, width = src.cols, height = src.rows;
	size_t planeSize = width * height;
	int yColor = GETYVAL(colorYUV);
	int uColor = GETUVAL(colorYUV);
	int vColor = GETVVAL(colorYUV);

	pObj->imem = 0;
	unsigned char *d_src = pObj->d_src;
	unsigned char *d_dst = pObj->d_mem[pObj->imem];
	unsigned char *d_osd = pObj->d_osd;

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	unsigned int nBCs = (src.rows * src.cols * src.channels())/STREAM_CNT;
	unsigned int nBCo = (osd.rows * osd.cols * osd.channels())/STREAM_CNT;
	unsigned int nBCd = (src.rows * src.cols)/STREAM_CNT;//I420
	unsigned int nBCduv = (src.rows/2) * ((src.cols/2)/STREAM_CNT);

	if(zoomx<=1){
		for(i = 0; i<STREAM_CNT; i++){
			ret = cudaMemcpyAsync(d_src + nBCs*i,
					src.data + nBCs*i, nBCs, cudaMemcpyHostToDevice, pObj->streams[i]);
		}
		for(i = 0; i<STREAM_CNT; i++){
			ret = cudaMemcpyAsync(d_osd + nBCo*i,
					osd.data + nBCo*i, nBCo, cudaMemcpyHostToDevice, pObj->streams[i]);
		}
		for(i = 0; i<STREAM_CNT; i++){
			gpuConvertYUYVtoI420AndOsd_kernel<<<numBlocks, blockSize, 0, pObj->streams[i]>>>(
					d_osd + nBCo*i, d_src + nBCs*i, d_dst + nBCd*i,
					d_dst+planeSize+nBCduv*i,
					d_dst+planeSize+(planeSize>>2)+nBCduv*i,
					width, height/STREAM_CNT,yColor, uColor, vColor);
		}
	}else{
		ret = cudaMemcpyAsync(d_src + planeSize*src.channels()/4,
				src.data + planeSize*src.channels()/4, planeSize*src.channels()/4,
				cudaMemcpyHostToDevice, pObj->streams[0]);
		ret = cudaMemcpyAsync(d_src + planeSize*src.channels()/2,
				src.data + planeSize*src.channels()/2, planeSize*src.channels()/4,
				cudaMemcpyHostToDevice, pObj->streams[1]);
		for(i = 0; i<STREAM_CNT; i++){
			ret = cudaMemcpyAsync(d_osd + nBCo*i,
					osd.data + nBCo*i, nBCo, cudaMemcpyHostToDevice, pObj->streams[i]);
		}
		for(i = 0; i<STREAM_CNT; i++){
			//gpuConvertYUYVtoI420AndZoomxAndOsd_async_kernel
			gpuConvertYUYVtoI420AndZoomxAndOsd_async_kernel<<<numBlocks, blockSize, 0, pObj->streams[i]>>>(
					d_osd + nBCo*i,
					d_src+(int)((float)height/2-((float)height/(zoomx<<1))+0.5f)*width*2+(width/2-(width/(zoomx<<1)))*2,
					d_dst + nBCd*i,
					d_dst+planeSize+nBCduv*i,
					d_dst+planeSize+(planeSize>>2)+nBCduv*i,
					width, height/STREAM_CNT, height*i/STREAM_CNT, zoomx>>1,yColor, uColor, vColor);
		}
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaMemcpyAsync(dst.data + nBCd*i,
				d_dst + nBCd*i, nBCd, cudaMemcpyDeviceToHost, pObj->streams[i]);
		ret = cudaMemcpyAsync(dst.data + planeSize + nBCduv*i,
				d_dst + planeSize + nBCduv*i, nBCduv, cudaMemcpyDeviceToHost, pObj->streams[i]);
		ret = cudaMemcpyAsync(dst.data + planeSize + (planeSize>>2) + nBCduv*i,
				d_dst + planeSize + (planeSize>>2) + nBCduv*i, nBCduv, cudaMemcpyDeviceToHost, pObj->streams[i]);
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaStreamSynchronize(pObj->streams[i]);
	}

	return ret;
}

static cudaError_t cuConvert_yuyvToI420andOsd_async(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int colorYUV)
{
	cudaError_t ret = cudaSuccess;
	int i, width = src.cols, height = src.rows;
	size_t planeSize = width * height;
	int yColor = GETYVAL(colorYUV);
	int uColor = GETUVAL(colorYUV);
	int vColor = GETVVAL(colorYUV);

	pObj->imem = 0;
	unsigned char *d_src = pObj->d_src;
	unsigned char *d_dst = pObj->d_mem[pObj->imem];
	unsigned char *d_osd = pObj->d_osd;

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	unsigned int nBCs = (src.rows * src.cols * src.channels())/STREAM_CNT;
	unsigned int nBCo = (osd.rows * osd.cols * osd.channels())/STREAM_CNT;
	unsigned int nBCd = (src.rows * src.cols)/STREAM_CNT;//I420
	unsigned int nBCduv = (src.rows/2) * ((src.cols/2)/STREAM_CNT);

	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaMemcpyAsync(d_src + nBCs*i, 
				src.data + nBCs*i, nBCs, cudaMemcpyHostToDevice, pObj->streams[i]);
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaMemcpyAsync(d_osd + nBCo*i, 
				osd.data + nBCo*i, nBCo, cudaMemcpyHostToDevice, pObj->streams[i]);
	}
	for(i = 0; i<STREAM_CNT; i++){
		gpuConvertYUYVtoI420AndOsd_kernel<<<numBlocks, blockSize, 0, pObj->streams[i]>>>(
				d_osd + nBCo*i, d_src + nBCs*i, d_dst + nBCd*i, 
				d_dst+planeSize+nBCduv*i, 
				d_dst+planeSize+(planeSize>>2)+nBCduv*i, 
				width, height/STREAM_CNT,yColor, uColor, vColor);
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaMemcpyAsync(dst.data + nBCd*i,
				d_dst + nBCd*i, nBCd, cudaMemcpyDeviceToHost, pObj->streams[i]);
		ret = cudaMemcpyAsync(dst.data + planeSize + nBCduv*i,
				d_dst + planeSize + nBCduv*i, nBCduv, cudaMemcpyDeviceToHost, pObj->streams[i]);
		ret = cudaMemcpyAsync(dst.data + planeSize + (planeSize>>2) + nBCduv*i,
				d_dst + planeSize + (planeSize>>2) + nBCduv*i, nBCduv, cudaMemcpyDeviceToHost, pObj->streams[i]);
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaStreamSynchronize(pObj->streams[i]);
	}

	return ret;
}

static cudaError_t cuConvert_yuyvToI420AndZoomx_async(PCUOBJ pObj, Mat src, Mat &dst, int zoomx)
{
	cudaError_t ret = cudaSuccess;
	int i, width = src.cols, height = src.rows;
	size_t planeSize = width * height;

	pObj->imem = 0;
	unsigned char *d_src = pObj->d_src;
	unsigned char *d_dst = pObj->d_mem[pObj->imem];

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	unsigned int nBCs = (src.rows * src.cols * src.channels())/STREAM_CNT;
	unsigned int nBCd = (src.rows * src.cols)/STREAM_CNT;//I420
	unsigned int nBCduv = (src.rows/2) * ((src.cols/2)/STREAM_CNT);

	if(zoomx<=1){
		for(i = 0; i<STREAM_CNT; i++){
			ret = cudaMemcpyAsync(d_src + nBCs*i,
					src.data + nBCs*i, nBCs, cudaMemcpyHostToDevice, pObj->streams[i]);
		}
		for(i = 0; i<STREAM_CNT; i++){
			gpuConvertYUYVtoI420_kernel<<<numBlocks, blockSize, 0, pObj->streams[i]>>>(
					d_src + nBCs*i, d_dst + nBCd*i,
					d_dst+planeSize+nBCduv*i,
					d_dst+planeSize+(planeSize>>2)+nBCduv*i,
					width, height/STREAM_CNT);
		}
	}else{
		ret = cudaMemcpyAsync(d_src + planeSize*src.channels()/4,
				src.data + planeSize*src.channels()/4, planeSize*src.channels()/4,
				cudaMemcpyHostToDevice, pObj->streams[0]);
		ret = cudaMemcpyAsync(d_src + planeSize*src.channels()/2,
				src.data + planeSize*src.channels()/2, planeSize*src.channels()/4,
				cudaMemcpyHostToDevice, pObj->streams[1]);
		for(i = 0; i<STREAM_CNT; i++){
			//gpuConvertYUYVtoI420AndZoomxAndOsd_async_kernel
			gpuConvertYUYVtoI420AndZoomx_async_kernel<<<numBlocks, blockSize, 0, pObj->streams[i]>>>(
					d_src+(int)((float)height/2-((float)height/(zoomx<<1))+0.5f)*width*2+(width/2-(width/(zoomx<<1)))*2,
					d_dst + nBCd*i,
					d_dst+planeSize+nBCduv*i,
					d_dst+planeSize+(planeSize>>2)+nBCduv*i,
					width, height/STREAM_CNT, height*i/STREAM_CNT, zoomx>>1);
		}
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaMemcpyAsync(dst.data + nBCd*i,
				d_dst + nBCd*i, nBCd, cudaMemcpyDeviceToHost, pObj->streams[i]);
		ret = cudaMemcpyAsync(dst.data + planeSize + nBCduv*i,
				d_dst + planeSize + nBCduv*i, nBCduv, cudaMemcpyDeviceToHost, pObj->streams[i]);
		ret = cudaMemcpyAsync(dst.data + planeSize + (planeSize>>2) + nBCduv*i,
				d_dst + planeSize + (planeSize>>2) + nBCduv*i, nBCduv, cudaMemcpyDeviceToHost, pObj->streams[i]);
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaStreamSynchronize(pObj->streams[i]);
	}

	return ret;
}

static cudaError_t cuConvert_grayToI420andOsd_async(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int colorYUV)
{
	cudaError_t ret = cudaSuccess;
	int i, width = src.cols, height = src.rows;
	size_t planeSize = width * height;
	int yColor = GETYVAL(colorYUV);
	int uColor = GETUVAL(colorYUV);
	int vColor = GETVVAL(colorYUV);

	pObj->imem = 0;
	unsigned char *d_src = pObj->d_src;
	unsigned char *d_dst = pObj->d_mem[pObj->imem];
	unsigned char *d_osd = pObj->d_osd;

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	unsigned int nBCs = (src.rows * src.cols * src.channels())/STREAM_CNT;
	unsigned int nBCo = (osd.rows * osd.cols * osd.channels())/STREAM_CNT;
	unsigned int nBCd = (src.rows * src.cols)/STREAM_CNT;//I420
	unsigned int nBCduv = (src.rows/2) * ((src.cols/2)/STREAM_CNT);

	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaMemcpyAsync(d_src + nBCs*i, 
				src.data + nBCs*i, nBCs, cudaMemcpyHostToDevice, pObj->streams[i]);
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaMemcpyAsync(d_osd + nBCo*i, 
				osd.data + nBCo*i, nBCo, cudaMemcpyHostToDevice, pObj->streams[i]);
	}
	for(i = 0; i<STREAM_CNT; i++){
		gpuConvertGRAYtoI420AndOsd_kernel<<<numBlocks, blockSize, 0, pObj->streams[i]>>>(
				d_osd + nBCo*i, d_src + nBCs*i, d_dst + nBCd*i, 
				d_dst+planeSize+nBCduv*i, 
				d_dst+planeSize+(planeSize>>2)+nBCduv*i, 
				width, height/STREAM_CNT, yColor, uColor, vColor);
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaMemcpyAsync(dst.data + nBCd*i,
				d_dst + nBCd*i, nBCd, cudaMemcpyDeviceToHost, pObj->streams[i]);
		ret = cudaMemcpyAsync(dst.data + planeSize + nBCduv*i,
				d_dst + planeSize + nBCduv*i, nBCduv, cudaMemcpyDeviceToHost, pObj->streams[i]);
		ret = cudaMemcpyAsync(dst.data + planeSize + (planeSize>>2) + nBCduv*i,
				d_dst + planeSize + (planeSize>>2) + nBCduv*i, nBCduv, cudaMemcpyDeviceToHost, pObj->streams[i]);
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaStreamSynchronize(pObj->streams[i]);
	}

	return ret;
}

static cudaError_t cuConvert_grayToI420AndZoomxAndOsd_async(PCUOBJ pObj, Mat src, Mat osd, Mat &dst, int zoomx, int colorYUV)
{
	cudaError_t ret = cudaSuccess;
	int i, width = src.cols, height = src.rows;
	size_t planeSize = width * height;
	int yColor = GETYVAL(colorYUV);
	int uColor = GETUVAL(colorYUV);
	int vColor = GETVVAL(colorYUV);

	pObj->imem = 0;
	unsigned char *d_src = pObj->d_src;
	unsigned char *d_dst = pObj->d_mem[pObj->imem];
	unsigned char *d_osd = pObj->d_osd;

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	unsigned int nBCs = (src.rows * src.cols * src.channels())/STREAM_CNT;
	unsigned int nBCo = (osd.rows * osd.cols * osd.channels())/STREAM_CNT;
	unsigned int nBCd = (src.rows * src.cols)/STREAM_CNT;//I420
	unsigned int nBCduv = (src.rows/2) * ((src.cols/2)/STREAM_CNT);

	if(zoomx<=1)
	{
		for(i = 0; i<STREAM_CNT; i++){
			ret = cudaMemcpyAsync(d_src + nBCs*i,
					src.data + nBCs*i, nBCs, cudaMemcpyHostToDevice, pObj->streams[i]);
		}
		for(i = 0; i<STREAM_CNT; i++){
			ret = cudaMemcpyAsync(d_osd + nBCo*i,
					osd.data + nBCo*i, nBCo, cudaMemcpyHostToDevice, pObj->streams[i]);
		}
		for(i = 0; i<STREAM_CNT; i++){
			gpuConvertGRAYtoI420AndOsd_kernel<<<numBlocks, blockSize, 0, pObj->streams[i]>>>(
					d_osd + nBCo*i, d_src + nBCs*i, d_dst + nBCd*i,
					d_dst+planeSize+nBCduv*i,
					d_dst+planeSize+(planeSize>>2)+nBCduv*i,
					width, height/STREAM_CNT, yColor, uColor, vColor);
		}
	}
	else
	{
		ret = cudaMemcpyAsync(d_src + planeSize*src.channels()/4,
				src.data + planeSize*src.channels()/4, planeSize*src.channels()/4,
				cudaMemcpyHostToDevice, pObj->streams[0]);
		ret = cudaMemcpyAsync(d_src + planeSize*src.channels()/2,
				src.data + planeSize*src.channels()/2, planeSize*src.channels()/4,
				cudaMemcpyHostToDevice, pObj->streams[1]);
		for(i = 0; i<STREAM_CNT; i++){
			ret = cudaMemcpyAsync(d_osd + nBCo*i,
					osd.data + nBCo*i, nBCo, cudaMemcpyHostToDevice, pObj->streams[i]);
		}
		for(i = 0; i<STREAM_CNT; i++){
			gpuConvertGRAYtoI420AndZoomxAndOsd_async_kernel<<<numBlocks, blockSize, 0, pObj->streams[i]>>>(
					d_osd + nBCo*i,
					d_src+(height/2-(height/(zoomx<<1)))*width+(width/2-(width/(zoomx<<1))),
					d_dst+nBCd*i,
					d_dst+planeSize+nBCduv*i,
					d_dst+planeSize+(planeSize>>2)+nBCduv*i,
					width, height/STREAM_CNT, height*i/STREAM_CNT, zoomx>>1,yColor, uColor, vColor);
		}
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaMemcpyAsync(dst.data + nBCd*i,
				d_dst + nBCd*i, nBCd, cudaMemcpyDeviceToHost, pObj->streams[i]);
		ret = cudaMemcpyAsync(dst.data + planeSize + nBCduv*i,
				d_dst + planeSize + nBCduv*i, nBCduv, cudaMemcpyDeviceToHost, pObj->streams[i]);
		ret = cudaMemcpyAsync(dst.data + planeSize + (planeSize>>2) + nBCduv*i,
				d_dst + planeSize + (planeSize>>2) + nBCduv*i, nBCduv, cudaMemcpyDeviceToHost, pObj->streams[i]);
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaStreamSynchronize(pObj->streams[i]);
	}

	return ret;
}

static cudaError_t cuConvert_grayToI420AndZoomx_async(PCUOBJ pObj, Mat src, Mat &dst, int zoomx)
{
	cudaError_t ret = cudaSuccess;
	int i, width = src.cols, height = src.rows;
	size_t planeSize = width * height;

	pObj->imem = 0;
	unsigned char *d_src = pObj->d_src;
	unsigned char *d_dst = pObj->d_mem[pObj->imem];

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	unsigned int nBCs = (src.rows * src.cols * src.channels())/STREAM_CNT;
	unsigned int nBCd = (src.rows * src.cols)/STREAM_CNT;//I420
	unsigned int nBCduv = (src.rows/2) * ((src.cols/2)/STREAM_CNT);

	if(zoomx<=1)
	{
		for(i = 0; i<STREAM_CNT; i++){
			ret = cudaMemcpyAsync(d_src + nBCs*i,
					src.data + nBCs*i, nBCs, cudaMemcpyHostToDevice, pObj->streams[i]);
		}
		for(i = 0; i<STREAM_CNT; i++){
			gpuConvertGRAYtoI420_kernel<<<numBlocks, blockSize, 0, pObj->streams[i]>>>(
					d_src + nBCs*i, d_dst + nBCd*i,
					d_dst+planeSize+nBCduv*i,
					d_dst+planeSize+(planeSize>>2)+nBCduv*i,
					width, height/STREAM_CNT);
		}
	}
	else
	{
		ret = cudaMemcpyAsync(d_src + planeSize*src.channels()/4,
				src.data + planeSize*src.channels()/4, planeSize*src.channels()/4,
				cudaMemcpyHostToDevice, pObj->streams[0]);
		ret = cudaMemcpyAsync(d_src + planeSize*src.channels()/2,
				src.data + planeSize*src.channels()/2, planeSize*src.channels()/4,
				cudaMemcpyHostToDevice, pObj->streams[1]);
		for(i = 0; i<STREAM_CNT; i++){
			gpuConvertGRAYtoI420AndZoomx_async_kernel<<<numBlocks, blockSize, 0, pObj->streams[i]>>>(
					d_src+(height/2-(height/(zoomx<<1)))*width+(width/2-(width/(zoomx<<1))),
					d_dst+nBCd*i,
					d_dst+planeSize+nBCduv*i,
					d_dst+planeSize+(planeSize>>2)+nBCduv*i,
					width, height/STREAM_CNT, height*i/STREAM_CNT, zoomx>>1);
		}
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaMemcpyAsync(dst.data + nBCd*i,
				d_dst + nBCd*i, nBCd, cudaMemcpyDeviceToHost, pObj->streams[i]);
		ret = cudaMemcpyAsync(dst.data + planeSize + nBCduv*i,
				d_dst + planeSize + nBCduv*i, nBCduv, cudaMemcpyDeviceToHost, pObj->streams[i]);
		ret = cudaMemcpyAsync(dst.data + planeSize + (planeSize>>2) + nBCduv*i,
				d_dst + planeSize + (planeSize>>2) + nBCduv*i, nBCduv, cudaMemcpyDeviceToHost, pObj->streams[i]);
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaStreamSynchronize(pObj->streams[i]);
	}

	return ret;
}

/*
#define DEBUGTM_begin(c)	float elapsedTime;{	\
	if(c==chId){ret = cudaEventRecord(gObjs[chId].start, gObjs[chId].streams[0]);}	\
}
#define DEBUGTM_end(c){	\
	if(c==chId){	\
		ret = cudaEventRecord(gObjs[chId].stop, gObjs[chId].streams[0]);	\
		ret = cudaEventElapsedTime(&elapsedTime, gObjs[chId].start, gObjs[chId].stop);	\
		gObjs[chId].count ++;	\
		gObjs[chId].elapsedTimeMax = max(gObjs[chId].elapsedTimeMax, elapsedTime);	\
		gObjs[chId].elapsedTimeMin = min(gObjs[chId].elapsedTimeMin, elapsedTime);	\
		gObjs[chId].elapsedTimeSum += elapsedTime;	\
		if(gObjs[chId].count == 1000){	\
			printf("\n%s: ch%d Time taken %3.1f(%3.1f, %3.1f, %3.1f) ms", __func__, chId,	\
					elapsedTime, gObjs[chId].elapsedTimeMax, gObjs[chId].elapsedTimeMin, gObjs[chId].elapsedTimeSum/gObjs[chId].count);	\
			gObjs[chId].elapsedTimeMax = 0.0f; gObjs[chId].elapsedTimeMin = 1000.0f; gObjs[chId].elapsedTimeSum = 0.0f;	\
			gObjs[chId].count = 0;	\
		}	\
	}	\
}
*/

__inline__ void elapsedTimeCals(int chId, const char *func)
{
	float elapsedTime = (gObjs[chId].stampEnd - gObjs[chId].stampBegin)*1000.f/getTickFrequency();
	gObjs[chId].count ++;
	gObjs[chId].elapsedTimeMax = max(gObjs[chId].elapsedTimeMax, elapsedTime);
	gObjs[chId].elapsedTimeMin = min(gObjs[chId].elapsedTimeMin, elapsedTime);
	gObjs[chId].elapsedTimeSum += elapsedTime;
	if(gObjs[chId].count == 30*10){
		//printf("\n[%d]%s: ch%d Time taken %3.1f(%3.1f, %3.1f, %3.1f) ms", OSA_getCurTimeInMsec(), func, chId,
		//		elapsedTime, gObjs[chId].elapsedTimeMax, gObjs[chId].elapsedTimeMin, gObjs[chId].elapsedTimeSum/gObjs[chId].count);
		printf("\n[%d]%s: ch%d Time taken %3.1f(%3.1f, %3.1f, %3.1f) ms", OSA_getCurTimeInMsec(), func, chId,
				elapsedTime, gObjs[chId].elapsedTimeMax, gObjs[chId].elapsedTimeMin,
				(gObjs[chId].elapsedTimeSum-gObjs[chId].elapsedTimeMax-gObjs[chId].elapsedTimeMin)/(gObjs[chId].count-2));
		fflush(stdout);
		gObjs[chId].elapsedTimeMax = 0.0f; gObjs[chId].elapsedTimeMin = 1000.0f; gObjs[chId].elapsedTimeSum = 0.0f;
		gObjs[chId].count = 0;
	}
}

cudaError_t cuConvert(int chId, Mat src, Mat osd, Mat &dst, int zoomx, int colorYUV)
{
	LOCK;
	cudaError_t ret = cudaSuccess;

	gObjs[chId].stampBegin = getTickCount();

	if(src.channels() == 1){
		//cuConvert_grayToI420andOsd(&gObjs[chId], src, osd, dst, colorYUV);
		cuConvert_grayToI420AndZoomxAndOsd(&gObjs[chId], src, osd, dst, zoomx, colorYUV);
	}else{
		//cuConvert_yuyvToI420andOsd(&gObjs[chId], src, osd, dst, colorYUV);
		cuConvert_yuyvToI420AndZoomxAndOsd(&gObjs[chId], src, osd, dst, zoomx, colorYUV);
	}
	if(ret != cudaSuccess){
		printf("%s(%i)  : cudaGetLastError() CUDA error: %d\n", __FILE__, __LINE__, (int)cudaGetLastError());
	}
	//ret = cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	gObjs[chId].stampEnd = getTickCount();
	elapsedTimeCals(chId, __func__);

	UNLOCK;
	return ret;
}
cudaError_t cuConvertEnh(int chId, cv::Mat src, cv::Mat osd, cv::Mat &dst, int zoomx, int colorYUV)
{
	LOCK;
	cudaError_t ret = cudaSuccess;
	gObjs[chId].stampBegin = getTickCount();

	if(src.channels() == 1){
		//cuConvert_grayToI420andOsd(&gObjs[chId], src, osd, dst, colorYUV);
		cuConvert_grayToI420AndZoomxAndOsd(&gObjs[chId], src, osd, dst, zoomx, colorYUV);
	}else{
		//cuConvert_yuyvToI420andOsd(&gObjs[chId], src, osd, dst, colorYUV);
		cuConvert_yuyvToI420AndZoomxAndOsd(&gObjs[chId], src, osd, dst, zoomx, colorYUV);
	}

	Mat enhSrc(src.rows, src.cols,CV_8UC1, gObjs[chId].d_mem[gObjs[chId].imem]);
	Mat enhDst(src.rows, src.cols,CV_8UC1, gObjs[chId].d_mem[gObjs[chId].imem+1]);
	cuClahe(enhSrc, enhDst, (src.cols==1920)?8:8, (src.cols==1920)?4:8);
	int planeSize = src.rows*src.cols;
	ret = cudaMemcpy(gObjs[chId].d_mem[gObjs[chId].imem+1]+planeSize,
			gObjs[chId].d_mem[gObjs[chId].imem]+planeSize, planeSize>>1, cudaMemcpyDeviceToDevice);
	ret = cudaMemcpy(dst.data,gObjs[chId].d_mem[gObjs[chId].imem+1], planeSize, cudaMemcpyDeviceToHost);
	gObjs[chId].imem++;

	if(ret != cudaSuccess){
		printf("%s(%i)  : cudaGetLastError() CUDA error: %d\n", __FILE__, __LINE__, (int)cudaGetLastError());
	}
	//ret = cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	gObjs[chId].stampEnd = getTickCount();
	elapsedTimeCals(chId, __func__);

	UNLOCK;
	return ret;
}

cudaError_t cuConvert_async(int chId, Mat src, Mat osd, Mat &dst, int zoomx, int colorYUV)
{
	LOCK;
	cudaError_t ret = cudaSuccess;
	gObjs[chId].stampBegin = getTickCount();
	if(src.channels() == 1){
		ret = cuConvert_grayToI420AndZoomxAndOsd_async(&gObjs[chId], src, osd, dst, zoomx, colorYUV);
	}else{
		ret = cuConvert_yuyvToI420AndZoomxAndOsd_async(&gObjs[chId], src, osd, dst, zoomx, colorYUV);
	}
	if(ret != cudaSuccess){
		printf("%s(%i)  : cudaGetLastError() CUDA error: %d\n", __FILE__, __LINE__, (int)cudaGetLastError());
	}
	//ret = cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	gObjs[chId].stampEnd = getTickCount();
	elapsedTimeCals(chId, __func__);

	UNLOCK;
	return ret;
}

cudaError_t cuConvert_async(int chId, Mat src, Mat &dst, int zoomx)
{
	LOCK;
	cudaError_t ret = cudaSuccess;
	gObjs[chId].stampBegin = getTickCount();
	if(src.channels() == 1){
		ret = cuConvert_grayToI420AndZoomx_async(&gObjs[chId], src, dst, zoomx);
	}else{
		ret = cuConvert_yuyvToI420AndZoomx_async(&gObjs[chId], src, dst, zoomx);
	}
	if(ret != cudaSuccess){
		printf("%s(%i)  : cudaGetLastError() CUDA error: %d\n", __FILE__, __LINE__, (int)cudaGetLastError());
	}
	//ret = cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	gObjs[chId].stampEnd = getTickCount();
	elapsedTimeCals(chId, __func__);

	UNLOCK;
	return ret;
}

cudaError_t cuConvertEnh_async(int chId, Mat src, Mat osd, Mat &dst, int zoomx, int colorYUV)
{
	LOCK;
	cudaError_t ret = cudaSuccess;
	gObjs[chId].stampBegin = getTickCount();

	if(src.channels() == 1){
		ret = cuConvert_grayToI420AndZoomxAndOsd_async(&gObjs[chId], src, osd, dst, zoomx, colorYUV);
	}else{
		ret = cuConvert_yuyvToI420AndZoomxAndOsd_async(&gObjs[chId], src, osd, dst, zoomx, colorYUV);
	}

	Mat enhSrc(src.rows, src.cols,CV_8UC1, gObjs[chId].d_mem[gObjs[chId].imem]);
	Mat enhDst(src.rows, src.cols,CV_8UC1, gObjs[chId].d_mem[gObjs[chId].imem+1]);
	cuClahe(enhSrc, enhDst, (src.cols==1920)?8:8, (src.cols==1920)?4:8);
	int planeSize = src.rows*src.cols;
	ret = cudaMemcpy(gObjs[chId].d_mem[gObjs[chId].imem+1]+planeSize,
			gObjs[chId].d_mem[gObjs[chId].imem]+planeSize, planeSize>>1, cudaMemcpyDeviceToDevice);
	ret = cudaMemcpy(dst.data,gObjs[chId].d_mem[gObjs[chId].imem+1], planeSize, cudaMemcpyDeviceToHost);
	gObjs[chId].imem++;

	if(ret != cudaSuccess){
		printf("%s(%i)  : cudaGetLastError() CUDA error: %d\n", __FILE__, __LINE__, (int)cudaGetLastError());
	}
	//ret = cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	gObjs[chId].stampEnd = getTickCount();
	elapsedTimeCals(chId, __func__);

	UNLOCK;
	return ret;
}

cudaError_t cuConvertEnh_async(int chId, Mat src, Mat &dst, int zoomx)
{
	LOCK;
	cudaError_t ret = cudaSuccess;
	gObjs[chId].stampBegin = getTickCount();

	if(src.channels() == 1){
		ret = cuConvert_grayToI420AndZoomx_async(&gObjs[chId], src, dst, zoomx);
	}else{
		ret = cuConvert_yuyvToI420AndZoomx_async(&gObjs[chId], src, dst, zoomx);
	}

	Mat enhSrc(src.rows, src.cols,CV_8UC1, gObjs[chId].d_mem[gObjs[chId].imem]);
	Mat enhDst(src.rows, src.cols,CV_8UC1, gObjs[chId].d_mem[gObjs[chId].imem+1]);
	cuClahe(enhSrc, enhDst, (src.cols==1920)?8:8, (src.cols==1920)?4:8);
	int planeSize = src.rows*src.cols;
	ret = cudaMemcpy(gObjs[chId].d_mem[gObjs[chId].imem+1]+planeSize,
			gObjs[chId].d_mem[gObjs[chId].imem]+planeSize, planeSize>>1, cudaMemcpyDeviceToDevice);
	ret = cudaMemcpy(dst.data,gObjs[chId].d_mem[gObjs[chId].imem+1], planeSize, cudaMemcpyDeviceToHost);
	gObjs[chId].imem++;

	if(ret != cudaSuccess){
		printf("%s(%i)  : cudaGetLastError() CUDA error: %d\n", __FILE__, __LINE__, (int)cudaGetLastError());
	}
	//ret = cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	gObjs[chId].stampEnd = getTickCount();
	elapsedTimeCals(chId, __func__);

	UNLOCK;
	return ret;
}

static cudaError_t cuConvert_yuv2bgr_yuyv_async(PCUOBJ pObj, Mat src, Mat &dst, int flag)
{
	cudaError_t ret = cudaSuccess;
	OSA_assert(dst.data != NULL);
	int i, width = src.cols, height = src.rows;

	unsigned char *d_src = pObj->d_src;
	unsigned char *d_dst = (flag == CUT_FLAG_devAlloc) ? dst.data : pObj->d_mem[0];

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	unsigned int nBCs = (src.rows * src.cols * src.channels())/STREAM_CNT;
	unsigned int nBCd = (dst.rows * dst.cols * dst.channels())/STREAM_CNT;

	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaMemcpyAsync(d_src + nBCs*i,
				src.data + nBCs*i, nBCs, cudaMemcpyHostToDevice, pObj->streams[i]);
	}
	for(i = 0; i<STREAM_CNT; i++){
		gpuConvertYUYVtoBGR_kernel<<<numBlocks, blockSize, 0, pObj->streams[i]>>>(
				d_src + nBCs*i, d_dst + nBCd*i,
				width, height/STREAM_CNT);
	}
	if(dst.data != d_dst)
	{
		for(i = 0; i<STREAM_CNT; i++){
			ret = cudaMemcpyAsync(dst.data + nBCd*i,
					d_dst + nBCd*i, nBCd, cudaMemcpyDeviceToHost, pObj->streams[i]);
		}
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaStreamSynchronize(pObj->streams[i]);
	}

	//printf(" dst.data = %p d_dst = %p\n", dst.data, d_dst);

	return ret;
}


static cudaError_t cuConvert_yuv2bgr_uyvy_async(PCUOBJ pObj, Mat src, Mat &dst, int flag)
{
	cudaError_t ret = cudaSuccess;
	OSA_assert(dst.data != NULL);
	int i, width = src.cols, height = src.rows;

	unsigned char *d_src = pObj->d_src;
	unsigned char *d_dst = (flag == CUT_FLAG_devAlloc) ? dst.data : pObj->d_mem[0];

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;
	unsigned int nBCs = (src.rows * src.cols * src.channels())/STREAM_CNT;
	unsigned int nBCd = (dst.rows * dst.cols * dst.channels())/STREAM_CNT;

	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaMemcpyAsync(d_src + nBCs*i,
				src.data + nBCs*i, nBCs, cudaMemcpyHostToDevice, pObj->streams[i]);
	}
	for(i = 0; i<STREAM_CNT; i++){
		gpuConvertUYVYtoBGR_kernel<<<numBlocks, blockSize, 0, pObj->streams[i]>>>(
				d_src + nBCs*i, d_dst + nBCd*i,
				width, height/STREAM_CNT);
	}
	if(dst.data != d_dst)
	{
		for(i = 0; i<STREAM_CNT; i++){
			ret = cudaMemcpyAsync(dst.data + nBCd*i,
					d_dst + nBCd*i, nBCd, cudaMemcpyDeviceToHost, pObj->streams[i]);
		}
	}
	for(i = 0; i<STREAM_CNT; i++){
		ret = cudaStreamSynchronize(pObj->streams[i]);
	}

	//printf(" dst.data = %p d_dst = %p\n", dst.data, d_dst);

	return ret;
}



static cudaError_t cunppConvert_yuv2bgr_yuyv_async(PCUOBJ pObj, const Mat& src, Mat &dst, int flag)
{
	cudaError_t ret = cudaSuccess;
	OSA_assert(dst.data != NULL);
	int width = src.cols, height = src.rows;
	unsigned char *d_src = pObj->d_src;
	unsigned char *d_dst = (flag == CUT_FLAG_devAlloc) ? dst.data : pObj->d_mem[0];

	NppiSize oSizeROI;
	oSizeROI.width = width;
	oSizeROI.height = height;
	//npp::ImageCPU_8u_C2 oHostSrc;
	//npp::ImageNPP_8u_C2 oDeviceSrc(oHostSrc);
	cudaMemcpy(d_src, src.data, width*2, cudaMemcpyHostToDevice);
	NppStatus stat = nppiYCbCr422ToBGR_8u_C2C3R(d_src, width*2, d_dst, width*3, oSizeROI);
	if(stat != NPP_NO_ERROR){
		OSA_printf("%s: stat = %d", __func__, stat);
		OSA_assert(stat == NPP_NO_ERROR);
	}

	if(dst.data != d_dst)
	{
		cudaMemcpy(dst.data, d_dst, width*3, cudaMemcpyDeviceToHost);
	}

	return ret;
}

cudaError_t cuConvert_yuv2bgr_yuyv_async(int chId, Mat src, Mat &dst, int flag)
{
	LOCK;
	cudaError_t ret = cudaSuccess;
	gObjs[chId].stampBegin = getTickCount();
	ret = cuConvert_yuv2bgr_yuyv_async(&gObjs[chId], src, dst, flag);
	//ret = cunppConvert_yuv2bgr_yuyv_async(&gObjs[chId], src, dst, flag);
	if(ret != cudaSuccess){
		printf("%s(%i)  : cudaGetLastError() CUDA error: %d\n", __FILE__, __LINE__, (int)cudaGetLastError());
	}
	//ret = cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	gObjs[chId].stampEnd = getTickCount();
	elapsedTimeCals(chId, __func__);
	UNLOCK;
	return ret;
}

cudaError_t cuConvert_yuv2bgr_uyvy_async(int chId, Mat src, Mat &dst, int flag)
{
	LOCK;
	cudaError_t ret = cudaSuccess;
	gObjs[chId].stampBegin = getTickCount();
	ret = cuConvert_yuv2bgr_uyvy_async(&gObjs[chId], src, dst, flag);
	if(ret != cudaSuccess){
		printf("%s(%i)  : cudaGetLastError() CUDA error: %d\n", __FILE__, __LINE__, (int)cudaGetLastError());
	}
	//ret = cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	gObjs[chId].stampEnd = getTickCount();
	elapsedTimeCals(chId, __func__);
	UNLOCK;
	return ret;
}

cudaError_t cuConvertEnh_yuv2bgr_yuyv_async(int chId, Mat src, Mat &dst, int flag)
{
	LOCK;
	cudaError_t ret = cudaSuccess;
	gObjs[chId].stampBegin = getTickCount();
	Mat enhSrc(src.rows, src.cols,CV_8UC3, gObjs[chId].d_mem[0]);
	ret = cuConvert_yuv2bgr_yuyv_async(&gObjs[chId], src, enhSrc, CUT_FLAG_devAlloc);
	if(ret != cudaSuccess){
		printf("%s(%i)  : cudaGetLastError() CUDA error: %d\n", __FILE__, __LINE__, (int)cudaGetLastError());
	}
	//ret = cudaMemcpy(dst.data,enhSrc.data, src.rows*src.cols*dst.channels(),cudaMemcpyDeviceToDevice);
	cuClahe(enhSrc, dst, (src.cols==1920)?8:8, (src.cols==1920)?4:8, 3.0f);
	//cuTemporalFilter(dst, dst);
	//ret = cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	gObjs[chId].stampEnd = getTickCount();
	elapsedTimeCals(chId, __func__);
	UNLOCK;
	return ret;
}

cudaError_t cuConvertEnh_yuv2bgr_uyvy_async(int chId, Mat src, Mat &dst, int flag)
{
	LOCK;
	cudaError_t ret = cudaSuccess;
	gObjs[chId].stampBegin = getTickCount();
	Mat enhSrc(src.rows, src.cols,CV_8UC3, gObjs[chId].d_mem[0]);
	ret = cuConvert_yuv2bgr_uyvy_async(&gObjs[chId], src, enhSrc, CUT_FLAG_devAlloc);
	if(ret != cudaSuccess){
		printf("%s(%i)  : cudaGetLastError() CUDA error: %d\n", __FILE__, __LINE__, (int)cudaGetLastError());
	}
	//ret = cudaMemcpy(dst.data,enhSrc.data, src.rows*src.cols*dst.channels(),cudaMemcpyDeviceToDevice);
	cuClahe(enhSrc, dst, (src.cols==1920)?8:8, (src.cols==1920)?4:8, 3.0f);
	//cuTemporalFilter(dst, dst);
	//ret = cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	gObjs[chId].stampEnd = getTickCount();
	elapsedTimeCals(chId, __func__);
	UNLOCK;
	return ret;
}

cudaError_t cuConvertEnh_gray(int chId, cv::Mat src, cv::Mat &dst, int flag)
{
	LOCK;
	cudaError_t ret = cudaSuccess;
	gObjs[chId].stampBegin = getTickCount();
	Mat enhSrc(src.rows, src.cols,CV_8UC1, gObjs[chId].d_src);
	ret = cudaMemcpy(enhSrc.data, src.data, src.rows*src.cols, cudaMemcpyHostToDevice);
	OSA_assert(ret == cudaSuccess);
	cuClahe(enhSrc, dst, (src.cols==1920)?8:8, (src.cols==1920)?4:8);
	//ret = cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	gObjs[chId].stampEnd = getTickCount();
	elapsedTimeCals(chId, __func__);
	UNLOCK;
	return ret;
}

/*****************************************************************
 *
 *  cutColor yuv2bgr i420
 */
__global__ void gpuConvertI420toBGR_kernel(
		unsigned char *src_y, unsigned char *src_u, unsigned char *src_v, unsigned char *dst,
		unsigned int width, unsigned int height)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx*2 >= width) {
		return;
	}
	int height_half = (height>>1);
	int width_half = (width>>1);

	for (int i = 0; i < height_half; ++i) {
		int cb = src_u[i*width_half+idx];
		int cr = src_v[i*width_half+idx];
		int b = DESCALE((cb - 128)*COEFFS_3, 14);
		int g = DESCALE((cb - 128)*COEFFS_2 + (cr - 128)*COEFFS_1, 14);
		int r = DESCALE((cr - 128)*COEFFS_0, 14);
		int y00 = src_y[(i*2+0)*width+idx*2+0];
		int y01 = src_y[(i*2+0)*width+idx*2+1];
		int y10 = src_y[(i*2+1)*width+idx*2+0];
		int y11 = src_y[(i*2+1)*width+idx*2+1];

		dst[(i*2+0)*width*3+idx*6+0] = clip(0, 255, y00 + b);//B
		dst[(i*2+0)*width*3+idx*6+1] = clip(0, 255, y00 + g);//G
		dst[(i*2+0)*width*3+idx*6+2] = clip(0, 255, y00 + r);//R
		dst[(i*2+0)*width*3+idx*6+3] = clip(0, 255, y01 + b);//B
		dst[(i*2+0)*width*3+idx*6+4] = clip(0, 255, y01 + g);//G
		dst[(i*2+0)*width*3+idx*6+5] = clip(0, 255, y01 + r);//R
		dst[(i*2+1)*width*3+idx*6+0] = clip(0, 255, y10 + b);//B
		dst[(i*2+1)*width*3+idx*6+1] = clip(0, 255, y10 + g);//G
		dst[(i*2+1)*width*3+idx*6+2] = clip(0, 255, y10 + r);//R
		dst[(i*2+1)*width*3+idx*6+3] = clip(0, 255, y11 + b);//B
		dst[(i*2+1)*width*3+idx*6+4] = clip(0, 255, y11 + g);//G
		dst[(i*2+1)*width*3+idx*6+5] = clip(0, 255, y11 + r);//R
	}
}
static cudaError_t cuConvert_yuv2bgr_i420(PCUOBJ pObj, Mat &dst)
{
	cudaError_t ret = cudaSuccess;
	int width = dst.cols, height = dst.rows;

	unsigned char *d_src = pObj->d_mem[pObj->imem];
	unsigned char *d_dst = dst.data;

	unsigned int blockSize = 1024;
	unsigned int numBlocks = (width / 2 + blockSize - 1) / blockSize;

	gpuConvertI420toBGR_kernel<<<numBlocks, blockSize>>>(
			d_src, d_src+width*height, d_src+width*height+((width>>1)*(height>>1)),d_dst,
			width, height);
	cudaStreamSynchronize(NULL);

	return ret;
}

cudaError_t cuConvertConn_yuv2bgr_i420(int chId, Mat &dst, int flag)
{
	LOCK;
	cudaError_t ret = cudaSuccess;
	ret = cuConvert_yuv2bgr_i420(&gObjs[chId], dst);
	if(ret != cudaSuccess){
		printf("%s(%i)  : cudaGetLastError() CUDA error: %d\n", __FILE__, __LINE__, (int)cudaGetLastError());
	}
	//ret = cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	UNLOCK;
	return ret;
}


