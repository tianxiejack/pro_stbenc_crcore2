/*
 * crvxMotionComp_lib.hpp
 *
 *  Created on: May 5, 2019
 *      Author: ubuntu
 */

#ifndef CRVX_MOTION_COMPENSATION_LIB_HPP_
#define CRVX_MOTION_COMPENSATION_LIB_HPP_

#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>
#ifdef _CUDA_
#include <opencv2/core/cuda.hpp>
#	define cvGpuMat cv::cuda::GpuMat
#else
#include <opencv2/gpu/gpu.hpp>
#	define cvGpuMat cv::gpu::GpuMat
#endif

class MotionComp
{
public:

	enum MotionModel
	{
	    MM_TRANSLATION = 0,
	    MM_TRANSLATION_AND_SCALE = 1,
	    MM_ROTATION = 2,
	    MM_RIGID = 3,
	    MM_SIMILARITY = 4,
	    MM_AFFINE = 5,
	    MM_HOMOGRAPHY = 6,
	    MM_STABILIZER = 7,
	    MM_UNKNOWN = 8
	};

	struct InitParams{
		cudaStream_t stream;
		double noise_cov;
		bool bBorderTransparent;
		//cv::Scalar borderValue;
		float cropMargin;
		bool bCropMarginScale;
		bool bPreprocess;

		InitParams(){
			stream = 0;
			noise_cov = 1E-4;
			bBorderTransparent = true;
			//borderValue=cv::Scalar();
			cropMargin = -1.0f;
			bCropMarginScale = false;
			bPreprocess = false;
		}
	};

	virtual int init(const cvGpuMat& frame, const InitParams& params = InitParams(), const cvGpuMat& mask = cvGpuMat()) = 0;

	virtual int process(const cvGpuMat& frame, const cvGpuMat& mask = cvGpuMat()) = 0;

	virtual cvGpuMat getOut() = 0;

	virtual int getFeatures(std::vector<cv::Point2f>& points) = 0;

	virtual cv::Matx33f getTransform() = 0;

	virtual cv::Point2f ptFrom(const cv::Point2f& pt) = 0;

	virtual cv::Point2f ptTo(const cv::Point2f& pt) = 0;

    virtual void printPerfs() const = 0;

    virtual ~MotionComp() {}

    static MotionComp* create(MotionModel mm = MM_HOMOGRAPHY, bool bStabilizer = false);
};



#endif /* CRVX_MOTION_COMPENSATION_LIB_HPP_ */
