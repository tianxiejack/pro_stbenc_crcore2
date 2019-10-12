
/*
 * PCTracker.h
 *
 *  Created on: 2014-5-9
 *      Author: Zhong
 */

#ifndef PCTRACKER_H_
#define PCTRACKER_H_

//typedef unsigned char bool;

typedef unsigned char UInt8;
typedef unsigned short UInt16;
typedef unsigned int UInt32;
typedef unsigned char Uint8;

//#ifndef bool
#define false 0
#define true 1
//#endif

#ifndef	UTILS_assert
#define UTILS_assert assert
#endif
#define Vps_printf printf

#ifndef __INLINE__
//#define __INLINE__ static
#define __INLINE__ static __inline
#endif

/************************************/
// here is same as alglink in trackLink.h

//#define MAX_DIM	3
#define MAT_DIM	3
#define MAT_float 	1
#define MAT_u8		0
typedef struct _mat_t{
	int dtype; //0: u8, 1: float
	int size;
	int width, height;
	int channels;
	int step[MAT_DIM];
	union{
	float *data;
	unsigned char *data_u8;
	};
}IMG_MAT, IMG_MAT_FLOAT, IMG_MAT_UCHAR;

typedef struct
{
	int  width;
	int  height;
}UTC_SIZE;

typedef struct
{
	int  x;
	int  y;
	int  width;
	int  height;
}UTC_Rect;

typedef struct
{
	float  x;
	float  y;
	float  width;
	float  height;
}UTC_RECT_float;

typedef struct _track_acq_param{
	int axisX;
	int axisY;
	UTC_Rect rcWin;
}UTC_ACQ_param;

typedef struct _utctrack_obj{
	bool     m_bInited;

	int axisX;
	int axisY;
	UTC_Rect rcWin;

//	void *pParent;

}UTCTRACK_OBJ, *UTCTRACK_HANDLE;

typedef struct _utctrack_dynamic_param{
	float 	occlusion_thred;
	float	retry_acq_thred;

}UTC_DYN_PARAM;

typedef enum{
	tPLT_TST = 0,
	tPLT_WRK = 1,
}tPLT;

typedef enum{
	BoreSight_Big	= 0,
	BoreSight_Mid	= 1,
	BoreSight_Sm	= 2,
}BS_Type;

typedef struct _trk_sech_t{
	int	res_distance;
	int	res_area;
}TRK_SECH_RESTRAINT;

typedef struct{
	double	proc_noise_cov_xy;
	double 	proc_noise_cov_vxvy;
	double 	measure_noise_cov_xy;
}UKF_COV_PARAM;

typedef enum{
	DO_NOT_JUDGE	= 0x00,
	SCENE_MOVE_JUDGE,
	PLT_MOVE_JUDGE,
	SCENE_PLT_JUDGE,
	MAX_SECH_TYPE,
}TRAJ_SECH_TYPE;

////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif /* PCTRACKER_H_ */
