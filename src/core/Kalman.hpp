#ifndef  _KALMAN_001_H_
#define  _KALMAN_001_H_
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


namespace crcore
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct _Kalman_t{
	int MP;  //number of measure vector dimensions
	int DP;  //number of state   vector dimensions
	int CP;  //number of control vector dimensions
	
	double* state_pre;            //[DP * 1] 
	double* state_post;           //[DP * 1] 
	double* measure_param;        //[MP * 1]
	double* transition_matrix;    //[DP * DP] 
	double* control_matrix;       //[DP * CP] if (CP > 0) 
	double* measurement_matrix;   //[MP * DP]
	double* process_noise_cov;    //[DP * DP]
	double* measurement_noise_cov;//[MP * MP]
	double* error_cov_pre;        //[DP * DP]
	double* error_cov_post;       //[DP * DP]
	double* gain;                 //[DP * MP]

	double   deltat; // ��Ƶ����ʱ����
	int     m_bInited;

	//temporary  variable 
	double *B_Uk;		 //[ DP * 1  ]
	double *A_Pk;		 //[ DP * DP ]
	double *A_T;		 //[ DP * DP ]
	double *APA_T;		 //[ DP * DP ]

	double *H_T;         //[ DP * MP ]
	double *Pk_Ht;       //[ DP * MP ]
	double *Pk_Ht_R;     //[ MP * MP ]
	double *Pk_Ht_R_Inv; //[ MP * MP ]
	double *H_Xk;        //[ MP * 1  ]
	double *Kk_H_Xk;     //[ DP * 1  ]
	double *H_Pk;        //[ MP * DP ]
	double *Kk_H_Pk;     //[ DP * DP ]

}tKalman, *HKalman;

class CKalmanFilter
{
	HKalman KalmanInit();
	void KalmanPredict(double * control );
	void KalmanCorrect(double * measurement );
public:
	CKalmanFilter();
	~CKalmanFilter();

	HKalman  KalmanOpen(int D, int M, int C );
	void KalmanClose();
	void KalmanInitParam(float theta, float delta_x, float delta_y, double DeltaT);
	int Kalman(double *measure, double *control);
	HKalman m_hKalman;

};
};//namespace crcore

#endif
