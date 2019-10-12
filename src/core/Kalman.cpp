//#include "stdafx.h"
//#include <windows.h>
#include "Kalman.hpp"
#include <assert.h>
#include <math.h>
#include <memory.h>

using namespace crcore;

CKalmanFilter::CKalmanFilter():m_hKalman(NULL)
{

}
CKalmanFilter::~CKalmanFilter()
{

}

static void MatrixMultiply(double* A, double* B, int m, int p, int n, double* C)
{
	// A = input matrix (m x p)
	// B = input matrix (p x n)
	// m = number of rows in A
	// p = number of columns in A = number of rows in B
	// n = number of columns in B
	// C = output matrix = A*B (m x n)
	int i, j, k;
	for (i=0;i<m;i++)
	{
		for(j=0;j<n;j++)
		{
			C[n*i+j]=0;
			for (k=0;k<p;k++)
			{
				C[n*i+j]= C[n*i+j]+A[p*i+k]*B[n*k+j];
			}
		}
	}
}

static void MatrixAddition(double* A, double* B, int m, int n, double* C)
{
	// A = input matrix (m x n)
	// B = input matrix (m x n)
	// m = number of rows in A = number of rows in B
	// n = number of columns in A = number of columns in B
	// C = output matrix = A+B (m x n)
	int i, j;
	for (i=0;i<m;i++)
	{
		for(j=0;j<n;j++)
		{
			C[n*i+j]=A[n*i+j]+B[n*i+j];
		}
	}
}

static void MatrixSubtraction(double* A, double* B, int m, int n, double* C)
{
	// A = input matrix (m x n)
	// B = input matrix (m x n)
	// m = number of rows in A = number of rows in B
	// n = number of columns in A = number of columns in B
	// C = output matrix = A-B (m x n)
	int i, j;
	for (i=0;i<m;i++)
	{
		for(j=0;j<n;j++)
		{
			C[n*i+j]=A[n*i+j]-B[n*i+j];
		}
	}
}

static void MatrixTranspose(double* A, int m, int n, double* C)
{
	// A = input matrix (m x n)
	// m = number of rows in A
	// n = number of columns in A
	// C = output matrix = the transpose of A (n x m)
	int i, j;
	for (i=0;i<m;i++)
	{
		for(j=0;j<n;j++)
		{
			C[m*j+i]=A[n*i+j];
		}
	}
}

static int MatrixInversion(double* A, int n, double* AInverse)
{
	// A = input matrix (n x n)
	// n = dimension of A 
	// AInverse = inverted matrix (n x n)
	// This function inverts a matrix based on the Gauss Jordan method.
	// The function returns 1 on success, 0 on failure.
	int i, j, iPass, imx, icol, irow;
	double det, temp, pivot, factor;
/*	double* ac = (double*)calloc(n*n, sizeof(double));*///modified by zhong
	det = 1;
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			AInverse[n*i+j] = 0;
/*			ac[n*i+j] = A[n*i+j];*/
		}
		AInverse[n*i+i] = 1;
	}
	// The current pivot row is iPass.  
	// For each pass, first find the maximum element in the pivot column.
	for (iPass = 0; iPass < n; iPass++)
	{
		imx = iPass;
		for (irow = iPass; irow < n; irow++)
		{
			if (fabs(A[n*irow+iPass]) > fabs(A[n*imx+iPass]))
				imx = irow;
		}
		// Interchange the elements of row iPass and row imx in both A and AInverse.
		if (imx != iPass)
		{
			for (icol = 0; icol < n; icol++)
			{
				temp = AInverse[n*iPass+icol];
				AInverse[n*iPass+icol] = AInverse[n*imx+icol];
				AInverse[n*imx+icol] = temp;
				if (icol >= iPass)
				{
					temp = A[n*iPass+icol];
					A[n*iPass+icol] = A[n*imx+icol];
					A[n*imx+icol] = temp;
				}
			}
		}
		// The current pivot is now A[iPass][iPass].
		// The determinant is the product of the pivot elements.
		pivot = A[n*iPass+iPass];
		det = det * pivot;
		if (det == 0) 
		{
/*			free(ac);*/
			return 0;
		}
		for (icol = 0; icol < n; icol++)
		{	
			// Normalize the pivot row by dividing by the pivot element.
			AInverse[n*iPass+icol] = AInverse[n*iPass+icol] / pivot;
			if (icol >= iPass)
				A[n*iPass+icol] = A[n*iPass+icol] / pivot;
		}
		for (irow = 0; irow < n; irow++)
		{	
			// Add a multiple of the pivot row to each row.  The multiple factor 
			// is chosen so that the element of A on the pivot column is 0.
			if (irow != iPass)
				factor = A[n*irow+iPass];
			for (icol = 0; icol < n; icol++)
			{
				if (irow != iPass)
				{
					AInverse[n*irow+icol] -= factor * AInverse[n*iPass+icol];
					A[n*irow+icol] -= factor * A[n*iPass+icol];
				}
			}
		}
	}
/*	free(ac);*/
	return 1;
}

/*
	��ȫѡ��ԪGauss-Jordan����n��ʵ����A�������A^{-1}
	�������
	double * A��     ԭ����Ϊһ������
	int n��          ����ά��
	�������
	double * A��     ��õ������
	����ֵ��
	���ر��Ϊ0����ʾ�������죻���򷵻ط�0ֵ
*/
static int MatrixBrinv( double * A, int n)
{
	int * is, * js, i, j, k, l, u, v;
	double d,p;

	is = (int *)malloc( n*sizeof(int) );
	js = (int *)malloc( n*sizeof(int) );

	for ( k = 0; k < n; k++ )
	{ 
		d = 0.0;
		for ( i = k; i < n; i++ )
		{
			for ( j = k; j < n; j++ )
			{ 
				l = i*n+j;
				p = fabs(A[l]);
				if ( p > d )
				{ 
					d = p; is[k] = i; js[k] = j;
				}
			}
		}
		if ( d+1.0 == 1.0 ) /* ����Ϊ������ */
		{ 
			free( is ); 
			free( js ); 
			return( 0 );
		}
		if ( is[k] != k )
		{
			for ( j = 0; j < n; j++ )
			{ 
				u = k*n+j;
				v = is[k]*n+j;
				p = A[u]; A[u] = A[v]; A[v] = p;
			}
		}
		if ( js[k] != k )
		{
			for ( i = 0; i < n; i++ )
			{ 
				u = i*n+k;
				v = i*n+js[k];
				p = A[u]; A[u] = A[v]; A[v] = p;
			}
		}
		l = k*n+k;
		A[l] = 1.0/A[l];
		for ( j = 0; j < n; j++ )
		{
			if ( j != k )
			{ 
				u = k*n+j;
				A[u] = A[u]*A[l];
			}
		}
		for ( i = 0; i < n; i++ )
		{
			if ( i != k )
			{
				for ( j = 0; j < n; j++ )
				{
					if ( j != k )
					{ 
						u = i*n+j;
						A[u] = A[u] - A[i*n+k]*A[k*n+j];
					}
				}
			}
		}
		for ( i = 0; i < n; i++ )
		{
			if ( i != k )
			{ 
				u = i*n+k;
				A[u] = -A[u]*A[l];
			}
		}
	}
	for ( k = n-1; k >= 0; k-- )
	{ 
		if ( js[k] != k )
		{
			for ( j = 0; j <= n-1; j++ )
			{ 
				u = k*n+j;
				v = js[k]*n+j;
				p = A[u]; A[u] = A[v]; A[v] = p;
			}
		}
		if ( is[k] != k )
		{
			for ( i = 0; i < n; i++ )
			{ 
				u = i*n+k;
				v = i*n+is[k];
				p = A[u]; A[u] = A[v]; A[v] = p;
			}
		}
	}
	free( is );
	free( js );

	return(1);
}

static void MatrixCopy(double *A, double *B, int m, int n)
{
	memcpy(A, B, sizeof(double)*m*n);
}

HKalman CKalmanFilter::KalmanInit()
{
	HKalman pKalamObj = new tKalman;
	pKalamObj->state_pre = NULL;
	pKalamObj->state_post = NULL;
	pKalamObj->measure_param = NULL;
	pKalamObj->transition_matrix = NULL;
	pKalamObj->process_noise_cov = NULL;
	pKalamObj->measurement_matrix = NULL;
	pKalamObj->measurement_noise_cov = NULL;
	pKalamObj->error_cov_pre = NULL;
	pKalamObj->error_cov_post = NULL;
	pKalamObj->gain = NULL;
	pKalamObj->control_matrix = NULL;

	pKalamObj->B_Uk = NULL;
	pKalamObj->A_Pk = NULL;
	pKalamObj->A_T = NULL;
	pKalamObj->APA_T = NULL;

	pKalamObj->H_T = NULL;
	pKalamObj->Pk_Ht = NULL;
	pKalamObj->Pk_Ht_R = NULL;
	pKalamObj->Pk_Ht_R_Inv = NULL;
	pKalamObj->H_Xk = NULL;
	pKalamObj->Kk_H_Xk = NULL;
	pKalamObj->H_Pk = NULL;
	pKalamObj->Kk_H_Pk = NULL;
	pKalamObj->deltat = 0.04; // ��Ƶ����ʱ����

	pKalamObj->m_bInited = false;

	return pKalamObj;
}

HKalman  CKalmanFilter::KalmanOpen(int D, int M, int C )
{
	if(m_hKalman != NULL)
		KalmanClose();
	m_hKalman = KalmanInit();
	if(m_hKalman == NULL)
		return NULL;
	if( D <= 0 || M <= 0 ){
		//AfxMessageBox("state and measurement vectors must have positive number of dimensions! ");	
		m_hKalman->m_bInited = false;
		return NULL;
	}
	if( C < 0 ){
        //AfxMessageBox("No Control!");
		m_hKalman->m_bInited = false;
		return NULL;
	}
    m_hKalman->DP = D;
    m_hKalman->MP = M;
    m_hKalman->CP = C;

	if(m_hKalman->state_pre == NULL){
		m_hKalman->state_pre =  (double*)malloc(m_hKalman->DP * 1*sizeof(double));
		memset( m_hKalman->state_pre ,  0 , sizeof (*m_hKalman->state_pre) );
	}
	if(m_hKalman->state_post == NULL){
		m_hKalman->state_post = (double*)malloc(m_hKalman->DP * 1*sizeof(double));
		memset( m_hKalman->state_post , 0 , sizeof (*m_hKalman->state_post) );
	}
	if (m_hKalman->measure_param == NULL){
		m_hKalman->measure_param =  (double*)malloc(m_hKalman->MP * 1*sizeof(double));
		memset( m_hKalman->measure_param , 0 , sizeof (*m_hKalman->measure_param) );
	}
	if(m_hKalman->transition_matrix == NULL){
		m_hKalman->transition_matrix =  (double*)malloc(m_hKalman->DP * m_hKalman->DP*sizeof(double));
		memset( m_hKalman->transition_matrix , 0 , sizeof (*m_hKalman->transition_matrix) );
	}
	if(m_hKalman->process_noise_cov == NULL){
		m_hKalman->process_noise_cov = (double*)malloc(m_hKalman->DP * m_hKalman->DP*sizeof(double));
		memset( m_hKalman->process_noise_cov , 0 , sizeof (*m_hKalman->process_noise_cov) );
	}
	if(m_hKalman->measurement_matrix == NULL){
		m_hKalman->measurement_matrix = (double*)malloc(m_hKalman->MP * m_hKalman->DP*sizeof(double));
		memset( m_hKalman->measurement_matrix , 0 , sizeof (*m_hKalman->measurement_matrix) );
	}
	if(m_hKalman->measurement_noise_cov == NULL){
		m_hKalman->measurement_noise_cov =  (double*)malloc(m_hKalman->MP * m_hKalman->MP*sizeof(double));
		memset( m_hKalman->measurement_noise_cov , 0 , sizeof (*m_hKalman->measurement_noise_cov) );
	}
	if(m_hKalman->error_cov_pre == NULL){
		m_hKalman->error_cov_pre = (double*)malloc(m_hKalman->DP * m_hKalman->DP*sizeof(double));
		memset( m_hKalman->error_cov_pre , 0 , sizeof (*m_hKalman->error_cov_pre) );
	}
	if(m_hKalman->error_cov_post == NULL){
		m_hKalman->error_cov_post = (double*)malloc(m_hKalman->DP * m_hKalman->DP*sizeof(double));
		memset( m_hKalman->error_cov_post , 0 , sizeof (*m_hKalman->error_cov_post) );
	}
	if(m_hKalman->gain == NULL){
		m_hKalman->gain = (double*)malloc(m_hKalman->DP * m_hKalman->MP*sizeof(double));
		memset( m_hKalman->gain , 0 , sizeof (*m_hKalman->gain) );
	}
	if( m_hKalman->CP > 0 )
    {
		if(m_hKalman->control_matrix == NULL){
			m_hKalman->control_matrix = (double*)malloc(m_hKalman->DP * m_hKalman->CP*sizeof(double));
			memset( m_hKalman->control_matrix , 0 , sizeof (*m_hKalman->control_matrix) );
		}
    }

	if(m_hKalman->B_Uk == NULL){
		m_hKalman->B_Uk  = (double*)malloc(m_hKalman->DP * m_hKalman->MP*sizeof(double));
		memset( m_hKalman->B_Uk ,  0 , sizeof (*m_hKalman->B_Uk) );
	}
	if(m_hKalman->A_Pk == NULL){
		m_hKalman->A_Pk  = (double*)malloc(m_hKalman->DP * m_hKalman->DP*sizeof(double));
		memset( m_hKalman->A_Pk ,  0 , sizeof (*m_hKalman->A_Pk) );
	}
	if(m_hKalman->A_T == NULL){
		m_hKalman->A_T   = (double*)malloc(m_hKalman->DP * m_hKalman->DP*sizeof(double));
		memset( m_hKalman->A_T ,  0 , sizeof (*m_hKalman->A_T) );
	}
	if(m_hKalman->APA_T == NULL){
		m_hKalman->APA_T = (double*)malloc(m_hKalman->DP * m_hKalman->DP*sizeof(double));
		memset( m_hKalman->APA_T ,  0 , sizeof (*m_hKalman->APA_T) );
	}
	
	if(m_hKalman->H_T == NULL){
		m_hKalman->H_T       = (double*)malloc(m_hKalman->DP * m_hKalman->MP*sizeof(double));
		memset( m_hKalman->H_T ,  0 , sizeof (*m_hKalman->H_T) );
	}
	if(m_hKalman->Pk_Ht == NULL){
		m_hKalman->Pk_Ht     = (double*)malloc(m_hKalman->DP * m_hKalman->MP*sizeof(double));
		memset( m_hKalman->Pk_Ht ,  0 , sizeof (*m_hKalman->Pk_Ht) );
	}
	if(m_hKalman->Pk_Ht_R == NULL){
		m_hKalman->Pk_Ht_R   = (double*)malloc(m_hKalman->MP * m_hKalman->MP*sizeof(double));
		memset( m_hKalman->Pk_Ht_R ,  0 , sizeof (*m_hKalman->Pk_Ht_R) );
	}
	if(m_hKalman->Pk_Ht_R_Inv == NULL){
		m_hKalman->Pk_Ht_R_Inv = (double*)malloc(m_hKalman->MP * m_hKalman->MP*sizeof(double));
		memset( m_hKalman->Pk_Ht_R_Inv ,  0 , sizeof (*m_hKalman->Pk_Ht_R_Inv) );
	}
	if(m_hKalman->H_Xk == NULL){
		m_hKalman->H_Xk      = (double*)malloc(m_hKalman->MP *  1*sizeof(double));
		memset( m_hKalman->H_Xk ,  0 , sizeof (*m_hKalman->H_Xk) );
	}
	if(m_hKalman->Kk_H_Xk == NULL){
		m_hKalman->Kk_H_Xk   = (double*)malloc(m_hKalman->DP *  1*sizeof(double));
		memset( m_hKalman->Kk_H_Xk ,  0 , sizeof (*m_hKalman->Kk_H_Xk) );
	}
	if(m_hKalman->H_Pk == NULL){
		m_hKalman->H_Pk      = (double*)malloc(m_hKalman->MP * m_hKalman->DP *sizeof(double));
		memset( m_hKalman->H_Pk ,  0 , sizeof (*m_hKalman->H_Pk) );
	}
	if(m_hKalman->Kk_H_Pk == NULL){
		m_hKalman->Kk_H_Pk   = (double*)malloc(m_hKalman->DP * m_hKalman->DP*sizeof(double));
		memset( m_hKalman->Kk_H_Pk ,  0 , sizeof (*m_hKalman->Kk_H_Pk) );
	}
	m_hKalman->m_bInited = true;

	return NULL;
}

void CKalmanFilter::KalmanClose()
{
	if(m_hKalman == NULL)
		return ;
	if(m_hKalman->m_bInited == false)
		return;
	if(m_hKalman->state_pre != NULL){
		free (m_hKalman->state_pre) ;        m_hKalman->state_pre = NULL;
	}
	if(m_hKalman->state_post != NULL){
		free (m_hKalman->state_post) ;        m_hKalman->state_post = NULL;
	}
	if (m_hKalman->measure_param != NULL){
		free (m_hKalman->measure_param) ;        m_hKalman->measure_param = NULL;
	}
	if(m_hKalman->transition_matrix != NULL){
		free (m_hKalman->transition_matrix) ;  m_hKalman->transition_matrix = NULL;
	}
	if(m_hKalman->CP >0)
	{
		if(m_hKalman->control_matrix != NULL){
			free (m_hKalman->control_matrix) ;  m_hKalman->control_matrix = NULL;
		}
	}
	if(m_hKalman->measurement_matrix != NULL){
		free (m_hKalman->measurement_matrix) ;  m_hKalman->measurement_matrix = NULL;
	}
	if(m_hKalman->process_noise_cov != NULL){
		free (m_hKalman->process_noise_cov) ;   m_hKalman->process_noise_cov = NULL;
	}
	if(m_hKalman->measurement_noise_cov != NULL){
		free (m_hKalman->measurement_noise_cov) ;  m_hKalman->measurement_noise_cov = NULL;
	}
	if(m_hKalman->error_cov_pre != NULL){
		free (m_hKalman->error_cov_pre) ;    m_hKalman->error_cov_pre = NULL;
	}
	if(m_hKalman->gain != NULL){
		free (m_hKalman->gain) ;  m_hKalman->gain = NULL;
	}
	if(m_hKalman->error_cov_post != NULL){
		free (m_hKalman->error_cov_post) ;      m_hKalman->error_cov_post = NULL;
	}
	
	if(m_hKalman->B_Uk != NULL){
		free (m_hKalman->B_Uk) ;   m_hKalman->B_Uk = NULL;
	}
	if(m_hKalman->A_Pk != NULL){
		free (m_hKalman->A_Pk) ;   m_hKalman->A_Pk = NULL;
	}
	if(m_hKalman->A_T != NULL){
		free (m_hKalman->A_T) ;   m_hKalman->A_T = NULL;
	}
	if(m_hKalman->APA_T != NULL){
		free (m_hKalman->APA_T) ;  m_hKalman->APA_T = NULL;
	}
	
	if(m_hKalman->H_T != NULL){
		free (m_hKalman->H_T) ;     m_hKalman->H_T = NULL;
	}
	if(m_hKalman->Pk_Ht != NULL){
		free (m_hKalman->Pk_Ht) ;    m_hKalman->Pk_Ht = NULL;
	}
	if(m_hKalman->Pk_Ht_R != NULL){
		free (m_hKalman->Pk_Ht_R) ;   m_hKalman->Pk_Ht_R = NULL;
	}
	if(m_hKalman->Pk_Ht_R_Inv != NULL){
		free (m_hKalman->Pk_Ht_R_Inv) ;  m_hKalman->Pk_Ht_R_Inv = NULL;
	}
	if(m_hKalman->H_Xk != NULL){
		free (m_hKalman->H_Xk) ;     m_hKalman->H_Xk = NULL;
	}
	if(m_hKalman->Kk_H_Xk != NULL){
		free (m_hKalman->Kk_H_Xk) ;  m_hKalman->Kk_H_Xk = NULL;
	}
	if(m_hKalman->H_Pk != NULL){
		free (m_hKalman->H_Pk) ;     m_hKalman->H_Pk = NULL;
	}
	if(m_hKalman->Kk_H_Pk != NULL){
		free (m_hKalman->Kk_H_Pk) ;   m_hKalman->Kk_H_Pk = NULL;
	}
	m_hKalman->m_bInited = false;

	delete m_hKalman;
}


void CKalmanFilter::KalmanInitParam(float theta, float delta_x, float delta_y, double DeltaT)
{
	int x, y;
	if(m_hKalman == NULL)
		return ;
	if (!m_hKalman->m_bInited){
		return;
	}
	m_hKalman->deltat = DeltaT;
	/* ��̼�������Э������� */
	for ( y = 0; y < m_hKalman->DP; y++ ){
		for ( x = 0; x < m_hKalman->DP; x++ ){
			m_hKalman->process_noise_cov[y*m_hKalman->DP+x] = 0;//1E-5;
		}
	}
	m_hKalman->process_noise_cov[1*m_hKalman->DP+1] = 1E-6;//0.00000001;//100.0;  /* Э���Ϊ100.0�����໥���� */
	m_hKalman->process_noise_cov[3*m_hKalman->DP+3] = 1E-6;//0.00000001;//100.0;  /* Э���Ϊ100.0�����໥���� */
	m_hKalman->process_noise_cov[5*m_hKalman->DP+5] = 1E-6;//0.00000001;//100.0;  /* Э���Ϊ100.0�����໥���� */

	/* �۲�����Э������� */
	for ( y = 0; y < m_hKalman->MP; y++ ){
		for ( x = 0; x < m_hKalman->MP; x++ ){
			m_hKalman->measurement_noise_cov[y*m_hKalman->MP+x] = 0;
		}
	}
	m_hKalman->measurement_noise_cov[0*m_hKalman->MP+0] = 0.25;  /* Э���Ϊ0.001�����໥���� */
	m_hKalman->measurement_noise_cov[1*m_hKalman->MP+1] = 0.25;  /* Э���Ϊ0.001�����໥���� */
	m_hKalman->measurement_noise_cov[2*m_hKalman->MP+2] = 0.25;  /* Э���Ϊ0.001�����໥���� */

	/* ״̬�������Э���� */
	for ( y = 0; y < m_hKalman->DP; y++ ){
		for ( x = 0; x < m_hKalman->DP; x++ ){
			m_hKalman->error_cov_post[y*m_hKalman->DP+x] = 0.0;
		}
	}
	for ( y = 0; y < m_hKalman->DP; y++ ){
		m_hKalman->error_cov_post[y*m_hKalman->DP+y] = 1.0;  /* �Խǳ�ʼЭ���Ϊ1�����໥���� */
	}

	/* ״̬ת���� */
	for ( y = 0; y < m_hKalman->DP; y++ ){
		for ( x = 0; x < m_hKalman->DP; x++ ){
			m_hKalman->transition_matrix[y*m_hKalman->DP+x] = 0.0;
		}
	}
	for ( y = 0; y < m_hKalman->DP; y++ ){
		m_hKalman->transition_matrix[y*m_hKalman->DP+y] = 1.0;  /* �Խ�Ϊ1 */
	}
	m_hKalman->transition_matrix[0*m_hKalman->DP+1] = 1;
	m_hKalman->transition_matrix[2*m_hKalman->DP+3] = 1;
	m_hKalman->transition_matrix[4*m_hKalman->DP+5] = 1;

	/* �۲���״̬�����۲�����ת�ƾ��� */
	for ( y = 0; y < m_hKalman->MP; y++ ){
		for ( x = 0; x < m_hKalman->DP; x++ ){
			m_hKalman->measurement_matrix[y*m_hKalman->DP+x] = 0.0;
		}
	}
	m_hKalman->measurement_matrix[0*m_hKalman->DP+0] = 1.0;
	m_hKalman->measurement_matrix[1*m_hKalman->DP+2] = 1.0;
	m_hKalman->measurement_matrix[2*m_hKalman->DP+4] = 1.0;

	// �۲���������˳��theta, x, y
	m_hKalman->measure_param[0] = (float)theta;
	m_hKalman->measure_param[1] = (float)delta_x;
	m_hKalman->measure_param[2] = (float)delta_y;
	/* ��ʼ��thelta, thelta_v, x, x_v, y, y_v��״̬�� */
    // ״̬��������˳��thelta, thelta_v,  x, y, x_v, y_v
	m_hKalman->state_post[0] = theta;
	m_hKalman->state_post[1] = 0.0;
	m_hKalman->state_post[2] = delta_x;
	m_hKalman->state_post[3] = 0.0;
	m_hKalman->state_post[4] = delta_y;
	m_hKalman->state_post[5] = 0.0;
}



void CKalmanFilter::KalmanPredict(double * control )
{
	if(m_hKalman == NULL)
		return;
	if (!m_hKalman->m_bInited){
		return;
	}

	/* update the state */
	/* x'(k) = A*x(k) */
	MatrixMultiply( m_hKalman->transition_matrix, m_hKalman->state_post, m_hKalman->DP , m_hKalman->DP , 1 , m_hKalman->state_pre );

	if( control!=NULL && m_hKalman->CP > 0 ){
		/* x'(k) = x'(k) + B*u(k) */
		MatrixMultiply( m_hKalman->control_matrix, control, m_hKalman->DP , m_hKalman->CP , 1 , m_hKalman->B_Uk);
		MatrixAddition( m_hKalman->state_pre, m_hKalman->B_Uk, m_hKalman->DP, 1, m_hKalman->state_pre);
	}
	/* update error covariance matrices */
	/* A_Pk = A*P(k) */
	MatrixMultiply( m_hKalman->transition_matrix, m_hKalman->error_cov_post, m_hKalman->DP, m_hKalman->DP, m_hKalman->DP, m_hKalman->A_Pk);
	
	/* P'(k) = A_Pk*At + Q */
	MatrixTranspose(m_hKalman->transition_matrix, m_hKalman->DP, m_hKalman->DP, m_hKalman->A_T);
	MatrixMultiply(m_hKalman->A_Pk, m_hKalman->A_T, m_hKalman->DP, m_hKalman->DP, m_hKalman->DP, m_hKalman->APA_T);
	MatrixAddition(m_hKalman->APA_T, m_hKalman->process_noise_cov, m_hKalman->DP, m_hKalman->DP, m_hKalman->error_cov_pre);
}

void CKalmanFilter::KalmanCorrect(double * measurement )
{
	int i;
	if(m_hKalman == NULL)
		return;
	if (!m_hKalman->m_bInited){
		return;
	}
	if( measurement == NULL)
	{
		;//AfxMessageBox("Measurement is Null!!!");
	}
	if(measurement != NULL){
		for (i=0; i<m_hKalman->MP; i++){
			m_hKalman->measure_param[i] = measurement[i];
		}
	}
	/* H_T = Ht*/
	MatrixTranspose( m_hKalman->measurement_matrix , m_hKalman->MP , m_hKalman->DP , m_hKalman->H_T);
	/* Pk_Ht = P'(k) * H_T */
    MatrixMultiply( m_hKalman->error_cov_pre, m_hKalman->H_T, m_hKalman->DP , m_hKalman->DP , m_hKalman->MP , m_hKalman->Pk_Ht);
	
    /* Pk_Ht_R = H*Pk_Ht + R */
    MatrixMultiply( m_hKalman->measurement_matrix ,m_hKalman->Pk_Ht , m_hKalman->MP , m_hKalman->DP , m_hKalman->MP , m_hKalman->Pk_Ht_R);
	MatrixAddition( m_hKalman->Pk_Ht_R , m_hKalman->measurement_noise_cov , m_hKalman->MP , m_hKalman->MP , m_hKalman->Pk_Ht_R);
	
    /* Pk_Ht_R_Inv = inv(Pk_Ht_R) */
#if 0
    MatrixInversion( m_hKalman->Pk_Ht_R , m_hKalman->MP, m_hKalman->Pk_Ht_R_Inv);
#else
    MatrixCopy(m_hKalman->Pk_Ht_R_Inv, m_hKalman->Pk_Ht_R, m_hKalman->MP, m_hKalman->MP);
	MatrixBrinv(m_hKalman->Pk_Ht_R_Inv, m_hKalman->MP);
#endif
    
    /* K(k) = Pk_Ht * Pk_Ht_R_Inv  */
    MatrixMultiply( m_hKalman->Pk_Ht , m_hKalman->Pk_Ht_R_Inv, m_hKalman->DP , m_hKalman->MP ,m_hKalman->MP , m_hKalman->gain);

    //update state_post
    /* H_Xk = z(k) - H*x'(k) */
	MatrixMultiply( m_hKalman->measurement_matrix , m_hKalman->state_pre , m_hKalman->MP , m_hKalman->DP , 1, m_hKalman->H_Xk);
	MatrixSubtraction( m_hKalman->measure_param , m_hKalman->H_Xk , m_hKalman->MP , 1, m_hKalman->H_Xk);
    /* x(k) = x'(k) + K(k)*H_Xk */
	MatrixMultiply( m_hKalman->gain , m_hKalman->H_Xk, m_hKalman->DP , m_hKalman->MP, 1, m_hKalman->Kk_H_Xk );
    MatrixAddition( m_hKalman->state_pre , m_hKalman->Kk_H_Xk , m_hKalman->DP ,1 , m_hKalman->state_post);
 
	//update error_cov_post
    /* P(k) = P'(k) - K(k)*H* P'(k) */
    MatrixMultiply( m_hKalman->measurement_matrix , m_hKalman->error_cov_pre , m_hKalman->MP , m_hKalman->DP , m_hKalman->DP , m_hKalman->H_Pk );
	MatrixMultiply( m_hKalman->gain , m_hKalman->H_Pk , m_hKalman->DP , m_hKalman->MP, m_hKalman->DP , m_hKalman->Kk_H_Pk );
	MatrixSubtraction( m_hKalman->error_cov_pre , m_hKalman->Kk_H_Pk , m_hKalman->DP ,m_hKalman->DP , m_hKalman->error_cov_post);
}

int CKalmanFilter::Kalman(double *measure, double *control)
{
	assert(m_hKalman != NULL);
	KalmanPredict(control);
	KalmanCorrect(measure);
	return 0;
}
