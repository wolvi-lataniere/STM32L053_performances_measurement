/*
 * orientation_estimation.c
 *
 *  This module estimates the 2D system orientation from IMU measurements
 *
 *  Created on: 12 sept. 2017
 *      Author: Aurélien VALADE
 */
#include "orientation_estimation.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/**
 * Constants definition for the Kalman filter
 */
#define G_INITIAL_NOISE 0.00001f
#define G_NOISE 0.0001f
#define THETA_INITIAL_NOISE 0.01f
#define THETA_NOISE 0.001f
#define SAMPLE_RATE 26.0f

#define ACCELERO_NOISE 0.01f

/*************************************/
/*** Matrix manipulation functions ***/
/*************************************/

/**
 * @brief Allocate matrix structure
 *
 * @param width matrix width
 * @param height matrix height
 */
MATRIX_F *f_matrix_alloc(uint8_t width, uint8_t height)
{
	MATRIX_F *res = (MATRIX_F*)malloc(sizeof(MATRIX_F));

	res->data = (float*) malloc(sizeof(float)*width*height);
	res->height = height;
	res->width = width;

	memset(res->data, 0, sizeof(float)*width*height);

	return res;
}

/**
 * @brief Allocate matrix structure from a diagonal vector
 *
 * Create a new diagonal matrix using diag data as values.
 *
 * @param diag digaonal vector
 * @param size size of the vector
 */
MATRIX_F *f_matrix_from_diag(float* diag, uint8_t size)
{
	if (diag == NULL)
		return NULL;

	MATRIX_F *res = (MATRIX_F*)malloc(sizeof(MATRIX_F));
	int i;

	res->data = (float*)malloc(sizeof(float)*size*size);
	res->height = size;
	res->width = size;

	memset(res->data, 0, sizeof(float)*size*size);

	for (i=0;i<size;i++)
		MATRIX_DATA(res, i, i) = diag[i];

	return res;
}

/**
 * @brief Free a matrix structure
 *
 * @param matrix matrix to free
 */
MATH_STATUS f_matrix_free(MATRIX_F* matrix)
{
	if (matrix == NULL || matrix->data == NULL)
		return MATH_ERROR;

	free(matrix->data);
	free(matrix);

	return MATH_OK;
}

/**
 * @brief Invert a symetric matrix using Cholesky method
 *
 * @param A matrix to invert
 * @param res result matrix to fill with data
 */
MATH_STATUS f_matrix_invert_cholesky(MATRIX_F* A, MATRIX_F* res)
{
	float r;
	int i,j,k;

	if (A==NULL || res==NULL ||
			A->data == NULL || res->data == NULL ||
			A->height != A->width ||
			A->height != res->height ||
			res->height != res->width)
		return MATH_ERROR;

	for (j=0;j<A->height;j++)
	{
		for (i=j;i<A->height;i++)
		{
			if (i==j)
				MATRIX_DATA(res,j,i) = 1/MATRIX_DATA(A,j,i);
			else
			{
				r=0.0;
				for (k=0;k<i;k++)
					r -= MATRIX_DATA(A,j,k)*MATRIX_DATA(res,k,i);
				if (MATRIX_DATA(A, i, i) == 0)
					return MATH_ERROR;
				r /= MATRIX_DATA(A,i,i);
				MATRIX_DATA(res,j,i) = r;
			}
		}
	}

	return MATH_OK;
}

/**
 * @brief Multitply two matrices
 *
 * @param A first matrix to multiply
 * @param B second matrix to multiply
 * @param result array to store the result matrix
 */
MATH_STATUS f_matrix_multiply(MATRIX_F *A, MATRIX_F *B, MATRIX_F *res)
{
	float r;

	int i,j,k;

	if (A==NULL || B==NULL || res == NULL ||
			A->width != B->height ||
			A->height != res->height ||
			B->width != res->width)
		return MATH_ERROR;

	for (i=0;i<A->height;i++)
		for (j=0;j<B->width;j++)
		{
			r=0.0f;
			for (k=0;k<A->width;k++)
				r += MATRIX_DATA(A, k, i) * MATRIX_DATA(B, j, k);

			MATRIX_DATA(res,j, i) = r;
		}

	return MATH_OK;
}

/**
 * @brief Multitply matrix A by the transpose of B
 *
 * @param A first matrix to multiply
 * @param B second matrix to multiply
 * @param result array to store the result matrix
 */
MATH_STATUS f_matrix_multiply_by_transpose(MATRIX_F *A, MATRIX_F *B, MATRIX_F *res)
{
	float r;

	int i,j,k;

	if (A==NULL || B==NULL || res == NULL ||
			A->width != B->width ||
			A->height != res->height ||
			B->height != res->width)
		return MATH_ERROR;

	for (i=0;i<A->height;i++)
		for (j=0;j<B->height;j++)
		{
			r=0.0;
			for (k=0;k<A->width;k++)
				r += MATRIX_DATA(A, k, i) * MATRIX_DATA(B, k, j);

			MATRIX_DATA(res,j, i) = r;
		}

	return MATH_OK;
}



/*************************************/
/***   Application specific part   ***/
/*************************************/

// Kalman filter matrices
static MATRIX_F *P, *X, *PE, *K, *E, *KE, *H, *PH, *HPH;

/**
 * @brief Initialize the filter and allocate related matrices
 *
 * @param theta Initial guess of theta (in rad)
 * @param g Initial guess of gravity factor (in g)
 */
MATH_STATUS orient_est_init(float theta, float g)
{
	float diag[2];

	// Allocate the state vector
	X = f_matrix_alloc(1, 2);
	if (X == NULL)
		return MATH_ERROR;

	// And initialize with initial parameters
	MATRIX_DATA(X, 0, 0) = g;
	MATRIX_DATA(X, 0, 1) = theta;

	// Initialize P0
	diag[0] = G_INITIAL_NOISE * G_INITIAL_NOISE;
	diag[1] = THETA_INITIAL_NOISE * THETA_INITIAL_NOISE;
	P = f_matrix_from_diag(diag, 2);
	if (P == NULL)
	{
		f_matrix_free(X);
		return MATH_ERROR;
	}

	// Initialize Kalman Gain matrix
	K = f_matrix_alloc(2, 2);
	if (K == NULL)
	{
		f_matrix_free(X);
		f_matrix_free(P);
		return MATH_ERROR;
	}

	// Initialize Innovation matrix
	E = f_matrix_alloc(1,2);
	if (E == NULL)
	{
		f_matrix_free(X);
		f_matrix_free(P);
		f_matrix_free(K);
		return MATH_ERROR;
	}

	// Initialize Jacobian matrix
	H = f_matrix_alloc(2,2);
	if (H == NULL)
	{
		f_matrix_free(X);
		f_matrix_free(P);
		f_matrix_free(K);
		f_matrix_free(E);
		return MATH_ERROR;
	}

	// Initialize intermediate matrix for P*H computation
	PH = f_matrix_alloc(2,2);
	if (PH == NULL)
	{
		f_matrix_free(X);
		f_matrix_free(P);
		f_matrix_free(K);
		f_matrix_free(E);
		f_matrix_free(H);
		return MATH_ERROR;
	}

	// Initialize intermediate matrix for H*P*H computation
	HPH = f_matrix_alloc(2,2);
	if (HPH == NULL)
	{
		f_matrix_free(X);
		f_matrix_free(P);
		f_matrix_free(K);
		f_matrix_free(E);
		f_matrix_free(H);
		f_matrix_free(PH);
		return MATH_ERROR;
	}

	// Initialize matrix for K*E Computation (Kalman gain times innovation)
	KE = f_matrix_alloc(1,2);
	if (KE == NULL)
	{
		f_matrix_free(X);
		f_matrix_free(P);
		f_matrix_free(K);
		f_matrix_free(E);
		f_matrix_free(H);
		f_matrix_free(PH);
		f_matrix_free(HPH);
		return MATH_ERROR;
	}

	// Initialize matrix for P estimation
	PE = f_matrix_alloc(2,2);
	if (PE == NULL)
	{
		f_matrix_free(X);
		f_matrix_free(P);
		f_matrix_free(K);
		f_matrix_free(E);
		f_matrix_free(H);
		f_matrix_free(PH);
		f_matrix_free(HPH);
		f_matrix_free(KE);
		return MATH_ERROR;
	}

	return MATH_OK;
}

/**
 * @brief Process an evolution estimate on the Kalman filter
 *
 * @param gyro Measured gyro speed (with offset pre-removed, in rad/s)
 */
MATH_STATUS orient_est_evolve(float gyro)
{
  // Evolve state
  MATRIX_DATA(X, 0, 1) += gyro / SAMPLE_RATE;        // The angle is increased by the rotation rate
  MATRIX_DATA(X, 0, 0) = fabs(MATRIX_DATA(X, 0, 0)); // Ensure a positive gravity

  // Update covariance (only diagonal elements are impacted as we use identity matrix for noise)
  MATRIX_DATA(PE, 0, 0) = MATRIX_DATA(P, 0, 0) + G_NOISE * G_NOISE;
  MATRIX_DATA(PE, 1, 1) = MATRIX_DATA(P, 1, 1) + THETA_NOISE * THETA_NOISE;
  
  return MATH_OK;
}

/**
 * @brief Do the measurement prediction and correction on the Kalman filter
 *
 * @param ax X accelerometer measurement (in g)
 * @param az Z accelerometer measurement (in g)
 */
MATH_STATUS orient_est_correct(float ax, float az)
{
	// Pre-compute frequently used values
	float stheta = sinf(MATRIX_DATA(X, 0, 1));
	float ctheta = cosf(MATRIX_DATA(X, 0, 1));
	float g = fabs(MATRIX_DATA(X, 0, 0));
	
	// Compute ax, az innovation,
	MATRIX_DATA(E, 0, 0) = ax - (g * stheta);
	MATRIX_DATA(E, 0, 1) = az - (g * ctheta);

	// Compute measurement jacobian
	MATRIX_DATA(H, 0, 0) = stheta / 1000.0f;
	MATRIX_DATA(H, 0, 1) = ctheta / 1000.0f;
	MATRIX_DATA(H, 1, 0) = g * ctheta / 1000.0f;
	MATRIX_DATA(H, 1, 1) = -g * stheta / 1000.0f;

	// Compute the Kalman gain :
	// [K] = [PE].[H]'.([H].[PE].[H]' + [R])^(-1)

	// Start by [PH] = [PE].[H]'
	if (f_matrix_multiply_by_transpose(PE, H, PH) == MATH_ERROR)
		return MATH_ERROR;
	// Then [H].[PH] = [H].[PE].[H]'
	if (f_matrix_multiply(H, PH, K) == MATH_ERROR)
		return MATH_ERROR;

	// Then manually add the noise ([H].[PH] + [R])
	// As the noise coupling is identity, we only compute needed elements
	MATRIX_DATA(K, 0, 0) += ACCELERO_NOISE * ACCELERO_NOISE;
	MATRIX_DATA(K, 1, 1) += ACCELERO_NOISE * ACCELERO_NOISE;

	// Invert the group ([H].[PH] + [R])^(-1)
	if (f_matrix_invert_cholesky(K, HPH) == MATH_ERROR)
		return MATH_ERROR;

	// Finish the K computation [K] = [PE].[H]'.([H].[PE].[H]' + [R])^(-1)
	if (f_matrix_multiply(PH, HPH, K) == MATH_ERROR)
		return MATH_ERROR;

	// Compute the product [K].[E] to propagate the innovation
	if (f_matrix_multiply(K, E, KE) == MATH_ERROR)
		return MATH_ERROR;

	// Update estimate
	MATRIX_DATA(X, 0, 0) += MATRIX_DATA(KE, 0, 0);
	MATRIX_DATA(X, 0, 1) += MATRIX_DATA(KE, 0, 1);

	// Update P
	if (f_matrix_multiply(K, H, PH) == MATH_ERROR)
		return MATH_ERROR;

	MATRIX_DATA(PH, 0, 0) = 1 - MATRIX_DATA(PH, 0, 0);
	MATRIX_DATA(PH, 1, 1) = 1 - MATRIX_DATA(PH, 1, 1);
	MATRIX_DATA(PH, 0, 1) = - MATRIX_DATA(PH, 0, 1);
	MATRIX_DATA(PH, 1, 0) = - MATRIX_DATA(PH, 1, 0);

	if (f_matrix_multiply(PH, PE, P) == MATH_ERROR)
		return MATH_ERROR;

	return MATH_OK;
}

/**
 * @brief Access the estimate angle
 *
 * @return The estimate angle (in rad)
 */
float orient_est_get_theta()
{
	return MATRIX_DATA(X, 0, 1);
}

/**
 * @brief Access the estimate gravity factor
 *
 * @return The estimate gravity factor
 */
float orient_est_get_g()
{
	return MATRIX_DATA(X, 0, 0);
}
