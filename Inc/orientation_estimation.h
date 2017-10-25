/*
 * orientation_estimation.h
 *
 *  Created on: 12 sept. 2017
 *      Author: Aurélien VALADE
 */

#ifndef ORIENTATION_ESTIMATION_H_
#define ORIENTATION_ESTIMATION_H_

#include <stdint.h>

typedef enum {
	MATH_OK,
	MATH_ERROR
}MATH_STATUS;

typedef struct {
	float *data;
	uint8_t height, width;
}MATRIX_F;

#define MATRIX_DATA(mat, x, y) mat->data[x + mat->width*y]

MATRIX_F *f_matrix_alloc(uint8_t width, uint8_t height);
MATRIX_F *f_matrix_from_diag(float* diag, uint8_t size);
MATH_STATUS f_matrix_free(MATRIX_F* matrix);

MATH_STATUS f_matrix_multiply(MATRIX_F *A, MATRIX_F *B, MATRIX_F *res);
MATH_STATUS f_matrix_multiply_by_transpose(MATRIX_F *A, MATRIX_F *B, MATRIX_F *res);
MATH_STATUS f_matrix_invert_cholesky(MATRIX_F* A, MATRIX_F* res);

MATH_STATUS orient_est_init(float theta, float g);
MATH_STATUS orient_est_evolve(float gyro);
MATH_STATUS orient_est_correct(float ax, float az);

float orient_est_get_g();
float orient_est_get_theta();

#endif /* ORIENTATION_ESTIMATION_H_ */
