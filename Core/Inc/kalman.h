/*
 * kalman.h
 *
 *  Created on: Jul 24, 2024
 *      Author: vedpa
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "stdint.h"
#include "arm_math.h"

typedef struct quaternion_s {
	float32_t w;
	float32_t vec[3];
}quaternion_t;

extern quaternion_t estimate;
extern int initialized;
extern float32_t estimate_covar_mat[15][15];
extern float32_t G_mat[15][15];
extern float32_t kalman_gyro[3];

void injectMatrix(float32_t* outMat, float32_t* inMat, uint8_t y0, uint8_t x0, uint8_t sizeOut, uint8_t sizeIn, uint8_t doFree);
void diagonalMat(float32_t* matrix, uint8_t size, float32_t value);
arm_matrix_instance_f32 generateDiagonalMatrix(float32_t* matrix, uint8_t size, float32_t value);
float32_t quatSquareMag(quaternion_t q);
quaternion_t quatDivideScalar(quaternion_t q, float32_t val);
quaternion_t quatMultiplyScalar(quaternion_t q, float32_t val);
void rotateVector3ByQuaternion(float32_t v[3], quaternion_t q);
void normalizeQuaternion(quaternion_t* q);
void addToQuat(quaternion_t* q1, quaternion_t q2);
quaternion_t quatConjugate(quaternion_t q);
quaternion_t quatInverse(quaternion_t q);
quaternion_t quatMultiply(quaternion_t q1, quaternion_t q2);
float32_t* addMatrices(arm_matrix_instance_f32 in1, arm_matrix_instance_f32 in2, uint8_t size);
arm_matrix_instance_f32 createScaleMatrix(arm_matrix_instance_f32* instanceIn, float32_t scaleFactor, uint8_t size);
arm_matrix_instance_f32 process_covariance(float32_t time_delta);
void subtractFromVector(float32_t* vec1, float32_t* vec2, uint8_t size);
void addToVector(float32_t* vec1, float32_t* vec2, uint8_t len);
float32_t getAccelHealth(float32_t* acc, float32_t* gyr);

float32_t vectorDot(float32_t* vec1, float32_t* vec2, uint8_t len);
float32_t* vector3Add(float32_t* vec1, float32_t* vec2, float32_t* out);
float32_t* vector3Scale(float32_t* vec, float32_t scale, float32_t* out);
float32_t* vector3Cross(float32_t* vec1, float32_t* vec2, float32_t* out);
arm_matrix_instance_f32 outerProductVec3(float32_t* vec1, float32_t* vec2);

arm_matrix_instance_f32 quatToMatrix(quaternion_t q);
arm_matrix_instance_f32 skewSymmetric(float32_t* v);

void initKalman(quaternion_t initial_est, float32_t estimate_cov, float32_t gyro_cov, float32_t gyro_bias_cov,
		float32_t accel_proc_cov, float32_t accel_bias_cov, float32_t accel_obs_cov);
void updateKalman(float32_t gyroMeas[3], float32_t accMeas[3], float32_t time_delta);


#endif /* INC_KALMAN_H_ */
