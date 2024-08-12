/*
 * math_util.h
 *
 *  Created on: Aug 9, 2024
 *      Author: vedpadu
 */

#ifndef INC_MATH_UTIL_H_
#define INC_MATH_UTIL_H_

#include "stdint.h"
#include "arm_math.h"
#include "stdlib.h"

typedef struct quaternion_s {
	float32_t w;
	float32_t vec[3];
}quaternion_t;

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
void quatToEuler(quaternion_t q, float32_t* outEuler);
float32_t* addMatrices(arm_matrix_instance_f32 in1, arm_matrix_instance_f32 in2, uint8_t size);
arm_matrix_instance_f32 createScaleMatrix(arm_matrix_instance_f32* instanceIn, float32_t scaleFactor, uint8_t size);
void subtractFromVector(float32_t* vec1, float32_t* vec2, uint8_t size);
void addToVector(float32_t* vec1, float32_t* vec2, uint8_t len);
float32_t vectorDot(float32_t* vec1, float32_t* vec2, uint8_t len);
float32_t* vector3Add(float32_t* vec1, float32_t* vec2, float32_t* out);
float32_t* vector3Scale(float32_t* vec, float32_t scale, float32_t* out);
float32_t* vector3Cross(float32_t* vec1, float32_t* vec2, float32_t* out);
arm_matrix_instance_f32 outerProductVec3(float32_t* vec1, float32_t* vec2);

arm_matrix_instance_f32 quatToMatrix(quaternion_t q);
arm_matrix_instance_f32 skewSymmetric(float32_t* v);

float32_t clamp(float32_t in, float32_t max, float32_t min);
int16_t int16Clamp(int16_t in, int16_t max, int16_t min);
float32_t absVal(float32_t val);
int32_t absInt(int32_t val);

#endif /* INC_MATH_UTIL_H_ */
