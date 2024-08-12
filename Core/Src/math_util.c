/*
 * math_util.c
 *
 *  Created on: Aug 9, 2024
 *      Author: vedpadu
 */

#include "math_util.h"

// IMPORTANT: DATA MUST BE FREED AFTER USE
arm_matrix_instance_f32 skewSymmetric(float32_t* v){
	float32_t* mat = calloc(9, sizeof(float32_t));
	mat[1] = -v[2];
	mat[2] = v[1];
	mat[3] = v[2];
	mat[5] = -v[0];
	mat[6] = -v[1];
	mat[7] = v[0];
	arm_matrix_instance_f32 matInst;
	arm_mat_init_f32(&matInst, 3, 3, mat);
	return matInst;
}

// IMPORTANT: DATA MUST BE FREED AFTER USE
arm_matrix_instance_f32 quatToMatrix(quaternion_t q){
	arm_matrix_instance_f32 mat1;
	arm_matrix_instance_f32 mat2;
	arm_matrix_instance_f32 mat3;

	mat1 = outerProductVec3(q.vec, q.vec);
	arm_mat_scale_f32(&mat1, 2.0, &mat1);

	float32_t mat2_vals[3][3] = {0}; // doesnt need to be calloced because it is only used locally
	mat2 = generateDiagonalMatrix(mat2_vals[0], 3, pow(q.w, 2.0) - vectorDot(q.vec, q.vec, 3));
	arm_mat_add_f32(&mat1, &mat2, &mat1);

	mat3 = skewSymmetric(q.vec);
	arm_mat_scale_f32(&mat3, 2 * q.w, &mat3);
	arm_mat_add_f32(&mat1, &mat3, &mat1);

	free(mat3.pData);

	return mat1;
}

// IMPORTANT: DATA MUST BE FREED AFTER USE
arm_matrix_instance_f32 outerProductVec3(float32_t* vec1, float32_t* vec2){
	float32_t* mat = calloc(9, sizeof(float32_t));
	int x;
	int y;
	for(y = 0;y < 3;y++){
		for(x = 0;x < 3;x++){
			mat[y * 3 + x] = vec1[y] * vec2[x];
		}
	}
	arm_matrix_instance_f32 matInst;
	arm_mat_init_f32(&matInst, 3, 3, mat);
	return matInst;
}

void subtractFromVector(float32_t* vec1, float32_t* vec2, uint8_t len){
	int i;
	for(i = 0;i < len;i++){
		vec1[i] -= vec2[i];
	}
}

void addToVector(float32_t* vec1, float32_t* vec2, uint8_t len){
	int i;
	for(i = 0;i < len;i++){
		vec1[i] += vec2[i];
	}
}

float32_t* vector3Add(float32_t* vec1, float32_t* vec2, float32_t* out){
	int i;
	for(i = 0;i < 3;i++){
		out[i] = vec1[i] + vec2[i];
	}
	return out;
}

float32_t* vector3Cross(float32_t* vec1, float32_t* vec2, float32_t* out){
	out[0] = vec1[1] * vec2[2] - vec2[1] * vec1[2];
	out[1] = -vec1[0] * vec2[2] + vec2[0] * vec1[2];
	out[2] = vec1[0] * vec2[1] - vec2[0] * vec1[1];
	return out;
}

float32_t* vector3Scale(float32_t* vec, float32_t scale, float32_t* out){
	out[0] = vec[0] * scale;
	out[1] = vec[1] * scale;
	out[2] = vec[2] * scale;
	return out;
}

float32_t vectorDot(float32_t* vec1, float32_t* vec2, uint8_t len){
	float32_t prod = 0.0;
	int i;
	for(i = 0;i < len;i++){
		prod += vec1[i] * vec2[i];
	}
	return prod;
}

arm_matrix_instance_f32 generateDiagonalMatrix(float32_t* matrix, uint8_t size, float32_t value){
	arm_matrix_instance_f32 instance;
	diagonalMat(matrix, size, value);
	arm_mat_init_f32(&instance, size, size, matrix);
	return instance;
}

// maybe make these
arm_matrix_instance_f32 createScaleMatrix(arm_matrix_instance_f32* instanceIn, float32_t scaleFactor, uint8_t size){
	arm_matrix_instance_f32 instanceOut;
	float32_t* mat = malloc(size * size * sizeof(float32_t));
	arm_mat_init_f32(&instanceOut, size, size, mat);
	arm_mat_scale_f32(instanceIn, scaleFactor, &instanceOut);
	return instanceOut;
}

// generally only called for malloced/calloced matrices atm, be careful to not free stack data
float32_t* addMatrices(arm_matrix_instance_f32 in1, arm_matrix_instance_f32 in2, uint8_t size){
	arm_matrix_instance_f32 instanceOut;
	float32_t* mat = malloc(size * size * sizeof(float32_t));
	arm_mat_init_f32(&instanceOut, size, size, mat);
	arm_mat_add_f32(&in1, &in2, &instanceOut);
	free(in1.pData);
	free(in2.pData);
	return mat;
}

void quatToEuler(quaternion_t q, float32_t* outEuler){

	float32_t roll = atan2(2*(q.w*q.vec[0] + q.vec[1]*q.vec[2]), 1 - 2*(q.vec[0]*q.vec[0] + q.vec[1]*q.vec[1]));
	outEuler[0] = roll;

	float32_t pitch = asin(2*(q.w*q.vec[1] - q.vec[2]*q.vec[0]));
	//float32_t pitch = -(M_PI/2.0) + 2.0 * atan2(sqrt(1 + 2.0 * (q.w * q.vec[1] - q.vec[0] * q.vec[2])), sqrt(1 - 2.0 * (q.w * q.vec[1]- q.vec[0] * q.vec[2])));
	outEuler[1] = pitch;

	float32_t yaw = atan2(2*(q.w*q.vec[2] + q.vec[0]*q.vec[1]), 1 - 2*(q.vec[1]*q.vec[1] + q.vec[2]*q.vec[2]));
	outEuler[2] = yaw;
}

// assumes matrix already initialized with zeros
// use 1.0 as value to create an identity matrix
void diagonalMat(float32_t* matrix, uint8_t size, float32_t value){
	int i;
	for(i = 0;i < size;i++){
		matrix[i * size + i] = value;
	}
}

// Inserts a smaller square matrix into a larger square matrix at a specific position. Can choose to free the data in the smaller matrix.
void injectMatrix(float32_t* outMat, float32_t* inMat, uint8_t y0, uint8_t x0, uint8_t sizeOut, uint8_t sizeIn, uint8_t doFree){
	int x;
	int y;
	for(y = 0;y < sizeIn;y++){
		for(x = 0;x < sizeIn;x++){
			outMat[((y0+y) * sizeOut) + x0+x] = inMat[y * sizeIn + x];
		}
	}
	if(doFree){
		free(inMat);
	}
}

// Inverse quaternion
quaternion_t quatInverse(quaternion_t q){
	float32_t ss = quatSquareMag(q);
	if(ss > 0){
		quaternion_t conj = quatConjugate(q);
		conj = quatDivideScalar(conj, ss);
		return conj;
	}else{
		// return q if quaternion is 0 to avoid div by 0
		return q;
	}

}

// Multiply two quaternions
quaternion_t quatMultiply(quaternion_t q1, quaternion_t q2){
	float32_t w = q1.w * q2.w - vectorDot(q1.vec, q2.vec, 3);
	float32_t prod1[3] = {0};
	float32_t prod2[3] = {0};
	float32_t cross[3] = {0};
	vector3Scale(q2.vec, q1.w, prod1);
	vector3Scale(q1.vec, q2.w, prod2);
	addToVector(prod1, prod2, 3);
	addToVector(prod1, vector3Cross(q1.vec, q2.vec, cross), 3);
	quaternion_t out = {w, {prod1[0], prod1[1], prod1[2]}};
	return out;
}

// Rotate a vector by a quaternion
void rotateVector3ByQuaternion(float32_t v[3], quaternion_t q) {
    quaternion_t v_q = {0, {v[0], v[1], v[2]}}; // Convert vector to quaternion

    quaternion_t q_inv = quatInverse(q);
    quaternion_t temp = quatMultiply(q, v_q);
    quaternion_t rotated_v_q = quatMultiply(temp, q_inv);

    // Extract the rotated vector
    v[0] = rotated_v_q.vec[0];
    v[1] = rotated_v_q.vec[1];
    v[2] = rotated_v_q.vec[2];
}

// Add one quaternion to another
void addToQuat(quaternion_t* q1, quaternion_t q2){
	q1->w += q2.w;
	addToVector(q1->vec, q2.vec, 3);
}

// Normalize a quaternion
void normalizeQuaternion(quaternion_t* q){
	*q = quatDivideScalar(*q, pow(quatSquareMag(*q), 0.5));
}

// Get the conjugate of a quaternion
quaternion_t quatConjugate(quaternion_t q){
	quaternion_t conj = {q.w, {-q.vec[0], -q.vec[1], -q.vec[2]}};
	return conj;
}

// Scalar multiply for quaternion
quaternion_t quatMultiplyScalar(quaternion_t q, float32_t val){
	q.vec[0] = q.vec[0]*val;
	q.vec[1] = q.vec[1]*val;
	q.vec[2] = q.vec[2]*val;
	q.w = q.w*val;
	return q;
}

// Scalar divide for quaternion
quaternion_t quatDivideScalar(quaternion_t q, float32_t val){
	q.vec[0] = q.vec[0]/val;
	q.vec[1] = q.vec[1]/val;
	q.vec[2] = q.vec[2]/val;
	q.w = q.w/val;
	return q;
}

// Quaternion square magnitude
float32_t quatSquareMag(quaternion_t q){
	return q.vec[0] * q.vec[0] + q.vec[1] * q.vec[1] + q.vec[2] * q.vec[2] + q.w * q.w;
}

float32_t absVal(float32_t val){
	if(val < 0){
		return -val;
	}
	return val;
}

int32_t absInt(int32_t val){
	if(val < 0){
		return -val;
	}
	return val;
}

int16_t int16Clamp(int16_t in, int16_t max, int16_t min){
	if(in > max){
		in = max;
	}
	if(in < min){
		in = min;
	}
	return in;
}

float32_t clamp(float32_t in, float32_t max, float32_t min){
	if(in > max){
		in = max;
	}
	if(in < min){
		in = min;
	}
	return in;
}

