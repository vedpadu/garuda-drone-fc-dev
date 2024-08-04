/*
 * kalman.c
 *
 *  Created on: Jul 24, 2024
 *      Author: vedpa
 */


#include "arm_math.h"
#include "kalman.h"
#include "main.h"
#include "stdlib.h"

quaternion_t estimate;
float32_t estimate_covar_mat[15][15] = {0};
float32_t observation_covar_mat[3][3] = {0};



float32_t gyro_covar_mat[3][3] = {0};
float32_t gyro_bias_covar_mat[3][3] = {0};
float32_t accel_covar_mat[3][3] = {0};
float32_t accel_bias_covar_mat[3][3] = {0};

arm_matrix_instance_f32 estimate_covariance;
arm_matrix_instance_f32 observation_covariance;
arm_matrix_instance_f32 gyro_covariance;
arm_matrix_instance_f32 gyro_bias_covariance;
arm_matrix_instance_f32 accel_covariance;
arm_matrix_instance_f32 accel_bias_covariance;

float32_t Q_mat[15][15] = {0};
arm_matrix_instance_f32 Q;
float32_t F_mat[15][15] = {0};
arm_matrix_instance_f32 F;
float32_t F_mat_transpose[15][15] = {0};
arm_matrix_instance_f32 Ft;
float32_t G_mat[15][15] = {0};
arm_matrix_instance_f32 G;
float32_t K_mat[15][3] = {0};
arm_matrix_instance_f32 K;
float32_t H_mat[3][15] = {0};
arm_matrix_instance_f32 H;
float32_t H_mat_transpose[15][3] = {0};
arm_matrix_instance_f32 Ht;
float32_t PHt_mat[15][3] = {0};
arm_matrix_instance_f32 PHt;
float32_t inn_cov_mat[3][3] = {0};
arm_matrix_instance_f32 inn_cov;
float32_t inverse_in_cov_mat[3][3] = {0};
arm_matrix_instance_f32 inverse_in_cov;
float32_t aposteriori_mat[15][1] = {0};
arm_matrix_instance_f32 aposteriori_state;

float32_t identity_mat[15][15] = {0};
arm_matrix_instance_f32 identity_inst;
float32_t zero_mat[15][15] = {0};
arm_matrix_instance_f32 zero_inst;

float32_t estimate_covar_mid_mat[15][15] = {0};
arm_matrix_instance_f32 estimate_covar_mid;
float32_t estimate_covar_copy_mat[15][15] = {0};
arm_matrix_instance_f32 estimate_covar_copy;


float32_t gyro_bias[3] = {0};
float32_t kalman_gyro[3] = {0};
float32_t kalman_accel[3] = {0};
float32_t accelerometer_bias[3] = {0};
int initialized = 0;

void initKalman(quaternion_t initial_est, float32_t estimate_cov, float32_t gyro_cov, float32_t gyro_bias_cov,
		float32_t accel_proc_cov, float32_t accel_bias_cov, float32_t accel_obs_cov){
	estimate = initial_est;
	estimate_covariance = generateDiagonalMatrix(estimate_covar_mat[0], 15, estimate_cov);

	observation_covariance = generateDiagonalMatrix(observation_covar_mat[0], 3, accel_obs_cov);

	float32_t G1[3][3] = {0};
	diagonalMat(G1[0], 3, -1.0);
	float32_t G2[3][3] = {0};
	diagonalMat(G2[0], 3, 1.0);
	injectMatrix(G_mat[0], G1[0], 0, 9, 15, 3, 0);
	injectMatrix(G_mat[0], G2[0], 6, 3, 15, 3, 0);

	gyro_covariance = generateDiagonalMatrix(gyro_covar_mat[0], 3, gyro_cov);
	gyro_bias_covariance = generateDiagonalMatrix(gyro_bias_covar_mat[0], 3, gyro_bias_cov);
	accel_covariance = generateDiagonalMatrix(accel_covar_mat[0], 3, accel_proc_cov);
	accel_bias_covariance = generateDiagonalMatrix(accel_bias_covar_mat[0], 3, accel_bias_cov);

	arm_mat_init_f32(&Q, 15, 15, Q_mat[0]);
	arm_mat_init_f32(&F, 15, 15, F_mat[0]);
	arm_mat_init_f32(&Ft, 15, 15, F_mat_transpose[0]);
	arm_mat_init_f32(&G, 15, 15, G_mat[0]);
	arm_mat_init_f32(&K, 15, 3, K_mat[0]);
	arm_mat_init_f32(&H, 3, 15, H_mat[0]);
	arm_mat_init_f32(&Ht, 15, 3, H_mat_transpose[0]);
	arm_mat_init_f32(&PHt, 15, 3, PHt_mat[0]);
	arm_mat_init_f32(&aposteriori_state, 15, 1, aposteriori_mat[0]);
	arm_mat_init_f32(&inn_cov, 3, 3, inn_cov_mat[0]);
	arm_mat_init_f32(&inverse_in_cov, 3, 3, inverse_in_cov_mat[0]);
	arm_mat_init_f32(&estimate_covar_mid, 15, 15, estimate_covar_mid_mat[0]);
	arm_mat_init_f32(&estimate_covar_copy, 15, 15, estimate_covar_copy_mat[0]);
	arm_mat_init_f32(&zero_inst, 15, 15, zero_mat[0]);

	identity_inst = generateDiagonalMatrix(identity_mat[0], 15, 1.0);
	initialized = 1;
}

// assumes Q instance already initialized
arm_matrix_instance_f32 process_covariance(float32_t time_delta){
	diagonalMat(Q_mat[0], 15, 0.0);
	injectMatrix(Q_mat[0], addMatrices(createScaleMatrix(&gyro_covariance, time_delta, 3), createScaleMatrix(&gyro_bias_covariance, pow(time_delta, 3.0) / 3.0, 3), 3),
			0, 0, 15, 3, 1);
	injectMatrix(Q_mat[0], createScaleMatrix(&gyro_bias_covariance, -pow(time_delta, 2.0)/2.0, 3).pData,
				0, 9, 15, 3, 1);
	injectMatrix(Q_mat[0], addMatrices(createScaleMatrix(&accel_covariance, time_delta, 3), createScaleMatrix(&accel_bias_covariance, pow(time_delta, 3.0) / 3.0, 3), 3),
				3, 3, 15, 3, 1);
	injectMatrix(Q_mat[0], addMatrices(createScaleMatrix(&accel_bias_covariance, pow(time_delta, 4.0)/8.0, 3), createScaleMatrix(&accel_covariance, pow(time_delta, 2.0) / 2.0, 3), 3),
					3, 6, 15, 3, 1);
	injectMatrix(Q_mat[0], createScaleMatrix(&accel_bias_covariance, -pow(time_delta, 2.0)/2.0, 3).pData,
					3, 12, 15, 3, 1);
	injectMatrix(Q_mat[0], addMatrices(createScaleMatrix(&accel_covariance, pow(time_delta, 2.0)/2.0, 3), createScaleMatrix(&accel_bias_covariance, pow(time_delta, 4.0) / 8.0, 3), 3),
					6, 3, 15, 3, 1);
	injectMatrix(Q_mat[0], addMatrices(createScaleMatrix(&accel_covariance, pow(time_delta, 3.0)/3.0, 3), createScaleMatrix(&accel_bias_covariance, pow(time_delta, 5.0) / 20.0, 3), 3),
						6, 6, 15, 3, 1);
	injectMatrix(Q_mat[0], createScaleMatrix(&accel_bias_covariance, -pow(time_delta, 3.0)/6.0, 3).pData,
						6, 12, 15, 3, 1);
	injectMatrix(Q_mat[0], createScaleMatrix(&gyro_bias_covariance, -pow(time_delta, 2.0)/2.0, 3).pData,
					9,0, 15, 3, 1);
	injectMatrix(Q_mat[0], createScaleMatrix(&gyro_bias_covariance, time_delta, 3).pData,
						9, 9, 15, 3, 1);
	injectMatrix(Q_mat[0], createScaleMatrix(&accel_bias_covariance, -pow(time_delta, 2.0)/2.0, 3).pData,
							12, 3, 15, 3, 1);
	injectMatrix(Q_mat[0], createScaleMatrix(&accel_bias_covariance, -pow(time_delta, 3.0)/6.0, 3).pData,
							12, 6, 15, 3, 1);
	injectMatrix(Q_mat[0], createScaleMatrix(&accel_bias_covariance, time_delta, 3).pData,
								12, 12, 15, 3, 1);
	return Q;
}

void updateKalman(float32_t gyroMeas[3], float32_t accMeas[3], float32_t time_delta){
	subtractFromVector(gyroMeas, gyro_bias, 3);
	subtractFromVector(accMeas, accelerometer_bias, 3);
	subtractFromVector(kalman_gyro, kalman_gyro, 3);
	addToVector(kalman_gyro, gyroMeas, 3);
	// integrate angular velocity
	quaternion_t foldQuat = {0.0, {gyroMeas[0], gyroMeas[1], gyroMeas[2]}};
	addToQuat(&estimate, quatMultiplyScalar(quatMultiply(estimate, foldQuat), time_delta * 0.5));
	normalizeQuaternion(&estimate); // slightly different values


	// form process model
	arm_matrix_instance_f32 G1_inst = skewSymmetric(gyroMeas);
	float32_t G2_mat[3][3] = {0};
	arm_matrix_instance_f32 G2_inst;
	arm_matrix_instance_f32 G3_inst = quatToMatrix(estimate);
	arm_mat_init_f32(&G2_inst, 3, 3, G2_mat[0]);

	arm_mat_scale_f32(&G1_inst, -1.0, &G1_inst);
	arm_mat_scale_f32(&G3_inst, -1.0, &G3_inst);
	arm_matrix_instance_f32 G2_temp = skewSymmetric(accMeas);
	arm_mat_mult_f32(&G3_inst, &G2_temp, &G2_inst);
	//displayFloats4("00", gyroMeas[0], "01", gyroMeas[1], "02", gyroMeas[2], "03", G1_inst.pData[3]);

	injectMatrix(G_mat[0], G1_inst.pData, 0, 0, 15, 3, 1);
	injectMatrix(G_mat[0], G2_inst.pData, 3, 0, 15, 3, 0);
	injectMatrix(G_mat[0], G3_inst.pData, 3, 12, 15, 3, 1);
	arm_mat_scale_f32(&G, time_delta, &F); // something wrong here ish?
	arm_status stat = arm_mat_add_f32(&identity_inst, &F, &F);
	if(!stat){
		//displayInt("success", 1);
	}

	free(G2_temp.pData);

	//displayFloats4("00", G_mat[12][3], "01", G_mat[12][4], "02", G_mat[12][5], "03", G_mat[13][3]);
	//return;
	// update with a priori covariance
	//arm_mat_mult_f32(&estimate_covariance, &estimate_covar_mid, &estimate_covariance);
	arm_status stat2 = arm_mat_mult_f32(&F, &estimate_covariance, &estimate_covar_mid);
	if(stat2 != 0){
		displayInt("failure", stat2);
		return;
	}
	arm_mat_trans_f32(&F, &Ft);
	arm_mat_mult_f32(&estimate_covar_mid, &Ft, &estimate_covariance);
	arm_matrix_instance_f32 proc = process_covariance(time_delta);
	arm_mat_add_f32(&estimate_covariance, &proc, &estimate_covariance);


	// form Kalman gain
	arm_mat_sub_f32(&H, &H, &H); // zero the matrix

	quaternion_t inverseEst = quatInverse(estimate);
	float32_t vec[3] = {0.0, 0.0, 1.0};
	rotateVector3ByQuaternion(vec, inverseEst); // verify this is working
	injectMatrix(H_mat[0], skewSymmetric(vec).pData, 0, 0, 15, 3, 1);
	float32_t identityTemp[3][3] = {0}; // permanent 3x3 ident?
	injectMatrix(H_mat[0], generateDiagonalMatrix(identityTemp[0], 3, 1.0).pData, 0, 12, 15, 3, 0);

	arm_mat_trans_f32(&H, &Ht);
	arm_mat_mult_f32(&estimate_covariance, &Ht, &PHt);
	arm_mat_mult_f32(&H, &PHt, &inn_cov);
	arm_mat_add_f32(&inn_cov, &observation_covariance, &inn_cov);
	arm_mat_inverse_f32(&inn_cov, &inverse_in_cov);
	arm_mat_mult_f32(&PHt, &inverse_in_cov, &K);

	// update with a posteriori covariance
	arm_mat_mult_f32(&K, &H, &estimate_covar_mid);
	arm_mat_sub_f32(&identity_inst, &estimate_covar_mid, &estimate_covar_mid); // is this legit?
	arm_mat_mult_f32(&estimate_covar_mid, &estimate_covariance, &estimate_covar_copy);
	arm_mat_add_f32(&estimate_covar_copy, &zero_inst, &estimate_covariance); // copying the matrix kind of scuffed

	subtractFromVector(accMeas, vec, 3);
	arm_matrix_instance_f32 accMeasInst;
	arm_mat_init_f32(&accMeasInst, 3, 1, accMeas);
	arm_mat_mult_f32(&K, &accMeasInst, &aposteriori_state);

	quaternion_t apost_fold_quat = {1.0, {0.5 * aposteriori_mat[0][0], 0.5 * aposteriori_mat[1][0], 0.5 * aposteriori_mat[2][0]}};
	estimate = quatMultiply(estimate, apost_fold_quat);
	normalizeQuaternion(&estimate);
	float32_t addToGyroBias[3] = {aposteriori_mat[9][0], aposteriori_mat[10][0], aposteriori_mat[11][0]};
	float32_t addToAccelBias[3] = {aposteriori_mat[12][0], aposteriori_mat[13][0], aposteriori_mat[14][0]};
	addToVector(gyro_bias, addToGyroBias, 3);
	addToVector(accelerometer_bias, addToAccelBias, 3);
}

// has to be freed
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

// needs to be freed
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

// needs to be freed
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
// might wanna return matrix instance
float32_t* addMatrices(arm_matrix_instance_f32 in1, arm_matrix_instance_f32 in2, uint8_t size){
	arm_matrix_instance_f32 instanceOut;
	float32_t* mat = malloc(size * size * sizeof(float32_t));
	arm_mat_init_f32(&instanceOut, size, size, mat);
	arm_mat_add_f32(&in1, &in2, &instanceOut);
	free(in1.pData);
	free(in2.pData);
	return mat;
}



// assumes matrix already initialized with zeros
// use 1.0 as value to create an identity matrix
void diagonalMat(float32_t* matrix, uint8_t size, float32_t value){
	int i;
	for(i = 0;i < size;i++){
		matrix[i * size + i] = value;
	}
}

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

void addToQuat(quaternion_t* q1, quaternion_t q2){
	q1->w += q2.w;
	addToVector(q1->vec, q2.vec, 3);
}

void normalizeQuaternion(quaternion_t* q){
	*q = quatDivideScalar(*q, pow(quatSquareMag(*q), 0.5));
}

quaternion_t quatConjugate(quaternion_t q){
	quaternion_t conj = {q.w, {-q.vec[0], -q.vec[1], -q.vec[2]}};
	return conj;
}

quaternion_t quatMultiplyScalar(quaternion_t q, float32_t val){
	q.vec[0] = q.vec[0]*val;
	q.vec[1] = q.vec[1]*val;
	q.vec[2] = q.vec[2]*val;
	q.w = q.w*val;
	return q;
}

quaternion_t quatDivideScalar(quaternion_t q, float32_t val){
	q.vec[0] = q.vec[0]/val;
	q.vec[1] = q.vec[1]/val;
	q.vec[2] = q.vec[2]/val;
	q.w = q.w/val;
	return q;
}

float32_t quatSquareMag(quaternion_t q){
	return q.vec[0] * q.vec[0] + q.vec[1] * q.vec[1] + q.vec[2] * q.vec[2] + q.w * q.w;
}
