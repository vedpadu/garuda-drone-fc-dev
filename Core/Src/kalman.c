/*
 * kalman.c
 * MEKF
 *
 *  Created on: Jul 24, 2024
 *      Author: vedpadu
 */
#include "kalman.h"

quaternion_t estimate;

// system covariance matrices
float32_t estimate_covar_mat[15][15] = {0};
float32_t observation_covar_mat[3][3] = {0};

// sensor covariance matrices
float32_t gyro_covar_mat[3][3] = {0};
float32_t gyro_bias_covar_mat[3][3] = {0};
float32_t accel_covar_mat[3][3] = {0};
float32_t accel_bias_covar_mat[3][3] = {0};

// kalman matrices
float32_t Q_mat[15][15] = {0};
float32_t F_mat[15][15] = {0};
float32_t F_mat_transpose[15][15] = {0};
float32_t G_mat[15][15] = {0};
float32_t K_mat[15][3] = {0};
float32_t H_mat[3][15] = {0};
float32_t H_mat_transpose[15][3] = {0};
float32_t PHt_mat[15][3] = {0};
float32_t inn_cov_mat[3][3] = {0};
float32_t inverse_in_cov_mat[3][3] = {0};
float32_t aposteriori_mat[15][1] = {0};

// util matrices
float32_t identity_mat[15][15] = {0};
float32_t zero_mat[15][15] = {0};

// transition matrices
float32_t estimate_covar_mid_mat[15][15] = {0};
float32_t estimate_covar_copy_mat[15][15] = {0};

// arm instances
arm_matrix_instance_f32 estimate_covariance;
arm_matrix_instance_f32 observation_covariance;
arm_matrix_instance_f32 gyro_covariance;
arm_matrix_instance_f32 gyro_bias_covariance;
arm_matrix_instance_f32 accel_covariance;
arm_matrix_instance_f32 accel_bias_covariance;
arm_matrix_instance_f32 Q;
arm_matrix_instance_f32 F;
arm_matrix_instance_f32 Ft;
arm_matrix_instance_f32 G;
arm_matrix_instance_f32 K;
arm_matrix_instance_f32 H;
arm_matrix_instance_f32 Ht;
arm_matrix_instance_f32 PHt;
arm_matrix_instance_f32 inn_cov;
arm_matrix_instance_f32 inverse_in_cov;
arm_matrix_instance_f32 aposteriori_state;
arm_matrix_instance_f32 identity_inst;
arm_matrix_instance_f32 zero_inst;
arm_matrix_instance_f32 estimate_covar_mid;
arm_matrix_instance_f32 estimate_covar_copy;

float32_t gyro_bias[3] = {0};
float32_t accelerometer_bias[3] = {0};
float32_t kalman_gyro[3] = {0};
float32_t kalman_accel[3] = {0};

int kalman_initialized = 0;

void initKalman(quaternion_t initialEst, float32_t estimateCov, float32_t gyroCov, float32_t gyroBiasCov,
		float32_t accelProcCov, float32_t accelBiasCov, float32_t accelObsCov){
	estimate = initialEst;
	estimate_covariance = generateDiagonalMatrix(estimate_covar_mat[0], 15, estimateCov);

	observation_covariance = generateDiagonalMatrix(observation_covar_mat[0], 3, accelObsCov);

	float32_t G1[3][3] = {0};
	diagonalMat(G1[0], 3, -1.0);
	float32_t G2[3][3] = {0};
	diagonalMat(G2[0], 3, 1.0);
	injectMatrix(G_mat[0], G1[0], 0, 9, 15, 3, 0);
	injectMatrix(G_mat[0], G2[0], 6, 3, 15, 3, 0);

	gyro_covariance = generateDiagonalMatrix(gyro_covar_mat[0], 3, gyroCov);
	gyro_bias_covariance = generateDiagonalMatrix(gyro_bias_covar_mat[0], 3, gyroBiasCov);
	accel_covariance = generateDiagonalMatrix(accel_covar_mat[0], 3, accelProcCov);
	accel_bias_covariance = generateDiagonalMatrix(accel_bias_covar_mat[0], 3, accelBiasCov);

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
	kalman_initialized = 1;
}

// assumes Q instance already initialized
arm_matrix_instance_f32 processCovariance(float32_t time_delta){
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

// TODO: check function statuses
void updateKalman(float32_t gyroMeas[3], float32_t accMeas[3], float32_t timeDelta){

	// create a copy of the measurement so that the gyro and accelerometer raw measurements are not changed
	subtractFromVector(kalman_accel, kalman_accel, 3);
	addToVector(kalman_accel, accMeas, 3);
	subtractFromVector(kalman_accel, accelerometer_bias, 3);

	// TODO: this gyro bias should be public hypothetically
	subtractFromVector(kalman_gyro, kalman_gyro, 3);
	addToVector(kalman_gyro, gyroMeas, 3);
	subtractFromVector(kalman_gyro, gyro_bias, 3);

	// integrate angular velocity
	quaternion_t foldQuat = {0.0, {kalman_gyro[0], kalman_gyro[1], kalman_gyro[2]}};
	addToQuat(&estimate, quatMultiplyScalar(quatMultiply(estimate, foldQuat), timeDelta * 0.5));
	normalizeQuaternion(&estimate);

	// form process model
	arm_matrix_instance_f32 G1_inst = skewSymmetric(kalman_gyro);
	float32_t G2_mat[3][3] = {0};
	arm_matrix_instance_f32 G2_inst;
	arm_matrix_instance_f32 G3_inst = quatToMatrix(estimate);
	arm_mat_init_f32(&G2_inst, 3, 3, G2_mat[0]);

	arm_mat_scale_f32(&G1_inst, -1.0, &G1_inst);
	arm_mat_scale_f32(&G3_inst, -1.0, &G3_inst);
	arm_matrix_instance_f32 G2_temp = skewSymmetric(kalman_accel);
	arm_mat_mult_f32(&G3_inst, &G2_temp, &G2_inst);

	injectMatrix(G_mat[0], G1_inst.pData, 0, 0, 15, 3, 1);
	injectMatrix(G_mat[0], G2_inst.pData, 3, 0, 15, 3, 0);
	injectMatrix(G_mat[0], G3_inst.pData, 3, 12, 15, 3, 1);
	arm_mat_scale_f32(&G, timeDelta, &F); // something wrong here ish?
	arm_mat_add_f32(&identity_inst, &F, &F);

	free(G2_temp.pData);

	// update with a priori covariance
	arm_mat_mult_f32(&F, &estimate_covariance, &estimate_covar_mid);
	arm_mat_trans_f32(&F, &Ft);
	arm_mat_mult_f32(&estimate_covar_mid, &Ft, &estimate_covariance);
	arm_matrix_instance_f32 proc = processCovariance(timeDelta); // why does this have to be called?
	arm_mat_add_f32(&estimate_covariance, &proc, &estimate_covariance);


	// form Kalman gain
	arm_mat_sub_f32(&H, &H, &H); // zero the matrix

	quaternion_t inverseEst = quatInverse(estimate);
	float32_t vec[3] = {0.0, 0.0, -1.0};
	rotateVector3ByQuaternion(vec, inverseEst);
	injectMatrix(H_mat[0], skewSymmetric(vec).pData, 0, 0, 15, 3, 1);
	float32_t identityTemp[3][3] = {0}; // permanent 3x3 ident?
	injectMatrix(H_mat[0], generateDiagonalMatrix(identityTemp[0], 3, 1.0).pData, 0, 12, 15, 3, 0);

	arm_mat_trans_f32(&H, &Ht);
	arm_mat_mult_f32(&estimate_covariance, &Ht, &PHt);
	arm_mat_mult_f32(&H, &PHt, &inn_cov);
	arm_matrix_instance_f32 curr_obs_cov = createScaleMatrix(&observation_covariance, getAccelHealth(kalman_accel, kalman_gyro), 3);
	arm_mat_add_f32(&inn_cov, &curr_obs_cov, &inn_cov);
	arm_mat_inverse_f32(&inn_cov, &inverse_in_cov);
	arm_mat_mult_f32(&PHt, &inverse_in_cov, &K);
	free(curr_obs_cov.pData);

	// update with a posteriori covariance
	arm_mat_mult_f32(&K, &H, &estimate_covar_mid);
	arm_mat_sub_f32(&identity_inst, &estimate_covar_mid, &estimate_covar_mid);
	arm_mat_mult_f32(&estimate_covar_mid, &estimate_covariance, &estimate_covar_copy);
	arm_mat_add_f32(&estimate_covar_copy, &zero_inst, &estimate_covariance); // copying the matrix kind of scuffed

	subtractFromVector(kalman_accel, vec, 3);
	arm_matrix_instance_f32 accMeasInst;
	arm_mat_init_f32(&accMeasInst, 3, 1, kalman_accel);
	arm_mat_mult_f32(&K, &accMeasInst, &aposteriori_state);

	quaternion_t apost_fold_quat = {1.0, {0.5 * aposteriori_mat[0][0], 0.5 * aposteriori_mat[1][0], 0.5 * aposteriori_mat[2][0]}};
	estimate = quatMultiply(estimate, apost_fold_quat);
	normalizeQuaternion(&estimate);
	float32_t addToGyroBias[3] = {aposteriori_mat[9][0], aposteriori_mat[10][0], aposteriori_mat[11][0]};
	float32_t addToAccelBias[3] = {aposteriori_mat[12][0], aposteriori_mat[13][0], aposteriori_mat[14][0]};
	addToVector(gyro_bias, addToGyroBias, 3);
	addToVector(accelerometer_bias, addToAccelBias, 3);
}

// If the accelerometer reads values that are not stationary, generate a larger scalar to increase the covariance
float32_t getAccelHealth(float32_t* acc, float32_t* gyr){
	float32_t mag = acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2];
	float32_t diff = mag - 1.0;
	diff = absVal(diff);
	return 1.0 + (diff * ACCEL_HEALTH_COEFFICIENT);
}

