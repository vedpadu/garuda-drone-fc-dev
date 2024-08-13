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
#include "math_util.h"

#define ESTIMATE_COV 0.0
#define GYRO_COV 0.004
#define GYRO_BIAS_COV 0.000005
#define ACCEL_PROC_COV 0.005
#define ACCEL_BIAS_COV 0.0
#define ACCEL_OBS_COV 0.01

#define ACCEL_HEALTH_COEFFICIENT 15.0

extern quaternion_t estimate;
extern int kalman_initialized;

arm_matrix_instance_f32 processCovariance(float32_t time_delta);

float32_t getAccelHealth(float32_t* acc, float32_t* gyr);

void initKalman(quaternion_t initial_est, float32_t estimate_cov, float32_t gyro_cov, float32_t gyro_bias_cov,
		float32_t accel_proc_cov, float32_t accel_bias_cov, float32_t accel_obs_cov);
void updateKalman(float32_t gyroMeas[3], float32_t accMeas[3], float32_t time_delta);


#endif /* INC_KALMAN_H_ */
