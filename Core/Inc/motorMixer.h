/*
 * motorMixer.h
 *
 *  Created on: Aug 1, 2024
 *      Author: vedpa
 */

#ifndef INC_MOTORMIXER_H_
#define INC_MOTORMIXER_H_

#include "pid.h"
#include "kalman.h"
#include "esc.h"

extern float32_t eulerAttitude[3];

#define PID_KP  0.045 // maybe raise this? //0.028 // 0.05
#define PID_KI  0.027 // 0.02
#define PID_KD  0.16 //0.023 //0.15

#define PID_RATE_KP  10.0
#define PID_RATE_KI  1.0
#define PID_RATE_KD  -2.0

#define PID_RATE_LIM_MIN -20.0
#define PID_RATE_LIM_MAX  20.0

#define PID_RATE_LIM_MIN_INT -0.05
#define PID_RATE_LIM_MAX_INT  0.05

#define PID_TAU 0.1 //0.1 // too high?
#define PID_RATE_TAU 0.1

#define PID_LIM_MIN -1.0
#define PID_LIM_MAX  1.0

#define PID_LIM_MIN_INT -0.045
#define PID_LIM_MAX_INT  0.045

#define SAMPLE_TIME_S 0.002
#define SAMPLE_TIME_KALMAN 0.01

#define DEADBAND 20

typedef struct RCInputs_s{
	int16_t roll;
	int16_t pitch;
	uint16_t throttle;
	int16_t yaw;
}RCInputs_t;

typedef struct outRates_s{
	float roll;
	float pitch;
	float yaw;
	float throttle;
}outRates_t;

typedef struct rateSetpoint_s {
	float rateRoll;
	float ratePitch;
	float rateYaw;
}rateSetpoint_t;

typedef struct angleSetpoint_s {
	float roll;
	float pitch;
	float yaw;
}angleSetpoint_t;

extern outRates_t motorSetpoints;
extern rateSetpoint_t desiredRate;

void motorMixerInit();
void motorMixerUpdate(uint16_t* rcData, uint16_t* motorOut, float32_t* currentRate, float32_t* currentAccel, quaternion_t attitude);
void getDesiredRatesAccel(float32_t* accel);
void getRCInputs(uint16_t* rcData);
void getMotorOutputs(outRates_t set, uint16_t* motorOut);
float32_t clamp(float32_t in, float32_t max, float32_t min);
void achieveDesiredRates(float32_t* currentRate);
void getDesiredRates(quaternion_t attitude);
void motorMixerOuterUpdate(quaternion_t attitude);
float32_t absVal(float32_t val);

#endif /* INC_MOTORMIXER_H_ */
