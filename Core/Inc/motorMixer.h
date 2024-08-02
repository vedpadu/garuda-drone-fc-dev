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

#define PID_KP  0.012 // maybe raise this?
#define PID_KI  0.011
#define PID_KD  0.002

#define PID_RATE_KP  24.0
#define PID_RATE_KI  4.0
#define PID_RATE_KD  0.0

#define PID_RATE_LIM_MIN -8.0
#define PID_RATE_LIM_MAX  8.0

#define PID_RATE_LIM_MIN_INT -3.0
#define PID_RATE_LIM_MAX_INT  3.0

#define PID_TAU 0.02

#define PID_LIM_MIN -1.0
#define PID_LIM_MAX  1.0

#define PID_LIM_MIN_INT -0.2
#define PID_LIM_MAX_INT  0.2

#define SAMPLE_TIME_S 0.015873 // TODO: make this sample rate work pwease, im assuming a phase issue

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

void motorMixerInit();
void motorMixerUpdate(uint16_t* rcData, uint16_t* motorOut, float32_t* currentRate, quaternion_t attitude);
void getRCInputs(uint16_t* rcData);
void getMotorOutputs(outRates_t set, uint16_t* motorOut);
float32_t clamp(float32_t in, float32_t max, float32_t min);
void achieveDesiredRates(float32_t* currentRate);
void quatToEuler(quaternion_t q, float32_t* outEuler);
void getDesiredRates(quaternion_t attitude);

#endif /* INC_MOTORMIXER_H_ */
