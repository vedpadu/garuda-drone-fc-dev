/*
 * motorMixer.h
 * Turns RC data to motor outputs
 * Executes control loops
 *
 *  Created on: Aug 1, 2024
 *      Author: vedpadu
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "pid.h"
#include "math_util.h"
#include "expresslrs.h"
#include "esc.h"
#include "main.h"
#include "elrs_rcdata_handler.h"

// pid constants
#define PID_MOTOR_KP  0.05 // maybe raise this? //0.028 // 0.045
#define PID_MOTOR_KI  0.07 // 0.02
#define PID_MOTOR_KD  0.0055 //0.023 //0.16 // 0.0045

#define PID_RATE_KP  3.5
#define PID_RATE_KI  0.0// 0.5
#define PID_RATE_KD  -0.4

#define PID_THROTTLE_KP  0.0 // 0.1
#define PID_THROTTLE_KI  0.0
#define PID_THROTTLE_KD  0.0 // 0.01


// max outputs for pid controllers
#define PID_MOTOR_LIM_MIN -1.0
#define PID_MOTOR_LIM_MAX  1.0

#define PID_RATE_LIM_MIN -10.0
#define PID_RATE_LIM_MAX  10.0

#define PID_THROTTLE_LIM_MIN -1.0
#define PID_THROTTLE_LIM_MAX  1.0


// integral limiting for pid controllers
#define PID_MOTOR_LIM_MIN_INT -0.005
#define PID_MOTOR_LIM_MAX_INT  0.005

#define PID_RATE_LIM_MIN_INT -0.2
#define PID_RATE_LIM_MAX_INT  0.2

#define PID_THROTTLE_LIM_MIN_INT -0.01
#define PID_THROTTLE_LIM_MAX_INT  0.01


// low pass filter constants + sample times
#define PID_MOTOR_TAU 0.01
#define PID_RATE_TAU 0.005
#define PID_THROTTLE_TAU 0.01

#define SAMPLE_TIME_INNER 0.002
#define SAMPLE_TIME_OUTER (1.0 / KALMAN_FILTER_SAMPLE_RATE)


#define DEADBAND 20
#define THROTTLE_STICK_NO_INPUT 988
#define REGULAR_STICK_NO_INPUT 1500
#define MAX_INPUT_VELOCITY 30
#define SWITCH_THRESHOLD 1250

#define ACCEL_TAKEOFF_THRESHOLD 0.1
#define HOVER_THROTTLE_RANGE 0.1

#define THROTTLE_STICK_SCALE 500.0
#define HOVER_THROTTLE_NOT_SET -1.0

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


void controlsInit();
void controlsInnerLoop(uint16_t* rcData, uint16_t* motorOut, float32_t* currentRate, float32_t* currentAccel, quaternion_t attitude);
void handleRCInputs(uint16_t* rcData);
void getMotorOutputs(outRates_t set, uint16_t* motorOut);
void findHoverThrottle(quaternion_t attitude, float32_t* currentAccel);

void achieveDesiredRates(float32_t* currentRate);
void getDesiredRates(float32_t* eulerAtt);
void controlsOuterUpdate(quaternion_t attitude, float32_t* accel);
void getDesiredThrottle(float32_t dotTarget, quaternion_t attitude, float32_t* accel);

#endif /* INC_CONTROL_H_ */
