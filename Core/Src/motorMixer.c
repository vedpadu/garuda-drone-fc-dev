/*
 * motorMixer.c
 *
 *  Created on: Aug 1, 2024
 *      Author: vedpa
 */
#include "motorMixer.h"
#include "imu.h"

//uint16_t motorOut[MOTOR_COUNT] = {INIT_THROTTLE_MIN, INIT_THROTTLE_MIN, INIT_THROTTLE_MIN, INIT_THROTTLE_MIN}; // values sent to motors -> between 0 and 2048
rateSetpoint_t desiredRate = {0.0, 0.0, 0.0}; // roll, pitch, yaw

int16_t rcIn[4] = {0, 0, 0, 0}; // roll, pitch, throttle, yaw -> same format as input rcData
int16_t oldRCIn[4] = {0, 0, 0, 0};
outRates_t motorSetpoints = {0.0, 0.0, 0.0, 0.0}; // values between -1 and 1 for roll, pitch, throttle, yaw

PIDController rollPID = {PID_KP, PID_KI, PID_KD,
						PID_TAU,
						PID_LIM_MIN, PID_LIM_MAX,
						PID_LIM_MIN_INT, PID_LIM_MAX_INT,
						SAMPLE_TIME_S };
PIDController pitchPID = {PID_KP, PID_KI, PID_KD,
						PID_TAU,
						PID_LIM_MIN, PID_LIM_MAX,
						PID_LIM_MIN_INT, PID_LIM_MAX_INT,
						SAMPLE_TIME_S };
PIDController yawPID = {PID_KP, PID_KI, PID_KD,
						PID_TAU,
						PID_LIM_MIN, PID_LIM_MAX,
						PID_LIM_MIN_INT, PID_LIM_MAX_INT,
						SAMPLE_TIME_S };

void motorMixerInit(){
	PIDController_Init(&rollPID);
	PIDController_Init(&pitchPID);
	PIDController_Init(&yawPID);
}

void motorMixerUpdate(uint16_t* rcData, uint16_t* motorOut, float32_t* currentRate){
	getRCInputs(rcData);
	motorSetpoints.throttle = (float32_t)rcIn[2]/1000.0;
	if(motorSetpoints.throttle > 0.01){
		achieveDesiredRates(currentRate);
	}else{
		motorSetpoints.roll = 0;
		motorSetpoints.pitch = 0;
		motorSetpoints.yaw = 0;
	}

	//motorSetpoints.pitch = (float32_t)rcIn[1]/1000.0;
	//motorSetpoints.roll = (float32_t)rcIn[0]/1000.0;
	//motorSetpoints.yaw = (float32_t)rcIn[3]/1000.0;
	getMotorOutputs(motorSetpoints, motorOut);
}

void achieveDesiredRates(float32_t* currentRate){
	PIDController_Update(&rollPID, desiredRate.rateRoll, currentRate[0]);
	PIDController_Update(&pitchPID, desiredRate.ratePitch, currentRate[1]);
	PIDController_Update(&yawPID, desiredRate.rateYaw, currentRate[2]);

	motorSetpoints.roll = clamp(rollPID.out, 1.0, -1.0); // don't have to clamp
	motorSetpoints.pitch = clamp(pitchPID.out, 1.0, -1.0);
	motorSetpoints.yaw = clamp(yawPID.out, 1.0, -1.0);
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

// TODO: switch handling?
void getRCInputs(uint16_t* rcData){
	int i;
	for(i = 0;i < MOTOR_COUNT;i++){
		if(i != 2){ // not throttle
			rcIn[i] = (int16_t)rcData[i] - 1500;
			if(rcIn[i] > DEADBAND + 30){
				if(oldRCIn[i] < rcIn[i]){
					if(rcIn[i] - oldRCIn[i] > 30){
						rcIn[i] = oldRCIn[i] + 30;
					}
				}else{
					if(oldRCIn[i] - rcIn[i] > 30){
						rcIn[i] = oldRCIn[i] - 30;
					}
				}
			}
			if(rcIn[i] < DEADBAND && rcIn[i] > -DEADBAND){
				rcIn[i] = 0;
			}
		}else{
			rcIn[i] = (int16_t)rcData[i] - 989;
			if(rcIn[i] > DEADBAND + 30){
				if(oldRCIn[i] < rcIn[i]){
					if(rcIn[i] - oldRCIn[i] > 30){
						rcIn[i] = oldRCIn[i] + 30;
					}
				}else{
					if(oldRCIn[i] - rcIn[i] > 30){
						rcIn[i] = oldRCIn[i] - 30;
					}
				}
			}
			if(rcIn[i] < DEADBAND){
				rcIn[i] = 0;
			}
		}
		oldRCIn[i] = rcIn[i];
	}
}



void getMotorOutputs(outRates_t set, uint16_t* motorOut){
	float32_t out[MOTOR_COUNT] = {0};
	out[0] = (set.throttle + set.roll + set.pitch - set.yaw) * 2000;
	out[1] = (set.throttle + set.roll - set.pitch + set.yaw) * 2000;
	out[2] = (set.throttle - set.roll - set.pitch - set.yaw) * 2000;
	out[3] = (set.throttle - set.roll + set.pitch + set.yaw) * 2000;
	// at extremes these motors will not be able to match exactly
	int i;
	for(i = 0;i < MOTOR_COUNT;i++){
		if(out[i] > 1999.0){
			out[i] = 1999.0;
		}
		if(out[i] < 0.0){
			out[i] = 0.0;
		}
		motorOut[i] = (uint16_t)out[i];
		motorOut[i] += 48;
	}
}


