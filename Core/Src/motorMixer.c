/*
 * motorMixer.c
 *
 *  Created on: Aug 1, 2024
 *      Author: vedpa
 */
#include "motorMixer.h"
#include "imu.h"
#include "main.h"
#include "math.h"
#include "expresslrs.h"

//uint16_t motorOut[MOTOR_COUNT] = {INIT_THROTTLE_MIN, INIT_THROTTLE_MIN, INIT_THROTTLE_MIN, INIT_THROTTLE_MIN}; // values sent to motors -> between 0 and 2048
rateSetpoint_t desiredRate = {0.0, 0.0, 0.0}; // roll, pitch, yaw
angleSetpoint_t desiredAngle = {0.0, 0.0, 0.0};
float32_t desiredAccel[3] = {0.0, 0.0, 0.0};
float32_t eulerAttitude[3] = {0.0};
float outThrott = 0.0;
float32_t throttleTarget = 0.0;

int16_t rcIn[4] = {0, 0, 0, 0}; // roll, pitch, throttle, yaw -> same format as input rcData
int16_t oldRCIn[4] = {0, 0, 0, 0};
outRates_t motorSetpoints = {0.0, 0.0, 0.0, 0.0}; // values between -1 and 1 for roll, pitch, throttle, yaw
float32_t velEst = 0.0;

float32_t hoverThrottle = -1.0;
float32_t lastHoverableThrott = 0.0;

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

PIDController rollRatePID = {PID_RATE_KP, PID_RATE_KI, PID_RATE_KD,
						PID_RATE_TAU,
						PID_RATE_LIM_MIN, PID_RATE_LIM_MAX,
						PID_RATE_LIM_MIN_INT, PID_RATE_LIM_MAX_INT,
						SAMPLE_TIME_KALMAN };
PIDController pitchRatePID = {PID_RATE_KP, PID_RATE_KI, PID_RATE_KD,
						PID_RATE_TAU,
						PID_RATE_LIM_MIN, PID_RATE_LIM_MAX,
						PID_RATE_LIM_MIN_INT, PID_RATE_LIM_MAX_INT,
						SAMPLE_TIME_KALMAN };
PIDController yawRatePID = {PID_RATE_KP, PID_RATE_KI, PID_RATE_KD,
						PID_RATE_TAU,
						PID_RATE_LIM_MIN, PID_RATE_LIM_MAX,
						PID_RATE_LIM_MIN_INT, PID_RATE_LIM_MAX_INT,
						SAMPLE_TIME_KALMAN };
PIDController throttlePID = {PID_THROTTLE_KP, PID_THROTTLE_KI, PID_THROTTLE_KD,
						PID_THROTTLE_TAU,
						PID_THROTTLE_LIM_MIN, PID_THROTTLE_LIM_MAX,
						PID_THROTTLE_LIM_MIN_INT, PID_THROTTLE_LIM_MAX_INT,
						SAMPLE_TIME_S };

void motorMixerInit(){
	PIDController_Init(&rollPID);
	PIDController_Init(&pitchPID);
	PIDController_Init(&yawPID);
	PIDController_Init(&rollRatePID);
	PIDController_Init(&pitchRatePID);
	PIDController_Init(&yawRatePID);

	PIDController_Init(&throttlePID);
}

void motorMixerUpdate(uint16_t* rcData, uint16_t* motorOut, float32_t* currentRate, float32_t* currentAccel, quaternion_t attitude){
	getRCInputs(rcData);

	float32_t pitchRate = (float32_t)rcIn[1]/125.0;
	float32_t rollRate = (float32_t)rcIn[0]/125.0;
	throttleTarget = (float32_t)rcIn[2]/500.0;

	if(!isDisconnected() && throttleTarget > 0){
		//motorSetpoints.throttle = throttleTarget;
		if(absVal(throttleTarget - 1.0) < 0.15){
			throttleTarget = 1.0;
		}
		getDesiredThrottle(throttleTarget, attitude, currentAccel);
	}else{
		motorSetpoints.throttle = 0;
	}
	//desiredRate.ratePitch = pitchRate;
	//desiredRate.rateRoll = rollRate;
	if(motorSetpoints.throttle >= 0.01){
		if(hoverThrottle < 0.0){
			float32_t vec[3] = {0.0, 0.0, -1.0};
			quaternion_t inverseEst = quatInverse(attitude);
			rotateVector3ByQuaternion(vec, inverseEst); // verify this is working
			float32_t dot = vec[0] * accel[0] + vec[1] * accel[1] + vec[2] * accel[2];
			if(dot < 1.0){
				lastHoverableThrott = motorSetpoints.throttle;
			}else if(dot > 1.1){
				hoverThrottle = lastHoverableThrott;
			}
		}
		achieveDesiredRates(currentRate);

	}else{
		motorSetpoints.roll = 0;
		motorSetpoints.pitch = 0;
		motorSetpoints.yaw = 0;
		hoverThrottle = -1.0;
		lastHoverableThrott = 0.0;
	}

	//motorSetpoints.pitch = (float32_t)rcIn[1]/1000.0;
	//motorSetpoints.roll = (float32_t)rcIn[0]/1000.0;
	//motorSetpoints.yaw = (float32_t)rcIn[3]/1000.0;
	getMotorOutputs(motorSetpoints, motorOut);
}

void motorMixerOuterUpdate(quaternion_t attitude, float32_t* accel){
	float32_t pitchRate = (float32_t)rcIn[1]/500.0;
	float32_t rollRate = (float32_t)rcIn[0]/500.0;
	float32_t yawRate = (float32_t)rcIn[3]/500.0;



	desiredAngle.pitch = pitchRate;
	desiredAngle.roll = rollRate;
	quatToEuler(attitude, eulerAttitude);
	if(motorSetpoints.throttle < 0.01){
		desiredAngle.yaw = eulerAttitude[2];
	}else{
		desiredAngle.yaw += yawRate * SAMPLE_TIME_KALMAN;
	}

	getDesiredRates(eulerAttitude);
}

void getDesiredThrottle(float32_t dotTarget, quaternion_t attitude, float32_t* accel){
	float32_t vec[3] = {0.0, 0.0, -1.0};
	quaternion_t inverseEst = quatInverse(attitude);
	rotateVector3ByQuaternion(vec, inverseEst); // verify this is working
	float32_t dot = vec[0] * accel[0] + vec[1] * accel[1] + vec[2] * accel[2];
	float32_t vec2[3] = {0.0, 0.0, 1.0};
	rotateVector3ByQuaternion(vec2, inverseEst);
	float32_t thrustScale = 1.0;
	if(absVal(vec2[2]) > 0.01){
		thrustScale = 1.0/vec2[2];
	}



	PIDController_Update(&throttlePID, dotTarget, dot);
	outThrott = throttlePID.out;

	if(hoverThrottle > 0){

		motorSetpoints.throttle = dotTarget * hoverThrottle * thrustScale + throttlePID.out;
	}else{
		motorSetpoints.throttle = dotTarget * 0.5;
	}


}

void getDesiredRatesAccel(float32_t* accel){
	//quatToEuler(attitude, eulerAttitude);
	// convert to roll pitch yaw
	PIDController_Update(&rollRatePID, -desiredAccel[1], -accel[1]);
	PIDController_Update(&pitchRatePID, desiredAccel[0], accel[0]);
	//PIDController_Update(&yawRatePID, desiredAccel[2], eulerAttitude[2]);

	desiredRate.rateRoll = rollRatePID.out;
	desiredRate.ratePitch = pitchRatePID.out;
	//desiredRate.rateYaw = yawRatePID.out;
}

void getDesiredRates(float32_t* eulerAtt){

	// convert to roll pitch yaw
	PIDController_Update(&rollRatePID, (desiredAngle.roll), eulerAtt[0]);
	PIDController_Update(&pitchRatePID, desiredAngle.pitch, eulerAtt[1]);
	PIDController_Update(&yawRatePID, desiredAngle.yaw, eulerAtt[2]);

	desiredRate.rateRoll = rollRatePID.out;
	desiredRate.ratePitch = pitchRatePID.out;
	desiredRate.rateYaw = yawRatePID.out;
}

void achieveDesiredRates(float32_t* currentRate){
	/*if(absVal(desiredRate.rateRoll - currentRate[0]) < 0.01){
		currentRate[0] = desiredRate.rateRoll;
	}
	if(absVal(desiredRate.ratePitch - currentRate[1]) < 0.01){
		currentRate[1] = desiredRate.ratePitch;
	}
	if(absVal(desiredRate.rateYaw - currentRate[2]) < 0.01){
		currentRate[2] = desiredRate.rateYaw;
	}*/
	PIDController_Update(&rollPID, desiredRate.rateRoll, currentRate[0]);
	PIDController_Update(&pitchPID, desiredRate.ratePitch, currentRate[1]);
	PIDController_Update(&yawPID, desiredRate.rateYaw, currentRate[2]);

	motorSetpoints.roll = rollPID.out;
	motorSetpoints.pitch = pitchPID.out;
	motorSetpoints.yaw = yawPID.out;
}

float32_t absVal(float32_t val){
	if(val < 0){
		return -val;
	}
	return val;
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
	/*if(set.throttle > 0.01){
		out[0] += 50;
		out[3] += 50;
	}*/
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


