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
#include "outputHandler.h"

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

	initOutputHandler(18.0 * SAMPLE_TIME_S, 3.0 * SAMPLE_TIME_S);
}

void motorMixerUpdate(uint16_t* rcData, uint16_t* motorOut, float32_t* currentRate, float32_t* currentAccel, quaternion_t attitude){
	getRCInputs(rcData);

	float32_t pitchRate = (float32_t)rcIn[1]/125.0;
	float32_t rollRate = (float32_t)rcIn[0]/125.0;
	throttleTarget = (float32_t)rcIn[2]/500.0;

	if(!isDisconnected() && throttleTarget > 0){
		// hovering area... hypothetically -> might be making stuff worse
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
		// zero everything on no throttle
		motorSetpoints.roll = 0;
		motorSetpoints.pitch = 0;
		motorSetpoints.yaw = 0;
		hoverThrottle = -1.0;
		lastHoverableThrott = 0.0;
	}

	getMotorOutputs(motorSetpoints, motorOut);
}

void motorMixerOuterUpdate(quaternion_t attitude, float32_t* accel){
	float32_t pitchRate = -(float32_t)rcIn[1]/500.0;
	float32_t rollRate = (float32_t)rcIn[0]/500.0;
	float32_t yawRate = (float32_t)rcIn[3]/200.0;



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

	// TODO: tune this ctrlr
	PIDController_Update(&throttlePID, dotTarget, dot);
	outThrott = throttlePID.out;

	if(hoverThrottle > 0){
		// only can help alt hold, not perfect as throttle not linearized and stuff
		motorSetpoints.throttle = dotTarget * hoverThrottle * thrustScale + throttlePID.out;
	}else{
		motorSetpoints.throttle = dotTarget * 0.5;
	}


}

void getDesiredRates(float32_t* eulerAtt){
	PIDController_Update(&rollRatePID, (desiredAngle.roll), eulerAtt[0]);
	PIDController_Update(&pitchRatePID, desiredAngle.pitch, eulerAtt[1]);
	PIDController_Update(&yawRatePID, desiredAngle.yaw, eulerAtt[2]);

	desiredRate.rateRoll = rollRatePID.out;
	desiredRate.ratePitch = pitchRatePID.out;
	desiredRate.rateYaw = yawRatePID.out;
}

void achieveDesiredRates(float32_t* currentRate){
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
	outputUpdate(&set);
	/*float32_t throttle_avg_max = 0.5 * 0.5 + 0.5 * set.throttle;;
	if(hoverThrottle > 0.0){
		throttle_avg_max = 0.5 * hoverThrottle + 0.5 * set.throttle;
	}


	throttle_avg_max = clamp(throttle_avg_max, 1.0, set.throttle);
	float32_t throttle_best_rpy = 0.5;
	if(throttle_avg_max < throttle_best_rpy){
		throttle_best_rpy = throttle_avg_max;
	}

	out[0] = set.roll + set.pitch;
	out[1] = set.roll - set.pitch;
	out[2] = -set.roll - set.pitch;
	out[3] = -set.roll + set.pitch;

	float32_t yawArr[MOTOR_COUNT] = {-set.yaw, set.yaw, -set.yaw, set.yaw};

	float32_t room[MOTOR_COUNT] = {0};
	float32_t yawAllowed = 10000.0;
	int i;
	for(i = 0;i < MOTOR_COUNT;i++){
		room[i] = out[i] + throttle_best_rpy;
		if(yawArr[i] != 0.0){
			room[i] = 1 - room[i];
		}
		if(room[i] < 0.0){
			room[i] = 0.0;
		}
		if(room[i] < yawAllowed){
			yawAllowed = room[i];
		}
	}

	set.yaw = clamp(set.yaw, yawAllowed, -yawAllowed);
	out[0] += -set.yaw;
	out[1] += set.yaw;
	out[2] += -set.yaw;
	out[3] += set.yaw;

	float32_t rpy_low = out[0];
	float32_t rpy_high = out[0];
	int j;
	for(j = 1; j < MOTOR_COUNT;j++){
		if(out[j] < rpy_low){
			rpy_low = out[j];
		}
		if(out[j] > rpy_high){
			rpy_high = out[j];
		}
	}
	float32_t rpy_scale = 1.0;
	if(rpy_high - rpy_low > 1.0){
		rpy_scale = 1.0/(rpy_high - rpy_low);
	}
	if(throttle_avg_max + rpy_low < 0.0){
		if(-throttle_avg_max / rpy_low < rpy_scale){
			rpy_scale = -throttle_avg_max / rpy_low;
		}
	}

	rpy_low *= rpy_scale;
	rpy_high *= rpy_scale;
	throttle_best_rpy = -rpy_low;
	float32_t thr_adj = set.throttle - throttle_best_rpy;
	if(rpy_scale < 1.0){
		thr_adj = 0.0;
	}
	thr_adj = clamp(thr_adj, 1.0 - (throttle_best_rpy + rpy_high), 0.0);

	int k;
	for(k = 0;k < MOTOR_COUNT;k++){
		out[k] = ((throttle_best_rpy + thr_adj) + out[k] * rpy_scale) * 2000.0;
	}*/



	out[0] = (set.throttle + set.roll + set.pitch - set.yaw) * 2000;
	out[1] = (set.throttle + set.roll - set.pitch + set.yaw) * 2000;
	out[2] = (set.throttle - set.roll - set.pitch - set.yaw) * 2000;
	out[3] = (set.throttle - set.roll + set.pitch + set.yaw) * 2000;
//	if(set.roll > 0){
//		out[0] += set.roll * 2;
//		out[1] += set.roll * 2;
//	}else{
//		out[2] -= set.roll * 2;
//		out[3] -= set.roll * 2;
//	}
//
//	if(set.pitch > 0){
//		out[0] += set.pitch * 2;
//		out[3] += set.pitch * 2;
//	}else{
//		out[1] -= set.pitch * 2;
//		out[2] -= set.pitch * 2;
//	}

	/*if(set.throttle > 0.01){
		out[0] += 50;
		out[3] += 50;
	}*/
	// at extremes these motors will not be able to match exactly
	int l;
	for(l = 0;l < MOTOR_COUNT;l++){
		if(out[l] > 1999.0){
			out[l] = 1999.0;
		}
		if(out[l] < 0.0){
			out[l] = 0.0;
		}
		motorOut[l] = (uint16_t)out[l];
		motorOut[l] += 48;
	}
}


