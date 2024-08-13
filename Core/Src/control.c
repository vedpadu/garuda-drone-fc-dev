/*
 * motorMixer.c
 *
 *  Created on: Aug 1, 2024
 *      Author: vedpa
 */
#include <control.h>

rateSetpoint_t desired_rate = {0.0, 0.0, 0.0}; // roll, pitch, yaw
angleSetpoint_t desired_attitude = {0.0, 0.0, 0.0};
float32_t current_euler_attitude[3] = {0.0};

// do with outputHandler
int16_t rc_inputs[8] = {0}; // roll, pitch, throttle, yaw, switch A, switch B, switch C, switch D -> same format as input rcData
int16_t old_rc_inputs[8] = {0}; // do these need to exist, output already smoothed

outRates_t motor_setpoints = {0.0, 0.0, 0.0, 0.0}; // values between -1 and 1 for roll, pitch, throttle, yaw

float32_t hover_throttle = HOVER_THROTTLE_NOT_SET; // throttle that the drone gets close to hovering at
float32_t last_hoverable_thrott = 0.0; // used at takeoff to find the hover throttle

uint8_t drone_armed = 0;

// rate to motor setpoint PIDs
PIDController roll_motor_PID = {PID_MOTOR_KP, PID_MOTOR_KI, PID_MOTOR_KD,
						PID_MOTOR_TAU,
						PID_MOTOR_LIM_MIN, PID_MOTOR_LIM_MAX,
						PID_MOTOR_LIM_MIN_INT, PID_MOTOR_LIM_MAX_INT,
						SAMPLE_TIME_INNER };
PIDController pitch_motor_PID = {PID_MOTOR_KP, PID_MOTOR_KI, PID_MOTOR_KD,
						PID_MOTOR_TAU,
						PID_MOTOR_LIM_MIN, PID_MOTOR_LIM_MAX,
						PID_MOTOR_LIM_MIN_INT, PID_MOTOR_LIM_MAX_INT,
						SAMPLE_TIME_INNER };
PIDController yaw_motor_PID = {PID_MOTOR_KP, 0.0, PID_MOTOR_KD,
						PID_MOTOR_TAU,
						PID_MOTOR_LIM_MIN, PID_MOTOR_LIM_MAX,
						PID_MOTOR_LIM_MIN_INT, PID_MOTOR_LIM_MAX_INT,
						SAMPLE_TIME_INNER };

// angle to rate PIDs
PIDController roll_rate_PID = {PID_RATE_KP, PID_RATE_KI, PID_RATE_KD,
						PID_RATE_TAU,
						PID_RATE_LIM_MIN, PID_RATE_LIM_MAX,
						PID_RATE_LIM_MIN_INT, PID_RATE_LIM_MAX_INT,
						SAMPLE_TIME_OUTER };
PIDController pitch_rate_PID = {PID_RATE_KP, PID_RATE_KI, PID_RATE_KD,
						PID_RATE_TAU,
						PID_RATE_LIM_MIN, PID_RATE_LIM_MAX,
						PID_RATE_LIM_MIN_INT, PID_RATE_LIM_MAX_INT,
						SAMPLE_TIME_OUTER };
PIDController yaw_rate_PID = {PID_RATE_KP, PID_RATE_KI, PID_RATE_KD,
						PID_RATE_TAU,
						PID_RATE_LIM_MIN, PID_RATE_LIM_MAX,
						PID_RATE_LIM_MIN_INT, PID_RATE_LIM_MAX_INT,
						SAMPLE_TIME_OUTER };

PIDController throttle_PID = {PID_THROTTLE_KP, PID_THROTTLE_KI, PID_THROTTLE_KD,
						PID_THROTTLE_TAU,
						PID_THROTTLE_LIM_MIN, PID_THROTTLE_LIM_MAX,
						PID_THROTTLE_LIM_MIN_INT, PID_THROTTLE_LIM_MAX_INT,
						SAMPLE_TIME_INNER };

void controlsInit()
{
	HAL_TIM_Base_Start_IT(htim_control_loop);

	PIDController_Init(&roll_motor_PID);
	PIDController_Init(&pitch_motor_PID);
	PIDController_Init(&yaw_motor_PID);
	PIDController_Init(&roll_rate_PID);
	PIDController_Init(&pitch_rate_PID);
	PIDController_Init(&yaw_rate_PID);
	PIDController_Init(&throttle_PID);

}

void controlsInnerLoop(uint16_t* rcData, uint16_t* motorOut, float32_t* currentRate, float32_t* currentAccel,
							quaternion_t attitude)
{
	handleRCInputs(rcData);
	drone_armed = rc_inputs[SWITCH_A_IND];
	if(!drone_armed){
		int i;
		for(i = 0;i < MOTOR_COUNT;i++){
			motorOut[i] = THROTTLE_MIN;
		}
		return;
	}

	// only runs if the rate control switch is flicked
	if(rc_inputs[SWITCH_B_IND]){
		float32_t pitchRate = (float32_t)-rc_inputs[PITCH_STICK_IND]/125.0;
		float32_t rollRate = (float32_t)rc_inputs[ROLL_STICK_IND]/125.0;

		desired_rate.ratePitch = pitchRate;
		desired_rate.rateRoll = rollRate;
	}

	float32_t throttle_target = (float32_t)rc_inputs[THROTTLE_STICK_IND]/500.0;

	if(!isDisconnected() && throttle_target > 0){
		// hovering area...
		if(absVal(throttle_target - 1.0) < HOVER_THROTTLE_RANGE){
			throttle_target = 1.0;
		}
		getDesiredThrottle(throttle_target, attitude, currentAccel);
	}else{
		motor_setpoints.throttle = 0;
	}

	// only do rate control and find hover throttle if we are throttling
	if(motor_setpoints.throttle >= 0.01){
		// + 0.1 to mitigate floating point precision errors
		if(hover_throttle < HOVER_THROTTLE_NOT_SET + 0.1){
			findHoverThrottle(attitude, currentAccel);
		}
		achieveDesiredRates(currentRate);

	}else{
		// reset everything on no throttle
		motor_setpoints.roll = 0;
		motor_setpoints.pitch = 0;
		motor_setpoints.yaw = 0;
		hover_throttle = HOVER_THROTTLE_NOT_SET;
		last_hoverable_thrott = 0.0;
	}

	// set outputs
	getMotorOutputs(motor_setpoints, motorOut);
}

void findHoverThrottle(quaternion_t attitude, float32_t* currentAccel)
{
	float32_t vec[3] = {0.0, 0.0, -1.0};
	quaternion_t inverseEst = quatInverse(attitude);
	rotateVector3ByQuaternion(vec, inverseEst);
	float32_t dot = vec[0] * currentAccel[0] + vec[1] * currentAccel[1] + vec[2] * currentAccel[2];
	if(dot < 1.0){ // while we are on the ground (accel shows < 1)
		last_hoverable_thrott = motor_setpoints.throttle;
	}else if(dot > 1.0 + ACCEL_TAKEOFF_THRESHOLD){ // only set the hover throttle when we are certain we are taken off
		hover_throttle = last_hoverable_thrott;
	}
}

void controlsOuterUpdate(quaternion_t attitude, float32_t* accel)
{
	float32_t pitchRate = -(float32_t)rc_inputs[PITCH_STICK_IND]/500.0;
	float32_t rollRate = (float32_t)rc_inputs[ROLL_STICK_IND]/500.0;
	float32_t yawRate = (float32_t)rc_inputs[YAW_STICK_IND]/200.0;

	desired_attitude.pitch = pitchRate;
	desired_attitude.roll = rollRate;

	quatToEuler(attitude, current_euler_attitude);

	if(motor_setpoints.throttle < 0.01){
		desired_attitude.yaw = current_euler_attitude[2];
	}else{
		desired_attitude.yaw += yawRate * SAMPLE_TIME_OUTER;
	}

	getDesiredRates(current_euler_attitude);
}

// throttle controller -> inner loop
void getDesiredThrottle(float32_t dotTarget, quaternion_t attitude, float32_t* accel)
{
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
	PIDController_Update(&throttle_PID, dotTarget, dot);

	if(hover_throttle > 0){
		// only can help alt hold, not perfect as throttle not linearized and stuff
		motor_setpoints.throttle = dotTarget * hover_throttle * thrustScale + throttle_PID.out;
	}else{
		motor_setpoints.throttle = dotTarget * 0.5;
	}
}

// angle to rate controls -> outer loop
void getDesiredRates(float32_t* eulerAtt)
{
	PIDController_Update(&roll_rate_PID, (desired_attitude.roll), eulerAtt[0]);
	PIDController_Update(&pitch_rate_PID, desired_attitude.pitch, eulerAtt[1]);
	PIDController_Update(&yaw_rate_PID, desired_attitude.yaw, eulerAtt[2]);

	desired_rate.rateRoll = roll_rate_PID.out;
	desired_rate.ratePitch = pitch_rate_PID.out;
	desired_rate.rateYaw = yaw_rate_PID.out;
}

// rate to motor setpoints -> inner loop
void achieveDesiredRates(float32_t* currentRate)
{
	PIDController_Update(&roll_motor_PID, desired_rate.rateRoll, currentRate[0]);
	PIDController_Update(&pitch_motor_PID, desired_rate.ratePitch, currentRate[1]);
	PIDController_Update(&yaw_motor_PID, desired_rate.rateYaw, currentRate[2]);

	motor_setpoints.roll = roll_motor_PID.out;
	motor_setpoints.pitch = pitch_motor_PID.out;
	motor_setpoints.yaw = yaw_motor_PID.out;
}

// clamps max velocity for rc inputs
void handleRCInputs(uint16_t* rcData)
{
	int i;
	for(i = 0;i < 4;i++){
		if(i != THROTTLE_STICK_IND){
			rc_inputs[i] = ((int16_t)rcData[i]) - REGULAR_STICK_NO_INPUT;
			if(rc_inputs[i] < DEADBAND && rc_inputs[i] > -DEADBAND){
				rc_inputs[i] = 0;
			}
		}else{
			rc_inputs[i] = ((int16_t)rcData[i]) - THROTTLE_STICK_NO_INPUT;
			if(rc_inputs[i] < DEADBAND){
				rc_inputs[i] = 0;
			}
		}
		if((rc_inputs[i]) > DEADBAND + MAX_INPUT_VELOCITY || rc_inputs[i] < -DEADBAND - MAX_INPUT_VELOCITY){
			rc_inputs[i] = intClamp(rc_inputs[i], old_rc_inputs[i] + MAX_INPUT_VELOCITY, old_rc_inputs[i] - MAX_INPUT_VELOCITY);
		}

		old_rc_inputs[i] = rc_inputs[i];
	}
	int j;
	for(j = 4;j < 8;j++){
		// converts switches to true false
		if(rcData[j] > SWITCH_THRESHOLD){ // lots of leeway for switch flicking so we don't flick the switch on for bad values
			rc_inputs[j] = 1;
		}else{
			rc_inputs[j] = 0;
		}
	}
}

// motor mixer
void getMotorOutputs(outRates_t set, uint16_t* motorOut)
{
	float32_t out[MOTOR_COUNT] = {0};
	out[0] = (set.throttle + set.roll + set.pitch - set.yaw) * 2000;
	out[1] = (set.throttle + set.roll - set.pitch + set.yaw) * 2000;
	out[2] = (set.throttle - set.roll - set.pitch - set.yaw) * 2000;
	out[3] = (set.throttle - set.roll + set.pitch + set.yaw) * 2000;

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
		motorOut[l] += THROTTLE_MIN;
	}
}


