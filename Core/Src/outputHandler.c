/*
 * outputHandler.c
 * Limits output velocity and acceleration
 *
 *  Created on: Aug 8, 2024
 *      Author: vedpadu
 */

#include "outputHandler.h"

outRates_t oldRates = {0.0, 0.0, 0.0};
float32_t oldRateOfChange[3] = {0.0};

float32_t maxVelocity = 0.0;
float32_t maxAcceleration = 0.0;

void initOutputHandler(float32_t maxVel, float32_t maxAccel){
	maxVelocity = maxVel;
	maxAcceleration = maxAccel;
}

void outputUpdate(outRates_t* out){
	out->roll = clamp(out->roll, oldRates.roll + maxVelocity, oldRates.roll - maxVelocity);
	out->pitch = clamp(out->pitch, oldRates.pitch + maxVelocity, oldRates.pitch - maxVelocity);
	out->yaw = clamp(out->yaw, oldRates.yaw + maxVelocity, oldRates.yaw - maxVelocity);


	float32_t currentRateOfChange[3] = {0.0};
	currentRateOfChange[0] = out->roll - oldRates.roll;
	currentRateOfChange[1] = out->pitch - oldRates.pitch;
	currentRateOfChange[2] = out->yaw - oldRates.yaw;

	float32_t diff[3] = {0.0};
	int i;
	for(i = 0;i < 3;i++){
		diff[i] = currentRateOfChange[i] - oldRateOfChange[i];
	}

	if(absVal(diff[0]) > maxAcceleration){
		currentRateOfChange[0] = oldRateOfChange[0] + clamp(diff[0], maxAcceleration, -maxAcceleration);
		out->roll = oldRates.roll + currentRateOfChange[0];
	}
	if(absVal(diff[1]) > maxAcceleration){
		currentRateOfChange[1] = oldRateOfChange[1] + clamp(diff[1], maxAcceleration, -maxAcceleration);
		out->pitch = oldRates.pitch + currentRateOfChange[1];
	}
	if(absVal(diff[2]) > maxAcceleration){
		currentRateOfChange[2] = oldRateOfChange[2] + clamp(diff[2], maxAcceleration, -maxAcceleration);
		out->yaw = oldRates.yaw + currentRateOfChange[2];
	}
	oldRates = *out;
	memcpy(oldRateOfChange, currentRateOfChange, 3 * sizeof(float32_t));
}
