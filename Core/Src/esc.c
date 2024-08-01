/*
 * esc.c
 * Creates DSHOT600 packets given motor values
 *
 *  Created on: Jul 24, 2024
 *      Author: vedpa
 */
#include "esc.h"
#include "main.h"

uint8_t armed = 0;
uint16_t initializationCtr = 0;
uint16_t masterCtr = 0;
uint8_t do_init_throttle_down = 0;
uint32_t motorDshotBuffers[MOTOR_COUNT][DSHOT_FRAME_SIZE] = {{0}};
uint16_t motorOutputs[MOTOR_COUNT] = {INIT_THROTTLE_MIN, INIT_THROTTLE_MIN, INIT_THROTTLE_MIN, INIT_THROTTLE_MIN};
motorPWMTim_t motorPWMTims[4] = {
		{0, &htim4, TIM_CHANNEL_2, &hdma_tim4_ch2}, // black red // + pitch + roll
		{1, &htim2, TIM_CHANNEL_3, &hdma_tim2_ch3_up}, // black black // - pitch + roll
		{2, &htim4, TIM_CHANNEL_3, &hdma_tim4_ch3}, // red red // - pitch - roll
		{3, &htim2, TIM_CHANNEL_1, &hdma_tim2_ch1}, // red black // + pitch - roll
};


// deals with the initialization as well as the motor setting
void updateESC(){
	masterCtr++;
	// wait power up
	if(masterCtr % 4 == 0){
		if (initializationCtr < ESC_POWER_UP_TIME) {
		  ++initializationCtr;
		  return;
	  }
	}

	if (initializationCtr < ESC_POWER_UP_TIME) {
		return;
	}

	HAL_TIM_PWM_Start_DMA(motorPWMTims[masterCtr % 4].tim, motorPWMTims[masterCtr % 4].channel, motorDshotBuffers[masterCtr % 4], 18);

	 if(!armed){

		 dshot600(motorDshotBuffers[masterCtr % 4],motorOutputs[masterCtr % 4]);
	 }
	 if(masterCtr % 4 != 0){
		 return;
	 }

	 if(armed){
		 return;
	 }

	 if (!armed && initializationCtr > ESC_POWER_UP_TIME+INIT_THROTTLE_MAX*6){
		 doBlink = 1;
		 armed = 1;
	 }

	  ++initializationCtr;

	  if (do_init_throttle_down) {
		 motorOutputs[0]--;
		 motorOutputs[1]--;
		 motorOutputs[2]--;
		 motorOutputs[3]--;
		 if (motorOutputs[0]<INIT_THROTTLE_MIN){
			 motorOutputs[0]=INIT_THROTTLE_MIN;
			 motorOutputs[1]=INIT_THROTTLE_MIN;
			 motorOutputs[2]=INIT_THROTTLE_MIN;
			 motorOutputs[3]=INIT_THROTTLE_MIN;
		 }

	 }
	 else {
		 motorOutputs[0]++;
		 motorOutputs[1]++;
		 motorOutputs[2]++;
		 motorOutputs[3]++;
		 if (motorOutputs[0] > INIT_THROTTLE_MAX) {
			 motorOutputs[0] = INIT_THROTTLE_MAX;
			 motorOutputs[1] = INIT_THROTTLE_MAX;
			 motorOutputs[2] = INIT_THROTTLE_MAX;
			 motorOutputs[3] = INIT_THROTTLE_MAX;
			 do_init_throttle_down = 1;
		 }
	 }
}

motorPWMTim_t* getTims(){
	return motorPWMTims;
}

void setMotorOutputs(uint16_t* desiredOut){
	if(armed){
		memcpy(motorOutputs, desiredOut, MOTOR_COUNT * sizeof(uint16_t));
		int i;
		for(i = 0;i < 4;i++){
			dshot600(motorDshotBuffers[i],motorOutputs[i]);
		}
	}
}

// constructs a dshot packet for a given value
void dshot600(uint32_t *motor, uint16_t value)
{
	uint16_t packet = value << 1;

	// compute checksum
	int csum = 0;
	int csum_data = packet;


	motor[0] = 0;
	for (int i = 1; i < 4; i++) {
    csum ^=  csum_data;   // xor data by nibbles
    	csum_data >>= 4;
	}
	csum &= 0xf;

	// append checksum
	packet = (packet << 4) | csum;

	// encoding
	int i;
	for (i = 1; i < 17; i++)
	{
		motor[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;  // MSB first
	    packet <<= 1;
	}

	motor[i++] = 0;
}
