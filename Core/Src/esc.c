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
uint8_t do_init_throttle_down = 0;
uint32_t motorDshotBuffers[MOTOR_COUNT][DSHOT_FRAME_SIZE] = {{0}};
uint16_t motorOutputs[MOTOR_COUNT] = {INIT_THROTTLE_MIN, INIT_THROTTLE_MIN, INIT_THROTTLE_MIN, INIT_THROTTLE_MIN};
motorPWMTim_t motorPWMTims[4] = {
		{1, &htim2, TIM_CHANNEL_3}, // black black
		{2, &htim2, TIM_CHANNEL_1}, // red black
		{3, &htim4, TIM_CHANNEL_3}, // red red
		{0, &htim4, TIM_CHANNEL_2}, // black red
};

// deals with the initialization as well as the motor setting
void updateESC(){
	// wait power up
	 if (initializationCtr < ESC_POWER_UP_TIME) {
		  ++initializationCtr;
		  return;
	  }

	 int i;
	 for(i = 0;i < 1;i++){
		 if(!armed){
			 dshot600(motorDshotBuffers[i],motorOutputs[i]);
		 }

		 HAL_TIM_PWM_Start_DMA(motorPWMTims[i].tim, motorPWMTims[i].channel, motorDshotBuffers[i], DSHOT_FRAME_SIZE);
	 }



	  if (initializationCtr > ESC_POWER_UP_TIME+INIT_THROTTLE_MAX*6){
		 armed = 1;	//the arming sequence is ended
		 return;
	  }

	  ++initializationCtr;

	  // creates a triangle necessary for the initialization
	  //int j;
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
			 do_init_throttle_down=1; // making sure they all hit the max
		 }
	 }

//	  for(j = 0;j < MOTOR_COUNT;j++){
//		  if (do_init_throttle_down) {
//			 motorOutputs[j]--;
//			 if (motorOutputs[j]<INIT_THROTTLE_MIN){
//				 motorOutputs[j]=INIT_THROTTLE_MIN;
//			 }
//
//		 }
//		 else {
//			 motorOutputs[j]++;
//			 if (motorOutputs[j] > INIT_THROTTLE_MAX) {
//				 motorOutputs[j] = INIT_THROTTLE_MAX;
//				 if(j == MOTOR_COUNT - 1){
//					do_init_throttle_down=1; // making sure they all hit the max
//				 }
//			 }
//		 }
//	  }

}

void setMotorOutputs(uint16_t* desiredOut){
	if(armed){
		memcpy(motorOutputs, desiredOut, MOTOR_COUNT);
		int i;
		for(i = 0;i < 1;i++){
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

  // preserves only last 4 bits
  csum &= 0xf;

  // append checksum
  packet = (packet << 4) | csum;

  // encoding
  int i;
  for (i = 0; i < 16; i++)
  {
      motor[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;  // MSB first
      packet <<= 1;
  }

  motor[16] = 0;
  motor[17] = 0;
}
