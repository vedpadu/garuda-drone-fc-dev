/*
 * esc.c
 * Arms ESC as well as creates DSHOT600 packets given motor values
 *
 *  Created on: Jul 24, 2024
 *      Author: vedpadu
 */
#include "esc.h"

uint8_t armed = 0;
uint16_t initialization_ctr = 0;
uint16_t arming_ctr = 0;
uint8_t do_init_throttle_down = 0;
uint32_t motor_dshot_buffers[MOTOR_COUNT][DSHOT_FRAME_SIZE] = { { 0 } };
uint16_t motor_outputs[MOTOR_COUNT] = { INIT_THROTTLE_MIN, INIT_THROTTLE_MIN, INIT_THROTTLE_MIN, INIT_THROTTLE_MIN };
motorPWMTim_t motor_PWM_tims[4] = {
		{ 0, &htim4, TIM_CHANNEL_2, &hdma_tim4_ch2 }, // black red // + pitch + roll
		{ 1, &htim2, TIM_CHANNEL_3, &hdma_tim2_ch3_up }, // black black // - pitch + roll
		{ 2, &htim4, TIM_CHANNEL_3, &hdma_tim4_ch3 }, // red red // - pitch - roll
		{ 3, &htim2, TIM_CHANNEL_1, &hdma_tim2_ch1 }, // red black // + pitch - roll
		};

// deals with the initialization of the ESC
void arm_ESC() {
	arming_ctr++;
	// wait power up
	if (arming_ctr % MOTOR_COUNT == 0) {
		if (initialization_ctr < ESC_POWER_UP_TIME) {
			++initialization_ctr;
			return;
		}
	}

	if (initialization_ctr < ESC_POWER_UP_TIME) {
		return;
	}

	if (!armed) {
		dshot600(motor_dshot_buffers[arming_ctr % MOTOR_COUNT],
				motor_outputs[arming_ctr % MOTOR_COUNT]);
		HAL_TIM_PWM_Start_DMA(motor_PWM_tims[arming_ctr % MOTOR_COUNT].tim,
				motor_PWM_tims[arming_ctr % MOTOR_COUNT].channel,
				motor_dshot_buffers[arming_ctr % MOTOR_COUNT], 18);
	}
	if (!armed && initialization_ctr > ESC_POWER_UP_TIME + INIT_THROTTLE_MAX * 6 && arming_ctr % MOTOR_COUNT == MOTOR_COUNT - 1) {
		armed = 1;
		// convert to 100 hz low prio kalman timer. TODO: make unconstant
		HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);
		htim5.Instance->PSC = 4799;
		htim5.Instance->ARR = 99;
	}
	if (armed || arming_ctr % MOTOR_COUNT != 0) {
		return;
	}

	++initialization_ctr;

	// TODO: for loop
	if (do_init_throttle_down) {
		motor_outputs[0]--;
		motor_outputs[1]--;
		motor_outputs[2]--;
		motor_outputs[3]--;
		if (motor_outputs[0] < INIT_THROTTLE_MIN) {
			motor_outputs[0] = INIT_THROTTLE_MIN;
			motor_outputs[1] = INIT_THROTTLE_MIN;
			motor_outputs[2] = INIT_THROTTLE_MIN;
			motor_outputs[3] = INIT_THROTTLE_MIN;
		}

	} else {
		motor_outputs[0]++;
		motor_outputs[1]++;
		motor_outputs[2]++;
		motor_outputs[3]++;
		if (motor_outputs[0] > INIT_THROTTLE_MAX) {
			motor_outputs[0] = INIT_THROTTLE_MAX;
			motor_outputs[1] = INIT_THROTTLE_MAX;
			motor_outputs[2] = INIT_THROTTLE_MAX;
			motor_outputs[3] = INIT_THROTTLE_MAX;
			do_init_throttle_down = 1;
		}
	}
}

// sends motor values to dma streams and performs dshot
void set_esc_outputs(uint16_t *desiredOut) {
	if (armed) {
		memcpy(motor_outputs, desiredOut, MOTOR_COUNT * sizeof(uint16_t));
		int i;
		for (i = 0; i < 4; i++) {
			dshot600(motor_dshot_buffers[i], motor_outputs[i]);
			HAL_TIM_PWM_Start_DMA(motor_PWM_tims[i].tim, motor_PWM_tims[i].channel,
					motor_dshot_buffers[i], 18);
		}
	}
}

// constructs a dshot packet for a given value
void dshot600(uint32_t *motor, uint16_t value) {
	uint16_t packet = value << 1;

	// compute checksum
	int csum = 0;
	int csum_data = packet;

	motor[0] = 0;
	for (int i = 1; i < 4; i++) {
		csum ^= csum_data;   // xor data by nibbles
		csum_data >>= 4;
	}
	csum &= 0xf;

	// append checksum
	packet = (packet << 4) | csum;

	// encoding
	int i;
	for (i = 1; i < 17; i++) {
		motor[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;  // MSB first
		packet <<= 1;
	}

	motor[i++] = 0;
}
