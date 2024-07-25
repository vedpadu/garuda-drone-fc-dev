/*
 * esc.h
 *
 *  Created on: Jul 24, 2024
 *      Author: vedpa
 */

#ifndef INC_ESC_H_
#define INC_ESC_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include "string.h"

#define MOTOR_BIT_0           7
#define MOTOR_BIT_1           14
#define MOTOR_BITLENGTH       20

#define MOTOR_COUNT 4

#define ESC_POWER_UP_TIME 3000	// wait 3s before arming sequence
#define INIT_THROTTLE_MAX 500	    // max throttle when initializing
#define INIT_THROTTLE_MIN 48	    // min throttle when initializing

#define DSHOT_FRAME_SIZE 18

typedef struct motorPWMTim_s {
	uint8_t motorIndex;
	TIM_HandleTypeDef* tim;
	uint32_t channel;
} motorPWMTim_t;

void updateESC();
void dshot600(uint32_t *motor, uint16_t value);
void setMotorOutputs(uint16_t* desiredOut);

#endif /* INC_ESC_H_ */
