/*
 * button_handler.c
 *
 *  Created on: Aug 11, 2024
 *      Author: vedpa
 */

#include "button_handler.h"

int last_button_press_micros = 0;
int button_press_burst_count = 0;



void processButtonPress(uint32_t timeMicros){
	if(getDeltaTime(timeMicros, last_button_press_micros) / 1000 < DOUBLE_CLICK_MILLIS){
		button_press_burst_count++;
	}else{
		button_press_burst_count = 1;
	}

	if(button_press_burst_count == 1){
		setBindingMode();
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	}else if(button_press_burst_count == 2){
		exitBindMode();
	}else if(button_press_burst_count == 4){
		eraseFlashSector();
	}

	last_button_press_micros = timeMicros;
}
