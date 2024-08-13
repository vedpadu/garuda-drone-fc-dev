/*
 * button_handler.c
 *
 *  Created on: Aug 11, 2024
 *      Author: vedpa
 */

#include "button_handler.h"

int last_button_press_micros = 0;
int button_press_burst_count = 0;



void processButtonPress(uint32_t timeMicros)
{
	if(getDeltaTime(timeMicros, last_button_press_micros) / 1000 < MULTI_CLICK_THRESHOLD_MILLIS){
		button_press_burst_count++;
	}else{
		button_press_burst_count = 1;
	}

	if(button_press_burst_count == START_BIND_BUTTON_PRESS_COUNT){
		setBindingMode();
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	}else if(button_press_burst_count == EXIT_BIND_BUTTON_PRESS_COUNT){
		exitBindMode();
	}else if(button_press_burst_count == ERASE_FLASH_MEMORY_BUTTON_PRESS_COUNT){
		eraseFlashSector();
	}

	last_button_press_micros = timeMicros;
}
