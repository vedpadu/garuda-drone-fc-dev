/*
 * button_handler.h
 *
 *  Created on: Aug 11, 2024
 *      Author: vedpa
 */

#ifndef INC_BUTTON_HANDLER_H_
#define INC_BUTTON_HANDLER_H_

#include "main.h"
#include "expresslrs.h"
#include "flash_memory_handler.h"

#define MULTI_CLICK_THRESHOLD_MILLIS 750

#define START_BIND_BUTTON_PRESS_COUNT 1
#define EXIT_BIND_BUTTON_PRESS_COUNT 2
#define ERASE_FLASH_MEMORY_BUTTON_PRESS_COUNT 4

void processButtonPress(uint32_t timeMicros);

#endif /* INC_BUTTON_HANDLER_H_ */
