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

#define DOUBLE_CLICK_MILLIS 750

void processButtonPress(uint32_t timeMicros);

#endif /* INC_BUTTON_HANDLER_H_ */
