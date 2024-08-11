/*
 * flashMemoryConfig.h
 *
 *  Created on: Jul 20, 2024
 *      Author: vedpa
 */

#ifndef INC_FLASH_MEMORY_HANDLER_H_
#define INC_FLASH_MEMORY_HANDLER_H_

#include "main.h"

uint8_t initFlashMemoryConfig(uint8_t sizeWords);
void getFirstFreeAddress();
void eraseFlashSector();
void readCurrentConfig(uint8_t* out);
void writeNewConfig(uint8_t* in);


#endif /* INC_FLASH_MEMORY_HANDLER_H_ */
