/*
 * flashMemoryConfig.c
 *
 *  This flash memory config manager is ONLY developed for config packets of multiples of 4 bytes.
 *  It becomes exponentially more annoying when making packets that are not
 *  multiples of 4, as the memory pointers point to sectors of 4 bytes.
 *  Created on: Jul 20, 2024
 *      Author: vedpa
 */

#include <flash_memory_handler.h>

uint32_t *initSectorPtr = (uint32_t*)0x08060000;
uint32_t *finalSectorPtr = (uint32_t*)0x0807FFFC;
uint8_t configSizeWords;

uint32_t *firstFreeAddress = (uint32_t*)0x08060000;



uint8_t initFlashMemoryConfig(uint8_t sizeWords){
	configSizeWords = sizeWords;
	getFirstFreeAddress();

	// Check if the last word is not free. if this is the case then clear everything. This case is only likely on startup.
	uint32_t word = *finalSectorPtr;
	if(word != 0xFFFFFFFF){
		eraseFlashSector();
		return 0x03;
	}

	return *initSectorPtr;

}

void readCurrentConfig(uint8_t* out){
	uint32_t* startPtr = firstFreeAddress - configSizeWords;
	// This should never be true if write is called properly.
	if(startPtr > finalSectorPtr - configSizeWords + 1){
		startPtr = finalSectorPtr - configSizeWords + 1;
	}
	// should only be called if we are empty buffered, on first startup.
	if(startPtr < initSectorPtr){
		for(int j = 0;j < configSizeWords;j++){
			for(int i = 0;i < 4;i++){
				out[j * 4 + i] = 0;
			}
		}
	}

	for(int j = 0;j < configSizeWords;j++){
		uint32_t word = *(startPtr + j);
		for(int i = 0;i < 4;i++){
			out[j * 4 + i] = (uint8_t)((word >> (8 * (3 - i))) & 0xFF);
		}
	}

}

void writeNewConfig(uint8_t* in){
	// if the buffer fills up, start writing at the top. Very rarely called.
	if(firstFreeAddress > finalSectorPtr - configSizeWords + 1){
		firstFreeAddress = initSectorPtr;
		eraseFlashSector();
	}
	HAL_FLASH_Unlock();
	for(int i = 0;i < configSizeWords;i++){
		uint32_t word = ((uint32_t)in[i * 4 + 0] << 24) + ((uint32_t)in[i * 4 + 1] << 16) + ((uint32_t)in[i * 4 + 2] << 8) + ((uint32_t)in[i * 4 + 3]);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(firstFreeAddress + i), word);
	}
	firstFreeAddress = firstFreeAddress + configSizeWords;
	HAL_FLASH_Lock();
}

void getFirstFreeAddress(){
	uint32_t* currentDat = initSectorPtr;
	while((uint8_t)(*(currentDat)) != 0xFF){
		currentDat++;
	}
	firstFreeAddress = currentDat;
}

void eraseFlashSector(){
	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(FLASH_SECTOR_7, VOLTAGE_RANGE_3);
	HAL_FLASH_Lock();
}
