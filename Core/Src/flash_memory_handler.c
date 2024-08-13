/*
 * flash_memory_handler.c
 *
 *  This flash memory config manager is ONLY developed for config packets of multiples of 4 bytes.
 *  It becomes exponentially more annoying when making packets that are not
 *  multiples of 4, as the memory pointers point to sectors of 4 bytes.
 *  Created on: Jul 20, 2024
 *      Author: vedpadu
 */

#include <flash_memory_handler.h>

uint32_t *init_sector_ptr = (uint32_t*)0x08060000;
uint32_t *final_sector_ptr = (uint32_t*)0x0807FFFC;
uint8_t config_size_words;

uint32_t *first_free_address = (uint32_t*)0x08060000;



uint8_t initFlashMemoryConfig(uint8_t sizeWords)
{
	config_size_words = sizeWords;
	getFirstFreeAddress();

	// Check if the last word is not free. if this is the case then clear everything. This case is only likely on startup.
	uint32_t word = *final_sector_ptr;
	if(word != EMPTY_VALUE){
		eraseFlashSector();
		return 0x03;
	}

	return *init_sector_ptr;

}

void readCurrentConfig(uint8_t* out)
{
	uint32_t* start_ptr = first_free_address - config_size_words;
	// This should never be true if write is called properly.
	if(start_ptr > final_sector_ptr - config_size_words + 1){
		start_ptr = final_sector_ptr - config_size_words + 1;
	}
	// should only be called if we are empty buffered, on first startup.
	if(start_ptr < init_sector_ptr){
		for(int j = 0;j < config_size_words;j++){
			for(int i = 0;i < 4;i++){
				out[j * 4 + i] = 0;
			}
		}
	}

	for(int j = 0;j < config_size_words;j++){
		uint32_t word = *(start_ptr + j);
		for(int i = 0;i < 4;i++){
			out[j * 4 + i] = (uint8_t)((word >> (8 * (3 - i))) & 0xFF);
		}
	}

}

void writeNewConfig(uint8_t* in)
{
	// if the buffer fills up, start writing at the top. Very rarely called.
	if(first_free_address > final_sector_ptr - config_size_words + 1){
		first_free_address = init_sector_ptr;
		eraseFlashSector();
	}
	HAL_FLASH_Unlock();
	for(int i = 0;i < config_size_words;i++){
		uint32_t word = ((uint32_t)in[i * 4 + 0] << 24) + ((uint32_t)in[i * 4 + 1] << 16) + ((uint32_t)in[i * 4 + 2] << 8) + ((uint32_t)in[i * 4 + 3]);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(first_free_address + i), word);
	}
	first_free_address = first_free_address + config_size_words;
	HAL_FLASH_Lock();
}

void getFirstFreeAddress()
{
	uint32_t* current_data_ptr = init_sector_ptr;
	while((uint8_t)(*(current_data_ptr)) != 0xFF){
		current_data_ptr++;
	}
	first_free_address = current_data_ptr;
}

void eraseFlashSector()
{
	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(FLASH_SECTOR_7, VOLTAGE_RANGE_3);
	HAL_FLASH_Lock();
}
