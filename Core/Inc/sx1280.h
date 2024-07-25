/*
 * sx1280.h
 *
 *  Created on: Jul 19, 2024
 *      Author: vedpa
 */

#ifndef INC_SX1280_H_
#define INC_SX1280_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "math.h"

#define CS_Pin_SX1280 SPI3_CS_Pin
#define CS_GPIO_Port_SX1280 SPI3_CS_GPIO_Port

#define hspi_sx1280 (&hspi3)

#define CLOCK_FREQ 52000000
#define SX1280_PLL_STEP (CLOCK_FREQ / (262144.0))

#define EXPRESSLRS_PACKET_SIZE 8

typedef enum {
    SX1280_SET_STANDBY = 0x80,
	SX1280_RADIO_SET_RFFREQUENCY = 0x86,
	SX1280_SET_PACKET_TYPE = 0x8A,
	SX1280_WRITE_REGISTER = 0x18,
	SX1280_SET_MODULATION_PARAMS = 0x8B,
	SX1280_READ_REGISTER = 0x19,
	SX1280_SET_PACKET_PARAMS = 0x8C,
	SX1280_SET_DIO_IRQ_PARAMS = 0x8D,
	SX1280_SET_RX_MODE = 0x82,
} sx1280Command_e;

typedef enum{
	LORA_SF_5 = 0x50,
	LORA_SF_6 = 0x60,
	LORA_SF_7 = 0x70,
	LORA_SF_8 = 0x80,
	LORA_SF_9 = 0x90,
	LORA_SF_10 = 0xA0,
	LORA_SF_11 = 0xB0,
	LORA_SF_12 = 0xC0,
	LORA_BW_1600 = 0x0A,
	LORA_BW_800 = 0x18,
	LORA_BW_400 = 0x26,
	LORA_BW_200 = 0x34,
	LORA_CR_4_5 = 0x01,
	LORA_CR_4_6 = 0x02,
	LORA_CR_4_7 = 0x03,
	LORA_CR_4_8 = 0x04,
	LORA_CR_LI_4_5 = 0x05,
	LORA_CR_LI_4_6 = 0x06,
	LORA_CR_LI_4_8 = 0x07,
} sx1280LORAModParams_e;

typedef enum{
	LORA_HEADER_EXPLICIT = 0x00,
	LORA_HEADER_IMPLICIT = 0x80,
	LORA_CRC_ENABLE = 0x20,
	LORA_CRC_DISABLE = 0x00,
	LORA_IQ_INVERTED = 0x00,
	LORA_IQ_STD = 0x40,
} sx1280LORAPacketParams_e;

void initSX1280();

void writeRFFrequency(uint32_t freq);
void sendSPIBuffer(uint8_t* buf, uint8_t size);
void setPacketTypeLORA();
void setStandby();
void configureInterrupts();
void setHighPower();
void setRXModeNoTimeout();
uint8_t readRegister(uint16_t reg);
void writeRegister(uint16_t reg, uint8_t data);
void setLORAModParameters(sx1280LORAModParams_e sF, sx1280LORAModParams_e bW, sx1280LORAModParams_e cR);
void setLORAPacketParameters(uint8_t preambleLen, sx1280LORAPacketParams_e headerType, uint8_t payloadLen, sx1280LORAPacketParams_e crcEnabled, sx1280LORAPacketParams_e invertIQ);
void setRFRate(sx1280LORAModParams_e sF, sx1280LORAModParams_e bW, sx1280LORAModParams_e cR, uint8_t preambleLen, uint32_t freqReg, uint8_t isInverted);
uint8_t sx1280PollBusy();
uint8_t getStatus();

#endif /* INC_SX1280_H_ */
