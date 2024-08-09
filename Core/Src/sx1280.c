/*
 * sx1280.c
 *
 *  Created on: Jul 19, 2024
 *      Author: vedpa
 */
#include "sx1280.h"
#include "expresslrs.h"
#include "string.h"
#include "usbd_cdc_if.h"

void initSX1280(){
	setRFRate(LORA_SF_8, LORA_BW_800, LORA_CR_LI_4_8, 12, fhssGetInitialFreq(), 1);
}


//TODO: interrupt clear
void setRFRate(uint8_t sF, uint8_t bW, uint8_t cR, uint8_t preambleLen, uint32_t freqReg, uint8_t isInverted){
	setHighPower();
	setStandby();
	setPacketTypeLORA();
	writeRFFrequency(freqReg);
	setLORAModParameters(sF, bW, cR);
	if(isInverted){
		setLORAPacketParameters(preambleLen, LORA_HEADER_IMPLICIT, 0x08, LORA_CRC_DISABLE, LORA_IQ_INVERTED);
	}else{
		setLORAPacketParameters(preambleLen, LORA_HEADER_IMPLICIT, 0x08, LORA_CRC_DISABLE, LORA_IQ_STD);
	}
	configureInterrupts();
	setRXModeNoTimeout();
}

uint8_t getStatus(){
	uint8_t getStatus = 0xC0;
	uint8_t status_buf[1] = {0x00};
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &getStatus, 1, 10);
	HAL_SPI_Receive(&hspi3, status_buf, 1, 10);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
	return status_buf[0];
}

// sets RX Mode without any timeout, continuous listening
void setRXModeNoTimeout(){
	uint8_t set_rx_mode_buff[4] = {SX1280_SET_RX_MODE, 0x00, 0xFF, 0xFF};
	sendSPIBuffer(set_rx_mode_buff, 4);
}

// boof, i just send RX done interrupts to all DIO pins because I cannot be bothered to test which DIO Pin is actually wired to the interrupt
void configureInterrupts(){
	// interrupt mask is 0x0002, as the rxdone interrupt is on the second bit of the mask, explained in the data sheet
	// TODO: make this method not constant for other receiver necessities, but atm this script is just for the receiver.
	uint8_t irq_configure_buf[9] = {SX1280_SET_DIO_IRQ_PARAMS, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02};
	sendSPIBuffer(irq_configure_buf, 9);
}

void setLORAPacketParameters(uint8_t preambleLen, uint8_t headerType, uint8_t payloadLen, uint8_t crcEnabled, uint8_t invertIQ){
	uint8_t write_packet_params_buff[8] = {SX1280_SET_PACKET_PARAMS, preambleLen, headerType, payloadLen, crcEnabled, invertIQ, 0x00, 0x00};
	sendSPIBuffer(write_packet_params_buff, 8);
}

void setLORAModParameters(uint8_t sF, uint8_t bW, uint8_t cR){
	uint8_t lora_mod_params_buff[4] = {SX1280_SET_MODULATION_PARAMS, sF, bW, cR};
	sendSPIBuffer(lora_mod_params_buff, 4);

	uint8_t sFCorrection = 0x00;

	if(sF == LORA_SF_5 || sF == LORA_SF_6){
		sFCorrection = 0x1E;
	}else if(sF == LORA_SF_7 || sF == LORA_SF_8){
		sFCorrection = 0x37;
	}else{
		sFCorrection = 0x32;
	}
	writeRegister(0x0925, sFCorrection);

	uint8_t freq_comp = readRegister(0x093c);
	freq_comp = freq_comp & 0b11111000;
	freq_comp = freq_comp | 0x01;

	writeRegister(0x093c, freq_comp);
}

void setStandby(){
	uint8_t standby_set_buff[2] = {SX1280_SET_STANDBY, 0x00};
	sendSPIBuffer(standby_set_buff, 2);
}

// could be modular but I don't care.
void setPacketTypeLORA(){
	uint8_t packet_type_buff[2] = {SX1280_SET_PACKET_TYPE, 0x01};
	sendSPIBuffer(packet_type_buff, 2);
}

uint8_t sx1280PollBusy(void)
{
    uint32_t startTime = HAL_GetTick();
    while ((uint8_t)HAL_GPIO_ReadPin(RX_SPI_BUSY_GPIO_Port, RX_SPI_BUSY_Pin)) {
        if ((HAL_GetTick() - startTime) > 1000) {
            return 0;
        } else {
            __asm__("nop");
        }
    }
    return 1;
}

void writeRFFrequency(uint32_t freqReg){
	uint8_t buf[4] = {SX1280_RADIO_SET_RFFREQUENCY};

	buf[1] = (uint8_t)((freqReg >> 16) & 0xFF);
	buf[2] = (uint8_t)((freqReg >> 8) & 0xFF);
	buf[3] = (uint8_t)((freqReg) & 0xFF);

	sendSPIBuffer(buf, 4);
}

void writeRegister(uint16_t reg, uint8_t data){
	uint8_t buf[4] = {SX1280_WRITE_REGISTER};
	buf[1] = (uint8_t)((reg >> 8) & 0xFF);
	buf[2] = (uint8_t)((reg) & 0xFF);
	buf[3] = data;

	sendSPIBuffer(buf, 4);
}

// I hope I'm doing this right.
uint8_t readRegister(uint16_t reg){
	uint8_t read_buf[5] = {0x00};
	uint8_t buf[5] = {SX1280_READ_REGISTER};

	buf[1] = (uint8_t)((reg >> 8) & 0xFF);
	buf[2] = (uint8_t)((reg) & 0xFF);

	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi_sx1280, buf, 5, 10);
	HAL_SPI_Receive(hspi_sx1280, read_buf, 5, 10);
	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_SET);

	return read_buf[4];
}

void setHighPower(){
	writeRegister(0x0891, (readRegister(0x0891) | 0xC0));
}

//TODO: better delays
void sendSPIBuffer(uint8_t* buf, uint8_t size){
	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi_sx1280, buf, size, 10);
	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_SET);
}

