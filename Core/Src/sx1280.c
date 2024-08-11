/*
 * sx1280.c
 *
 *  Created on: Jul 19, 2024
 *      Author: vedpa
 */
#include "sx1280.h"

uint8_t packet_pointer_buf[4] = {0x00};

void sx1280_init(){
	sx1280_set_RF_rate(LORA_SF_8, LORA_BW_800, LORA_CR_LI_4_8, 12, fhssGetInitialFreq(), 1);
}

//TODO: interrupt clear
void sx1280_set_RF_rate(uint8_t sF, uint8_t bW, uint8_t cR, uint8_t preambleLen, uint32_t freqReg, uint8_t isInverted){
	sx1280_set_high_power();
	sx1280_set_standby();
	sx1280_set_packet_type_LORA();
	sx1280_write_RF_frequency(freqReg);
	sx1280_set_LORA_mod_parameters(sF, bW, cR);
	if(isInverted){
		sx1280_set_LORA_packet_parameters(preambleLen, LORA_HEADER_IMPLICIT, 0x08, LORA_CRC_DISABLE, LORA_IQ_INVERTED);
	}else{
		sx1280_set_LORA_packet_parameters(preambleLen, LORA_HEADER_IMPLICIT, 0x08, LORA_CRC_DISABLE, LORA_IQ_STD);
	}
	sx1280_configure_EXTI();
	sx1280_set_RX_mode_no_timeout();
}

// TODO: next two methods with constants
void sx1280_read_interrupt(uint8_t* out){
	uint8_t clear_irq[3] = {SX1280_CLEAR_INTERRUPTS, 0xFF, 0xFF};
	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi_sx1280, clear_irq, 3, 10);
	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_SET);

	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_RESET); // get pointer
	uint8_t transmit_buf[4] = {SX1280_GET_PACKET_POINTER, 0x00, 0x00, 0x00};
	HAL_SPI_Transmit(hspi_sx1280, transmit_buf, 4, 10);
	HAL_SPI_Receive(hspi_sx1280, packet_pointer_buf, 4, 10);
	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_SET);

	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_RESET); // read buffer data
	uint8_t transmit_buf_2[11] = {SX1280_GET_RX_DATA, packet_pointer_buf[0] - 16, 0x00}; // - 16 is kind of a magic number, it worked from my testing
	HAL_SPI_Transmit(hspi_sx1280, transmit_buf_2, 11, 1);
	HAL_SPI_Receive(hspi_sx1280, transmit_buf_2, 11, 10);
	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_SET);
	memcpy((uint8_t *) out, (uint8_t *) transmit_buf_2, 8);
}

uint8_t sx1280_get_status(){
	uint8_t getStatus = SX1280_GET_STATUS;
	uint8_t status_buf[1] = {0x00};
	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi_sx1280, &getStatus, 1, 10);
	HAL_SPI_Receive(hspi_sx1280, status_buf, 1, 10);
	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_SET);
	return status_buf[0];
}

// sets RX Mode without any timeout, continuous listening
void sx1280_set_RX_mode_no_timeout(){
	uint8_t set_rx_mode_buff[4] = {SX1280_SET_RX_MODE, 0x00, 0xFF, 0xFF};
	sx1280_send_SPI_buffer(set_rx_mode_buff, 4);
}

// boof, i just send RX done interrupts to all DIO pins because I cannot be bothered to test which DIO Pin is actually wired to the interrupt
void sx1280_configure_EXTI(){
	// interrupt mask is 0x0002, as the rxdone interrupt is on the second bit of the mask, explained in the data sheet
	// TODO: make this method not constant for other receiver necessities, but atm this script is just for the receiver.
	uint8_t irq_configure_buf[9] = {SX1280_SET_DIO_IRQ_PARAMS, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02};
	sx1280_send_SPI_buffer(irq_configure_buf, 9);
}

void sx1280_set_LORA_packet_parameters(uint8_t preambleLen, uint8_t headerType, uint8_t payloadLen, uint8_t crcEnabled, uint8_t invertIQ){
	uint8_t write_packet_params_buff[8] = {SX1280_SET_PACKET_PARAMS, preambleLen, headerType, payloadLen, crcEnabled, invertIQ, 0x00, 0x00};
	sx1280_send_SPI_buffer(write_packet_params_buff, 8);
}

void sx1280_set_LORA_mod_parameters(uint8_t sF, uint8_t bW, uint8_t cR){
	uint8_t lora_mod_params_buff[4] = {SX1280_SET_MODULATION_PARAMS, sF, bW, cR};
	sx1280_send_SPI_buffer(lora_mod_params_buff, 4);

	uint8_t sF_correction = 0x00;

	if(sF == LORA_SF_5 || sF == LORA_SF_6){
		sF_correction = 0x1E;
	}else if(sF == LORA_SF_7 || sF == LORA_SF_8){
		sF_correction = 0x37;
	}else{
		sF_correction = 0x32;
	}
	sx1280_write_register(0x0925, sF_correction);

	uint8_t freq_comp = sx1280_read_register(0x093c);
	freq_comp = freq_comp & 0b11111000;
	freq_comp = freq_comp | 0x01;

	sx1280_write_register(0x093c, freq_comp);
}

void sx1280_set_standby(){
	uint8_t standby_set_buff[2] = {SX1280_SET_STANDBY, 0x00};
	sx1280_send_SPI_buffer(standby_set_buff, 2);
}

// could be modular but I don't care.
void sx1280_set_packet_type_LORA(){
	uint8_t packet_type_buff[2] = {SX1280_SET_PACKET_TYPE, 0x01};
	sx1280_send_SPI_buffer(packet_type_buff, 2);
}

uint8_t sx1280_poll_busy(void)
{
    uint32_t start_time = HAL_GetTick();
    while ((uint8_t)HAL_GPIO_ReadPin(RX_SPI_BUSY_GPIO_Port, RX_SPI_BUSY_Pin)) {
        if ((HAL_GetTick() - start_time) > 1000) {
            return 0;
        } else {
            __asm__("nop");
        }
    }
    return 1;
}

void sx1280_write_RF_frequency(uint32_t freqReg){
	uint8_t buf[4] = {SX1280_RADIO_SET_RFFREQUENCY};

	buf[1] = (uint8_t)((freqReg >> 16) & 0xFF);
	buf[2] = (uint8_t)((freqReg >> 8) & 0xFF);
	buf[3] = (uint8_t)((freqReg) & 0xFF);

	sx1280_send_SPI_buffer(buf, 4);
}

void sx1280_write_register(uint16_t reg, uint8_t data){
	uint8_t buf[4] = {SX1280_WRITE_REGISTER};
	buf[1] = (uint8_t)((reg >> 8) & 0xFF);
	buf[2] = (uint8_t)((reg) & 0xFF);
	buf[3] = data;

	sx1280_send_SPI_buffer(buf, 4);
}

uint8_t sx1280_read_register(uint16_t reg){
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

void sx1280_set_high_power(){
	sx1280_write_register(0x0891, (sx1280_read_register(0x0891) | 0xC0));
}

//TODO: better delays
void sx1280_send_SPI_buffer(uint8_t* buf, uint8_t size){
	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi_sx1280, buf, size, 10);
	HAL_GPIO_WritePin(CS_GPIO_Port_SX1280, CS_Pin_SX1280, GPIO_PIN_SET);
}

