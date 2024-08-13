#include "bmi270.h"
#include "imu.h"

extern const uint8_t bmi270_config_file[8193];
uint8_t bmi270_spi_working = 0;
uint8_t bmi270_init_working = 0;
uint8_t bmi270_ready = 0;

// buffer to use for internal read spi calls. save memory
uint8_t bmi270_init_spi_buf[2] = { 0x00, 0x00 };

uint8_t bmi270_data_read_buf[BMI_DATA_BUF_SIZE] = { 0x00 };
uint8_t bmi270_data_transmit_buf[BMI_DATA_BUF_SIZE] = { BMI270_REG_ACC_DATA_X_LSB | 0x80, 0x00 };

void bmi270_cs_low()
{
	HAL_GPIO_WritePin(CS_GPIO_Port_BMI270, CS_Pin_BMI270, GPIO_PIN_RESET);
}
void bmi270_cs_high()
{
	HAL_GPIO_WritePin(CS_GPIO_Port_BMI270, CS_Pin_BMI270, GPIO_PIN_SET);
}

void bmi270_read_data() // This is answered in an interrupt callback in imu.c (HAL_SPI_TxRxCpltCallback)
{
	bmi270_cs_low();
	HAL_SPI_TransmitReceive_DMA(hspi_imu, bmi270_data_transmit_buf, bmi270_data_read_buf,
			BMI_DATA_BUF_SIZE);
}

int16_t bmi270_get_CAS()
{
	//TODO: get constant register from enum
	uint8_t gyro_cas = bmi270_read_register(BMI270_REG_CAS, bmi270_init_spi_buf);
	int16_t gyro_cas_signed;
	gyro_cas = gyro_cas & 0x7F; //7 bit value
	if (gyro_cas & 0x40) // 7 bit signed integer
			{
		gyro_cas_signed = (int16_t) (((int16_t) gyro_cas) - 128);
	} else {
		gyro_cas_signed = (int16_t) (gyro_cas);
	}
	return gyro_cas_signed;
}

// Details for this init sequence are described in the BMI270 datasheet.
void bmi270_init()
{
	bmi270_enable_SPI();
	HAL_TIM_Base_Start_IT(htim_imu);
	bmi270_spi_working = bmi270_read_register(BMI270_REG_CHIP_ID, bmi270_init_spi_buf) == 0x24;

	if (bmi270_spi_working) {
		bmi270_write_register(BMI270_REG_PWR_CONF, 0x00);
		HAL_Delay(10);
		bmi270_write_register(BMI270_REG_INIT_CTRL, 0x00);
		HAL_Delay(1);
		// config file has target register 0x5e included, allows us to just pass in the array without doing any appending
		bmi270_burst_transmit((uint8_t*) bmi270_config_file, 100,
				sizeof(bmi270_config_file));
		HAL_Delay(1);
		bmi270_write_register(BMI270_REG_INIT_CTRL, 0x01);
		HAL_Delay(40);
		HAL_DMA_Init(&hdma_spi1_rx);
		HAL_DMA_Init(&hdma_spi1_tx);
		bmi270_init_working = bmi270_read_register(BMI270_REG_INTERNAL_STATUS,
				bmi270_init_spi_buf) == 0x01;
		if (bmi270_init_working) {
			bmi270_configure_settings();
			bmi270_configure_EXTI();
			bmi270_ready = 1;
		}
	}
}

// TODO: not constants
void bmi270_configure_settings()
{
	bmi270_write_register(BMI270_REG_PWR_CTRL, 0x0E);
	HAL_Delay(1);
	bmi270_write_register(BMI270_REG_ACC_CONF, 0xA8);
	HAL_Delay(1);
	bmi270_write_register(BMI270_REG_GYRO_CONF, 0xA9);
	HAL_Delay(1);
	bmi270_write_register(BMI270_REG_PWR_CONF, 0x02);
	HAL_Delay(1);
	bmi270_write_register(BMI270_REG_ACC_RANGE, 0x00);
	HAL_Delay(1);
	bmi270_write_register(BMI270_REG_GYRO_RANGE, 0b00001000);
	HAL_Delay(1);
}

void bmi270_configure_EXTI()
{
	bmi270_write_register(BMI270_REG_INT_MAP_DATA, 0b01000100);
	HAL_Delay(10);
	bmi270_write_register(BMI270_REG_INT1_IO_CTRL, 0b00001010);
	HAL_Delay(10);
	bmi270_write_register(BMI270_REG_INT2_IO_CTRL, 0b00001010);
}

void bmi270_enable_SPI()
{
	bmi270_cs_low();
	HAL_Delay(1);
	bmi270_cs_high();
	HAL_Delay(10);
}

//TODO: CHECK HAL_OK
uint8_t bmi270_read_register(uint8_t rgstr, uint8_t *out_buf)
{
	rgstr = rgstr | 0x80;
	bmi270_cs_low();
	//TODO: remove dummy delay values
	HAL_SPI_Transmit(hspi_imu, &rgstr, 1, 10);
	HAL_SPI_Receive(hspi_imu, out_buf, 2, 10);
	bmi270_cs_high();
	return out_buf[1];
}

void bmi270_write_register(uint8_t rgstr, uint8_t data)
{
	bmi270_cs_low();
	uint8_t buf[2] = { rgstr, data };
	HAL_SPI_Transmit(hspi_imu, buf, 2, 10);
	bmi270_cs_high();
}

uint8_t* bmi270_burst_read(uint8_t rgstr, uint8_t *out_buf, uint16_t size,
		uint32_t timeout)
{
	rgstr = rgstr | 0x80;
	bmi270_cs_low();
	//TODO: remove dummy delay values
	HAL_SPI_Transmit(hspi_imu, &rgstr, 1, 10);
	HAL_SPI_Receive(hspi_imu, out_buf, size + 1, timeout);
	bmi270_cs_high();
	return out_buf;
}

void bmi270_burst_transmit(uint8_t *transmit_buf, uint32_t timeout, uint16_t size)
{
	bmi270_cs_low();
	HAL_SPI_Transmit(hspi_imu, transmit_buf, size, timeout);
	bmi270_cs_high();
}

// BMI 270 binary to decimal converters
//accelerometer
float bmi270_lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
	double power = 2;

	float half_scale =
			(float) ((pow((double) power, (double) bit_width) / 2.0));

	return (val * g_range) / half_scale;
}

//gyro
float bmi270_lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
	double power = 2;

	float half_scale =
			(float) ((pow((double) power, (double) bit_width) / 2.0));

	return (dps / (half_scale)) * (val);
}
