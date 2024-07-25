#include "bmi270.h"
#include "spi.h"
#include "gpio.h"

extern const uint8_t bmi270_config_file[8193];
uint8_t spiWorking = 0;
uint8_t initWorking = 0;

// buffer to use for internal read spi calls. save memory
uint8_t bmi270_init_spi_buf[2] = {0x00, 0x00};
uint8_t bmi270_data_spi_buf[13] = {0x00};

void cs_low(){ HAL_GPIO_WritePin(CS_GPIO_Port_BMI270, CS_Pin_BMI270, GPIO_PIN_RESET); }
void cs_high(){ HAL_GPIO_WritePin(CS_GPIO_Port_BMI270, CS_Pin_BMI270, GPIO_PIN_SET); }

void BMI270ReadData(float* accelBuf, float* gyroBuf)
{
	//accelX = 1,2; accelY = 3,4; accelZ = 5,6
	burst_read(BMI270_REG_ACC_DATA_X_LSB, bmi270_data_spi_buf, 12, 10);
	uint16_t accelXBin = ((uint16_t)bmi270_data_spi_buf[2]) << 8 | bmi270_data_spi_buf[1];
	uint16_t accelYBin = ((uint16_t)bmi270_data_spi_buf[4]) << 8 | bmi270_data_spi_buf[3];
	uint16_t accelZBin = ((uint16_t)bmi270_data_spi_buf[6]) << 8 | bmi270_data_spi_buf[5];
	uint16_t gyroXBin = ((uint16_t)bmi270_data_spi_buf[8]) << 8 | bmi270_data_spi_buf[7];
	uint16_t gyroYBin = ((uint16_t)bmi270_data_spi_buf[10]) << 8 | bmi270_data_spi_buf[9];
	uint16_t gyroZBin = ((uint16_t)bmi270_data_spi_buf[12]) << 8 | bmi270_data_spi_buf[11];
	int16_t gyro_cas = getCAS();
	int16_t gyroXSigned = (int16_t)gyroXBin;
	int16_t gyroZSigned = (int16_t)gyroZBin;
	gyroXSigned = gyroXSigned - (int16_t)(((int32_t) gyro_cas * (int32_t) gyroZSigned) / 512);

	accelBuf[0] = lsb_to_mps2((int16_t)accelXBin, (float)2.0, 16);
	accelBuf[1] = lsb_to_mps2((int16_t)accelYBin, (float)2.0, 16);
	accelBuf[2] = lsb_to_mps2((int16_t)accelZBin, (float)2.0, 16);

	gyroBuf[0] = lsb_to_dps(gyroXSigned, (float)2000.0, 16);
	gyroBuf[1] = lsb_to_dps((int16_t)gyroYBin, (float)2000.0, 16);
	gyroBuf[2] = lsb_to_dps(gyroZSigned, (float)2000.0, 16);
}

int16_t getCAS(){
	//TODO: get constant register from enum
	uint8_t gyro_cas = read_register(0x3c, bmi270_init_spi_buf);
	int16_t gyro_cas_ret;
	gyro_cas = gyro_cas & 0x7F;
	if(gyro_cas & 0x40)
	{
		gyro_cas_ret = (int16_t)(((int16_t)gyro_cas) - 128);
	}else{
		gyro_cas_ret = (int16_t)(gyro_cas);
	}
	return gyro_cas_ret;
}

void BMI270Init()
{
	bmi270EnableSPI();
	HAL_TIM_Base_Start_IT(exti_tim);
	spiWorking = read_register(BMI270_REG_CHIP_ID, bmi270_init_spi_buf) == 0x24;

	if(spiWorking){
		// Details for this init sequence in the datasheet.
		write_register(BMI270_REG_PWR_CONF, 0x00);
		HAL_Delay(10);
		write_register(BMI270_REG_INIT_CTRL, 0x00);
		HAL_Delay(1);
		// config file has target register included
		burst_transmit((uint8_t*)bmi270_config_file, 100, sizeof(bmi270_config_file));
		HAL_Delay(1);
		write_register(BMI270_REG_INIT_CTRL, 0x01);
		HAL_Delay(40);
		initWorking = read_register(BMI270_REG_INTERNAL_STATUS, bmi270_init_spi_buf) == 0x01;
		if(initWorking)
		{
			configureBMI270();
			configureBMI270EXTI();
		}
	}
}

void configureBMI270()
{
	write_register(BMI270_REG_PWR_CTRL, 0x0E);
	HAL_Delay(1);
	write_register(BMI270_REG_ACC_CONF, 0xA8);
	HAL_Delay(1);
	write_register(BMI270_REG_GYRO_CONF, 0xA9);
	HAL_Delay(1);
	write_register(BMI270_REG_PWR_CONF, 0x02);
	HAL_Delay(1);
	write_register(BMI270_REG_ACC_RANGE, 0x00);
	HAL_Delay(1);
	write_register(BMI270_REG_GYRO_RANGE, 0b00001000);
	HAL_Delay(1);
}

void configureBMI270EXTI()
{
	write_register(BMI270_REG_INT_MAP_DATA, 0b01000100);
	HAL_Delay(10);
	write_register(BMI270_REG_INT1_IO_CTRL, 0b00001010);
	HAL_Delay(10);
	write_register(BMI270_REG_INT2_IO_CTRL, 0b00001010);
}

void bmi270EnableSPI()
{
	cs_low();
    HAL_Delay(1);
    cs_high();
    HAL_Delay(10);
}

//TODO: CHECK HAL_OK
uint8_t read_register(uint8_t rgstr, uint8_t* out_buf)
{
	rgstr = rgstr | 0x80;
	cs_low();
	//TODO: remove dummy delay values
    HAL_SPI_Transmit(hspi_bmi270, &rgstr, 1, 10);
    HAL_SPI_Receive(hspi_bmi270, out_buf, 2, 10);
    cs_high();
    return out_buf[1];
}

void write_register(uint8_t rgstr, uint8_t data)
{
	cs_low();
	uint8_t buf[2] = {rgstr, data};
	HAL_SPI_Transmit(hspi_bmi270, buf, 2, 10);
	cs_high();
}

uint8_t* burst_read(uint8_t rgstr, uint8_t* out_buf, uint16_t size, uint32_t timeout)
{
	rgstr = rgstr | 0x80;
	cs_low();
	//TODO: remove dummy delay values
	HAL_SPI_Transmit(hspi_bmi270, &rgstr, 1, 10);
	HAL_SPI_Receive(hspi_bmi270, out_buf, size + 1, timeout);
	cs_high();
	return out_buf;
}

void burst_transmit(uint8_t* transmit_buf, uint32_t timeout, uint16_t size)
{
	cs_low();
	HAL_SPI_Transmit(hspi_bmi270, transmit_buf, size, timeout);
	cs_high();
}

float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (9.8 * val * g_range) / half_scale;
}

float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}
