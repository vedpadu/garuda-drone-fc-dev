#include "imu.h"

float32_t gyro[3] = {0};
float32_t accel[3] = {0};
float32_t gyro_pre_filt[3] = {0};
float32_t accel_pre_filt[3] = {0};
BiquadLPF gyro_biquad_filt;
BiquadLPF accel_biquad_filt;

//float32_t gyroB[3] = {(float32_t)-0.001, (float32_t)-0.002, (float32_t)-0.00106}; -> drone 1
float32_t gyro_biases[3] = {(float32_t)-0.0013, (float32_t)-0.0035, (float32_t)-0.0019};
float32_t accel_biases[3] = {(float32_t)0.295/9.8, (float32_t)-0.032/9.8, (float32_t)0.013/9.8};

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi == hspi_bmi270){
		cs_high();
		uint16_t accelXBin = ((uint16_t)bmi270_data_read_buf[3]) << 8 | bmi270_data_read_buf[2];
		uint16_t accelYBin = ((uint16_t)bmi270_data_read_buf[5]) << 8 | bmi270_data_read_buf[4];
		uint16_t accelZBin = ((uint16_t)bmi270_data_read_buf[7]) << 8 | bmi270_data_read_buf[6];
		uint16_t gyroXBin = ((uint16_t)bmi270_data_read_buf[9]) << 8 | bmi270_data_read_buf[8];
		uint16_t gyroYBin = ((uint16_t)bmi270_data_read_buf[11]) << 8 | bmi270_data_read_buf[10];
		uint16_t gyroZBin = ((uint16_t)bmi270_data_read_buf[13]) << 8 | bmi270_data_read_buf[12];

		int16_t gyro_cas = bmi270_get_CAS();
		int16_t gyroXSigned = (int16_t)gyroXBin;
		int16_t gyroZSigned = (int16_t)gyroZBin;
		gyroXSigned = gyroXSigned - (int16_t)(((int32_t) gyro_cas * (int32_t) gyroZSigned) / 512);

		accel_pre_filt[0] = (float32_t)lsb_to_mps2((int16_t)accelXBin, (float)2.0, 16);
		accel_pre_filt[1] = (float32_t)lsb_to_mps2((int16_t)accelYBin, (float)2.0, 16);
		accel_pre_filt[2] = (float32_t)lsb_to_mps2((int16_t)accelZBin, (float)2.0, 16);

		gyro_pre_filt[0] = (float32_t)lsb_to_dps(gyroXSigned, (float)2000.0, 16) * M_PI / 180.0;
		gyro_pre_filt[1] = (float32_t)lsb_to_dps((int16_t)gyroYBin, (float)2000.0, 16) * M_PI / 180.0;
		gyro_pre_filt[2] = (float32_t)lsb_to_dps(gyroZSigned, (float)2000.0, 16) * M_PI / 180.0;

		biquadLPFApply(&gyro_biquad_filt, gyro_pre_filt, gyro);
		biquadLPFApply(&accel_biquad_filt, accel_pre_filt, accel);

		// removing biases as well as converting axes
		gyro[0] = -(gyro[0] - gyro_biases[0]);
		gyro[1] = gyro[1] - gyro_biases[1];
		gyro[2] = -(gyro[2] - gyro_biases[2]);

		accel[0] = -(accel[0] - accel_biases[0]);
		accel[1] = (accel[1] - accel_biases[1]);
		accel[2] = -(accel[2] - accel_biases[2]);
	}

}

void IMUInit()
{
	bmi270_init();
	biquadLPFInit(&gyro_biquad_filt, 30.0, 500.0);
	biquadLPFInit(&accel_biquad_filt, 10.0, 500.0);
}

void readIMUData()
{
	bmi270_read_data();
}

