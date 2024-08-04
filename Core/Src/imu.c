#include "imu.h"

float32_t gyro[3];
float32_t accel[3];
float32_t gyroPreFilt[3];
float32_t accelPreFilt[3];
BiquadLPF gyroBiquad;
BiquadLPF accelBiquad;

// come on does this file need to exist?
void IMUInit()
{
	BMI270Init();
	biquadLPFInit(&gyroBiquad, 30.0, 500.0);
	biquadLPFInit(&accelBiquad, 10.0, 500.0);
}

void readIMUData()
{
	BMI270ReadData(accelPreFilt, gyroPreFilt);
	biquadLPFApply(&gyroBiquad, gyroPreFilt, gyro);
	biquadLPFApply(&accelBiquad, accelPreFilt, accel);
}
// need accessible gyro and accel

