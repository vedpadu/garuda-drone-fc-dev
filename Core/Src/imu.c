#include "imu.h"

float32_t gyro[3];
float32_t accel[3];

// come on does this file need to exist?
void IMUInit()
{
	BMI270Init();
}

void readIMUData()
{
	BMI270ReadData(accel, gyro);
}
// need accessible gyro and accel

