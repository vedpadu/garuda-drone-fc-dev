#include "imu.h"

float gyro[3];
float accel[3];

void IMUInit()
{
	BMI270Init();
}

void readIMUData()
{
	BMI270ReadData(accel, gyro);
}
// need accessible gyro and accel

