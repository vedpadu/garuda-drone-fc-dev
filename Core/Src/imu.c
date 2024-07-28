#include "imu.h"

float gyro[3];
float accel[3];

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

