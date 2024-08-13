#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "bmi270.h"
#include "biquadlpf.h"
#include "main.h"

extern float gyro[3];
extern float accel[3];

#define GYRO_RANGE_DPS 2000.0
#define ACCEL_G_RANGE 2.0

void IMUInit();
void readIMUData();

#endif
