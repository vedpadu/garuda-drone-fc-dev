#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "bmi270.h"
#include "biquadlpf.h"

extern float gyro[3];
extern float accel[3];

void IMUInit();
void readIMUData();

#endif
