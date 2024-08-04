#include "bmi270.h"
#include "biquadlpf.h"

extern float gyro[3];
extern float gyroPreFilt[3];
extern float accel[3];

void IMUInit();
void readIMUData();
