#ifndef _IMU_CONFIG_H_
#define _IMU_CONFIG_H_

#include <MPU9250.h>

#define G_TO_ACCEL 9.81
#define MGAUSS_TO_UTESLA 0.1
#define UTESLA_TO_TESLA 0.000001
#define ACCEL_SCALE 1 / 16384 // LSB/g
#define GYRO_SCALE 1 / 131 // LSB/(deg/s)
#define MAG_SCALE 0.6 // uT/LSB
    
MPU9250 mpuUnit;

#endif
