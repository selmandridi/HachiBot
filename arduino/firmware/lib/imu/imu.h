#ifndef _IMU2_H_
#define _IMU2_H_


#include "imu_config.h"

#include <Wire.h>
#include "geometry_msgs/Vector3.h"

#define GYRO_SENSITIVITY_2000DPS          (0.070F)
#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */

bool initIMU()
{
    Wire.begin();
   

    delay(2000);

    if (!mpuUnit.setup(0x68))
     {  
        while (1) 
        {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    
    return true;
}

bool updateIMU()
{
    return mpuUnit.update();
}

geometry_msgs::Vector3 readLinearAcceleration()
{
    geometry_msgs::Vector3 accel;    

    accel.x = mpuUnit.getLinearAccX();
    accel.y = mpuUnit.getLinearAccY();
    accel.z = mpuUnit.getLinearAccZ();

    return accel;
}

geometry_msgs::Vector3 readAngularVelocity()
{
    geometry_msgs::Vector3 gyro;

    gyro.x = mpuUnit.getGyroX() * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
    gyro.y = mpuUnit.getGyroY() * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
    gyro.z = mpuUnit.getGyroZ() * GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;

    return gyro;
}

geometry_msgs::Quaternion readOrientation()
{
    geometry_msgs::Quaternion qua;

    qua.x = mpuUnit.getQuaternionX();
    qua.y = mpuUnit.getQuaternionY();
    qua.z = mpuUnit.getQuaternionZ();
    qua.w = mpuUnit.getQuaternionW();

    return qua;
}

void calibrateAccelGyro()
{
    mpuUnit.calibrateAccelGyro();
}

void calibrateMag()
{
    mpuUnit.verbose(true);
    mpuUnit.calibrateMag();
    mpuUnit.verbose(false);
}

#endif
