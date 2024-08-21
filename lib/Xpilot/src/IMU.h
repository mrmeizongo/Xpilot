#ifndef _IMU_H
#define _IMU_H
#include <MPU6050.h>
#include "config.h"

class IMU
{
public:
    IMU(void);
    void init(void);
    void processIMU(void);

    float getRoll(void) { return ahrs_roll; }
    float getPitch(void) { return ahrs_pitch; }
    float getYaw(void) { return ahrs_yaw; }

    float getGyroX(void) { return gyroX; }
    float getGyroY(void) { return gyroY; }
    float getGyroZ(void) { return gyroZ; }

private:
    /*
     * Inertial measurement unit variable
     */
    MPU6050 mpu6050;

    float ahrs_pitch, ahrs_roll, ahrs_yaw = 0; // Airplane coordinate system values
    float gyroX, gyroY, gyroZ = 0;             // Angular velocity around the respective axis
};

extern IMU imu;
#endif // _IMU_H