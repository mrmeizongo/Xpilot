#ifndef _IMU_H
#define _IMU_H
#include <MPU9250.h>
#include "config.h"

class IMU
{
public:
    IMU(void);
    void init(void);

    bool processIMU(void);

#if defined(CALIBRATE_DEBUG)
    void print_calibration(void);
#endif

#if defined(IMU_DEBUG)
    void print_imu(void);
#endif

    float ahrs_pitch, ahrs_roll, ahrs_yaw = 0; // Airplane coordinate system values
    float gyroX, gyroY, gyroZ = 0;             // Angular velocity around the respective axis

private:
    /*
     * Inertial measurement unit variable
     */
    MPU9250 mpu9250;
};

extern IMU imu;
#endif // _IMU_H