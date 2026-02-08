#ifndef _IMU_H
#define _IMU_H
#include <MPU6050.h>

class IMU
{
public:
    IMU(void);
    void init(void);
    void calibrate(void);          // Calibrate the IMU and store the biases in EEPROM
    void restoreCalibration(void); // Restore the calibration values from EEPROM
    void getLatestReadings(void);  // Process the IMU data and update the AHRS values

    float getRoll(void) { return ahrs_roll; }
    float getPitch(void) { return ahrs_pitch; }
    float getYaw(void) { return ahrs_yaw; }

    float getGyroX(void) { return gyroX; }
    float getGyroY(void) { return gyroY; }
    float getGyroZ(void) { return gyroZ; }

private:
    /*
     * Inertial measurement unit
     */
    MPU6050 mpu6050;

    float ahrs_pitch, ahrs_roll, ahrs_yaw; // Airplane coordinate system values
    float gyroX, gyroY, gyroZ;             // Angular velocity about the respective axis
};

extern IMU imu;
#endif // _IMU_H