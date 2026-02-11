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

    float getRoll(void) { return rpy[0]; }
    float getPitch(void) { return rpy[1]; }
    float getYaw(void) { return rpy[2]; }

    float getGyroX(void) { return g[0]; }
    float getGyroY(void) { return g[1]; }
    float getGyroZ(void) { return g[2]; }

private:
    /*
     * Inertial measurement unit
     */
    MPU6050 mpu6050;

    float rpy[3]; // Airplane coordinate system values
    float g[3];   // Angular velocity about the respective axis - xyz
};

extern IMU imu;
#endif // _IMU_H