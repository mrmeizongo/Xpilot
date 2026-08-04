#ifndef _IMU_H
#define _IMU_H
#include <MPU6050.h>

class IMU
{
public:
    IMU(void);
    void init(void);
    void calibrate(void);                        // Calibrate the IMU and store the biases in EEPROM
    void restoreCalibration(void);               // Restore the calibration values from EEPROM
    void getLatestReadings(void);                // Process the IMU data and update the AHRS values
    static void getLatestReadingsTask(void *ctx) // Trampoline function for the scheduler to call the getLatestReadings function
    {
        static_cast<IMU *>(ctx)->getLatestReadings();
    }

    int16_t getRoll(void) { return static_cast<int16_t>(rpy[0]); }
    int16_t getPitch(void) { return static_cast<int16_t>(rpy[1]); }
    int16_t getYaw(void) { return static_cast<int16_t>(rpy[2]); }

    int16_t getGyroX(void) { return static_cast<int16_t>(g[0]); }
    int16_t getGyroY(void) { return static_cast<int16_t>(g[1]); }
    int16_t getGyroZ(void) { return static_cast<int16_t>(g[2]); }

    bool consumeNewData(void); // Call before calling the individual getter functions

private:
    /*
     * Inertial measurement unit
     */
    MPU6050 mpu6050;

    float rpy[3]; // Airplane coordinate system values
    float g[3];   // Angular velocity about the respective axis - xyz

    bool dataReady; // Flag to indicate if new data is available
};

extern IMU imu;
#endif // _IMU_H