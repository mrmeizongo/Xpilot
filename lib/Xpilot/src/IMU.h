#ifndef _IMU_H
#define _IMU_H
#include "MPU6050.h"

class IMU
{
public:
    using Callback = void (*)(float (&)[3], float (&)[3], void *);

    struct Consumer
    {
        Callback cb;
        void *ctx;
    };

    IMU(void);
    void init(void);
    void calibrate(void);          // Calibrate the IMU and store the biases in EEPROM
    void restoreCalibration(void); // Restore the calibration values from EEPROM
    void getLatestReadings(void);  // Process the IMU data and update the AHRS values

    static void getLatestReadingsTask(void *ctx) // Trampoline function for the scheduler to call the getLatestReadings function
    {
        static_cast<IMU *>(ctx)->getLatestReadings();
    }

    int16_t getRoll(void) { return static_cast<int16_t>(_rpy[0]); }
    int16_t getPitch(void) { return static_cast<int16_t>(_rpy[1]); }
    int16_t getYaw(void) { return static_cast<int16_t>(_rpy[2]); }

    int16_t getGyroX(void) { return static_cast<int16_t>(_g[0]); }
    int16_t getGyroY(void) { return static_cast<int16_t>(_g[1]); }
    int16_t getGyroZ(void) { return static_cast<int16_t>(_g[2]); }

    /// @brief              Register a callback to be invoked when new imu data is received
    /// @param callback     Function to execute
    /// @param ctx          Context pointer passed to the callback
    void registerConsumer(Callback, void *);

private:
    /*
     * Inertial measurement unit
     */
    MPU6050 mpu6050;

    float _rpy[3]; // Airplane coordinate system values
    float _g[3];   // Angular velocity about the respective axis - xyz

    Consumer _consumers[MAX_CONSUMERS]; // IMU values consumer
    uint8_t _consumerCount;
};

extern IMU imu;
#endif // _IMU_H