#ifndef _IMU_H
#define _IMU_H
#include "MPU6050.h"

// Xpilot body/control convention
// +Roll = right wind down
// +Pitch = nose up
// +Yaw = nose right

class IMU
{
public:
    enum Axis : uint8_t
    {
        X_AXIS = 0U,
        Y_AXIS,
        Z_AXIS,
        AXIS_COUNT
    };

    using Consumer = void (*)(const float (&)[Axis::AXIS_COUNT], const float (&)[Axis::AXIS_COUNT]);

    IMU(void);
    void init(void);
    void getLatestReadings(void); // Process the IMU data and update the AHRS values
    void calibrate(void);         // Calibrate IMU to obtain sensor bias values
    void getCalibration(float (&)[Axis::AXIS_COUNT], float (&)[Axis::AXIS_COUNT]);

    static void getLatestReadingsTask(void* ctx) // Trampoline function for the scheduler
    {
        static_cast<IMU*>(ctx)->getLatestReadings();
    }

    int16_t getRoll(void) { return static_cast<int16_t>(_rpy[Axis::X_AXIS]); }
    int16_t getPitch(void) { return static_cast<int16_t>(_rpy[Axis::Y_AXIS]); }
    int16_t getYaw(void) { return static_cast<int16_t>(_rpy[Axis::Z_AXIS]); }

    int16_t getGyroX(void) { return static_cast<int16_t>(_g[Axis::X_AXIS]); }
    int16_t getGyroY(void) { return static_cast<int16_t>(_g[Axis::Y_AXIS]); }
    int16_t getGyroZ(void) { return static_cast<int16_t>(_g[Axis::Z_AXIS]); }

    /// @brief              Register a single callback to be invoked when new imu data is received
    /// @param callback     Function to execute
    /// @param ctx          Context pointer passed to the callback
    void registerConsumer(Consumer);

private:
    /*
     * Inertial measurement unit
     */
    MPU6050 mpu6050;

    float _rpy[Axis::AXIS_COUNT]; // Airplane coordinate system values
    float _g[Axis::AXIS_COUNT];   // Angular velocity about the respective axis - xyz

    Consumer _consumer; // IMU values consumer
};

extern IMU imu;
#endif // _IMU_H