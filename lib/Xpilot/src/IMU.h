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

    IMU(void);
    void init(void);
    void updateAHRS(void); // Process the IMU data and update the AHRS values
    void printAHRS(void);  // Debugging
    void calibrate(void);  // Obtain sensor bias values

    int32_t getRPY(Axis axis) const;
    int32_t getG(Axis axis) const;

    static void updateAHRSTask(void* ctx) { static_cast<IMU*>(ctx)->updateAHRS(); }
    static void printAHRSTask(void* ctx) { static_cast<IMU*>(ctx)->printAHRS(); }

private:
    /*
     * Inertial measurement unit
     */
    MPU6050 mpu6050;

    float raw_rpy[Axis::AXIS_COUNT]; // Airplane coordinate system values
    float raw_g[Axis::AXIS_COUNT];   // Angular velocity about the respective axis - xyz

    int32_t scaled_rpy[Axis::AXIS_COUNT]; // To hold imu rpy values
    int32_t scaled_g[Axis::AXIS_COUNT];   // To hold imu g values
};

extern IMU imu;
#endif // _IMU_H