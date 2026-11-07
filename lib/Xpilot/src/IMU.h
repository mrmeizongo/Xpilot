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
    enum AXIS : uint8_t
    {
        X_AXIS = 0U,
        Y_AXIS,
        Z_AXIS,
        AXIS_COUNT
    };

    IMU(void);
    void init(void);
    void updateAHRS(void); // Collect and update the AHRS values from sensor
    void calibrate(void);  // Obtain bias values for the IMU sensors
    void printAHRS(void);  // Debugging

    int32_t getRPY(AXIS axis) const;
    int32_t getG(AXIS axis) const;

    static void updateAHRSTask(void* ctx) { static_cast<IMU*>(ctx)->updateAHRS(); }
    static void printAHRSTask(void* ctx) { static_cast<IMU*>(ctx)->printAHRS(); }

private:
    /*
     * Inertial measurement unit
     */
    MPU6050 mpu6050;

    float raw_rpy[AXIS::AXIS_COUNT]; // Airplane coordinate system values
    float raw_g[AXIS::AXIS_COUNT];   // Angular velocity about the respective axis - xyz

    int32_t scaled_rpy[AXIS::AXIS_COUNT]; // To hold imu rpy values
    int32_t scaled_g[AXIS::AXIS_COUNT];   // To hold imu g values
};

extern IMU imu;
#endif // _IMU_H