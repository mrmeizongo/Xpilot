#include <Arduino.h>
#include "IMU.h"

IMU::IMU(void) {}

void IMU::init(void)
{
    // Initialize MPU
    if (!mpu6050.setup(0x68))
    { // change to your own address
        for (;;)
        {
#if defined(IMU_DEBUG)
            Serial.println("No MPU found! Check connection");
#endif
            delay(1000);
        }
    }

#if defined(SELF_TEST_ACCEL_GYRO)
    mpu6050.selftest() ? Serial.println("Self test passed.") : Serial.println("Self test failed");
    while (true)
        ;
#endif
}

bool IMU::processIMU(void)
{
    if (mpu6050.update())
    {
#if defined(REVERSE_ROLL_STABILIZE)
        ahrs_roll = -((mpu6050.getRoll() + IMU_ROLL_TRIM));
#else
        ahrs_roll = (mpu6050.getRoll() + IMU_ROLL_TRIM);
#endif

#if defined(REVERSE_PITCH_STABILIZE)
        ahrs_pitch = -((mpu6050.getPitch() + IMU_PITCH_TRIM));
#else
        ahrs_pitch = (mpu6050.getPitch() + IMU_PITCH_TRIM);
#endif

#if defined(REVERSE_YAW_STABILIZE)
        ahrs_yaw = -mpu6050.getYaw();
#else
        ahrs_yaw = mpu6050.getYaw();
#endif

#if defined(REVERSE_ROLL_GYRO)
        gyroX = -(mpu6050.getGyroX());
#else
        gyroX = mpu6050.getGyroX();
#endif
#if defined(REVERSE_PITCH_GYRO)
        gyroY = -(mpu6050.getGyroY());
#else
        gyroY = mpu6050.getGyroY();
#endif
#if defined(REVERSE_YAW_GYRO)
        gyroZ = -(mpu6050.getGyroZ());
#else
        gyroZ = mpu6050.getGyroZ();
#endif
        return true;
    }

    return false;
}

// IMU Debug functions
#if defined(IMU_DEBUG)
void IMU::print_imu(void)
{
    Serial.print("Roll: ");
    Serial.println(ahrs_roll);
    Serial.print("Pitch: ");
    Serial.println(ahrs_pitch);
    Serial.print("Yaw: ");
    Serial.println(ahrs_yaw);
    Serial.println();
}
#endif

IMU imu;