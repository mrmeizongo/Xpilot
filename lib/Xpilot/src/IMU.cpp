#include <Arduino.h>
#include "IMU.h"

/*
 * ACCEL & GYRO biases obtained from running calibration function
 * Uncomment CALIBRATION_DEBUG to obtain these values
 * See NOTICE section in README.md for more information
 */
#define ACC_X_BIAS 1625.16f
#define ACC_Y_BIAS -244.20f
#define ACC_Z_BIAS 3652.10f
#define GYRO_X_BIAS -343.88f
#define GYRO_Y_BIAS -56.52f
#define GYRO_Z_BIAS -124.02f

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
    mpu6050.verbose(true);
    mpu6050.selftest() ? Serial.println("Self test passed.") : Serial.println("Self test failed");
    while (true)
        ;
#endif

/*
 * After calibration, be sure to record the obtained values and enter them in the XXX_X_BIAS definitions above
 */
#if defined(CALIBRATE_DEBUG)
    Serial.println("Calibrating...");
    mpu6050.verbose(true);
    mpu6050.calibrateAccelGyro();
#elif defined(CALIBRATE)
    mpu6050.verbose(false);
    mpu6050.calibrateAccelGyro();
#else
    mpu6050.setAccBias(ACC_X_BIAS, ACC_Y_BIAS, ACC_Z_BIAS);
    mpu6050.setGyroBias(GYRO_X_BIAS, GYRO_Y_BIAS, GYRO_Z_BIAS);
#endif
}

void IMU::processIMU(void)
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
    }
}

// IMU Debug functions
#if defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
void IMU::printIMU(void)
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