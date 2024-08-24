#include <Arduino.h>
#include "IMU.h"

/*
 * ACCEL & GYRO biases obtained from running calibration function
 * Uncomment CALIBRATION_DEBUG to obtain these values
 * See NOTICE section in README.md for more information
 */
#define ACC_X_BIAS 919.48f
#define ACC_Y_BIAS -437.80f
#define ACC_Z_BIAS 3838.37f
#define GYRO_X_BIAS -344.23f
#define GYRO_Y_BIAS -31.68f
#define GYRO_Z_BIAS -105.15f

#define IMU_WARMUP_LOOP 1000U

IMU::IMU(void) {}

void IMU::init(void)
{
    // Initialize MPU
    if (!mpu6050.setup(0x68))
    { // change to your own address
        for (;;)
        {
#if defined(IMU_DEBUG)
            Serial.println("MPU6050 not found! Check device address or I2C connection");
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

    // Warm up the IMU before initial use
    for (uint16_t i = 0; i < IMU_WARMUP_LOOP; i++)
    {
        processIMU();
    }
}

void IMU::processIMU(void)
{
    if (mpu6050.update())
    {
#if defined(REVERSE_ROLL_STABILIZE)
        ahrs_roll = -(mpu6050.getRoll() + IMU_ROLL_TRIM);
#else
        ahrs_roll = mpu6050.getRoll() + IMU_ROLL_TRIM;
#endif

#if defined(REVERSE_PITCH_STABILIZE)
        ahrs_pitch = -(mpu6050.getPitch() + IMU_PITCH_TRIM);
#else
        ahrs_pitch = mpu6050.getPitch() + IMU_PITCH_TRIM;
#endif

#if defined(REVERSE_YAW_STABILIZE)
        ahrs_yaw = -(mpu6050.getYaw());
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

IMU imu;