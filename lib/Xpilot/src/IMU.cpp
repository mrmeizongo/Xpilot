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
#if defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
            Serial.println("No MPU found! Check connection");
#endif
            delay(1000);
        }
    }
#if defined(CALIBRATE_DEBUG)
    Serial.println("Accel Gyro calibration will start in 3sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu6050.verbose(true);
    delay(3000);
    // Calibrate IMU accelerometer and gyro
    mpu6050.calibrateAccelGyro();
    Serial.println("Calibration complete.");
    print_calibration();
#elif defined(CALIBRATE)
    mpu6050.verbose(false);
    mpu6050.calibrateAccelGyro();
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
        ahrs_yaw = -((mpu6050.getYaw() + IMU_YAW_TRIM));
#else
        ahrs_yaw = (mpu6050.getYaw() + IMU_YAW_TRIM);
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

// IO Debug functions
#if defined(IMU_DEBUG) || defined(CALIBRATE_DEBUG)
void IMU::print_imu(void)
{
    // Serial.print("Yaw: ");
    // Serial.println(ahrs_yaw);
    Serial.print("Roll: ");
    Serial.println(ahrs_roll);
    Serial.print("Pitch: ");
    Serial.println(ahrs_pitch);
    Serial.print("Yaw: ");
    Serial.println(ahrs_yaw);
    Serial.println();
}
#endif

#if defined(CALIBRATE_DEBUG)
void IMU::print_calibration(void)
{
    Serial.println("< Calibration Parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu6050.getAccBiasX() * 1000.f / (float)MPU6050::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu6050.getAccBiasY() * 1000.f / (float)MPU6050::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu6050.getAccBiasZ() * 1000.f / (float)MPU6050::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu6050.getGyroBiasX() / (float)MPU6050::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu6050.getGyroBiasY() / (float)MPU6050::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu6050.getGyroBiasZ() / (float)MPU6050::CALIB_GYRO_SENSITIVITY);
    Serial.println();
}
#endif

IMU imu;