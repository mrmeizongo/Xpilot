#include <Arduino.h>
#include "IMU.h"

IMU::IMU(void) {}

void IMU::init(void)
{
    // Initialize MPU
    if (!mpu9250.setup(0x68))
    { // change to your own address
        for (;;)
        {
#if defined(IMU_DEBUG)
            Serial.println("No MPU found! Check connection");
#endif
            delay(1000);
        }
    }
#if defined(CALIBRATE_DEBUG)
    Serial.println("Accel Gyro calibration will start in 3sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu9250.verbose(true);
    delay(3000);
    // Calibrate IMU accelerometer and gyro
    mpu9250.calibrateAccelGyro();
    Serial.println("Calibration complete.");
    print_calibration();
#elif defined(CALIBRATE)
    mpu9250.verbose(false);
    mpu9250.calibrateAccelGyro();
#endif
}

bool IMU::processIMU(void)
{
    if (mpu9250.update())
    {
#if defined(REVERSE_ROLL_STABILIZE)
        ahrs_roll = -((int16_t)(mpu9250.getRoll() + IMU_ROLL_TRIM));
#else
        ahrs_roll = (int16_t)(mpu9250.getRoll() + IMU_ROLL_TRIM);
#endif

#if defined(REVERSE_PITCH_STABILIZE)
        ahrs_pitch = -((int16_t)(mpu9250.getPitch() + IMU_PITCH_TRIM));
#else
        ahrs_pitch = (int16_t)(mpu9250.getPitch() + IMU_PITCH_TRIM);
#endif

#if defined(REVERSE_YAW_STABILIZE)
        ahrs_yaw = -((int16_t)(mpu9250.getYaw() + IMU_YAW_TRIM));
#else
        ahrs_yaw = (int16_t)(mpu9250.getYaw() + IMU_YAW_TRIM);
#endif

#if defined(REVERSE_ROLL_GYRO)
        gyroX = -((int16_t)mpu9250.getGyroX());
#else
        gyroX = (int16_t)mpu9250.getGyroX();
#endif
#if defined(REVERSE_PITCH_GYRO)
        gyroY = -((int16_t)mpu9250.getGyroY());
#else
        gyroY = (int16_t)mpu9250.getGyroY();
#endif
#if defined(REVERSE_YAW_GYRO)
        gyroZ = -((int16_t)mpu9250.getGyroZ());
#else
        gyroZ = (int16_t)mpu9250.getGyroZ();
#endif
        return true;
    }

    return false;
}

// IO Debug functions
#if defined(IMU_DEBUG)
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
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu9250.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu9250.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu9250.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu9250.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu9250.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu9250.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu9250.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu9250.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu9250.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu9250.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu9250.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu9250.getMagScaleZ());
    Serial.println();
}
#endif

IMU imu;