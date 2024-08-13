#include "../src/MPU6050.h"

#define MPU6050_ADDRESS 0x68 // Original device address when ADO = 0

MPU6050 mpu6050;
float ahrs_roll, ahrs_pitch, ahrs_yaw = 0.f;
float gyroX, gyroY, gyroZ = 0.f;

void printCalibrationValues(void);

int main()
{
    // If using in an Arduino environment this part of the code should be in setup()
    Wire.begin();
    Serial.begin(9600);
    while (!Serial)
        ;

    if (!mpu6050.setup(MPU6050_ADDRESS))
    { // change to your own address
        for (;;)
        {
            Serial.println("No MPU found! Check connection");
            delay(1000);
        }
    }

    // To display debug message set verbose to true
    mpu6050.verbose(true);
    // Calibrate MPU6050 accelerometer and gyro
    mpu6050.calibrateAccelGyro();
    // Print values from calibration
    printCalibrationValues();

    // This part of the code should be in loop()
    for (;;)
    {
        // If we have data from MPU6050, use it
        if (mpu6050.update())
        {
            ahrs_roll = mpu6050.getRoll();
            ahrs_pitch = mpu6050.getPitch();
            ahrs_yaw = mpu6050.getYaw();

            gyroX = mpu6050.getGyroX();
            gyroY = mpu6050.getGyroY();
            gyroZ = mpu6050.getGyroZ();

            Serial.print("Roll: ");
            Serial.println(ahrs_roll);
            Serial.print("Pitch: ");
            Serial.println(ahrs_pitch);
            Serial.print("Yaw: ");
            Serial.println(ahrs_yaw);
            Serial.println();

            Serial.print("GyroX: ");
            Serial.println(gyroX);
            Serial.print("GyroY: ");
            Serial.println(gyroY);
            Serial.print("GyroZ: ");
            Serial.println(gyroZ);
            Serial.println();
        }
    }
}

void printCalibrationValues(void)
{
    Serial.println("< Calibration Parameters >");
    Serial.println("Accel Bias [g]: ");
    Serial.print(mpu6050.getAccBiasX() * 1000.f / (float)MPU6050::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu6050.getAccBiasY() * 1000.f / (float)MPU6050::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu6050.getAccBiasZ() * 1000.f / (float)MPU6050::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("Gyro Bias [deg/s]: ");
    Serial.print(mpu6050.getGyroBiasX() / (float)MPU6050::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu6050.getGyroBiasY() / (float)MPU6050::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu6050.getGyroBiasZ() / (float)MPU6050::CALIB_GYRO_SENSITIVITY);
    Serial.println();
}