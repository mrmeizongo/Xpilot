#include <Arduino.h>
#include "IMU.h"

/*
 * ACCEL & GYRO biases obtained from running calibration function
 * Uncomment CALIBRATION_DEBUG to obtain these values
 * See NOTICE section in README.md for more information
 */
#define ACC_X_BIAS 113.87f
#define ACC_Y_BIAS -79.00f
#define ACC_Z_BIAS -339.75f
#define GYRO_X_BIAS -559.22f
#define GYRO_Y_BIAS 5.75f
#define GYRO_Z_BIAS -21.94f

#define IMU_WARMUP_LOOP 1000U
#define MPU6050_ADDRESS 0x68

IMU::IMU(void) {}

void IMU::init(void)
{
    // Disable External FSYNC and set accelerometer and gyroscope DLPF to 3; bandwidths 44Hz and 42Hz respectively
    // DLPF 3 also introduces a processing delay of 4.8ms which will set the filtering frequency to 208Hz
    // We don't want to go below this value for our sample rate so will use 250Hz(i.e. SMPRT_DIV = 3)
    // Setting DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1kHz for both accelerometer and gyroscope
    // This is further reduced by a factor of 4 to 250 Hz because of the SMPLRT_DIV setting (gyroscope output rate/(1 + SMPLRT_DIV))
    MPU6050Setting setting = MPU6050Setting(ACCEL_FS_SEL::A2G, GYRO_FS_SEL::G250DPS, FIFO_SAMPLE_RATE::SMPL_250HZ, ACCEL_GYRO_DLPF_CFG::DLPF_42HZ);

    // Initialize MPU
    if (!mpu6050.setup(MPU6050_ADDRESS, setting))
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
    for (;;)
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