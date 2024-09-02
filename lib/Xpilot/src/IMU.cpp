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
#define MPU6050_ADDRESS 0x68
#define I2C_CLOCK_400KHZ 400000U

IMU::IMU(void)
{
}

void IMU::init(void)
{
    Wire.begin();
    Wire.setClock(I2C_CLOCK_400KHZ); // Setting I2C clock to 400Khz
    /*
     * Disable External FSYNC and set DLPF_CFG to 3; bandwidths 44Hz and 42Hz respectively to eliminate effects of vibration from airplane
     * Setting DLPF_CFG to 3 limits the sample rate to 1kHz for both accelerometer and gyroscope and introduces a delay of 4.8ms
     * The sample rate is further reduced by a factor of 4 because of the SMPLRT_DIV setting (gyroscope output rate/(1 + SMPLRT_DIV))
     * Due to the filtering delay of 4.8ms by the DLPF, gyroscope data is processed and stabilized at a frequency of 208Hz
     * We don't want to go below this value for our sample rate so we will use the next highest value of 250Hz(i.e. SMPRT_DIV 3 = SMPL_250HZ).
     * The Arduino Nano runs the entire loop in ~333Hz so it will handle IMU processing at 250Hz comfortably
     *
     * Settings options are;
     *
     * Accel sensitivity            Gyro sensitivity            FIFO sample rate            Accel & Gyro DLPF config
     * enum class ACCEL_FS_SEL      enum class GYRO_FS_SEL      enum class SAMPLE_RATE      enum class ACCEL_GYRO_DLPF_CFG
     * {                            {                           {                           {
     *      A2G,                        G250DPS,                    SMPL_1000HZ = 0,            DLPF_260HZx256HZ = 0,
     *      A4G,                        G500DPS,                    SMPL_500HZ,                 DLPF_184HZx188HZ,
     *      A8G,                        G1000DPS,                   SMPL_333HZ,                 DLPF_94HZx98HZ,
     *      A16G                        G2000DPS                    SMPL_250HZ,                 DLPF_44HZx42HZ,
     * };                           };                              SMPL_200HZ,                 DLPF_21HZx20HZ,
     *                                                              SMPL_167HZ,                 DLPF_10HZx10HZ,
     *                                                              SMPL_143HZ,                 DLPF_5HZx5HZ,
     *                                                              SMPL_125HZ                  DLPF_RESERVED
     *                                                          };                          };
     * See MPU6050 library for more details
     */
    MPU6050Setting setting = MPU6050Setting(ACCEL_FS_SEL::A2G, GYRO_FS_SEL::G250DPS, SAMPLE_RATE::SMPL_250HZ, ACCEL_GYRO_DLPF_CFG::DLPF_44HZx42HZ);

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
        ahrs_yaw = -(mpu6050.getYaw() + IMU_YAW_TRIM);
#else
        ahrs_yaw = mpu6050.getYaw() + IMU_YAW_TRIM;
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