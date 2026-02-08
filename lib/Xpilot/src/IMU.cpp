#include <Arduino.h>
#include <PlaneConfig.h>
#include <SystemConfig.h>
#include <PinChangeInterrupt.h>
#include <EEPROM.h>
#include "IMU.h"

// Functions to save and restore calibration data to/from EEPROM
static void saveToEEPROM(float accBiasX, float accBiasY, float accBiasZ,
                         float gyroBiasX, float gyroBiasY, float gyroBiasZ, uint8_t startOffset = CALIBRATE_MEMORY_OFFSET) __attribute__((unused));
static void restoreFromEEPROM(float &accBiasX, float &accBiasY, float &accBiasZ,
                              float &gyroBiasX, float &gyroBiasY, float &gyroBiasZ, uint8_t startOffset = CALIBRATE_MEMORY_OFFSET) __attribute__((unused));
static void readFromEEPROM(void) __attribute__((unused));

volatile static bool imuDataReady = false;

IMU::IMU(void)
{
    ahrs_pitch = 0.f;
    ahrs_roll = 0.f;
    ahrs_yaw = 0.f;
    gyroX = 0.f;
    gyroY = 0.f;
    gyroZ = 0.f;
}

void IMU::init(void)
{
    Wire.begin();
    Wire.setClock(I2C_CLOCK_400KHZ);
    /*
     * Disable External FSYNC and set DLPF_CFG to 3; bandwidths 44Hz and 42Hz respectively to eliminate effects of vibration from airplane
     * Setting DLPF_CFG to 3 limits the sample rate to 1kHz for both accelerometer and gyroscope and introduces a delay of 4.8ms
     * The sample rate is further reduced by a factor of 4 because of the SMPLRT_DIV setting (gyroscope output rate/(1 + SMPLRT_DIV))
     * Due to the filtering delay of 4.8ms by the DLPF, gyroscope data is processed and stabilized at a frequency of 208Hz
     * We don't want to go below this value for our sample rate so we will use the next highest value of 250Hz(i.e. SMPLRT_DIV 3 = SMPL_250HZ).
     * The Arduino Nano runs the entire loop in ~333Hz so it will handle IMU processing at 250Hz comfortably
     *
     * Settings options are;
     *
     * Accel sensitivity            Gyro sensitivity            FIFO sample rate            Accel & Gyro DLPF config
     * enum class ACCEL_FS_SEL      enum class GYRO_FS_SEL      enum class SAMPLE_RATE      enum class ACCEL_GYRO_DLPF_CFG
     * {                            {                           {                           {
     *      A2G = 0,                    G250DPS = 0,                SMPL_1000HZ = 0,            DLPF_260HZx256HZ = 0,
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
    {
        while (true)
        {
#if defined(IMU_DEBUG)
            Serial.println("MPU6050 not found! Check device address or I2C connection");
#endif
            delay(1000);
        }
    }

#if defined(READ_CALIBRATION_FROM_EEPROM)
    readFromEEPROM();
    while (true)
        ;
#else
    calibrate();
#endif

    pinMode(IMUPIN_INPUT, INPUT);
    attachPinChangeInterrupt(IMUPIN_INT, RISING);
}

void IMU::getLatestReadings(void)
{
    if (imuDataReady)
    {
        mpu6050.update();
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

        imuDataReady = false;
    }
}

void IMU::calibrate(void)
{
#if defined(SELF_TEST_ACCEL_GYRO)
    mpu6050.verbose(true);
    mpu6050.selftest() ? Serial.println("Self test passed.") : Serial.println("Self test failed.");
    while (true)
        ;
#elif defined(CALIBRATE_DEBUG)
    Serial.println("Calibrating...");
    mpu6050.verbose(true);
    mpu6050.calibrateAccelGyro();
    saveToEEPROM(mpu6050.getAccBiasX(), mpu6050.getAccBiasY(), mpu6050.getAccBiasZ(),
                 mpu6050.getGyroBiasX(), mpu6050.getGyroBiasY(), mpu6050.getGyroBiasZ());
    Serial.println("Calibration values saved to EEPROM.");
    delay(1000);
#elif defined(CALIBRATE)
    mpu6050.verbose(false);
    mpu6050.calibrateAccelGyro();
    saveToEEPROM(mpu6050.getAccBiasX(), mpu6050.getAccBiasY(), mpu6050.getAccBiasZ(),
                 mpu6050.getGyroBiasX(), mpu6050.getGyroBiasY(), mpu6050.getGyroBiasZ());
#else
    restoreCalibration();
#endif
}

void IMU::restoreCalibration(void)
{
    float accBiasX = 0.0f, accBiasY = 0.0f, accBiasZ = 0.0f;
    float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
    restoreFromEEPROM(accBiasX, accBiasY, accBiasZ,
                      gyroBiasX, gyroBiasY, gyroBiasZ);
    mpu6050.setAccBias(accBiasX, accBiasY, accBiasZ);
    mpu6050.setGyroBias(gyroBiasX, gyroBiasY, gyroBiasZ);
}

static void saveToEEPROM(float accBiasX, float accBiasY, float accBiasZ,
                         float gyroBiasX, float gyroBiasY, float gyroBiasZ, uint8_t startOffset)
{
    uint8_t eeAdd = startOffset;
    uint8_t eeSize = sizeof(float);
    // Save the calibration values to EEPROM
    EEPROM.put(eeAdd, accBiasX);
    eeAdd += eeSize;
    EEPROM.put(eeAdd, accBiasY);
    eeAdd += eeSize;
    EEPROM.put(eeAdd, accBiasZ);
    eeAdd += eeSize;
    EEPROM.put(eeAdd, gyroBiasX);
    eeAdd += eeSize;
    EEPROM.put(eeAdd, gyroBiasY);
    eeAdd += eeSize;
    EEPROM.put(eeAdd, gyroBiasZ);
}

static void restoreFromEEPROM(float &accBiasX, float &accBiasY, float &accBiasZ,
                              float &gyroBiasX, float &gyroBiasY, float &gyroBiasZ, uint8_t startOffset)
{
    uint8_t eeAdd = startOffset;
    uint8_t eeSize = sizeof(float);
    // Restore the calibration values from EEPROM
    EEPROM.get(eeAdd, accBiasX);
    eeAdd += eeSize;
    EEPROM.get(eeAdd, accBiasY);
    eeAdd += eeSize;
    EEPROM.get(eeAdd, accBiasZ);
    eeAdd += eeSize;
    EEPROM.get(eeAdd, gyroBiasX);
    eeAdd += eeSize;
    EEPROM.get(eeAdd, gyroBiasY);
    eeAdd += eeSize;
    EEPROM.get(eeAdd, gyroBiasZ);
}

static void readFromEEPROM(void)
{
    float accBiasX = 0.0f, accBiasY = 0.0f, accBiasZ = 0.0f;
    float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
    restoreFromEEPROM(accBiasX, accBiasY, accBiasZ,
                      gyroBiasX, gyroBiasY, gyroBiasZ);

    Serial.println("Calibration values from EEPROM:");
    Serial.print("Accel Bias X: ");
    Serial.println(accBiasX);
    Serial.print("Accel Bias Y: ");
    Serial.println(accBiasY);
    Serial.print("Accel Bias Z: ");
    Serial.println(accBiasZ);
    Serial.print("Gyro Bias X: ");
    Serial.println(gyroBiasX);
    Serial.print("Gyro Bias Y: ");
    Serial.println(gyroBiasY);
    Serial.print("Gyro Bias Z: ");
    Serial.println(gyroBiasZ);
}

void PinChangeInterruptEvent(IMUPIN_INT)(void)
{
    imuDataReady = true;
}

IMU imu;