#include <Arduino.h>
#include <PlaneConfig.h>
#include <SystemConfig.h>
#include <EEPROM.h>
#include "IMU.h"

// Functions to save and restore calibration data to/from EEPROM
static void saveToEEPROM(float accBiasX, float accBiasY, float accBiasZ,
                         float gyroBiasX, float gyroBiasY, float gyroBiasZ, uint8_t startOffset = CALIBRATE_MEMORY_OFFSET) __attribute__((unused));
static void restoreFromEEPROM(float &accBiasX, float &accBiasY, float &accBiasZ,
                              float &gyroBiasX, float &gyroBiasY, float &gyroBiasZ, uint8_t startOffset = CALIBRATE_MEMORY_OFFSET) __attribute__((unused));
static void readFromEEPROM(void) __attribute__((unused));

IMU::IMU(void)
{
    rpy[0] = 0.f;
    rpy[1] = 0.f;
    rpy[2] = 0.f;
    g[0] = 0.f;
    g[1] = 0.f;
    g[2] = 0.f;
}

void IMU::init(void)
{
    Wire.begin();
    Wire.setClock(I2C_CLOCK_400KHZ);
    /*
     * Settings options are;
     *
     * Accel sensitivity            Gyro sensitivity            FIFO sample rate            Accel & Gyro DLPF config
     * enum class ACCEL_FS_SEL      enum class GYRO_FS_SEL      enum class SAMPLE_RATE      enum class ACCEL_GYRO_DLPF_CFG
     * {                            {                           {                           {
     *      A2G = 0,                    G250DPS = 0,                SMPL_8KHZ = 0,            DLPF_260HZx256HZ = 0,
     *      A4G,                        G500DPS,                    SMPL_500HZ,                 DLPF_184HZx188HZ,
     *      A8G,                        G1000DPS,                   SMPL_333HZ,                 DLPF_94HZx98HZ,
     *      A16G                        G2000DPS                    SMPL_250HZ,                 DLPF_44HZx42HZ,
     * };                           };                              SMPL_200HZ,                 DLPF_21HZx20HZ,
     *                                                              SMPL_167HZ,                 DLPF_10HZx10HZ,
     *                                                              SMPL_143HZ,                 DLPF_5HZx5HZ,
     *                                                              SMPL_1KHZ                  DLPF_RESERVED
     *                                                          };                          };
     * See MPU6050 library for more details
     */
    MPU6050Setting setting = MPU6050Setting(ACCEL_FS_SEL::A2G, GYRO_FS_SEL::G250DPS, SAMPLE_RATE::SMPL_250HZ, ACCEL_GYRO_DLPF_CFG::DLPF_21HZx20HZ);

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
}

void IMU::getLatestReadings(void)
{
    mpu6050.update(rpy, g);
#if defined(REVERSE_ROLL)
    rpy[0] = -rpy[0];
#endif
#if defined(REVERSE_PITCH)
    rpy[1] = -rpy[1];
#endif
#if defined(REVERSE_YAW)
    rpy[2] = -rpy[2];
#endif
#if defined(REVERSE_X_GYRO)
    g[0] = -g[0];
#endif
#if defined(REVERSE_Y_GYRO)
    g[1] = -g[1];
#endif
#if defined(REVERSE_Z_GYRO)
    g[2] = -g[2];
#endif
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

IMU imu;