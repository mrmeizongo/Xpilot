#include <Arduino.h>
#include "SysConfig.h"
#include "FlightConfigAccess.h"
#include "IMU.h"

#define MPU6050_ADDRESS 0x68    // I2C address of MPU6050
#define I2C_CLOCK_400KHZ 400000 // I2C clock speed in Hz

IMU::IMU(void)
    : _rpy{0.f, 0.f, 0.f}
    , _g{0.f, 0.f, 0.f}
{
}

void IMU::init(void)
{
    Wire.begin();
    Wire.setClock(I2C_CLOCK_400KHZ);
    /*
     * Settings options are;
     *
     * Accel sensitivity            Gyro sensitivity            FIFO sample rate                Accel & Gyro DLPF config
     * enum class ACCEL_FS_SEL      enum class GYRO_FS_SEL      enum class SAMPLE_RATE_DIV      enum class ACCEL_GYRO_DLPF_CFG
     * {                            {                           {                               {
     *      A2G = 0,                    G250DPS = 0,                SMPL_8KHZ = 0,                  DLPF_260HZx256HZ = 0,
     *      A4G,                        G500DPS,                    SMPL_500HZ,                     DLPF_184HZx188HZ,
     *      A8G,                        G1000DPS,                   SMPL_333HZ,                     DLPF_94HZx98HZ,
     *      A16G                        G2000DPS                    SMPL_250HZ,                     DLPF_44HZx42HZ,
     * };                           };                              SMPL_200HZ,                     DLPF_21HZx20HZ,
     *                                                              SMPL_167HZ,                     DLPF_10HZx10HZ,
     *                                                              SMPL_143HZ,                     DLPF_5HZx5HZ,
     *                                                              SMPL_1KHZ                       DLPF_RESERVED
     *                                                          };                              };
     * See MPU6050 library for more details
     */
    MPU6050Setting setting = MPU6050Setting(
        ACCEL_FS_SEL::A2G, GYRO_FS_SEL::G250DPS, SAMPLE_RATE_DIV::SMPL_1KHZ, ACCEL_GYRO_DLPF_CFG::DLPF_44HZx42HZ);

    // Initialize MPU
    if (!mpu6050.setup(MPU6050_ADDRESS, setting, config().controlConfig.dt))
    {
        Serial.println("Invalid MPU!");
        while (true)
            ;
    }

    mpu6050.setAccBias(config().imuConfig.accBiasX, config().imuConfig.accBiasY, config().imuConfig.accBiasZ);
    mpu6050.setGyroBias(config().imuConfig.gyroBiasX, config().imuConfig.gyroBiasY, config().imuConfig.gyroBiasZ);
}

void IMU::getLatestReadings(void)
{
    if (!mpu6050.update(_rpy, _g))
    {
        return;
    }

    if (_consumer != nullptr)
        _consumer(_rpy, _g);
}

void IMU::registerConsumer(Consumer cb) { _consumer = cb; }

void IMU::calibrate(void) { mpu6050.calibrateAccelGyro(); }

void IMU::getCalibration(float (&accel)[Axis::AXIS_COUNT], float (&gyro)[Axis::AXIS_COUNT])
{
    for (uint8_t i = Axis::X_AXIS; i < Axis::AXIS_COUNT; i++)
    {
        accel[i] = mpu6050.getAccBias(i);
        gyro[i] = mpu6050.getGyroBias(i);
    }
}

void IMU::printIMU(void)
{
    // Clear screen
    Serial.print("\033[2J");
    Serial.print("\033[H");

    Serial.print(F("Roll: "));
    Serial.println(_rpy[Axis::X_AXIS]);
    Serial.print(F("Pitch: "));
    Serial.println(_rpy[Axis::Y_AXIS]);
    Serial.print(F("Yaw: "));
    Serial.println(_rpy[Axis::Z_AXIS]);
    Serial.print(F("Gyro X: "));
    Serial.println(_g[Axis::X_AXIS]);
    Serial.print(F("Gyro Y: "));
    Serial.println(_g[Axis::X_AXIS]);
    Serial.print(F("Gyro Z: "));
    Serial.println(_g[Axis::X_AXIS]);
}

IMU imu;