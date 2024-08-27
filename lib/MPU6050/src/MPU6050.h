#pragma once
#ifndef _MPU6050_H
#define _MPU6050_H

#include <Wire.h>
#include <Arduino.h>
#include "MPU6050RegisterMap.h"
#include "QuaternionFilter.h"

// Accel sensitivity - A2G (most sensitive)
enum class ACCEL_FS_SEL : uint8_t
{
    A2G,
    A4G,
    A8G,
    A16G
};

// Gyro sensitivity - G250DPS (most sensitive)
enum class GYRO_FS_SEL : uint8_t
{
    G250DPS,
    G500DPS,
    G1000DPS,
    G2000DPS
};

// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
// This assumes ACCEL_GYRO_DLPF_CFG is set to DLPF_184HZx188HZ - DLPF_5HZx5HZ, setting gyro and accelerometer output to 1kHz
// If ACCEL_GYRO_DLPF_CFG is set to DLPF_260HZx256HZ or DLPF_RESERVED, gyro output and Sample Rate is 8kHz, accelerometer is still 1kHz
enum class SAMPLE_RATE : uint8_t
{
    SMPL_1000HZ = 0,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ
};

// AccelxGyro filter Bandwidth
enum class ACCEL_GYRO_DLPF_CFG : uint8_t
{
    DLPF_260HZx256HZ = 0, // Accel delay 0ms, Gyro delay 0.98ms
    DLPF_184HZx188HZ,     // Accel delay 2.0ms, Gyro delay 1.9ms
    DLPF_94HZx98HZ,       // Accel delay 3.0ms, Gyro delay 2.8ms
    DLPF_44HZx42HZ,       // Accel delay 4.9ms, Gyro delay 4.8ms
    DLPF_21HZx20HZ,       // Accel delay 8.5ms, Gyro delay 8.3ms
    DLPF_10HZx10HZ,       // Accel delay 13.8ms, Gyro delay 13.4ms
    DLPF_5HZx5HZ,         // Accel delay 19.0ms, Gyro delay 18.6ms
    DLPF_RESERVED
};

struct MPU6050Setting
{
    ACCEL_FS_SEL accel_fs_sel;
    GYRO_FS_SEL gyro_fs_sel;
    SAMPLE_RATE sample_rate;
    ACCEL_GYRO_DLPF_CFG accel_gyro_dlpf_cfg;

    MPU6050Setting()
        : accel_fs_sel{ACCEL_FS_SEL::A2G}, gyro_fs_sel{GYRO_FS_SEL::G250DPS},
          sample_rate{SAMPLE_RATE::SMPL_1000HZ}, accel_gyro_dlpf_cfg{ACCEL_GYRO_DLPF_CFG::DLPF_260HZx256HZ} {}

    MPU6050Setting(ACCEL_FS_SEL _accel_fs_sel, GYRO_FS_SEL _gyro_fs_sel, SAMPLE_RATE _sample_rate, ACCEL_GYRO_DLPF_CFG _accel_gyro_dlpf_cfg)
        : accel_fs_sel{_accel_fs_sel}, gyro_fs_sel{_gyro_fs_sel},
          sample_rate{_sample_rate}, accel_gyro_dlpf_cfg{_accel_gyro_dlpf_cfg} {}
};

template <typename WireType>
class MPU6050_
{
public:
    static constexpr uint16_t CALIB_GYRO_SENSITIVITY{131};    // LSB/degrees/sec
    static constexpr uint16_t CALIB_ACCEL_SENSITIVITY{16384}; // LSB/g

    bool setup(const uint8_t addr = MPU6050_DEFAULT_ADDRESS, const MPU6050Setting &mpu_setting = MPU6050Setting(), WireType &w = Wire)
    {
        // addr should be valid for MPU
        if ((addr < MPU6050_DEFAULT_ADDRESS) || (addr > MPU6050_DEFAULT_ADDRESS + 7))
        {
            Serial.print("I2C address 0x");
            Serial.print(addr, HEX);
            Serial.println(" is not valid for MPU. Please check your I2C address.");
            return false;
        }
        mpu_i2c_addr = addr;
        setting = mpu_setting;
        wire = &w;

        if (isConnected())
        {
            initMPU6050();
        }
        else
        {
            if (b_verbose)
                Serial.println("Could not connect to MPU6050");
            has_connected = false;
            return false;
        }
        has_connected = true;
        return true;
    }

    void sleep(bool b)
    {
        byte c = read_byte(PWR_MGMT_1); // read the value, change sleep bit to match b, write byte back to register
        if (b)
        {
            c = c | 0x40; // sets the sleep bit
        }
        else
        {
            c = c & 0xBF; // mask 1011111 keeps all the previous bits
        }
        write_byte(PWR_MGMT_1, c);
    }

    void verbose(const bool b)
    {
        b_verbose = b;
    }

    void ahrs(const bool b)
    {
        b_ahrs = b;
    }

    void calibrateAccelGyro()
    {
        calibrate_acc_gyro_impl();
    }

    bool isConnected()
    {
        byte c = read_byte(WHO_AM_I_MPU6050);
        if (b_verbose)
        {
            Serial.print("MPU6050 WHO AM I = ");
            Serial.println(c, HEX);
        }
        return (c == mpu_i2c_addr);
    }

    bool isSleeping()
    {
        byte c = read_byte(PWR_MGMT_1);
        return (c & 0x40) == 0x40;
    }

    bool update()
    {
        if (!(has_connected && (read_byte(INT_STATUS) & 0x01)))
            return false;

        update_accel_gyro();

        /*
         * Madgwick function needs to be fed North, East, and Down direction like
         * (AN, AE, AD, GN, GE, GD)
         * Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
         * Magneto direction is Right-Hand, Y-Forward, Z-Down
         * So to adopt to the general Aircraft coordinate system (Right-Hand, X-Forward, Z-Down),
         * we need to feed (ax, -ay, -az, gx, -gy, -gz) but we pass (-ax, ay, az, gx, -gy, -gz)
         * because gravity is by convention positive down, we need to invert the accel data.
         * Get quaternion based on aircraft coordinate (Right-Hand, X-Forward, Z-Down)
         * Gyro will be converted from [deg/s] to [rad/s] inside of this function.
         */

        float an = -a[0];
        float ae = +a[1];
        float ad = +a[2];
        float gn = +g[0] * DEG_TO_RAD;
        float ge = -g[1] * DEG_TO_RAD;
        float gd = -g[2] * DEG_TO_RAD;

        for (size_t i = 0; i < n_filter_iter; ++i)
        {
            quat_filter.update(an, ae, ad, gn, ge, gd, q);
        }

        update_rpy(q[0], q[1], q[2], q[3]);
        return true;
    }

    float getRoll() const { return rpy[0]; }
    float getPitch() const { return rpy[1]; }
    float getYaw() const { return rpy[2]; }

    float getEulerX() const { return rpy[0]; }
    float getEulerY() const { return -rpy[1]; }
    float getEulerZ() const { return -rpy[2]; }

    float getQuaternionX() const { return q[1]; }
    float getQuaternionY() const { return q[2]; }
    float getQuaternionZ() const { return q[3]; }
    float getQuaternionW() const { return q[0]; }

    float getAcc(const uint8_t i) const { return (i < 3) ? a[i] : 0.f; }
    float getGyro(const uint8_t i) const { return (i < 3) ? g[i] : 0.f; }
    float getLinearAcc(const uint8_t i) const { return (i < 3) ? lin_acc[i] : 0.f; }

    float getAccX() const { return a[0]; }
    float getAccY() const { return a[1]; }
    float getAccZ() const { return a[2]; }
    float getGyroX() const { return g[0]; }
    float getGyroY() const { return g[1]; }
    float getGyroZ() const { return g[2]; }
    float getLinearAccX() const { return lin_acc[0]; }
    float getLinearAccY() const { return lin_acc[1]; }
    float getLinearAccZ() const { return lin_acc[2]; }

    float getAccBias(const uint8_t i) const { return (i < 3) ? acc_bias[i] : 0.f; }
    float getGyroBias(const uint8_t i) const { return (i < 3) ? gyro_bias[i] : 0.f; }

    float getAccBiasX() const { return acc_bias[0]; }
    float getAccBiasY() const { return acc_bias[1]; }
    float getAccBiasZ() const { return acc_bias[2]; }
    float getGyroBiasX() const { return gyro_bias[0]; }
    float getGyroBiasY() const { return gyro_bias[1]; }
    float getGyroBiasZ() const { return gyro_bias[2]; }

    float getTemperature() const { return temperature; }

    void setAccBias(const float x, const float y, const float z)
    {
        acc_bias[0] = x;
        acc_bias[1] = y;
        acc_bias[2] = z;
    }
    void setGyroBias(const float x, const float y, const float z)
    {
        gyro_bias[0] = x;
        gyro_bias[1] = y;
        gyro_bias[2] = z;
    }

    void setFilterIterations(const size_t n)
    {
        if (n > 0)
            n_filter_iter = n;
    }

    bool selftest()
    {
        return self_test_impl();
    }

private:
    static constexpr uint8_t MPU6050_DEFAULT_ADDRESS{0x68}; // Device address when ADO = 0
    uint8_t mpu_i2c_addr;

    // settings
    MPU6050Setting setting;
    float acc_resolution{0.f};  // scale resolutions per LSB for the sensors
    float gyro_resolution{0.f}; // scale resolutions per LSB for the sensors

    // Calibration Parameters
    float acc_bias[3]{0.0f, 0.0f, 0.0f};  // acc calibration value in ACCEL_FS_SEL: 2g
    float gyro_bias[3]{0.0f, 0.0f, 0.0f}; // gyro calibration value in GYRO_FS_SEL: 250dps

    // Temperature
    int16_t temperature_count{0}; // temperature raw count output
    float temperature{0.f};       // Stores the real internal chip temperature in degrees Celsius

    // Self Test
    float self_test_result[6]{0.f}; // holds results of gyro and accelerometer self test

    // IMU Data
    float a[3]{0.f, 0.f, 0.f};
    float g[3]{0.f, 0.f, 0.f};
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
    float rpy[3]{0.f, 0.f, 0.f};
    float lin_acc[3]{0.f, 0.f, 0.f}; // linear acceleration (acceleration with gravity component subtracted)
    QuaternionFilter quat_filter;
    size_t n_filter_iter{1};

    // Other settings
    bool has_connected{false};
    bool b_ahrs{true};
    bool b_verbose{false};

    // I2C
    WireType *wire;
    uint8_t i2c_err_;

    void initMPU6050()
    {
        acc_resolution = get_acc_resolution(setting.accel_fs_sel);
        gyro_resolution = get_gyro_resolution(setting.gyro_fs_sel);

        // reset device
        write_byte(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
        delay(100);

        // wake up device
        write_byte(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
        delay(100);                   // Wait for all registers to reset

        // get stable time source
        write_byte(PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference
        delay(200);

        // Configure Accelerometer & Gyro
        // If MPUSetting is not defined and passed to setup, the default settings are as follows
        // Accelerometer full scale range is +/-250 deg/s and Gyro is +/-2g (highest sensitivity)
        // Accelerometer outputs data at 1kHz and Gyro at 8kHz
        // SMPLRT_DIV is 0; making the Sample Rate 8kHz ( Sample Rate = Gyroscope Output Rate / 1 + SMPLRT_DIV)
        // DLPF is set to 256Hz bandwidth and has a delay of 0.98ms
        uint8_t mpu_config = (uint8_t)setting.accel_gyro_dlpf_cfg;
        write_byte(MPU_CONFIG, mpu_config);
        uint8_t sample_rate_div = (uint8_t)setting.sample_rate;
        write_byte(SMPLRT_DIV, sample_rate_div);

        // Set gyroscope full scale range
        // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        uint8_t c = read_byte(GYRO_CONFIG);          // get current GYRO_CONFIG register value
        c = c & ~0xE0;                               // Clear self-test bits [7:5]
        c = c & ~0x18;                               // Clear GYRO_FS_SEL bits [4:3]
        c = c | (uint8_t(setting.gyro_fs_sel) << 3); // Set full scale range for the gyro
        write_byte(GYRO_CONFIG, c);                  // Write new GYRO_CONFIG value to register

        // Set accelerometer full-scale range configuration
        c = read_byte(ACCEL_CONFIG);                  // get current ACCEL_CONFIG register value
        c = c & ~0xE0;                                // Clear self-test bits [7:5]
        c = c & ~0x18;                                // Clear ACCEL_FS_SEL bits [4:3]
        c = c | (uint8_t(setting.accel_fs_sel) << 3); // Set full scale range for the accelerometer
        write_byte(ACCEL_CONFIG, c);                  // Write new ACCEL_CONFIG register value

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
        // clear INT_STATUS on any read operation, and enable I2C_BYPASS_EN so additional chips
        // can join the I2C bus and all can be controlled by the Arduino as master
        write_byte(INT_PIN_CFG, 0x32);
        write_byte(INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
        delay(100);
    }

    void update_rpy(float qw, float qx, float qy, float qz)
    {
        // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
        // In this coordinate system, the positive z-axis is down toward Earth.
        // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
        // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
        // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
        // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
        // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
        // applied in the correct order which for this configuration is yaw, pitch, and then roll.
        // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
        float a12, a22, a31, a32, a33; // rotation matrix coefficients for Euler angles and gravity components
        a12 = 2.0f * (qx * qy + qw * qz);
        a22 = qw * qw + qx * qx - qy * qy - qz * qz;
        a31 = 2.0f * (qw * qx + qy * qz);
        a32 = 2.0f * (qx * qz - qw * qy);
        a33 = qw * qw - qx * qx - qy * qy + qz * qz;
        rpy[0] = atan2f(a31, a33);
        rpy[1] = -asinf(a32);
        rpy[2] = atan2f(a12, a22);
        rpy[0] *= 57.29577951;
        rpy[1] *= 57.29577951;
        rpy[2] *= 57.29577951;
        if (rpy[2] >= +180.f)
            rpy[2] -= 360.f;
        else if (rpy[2] < -180.f)
            rpy[2] += 360.f;

        lin_acc[0] = a[0] + a31;
        lin_acc[1] = a[1] + a32;
        lin_acc[2] = a[2] - a33;
    }

    void update_accel_gyro()
    {
        int16_t raw_acc_gyro_data[7];       // used to read all 14 bytes at once from the MPU6050 accel/gyro
        read_accel_gyro(raw_acc_gyro_data); // INT cleared on any read

        // Now we'll calculate the accleration value into actual g's
        a[0] = ((float)raw_acc_gyro_data[0] - acc_bias[0]) * acc_resolution; // get actual g value, this depends on scale being set
        a[1] = ((float)raw_acc_gyro_data[1] - acc_bias[1]) * acc_resolution;
        a[2] = ((float)raw_acc_gyro_data[2] - acc_bias[2]) * acc_resolution;

        temperature_count = raw_acc_gyro_data[3];               // Read the adc values
        temperature = (float)(temperature_count / 340) + 36.53; // Temperature in degrees Centigrade

        // Calculate the gyro value into actual degrees per second
        g[0] = ((float)raw_acc_gyro_data[4] - gyro_bias[0]) * gyro_resolution; // get actual gyro value, this depends on scale being set
        g[1] = ((float)raw_acc_gyro_data[5] - gyro_bias[1]) * gyro_resolution;
        g[2] = ((float)raw_acc_gyro_data[6] - gyro_bias[2]) * gyro_resolution;
    }

    void read_accel_gyro(int16_t *destination)
    {
        uint8_t raw_data[14];                                                // x/y/z accel register data stored here
        read_bytes(ACCEL_XOUT_H, 14, &raw_data[0]);                          // Read the 14 raw data registers into data array
        destination[0] = ((int16_t)raw_data[0] << 8) | (int16_t)raw_data[1]; // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)raw_data[2] << 8) | (int16_t)raw_data[3];
        destination[2] = ((int16_t)raw_data[4] << 8) | (int16_t)raw_data[5];
        destination[3] = ((int16_t)raw_data[6] << 8) | (int16_t)raw_data[7];
        destination[4] = ((int16_t)raw_data[8] << 8) | (int16_t)raw_data[9];
        destination[5] = ((int16_t)raw_data[10] << 8) | (int16_t)raw_data[11];
        destination[6] = ((int16_t)raw_data[12] << 8) | (int16_t)raw_data[13];
    }

    int16_t read_temperature_data()
    {
        uint8_t raw_data[2];                              // x/y/z gyro register data stored here
        read_bytes(TEMP_OUT_H, 2, &raw_data[0]);          // Read the two raw data registers sequentially into data array
        return ((int16_t)raw_data[0] << 8) | raw_data[1]; // Turn the MSB and LSB into a 16-bit value
    }

    // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
    // of the at-rest readings and then stores the resulting offsets into accelerometer and gyro bias variables
    // ACCEL_FS_SEL: 2g (maximum sensitivity)
    // GYRO_FS_SEL: 250dps (maximum sensitivity)
    void calibrate_acc_gyro_impl()
    {
        set_acc_gyro_to_calibration();
        collect_acc_gyro_data_to(acc_bias, gyro_bias);
        delay(100);
        if (b_verbose)
        {
            Serial.println("< Calibration Parameters >");
            Serial.println("Accel bias [g]: ");
            Serial.print(acc_bias[0]);
            Serial.print(", ");
            Serial.print(acc_bias[1]);
            Serial.print(", ");
            Serial.println(acc_bias[2]);
            Serial.println("Gyro bias [deg/s]: ");
            Serial.print(gyro_bias[0]);
            Serial.print(", ");
            Serial.print(gyro_bias[1]);
            Serial.print(", ");
            Serial.println(gyro_bias[2]);
        }
        initMPU6050();
        delay(1000);
    }

    void set_acc_gyro_to_calibration()
    {
        // reset device
        write_byte(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
        delay(100);

        // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
        write_byte(PWR_MGMT_1, 0x01);
        write_byte(PWR_MGMT_2, 0x00);
        delay(200);

        // Configure device for bias calculation
        write_byte(INT_ENABLE, 0x00);   // Disable all interrupts
        write_byte(FIFO_EN, 0x00);      // Disable FIFO
        write_byte(PWR_MGMT_1, 0x00);   // Turn on internal clock source
        write_byte(I2C_MST_CTRL, 0x00); // Disable I2C master
        write_byte(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
        write_byte(USER_CTRL, 0x04);    // Reset FIFO
        delay(15);

        // Configure MPU6050 gyro and accelerometer for bias calculation
        write_byte(MPU_CONFIG, 0x01);   // Set gyro low-pass filter to 188 Hz and accel to 184 Hz
        write_byte(SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
        write_byte(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
        write_byte(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

        // Configure FIFO to capture accelerometer and gyro data for bias calculation
        write_byte(USER_CTRL, 0x40); // Enable FIFO
        write_byte(FIFO_EN, 0x78);   // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
        delay(80);                   // accumulate 80 samples in 80 milliseconds = 960 bytes to prevent FIFO overflow
        write_byte(FIFO_EN, 0x00);   // Disable gyro and accelerometer sensors for FIFO after samples collected
    }

    void collect_acc_gyro_data_to(float *a_bias, float *g_bias)
    {
        uint8_t data[12];                     // data array to hold accelerometer and gyro x, y, z, data
        read_bytes(FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
        uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
        uint16_t packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

        for (uint16_t ii = 0; ii < packet_count; ii++)
        {
            int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
            read_bytes(FIFO_R_W, 12, &data[0]);                           // read data for averaging
            accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
            accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
            accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
            gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
            gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
            gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

            a_bias[0] += (float)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            a_bias[1] += (float)accel_temp[1];
            a_bias[2] += (float)accel_temp[2];
            g_bias[0] += (float)gyro_temp[0];
            g_bias[1] += (float)gyro_temp[1];
            g_bias[2] += (float)gyro_temp[2];

            delay(2);
        }
        a_bias[0] /= (float)packet_count; // Normalize sums to get average count biases
        a_bias[1] /= (float)packet_count;
        a_bias[2] /= (float)packet_count;
        g_bias[0] /= (float)packet_count;
        g_bias[1] /= (float)packet_count;
        g_bias[2] /= (float)packet_count;

        if (a_bias[2] > 0L)
        {
            a_bias[2] -= (float)CALIB_ACCEL_SENSITIVITY;
        } // Remove gravity from the z-axis accelerometer bias calculation
        else
        {
            a_bias[2] += (float)CALIB_ACCEL_SENSITIVITY;
        }
    }

    // Accelerometer and gyroscope self test; check calibration wrt factory settings
    bool self_test_impl() // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
    {
        uint8_t raw_data[6] = {0, 0, 0, 0, 0, 0};
        int16_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
        float factoryTrim[6];
        uint8_t FS = 0;

        write_byte(SMPLRT_DIV, 0x00);      // Set gyro sample rate to 1 kHz
        write_byte(MPU_CONFIG, 0x02);      // Set gyro and accel sample rate to 1 kHz and DLPF to 98 Hz
        write_byte(GYRO_CONFIG, FS << 3);  // Set full scale range for the gyro to 250 dps
        write_byte(ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g

        for (int ii = 0; ii < 200; ii++)
        { // get average current values of gyro and acclerometer

            read_bytes(ACCEL_XOUT_H, 6, &raw_data[0]);                       // Read the six raw data registers into data array
            aAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]); // Turn the MSB and LSB into a signed 16-bit value
            aAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
            aAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

            read_bytes(GYRO_XOUT_H, 6, &raw_data[0]);                        // Read the six raw data registers sequentially into data array
            gAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]); // Turn the MSB and LSB into a signed 16-bit value
            gAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
            gAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
        }

        for (int ii = 0; ii < 3; ii++)
        { // Get average of 200 values and store as average current readings
            aAvg[ii] /= 200;
            gAvg[ii] /= 200;
        }

        // Configure the accelerometer for self-test
        write_byte(ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
        write_byte(GYRO_CONFIG, 0xE0);  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
        delay(25);                      // Delay a while to let the device stabilize

        for (int ii = 0; ii < 200; ii++)
        { // get average self-test values of gyro and acclerometer

            read_bytes(ACCEL_XOUT_H, 6, &raw_data[0]);                         // Read the six raw data registers into data array
            aSTAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]); // Turn the MSB and LSB into a signed 16-bit value
            aSTAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
            aSTAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

            read_bytes(GYRO_XOUT_H, 6, &raw_data[0]);                          // Read the six raw data registers sequentially into data array
            gSTAvg[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]); // Turn the MSB and LSB into a signed 16-bit value
            gSTAvg[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
            gSTAvg[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);
        }

        for (int ii = 0; ii < 3; ii++)
        { // Get average of 200 values and store as average self-test readings
            aSTAvg[ii] /= 200;
            gSTAvg[ii] /= 200;
        }

        // Configure the gyro and accelerometer for normal operation
        write_byte(ACCEL_CONFIG, 0x00);
        write_byte(GYRO_CONFIG, 0x00);
        delay(25); // Delay a while to let the device stabilize

        // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
        uint8_t self_test_data[4];
        self_test_data[0] = read_byte(SELF_TEST_X);
        self_test_data[1] = read_byte(SELF_TEST_Y);
        self_test_data[2] = read_byte(SELF_TEST_Z);
        self_test_data[3] = read_byte(SELF_TEST_A);

        // Accel self test values are stored in 2 registers; i.e. SELF_TEST_X, Y and Z and A
        // First 2 bits of accel are stored in SELF_TEST_A X[5,4], Y[3,2] and Z[1,0] and the last 3 bits are stored in SELF_TEST_X, Y, and Z
        self_test_result[0] = ((self_test_data[0] & 0xE0) >> 3) | ((self_test_data[3] & 0x30) >> 4); // Xa
        self_test_result[1] = ((self_test_data[1] & 0xE0) >> 3) | ((self_test_data[3] & 0x0C) >> 2); // Ya
        self_test_result[2] = ((self_test_data[2] & 0xE0) >> 3) | ((self_test_data[3] & 0x03));      // Za

        // Gyro X, Y, and Z self test values are stored in the first 5 bits of SELF_TEST_X, Y and Z
        self_test_result[3] = self_test_data[0] & 0x1F; // Xg
        self_test_result[4] = self_test_data[1] & 0x1F; // Yg
        self_test_result[5] = self_test_data[2] & 0x1F; // Zg

        // Retrieve factory self - test value from self - test code reads
        factoryTrim[0] = self_test_result[0] != 0 ? (float)(4096 * 0.34 * (pow(0.92, (self_test_result[0] - 1) / pow(2, 5) - 2) / 0.34)) : 0.0f; // FT[Xa]
        factoryTrim[1] = self_test_result[1] != 0 ? (float)(4096 * 0.34 * (pow(0.92, (self_test_result[1] - 1) / pow(2, 5) - 2) / 0.34)) : 0.0f; // FT[Ya]
        factoryTrim[2] = self_test_result[2] != 0 ? (float)(4096 * 0.34 * (pow(0.92, (self_test_result[2] - 1) / pow(2, 5) - 2) / 0.34)) : 0.0f; // FT[Za]
        factoryTrim[3] = self_test_result[3] != 0 ? (float)(25 * 131 * pow(1.046, (self_test_result[3] - 1))) : 0.0f;                            // FT[Xg]
        factoryTrim[4] = self_test_result[4] != 0 ? (float)(-25 * 131 * pow(1.046, (self_test_result[4] - 1))) : 0.0f;                           // FT[Yg]
        factoryTrim[5] = self_test_result[5] != 0 ? (float)(25 * 131 * pow(1.046, (self_test_result[5] - 1))) : 0.0f;                            // FT[Zg]

        // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
        // To get percent, must multiply by 100
        for (int i = 0; i < 3; i++)
        {
            self_test_result[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i];         // Report percent differences
            self_test_result[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3]; // Report percent differences
        }

        if (b_verbose)
        {
            Serial.print("x-axis self test: acceleration trim within : ");
            Serial.print(self_test_result[0], 1);
            Serial.println("% of factory value");
            Serial.print("y-axis self test: acceleration trim within : ");
            Serial.print(self_test_result[1], 1);
            Serial.println("% of factory value");
            Serial.print("z-axis self test: acceleration trim within : ");
            Serial.print(self_test_result[2], 1);
            Serial.println("% of factory value");
            Serial.print("x-axis self test: gyration trim within : ");
            Serial.print(self_test_result[3], 1);
            Serial.println("% of factory value");
            Serial.print("y-axis self test: gyration trim within : ");
            Serial.print(self_test_result[4], 1);
            Serial.println("% of factory value");
            Serial.print("z-axis self test: gyration trim within : ");
            Serial.print(self_test_result[5], 1);
            Serial.println("% of factory value");
        }

        bool b = true;
        for (uint8_t i = 0; i < 6; ++i)
        {
            b &= fabs(self_test_result[i]) <= 14.f;
        }
        return b;
    }

    float get_acc_resolution(const ACCEL_FS_SEL accel_af_sel) const
    {
        switch (accel_af_sel)
        {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case ACCEL_FS_SEL::A2G:
            return 2.0 / 32768.0;
        case ACCEL_FS_SEL::A4G:
            return 4.0 / 32768.0;
        case ACCEL_FS_SEL::A8G:
            return 8.0 / 32768.0;
        case ACCEL_FS_SEL::A16G:
            return 16.0 / 32768.0;
        default:
            return 0.;
        }
    }

    float get_gyro_resolution(const GYRO_FS_SEL gyro_fs_sel) const
    {
        switch (gyro_fs_sel)
        {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case GYRO_FS_SEL::G250DPS:
            return 250.0 / 32768.0;
        case GYRO_FS_SEL::G500DPS:
            return 500.0 / 32768.0;
        case GYRO_FS_SEL::G1000DPS:
            return 1000.0 / 32768.0;
        case GYRO_FS_SEL::G2000DPS:
            return 2000.0 / 32768.0;
        default:
            return 0.;
        }
    }
    void write_byte(uint8_t subAddress, uint8_t data)
    {
        wire->beginTransmission(mpu_i2c_addr); // Initialize the Tx buffer
        wire->write(subAddress);               // Put slave register address in Tx buffer
        wire->write(data);                     // Put data in Tx buffer
        i2c_err_ = wire->endTransmission();    // Send the Tx buffer
        if (i2c_err_)
            print_i2c_error();
    }

    uint8_t read_byte(uint8_t subAddress)
    {
        uint8_t data = 0;                        // `data` will store the register data
        wire->beginTransmission(mpu_i2c_addr);   // Initialize the Tx buffer
        wire->write(subAddress);                 // Put slave register address in Tx buffer
        i2c_err_ = wire->endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
        if (i2c_err_)
            print_i2c_error();
        wire->requestFrom(mpu_i2c_addr, (size_t)1); // Read one byte from slave register address
        if (wire->available())
            data = wire->read(); // Fill Rx buffer with result
        return data;             // Return data read from slave register
    }

    void read_bytes(uint8_t subAddress, uint8_t count, uint8_t *dest)
    {
        wire->beginTransmission(mpu_i2c_addr);   // Initialize the Tx buffer
        wire->write(subAddress);                 // Put slave register address in Tx buffer
        i2c_err_ = wire->endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
        if (i2c_err_)
            print_i2c_error();
        uint8_t i = 0;
        wire->requestFrom(mpu_i2c_addr, count); // Read bytes from slave register address
        while (wire->available())
        {
            dest[i++] = wire->read();
        } // Put read results in the Rx buffer
    }

    void print_i2c_error()
    {
        if (i2c_err_ == 7)
            return; // to avoid stickbreaker-i2c branch's error code
        Serial.print("I2C ERROR CODE : ");
        Serial.println(i2c_err_);
    }
};

using MPU6050 = MPU6050_<TwoWire>;

#endif // _MPU6050_H
