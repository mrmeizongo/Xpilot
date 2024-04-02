// 03/13/2024 by Jamal Meizongo (mrmeizongo@outlook.com)
// This and other library code in this repository
// are partial releases and work is still in progress.
// Please keep this in mind as you use this piece of software.

/* ============================================
Xpilot library code is placed under the MIT license
Copyright (c) 2024 by
Author: Jamal Meizongo (mrmeizongo@outlook.com)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#pragma once

#include <I2Cdev.h>
#include <MPU9250.h>
#include <Servo.h>
#include "xpilot_config.h"

class Xpilot
{
public:
    Xpilot(void);

    // Prevent this class from being copied
    CLASS_NO_COPY(Xpilot);

    enum class FLIGHT_MODE : uint8_t
    {
        MANUAL = 1,
        FBW,
        STABILIZE
    };

    friend class Mode; // Flight mode controller

    void setup(void);
    void loop(void);

    void processIMU(void);
    void processInput(void);
    void processOutput(void);

    void get_imu_scaled(void);
    void mahoganyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

    FLIGHT_MODE getCurrentMode() { return currentMode; }
    void setCurrentMode(FLIGHT_MODE _currentMode)
    {
#if DEBUG
        Serial.print("Flight mode: ");
        Serial.println((uint8_t)_currentMode);
#endif
        currentMode = _currentMode;
    }

#if IO_DEBUG
    void print_imu(void);
    void print_input(void);
    void print_output(void);
#endif

private:
    MPU9250 imu;
    I2Cdev i2c;

    uint8_t aileronPinInt;      // Interrupt pin for aileron
    uint8_t aileron_out = 0;    // Aileron servo val
    byte rollDeflectionLim = 0; // Aileron deflection limit
    int aileronPulseWidth = 0;  // Aileron values obtained from transmitter

    uint8_t elevatorPinInt;      // Interrupt pin for elevator
    uint8_t elevator_out = 0;    // Elevator servo val
    byte pitchDeflectionLim = 0; // Elevator deflection limit
    int elevatorPulseWidth = 0;  // Elevator values obtained from transmitter

    FLIGHT_MODE currentMode = FLIGHT_MODE::FBW; // FBW is default mode

    Servo aileronServo;  // Aileron servo channel
    Servo elevatorServo; // Elevator servo channel

    // vvvvvvvvvvvvvvvvvv  VERY VERY IMPORTANT vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    // These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto 1.2
    float A_B[3]{539.75, 218.36, 834.53};
    float A_Ainv[3][3]{{0.51280, 0.00230, 0.00202},
                       {0.00230, 0.51348, -0.00126},
                       {0.00202, -0.00126, 0.50368}};

    // mag offsets and correction matrix
    float M_B[3]{18.15, 28.05, -36.09};
    float M_Ainv[3][3]{{0.68093, 0.00084, 0.00923},
                       {0.00084, 0.69281, 0.00103},
                       {0.00923, 0.00103, 0.64073}};

    float G_off[3] = {-299.7, 113.2, 202.4}; // raw offsets, determined for gyro at rest
    // ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // Raw data from IMU and scaled as vector
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
    float Axyz[3];
    float Gxyz[3];
    float Mxyz[3];

    // Vector to hold quaternion values from AHRS algorithm
    float q[4] = {1.0, 0.0, 0.0, 0.0};
    float ahrs_yaw, ahrs_pitch, ahrs_roll = 0; // Euler angle output
};

extern Xpilot xpilot;