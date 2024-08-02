// 03/13/2024 by Jamal Meizongo (mrmeizongo@outlook.com)
// This and other library code in this repository
// are partial releases and work is still in progress.
// Please keep this in mind as you use this piece of software.

/* ============================================
Flight stabilization software
    Copyright (C) 2024 Jamal Meizongo (mrmeizongo@outlook.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
===============================================
*/

#include "Xpilot.h"
#include "FlightModeController.h"
#include "Radio.h"

#define IMU_WARMUP_LOOP 1000U
#define I2C_CLOCK_1MHZ 1000000U

// Timer variables
unsigned long nowMs, outputLastMs = 0;
// -------------------------

Xpilot::Xpilot(void)
{
}

void Xpilot::setup(void)
{
    Wire.begin();
    Wire.setClock(I2C_CLOCK_1MHZ); // Overclocking I2C to 1Mhz

#if defined(MIXING_DEBUG) || defined(IO_DEBUG) || defined(LOOP_DEBUG) || defined(CALIBRATE_DEBUG) || defined(IMU_DEBUG)
    Serial.begin(9600);
    while (!Serial)
    {
        Serial.println("Serial port communication cannot be established.");
        delay(1000);
    }
#endif

    // Initialize MPU
    if (!imu.setup(0x68))
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
    imu.verbose(true);
    delay(3000);
    // Calibrate IMU accelerometer and gyro
    imu.calibrateAccelGyro();
    Serial.println("Calibration complete.");
    print_calibration();
#elif defined(CALIBRATE)
    imu.verbose(false);
    imu.calibrateAccelGyro();
#endif

    rx.init(); // Initialize radio

    // All input pins use pin change interrupts
    // Aileron setup
    aileron1Servo.attach(AILPIN1_OUTPUT);
    aileron2Servo.attach(AILPIN2_OUTPUT);
    pinMode(AILPIN_INPUT, INPUT_PULLUP);

    // Elevator setup
    elevatorServo.attach(ELEVPIN_OUTPUT);
    pinMode(ELEVPIN_INPUT, INPUT_PULLUP);

    // Rudder setup
    rudderServo.attach(RUDDPIN_OUTPUT);
    pinMode(RUDDPIN_INPUT, INPUT_PULLUP);

    // Mode setup
    pinMode(MODEPIN_INPUT, INPUT_PULLUP);

    // Warm up the IMU
    for (uint16_t i = 0; i < IMU_WARMUP_LOOP; i++)
    {
        processIMU();
    }
}

/*
    Main xpilot execution loop
    Read input, read imu data, process output to servos
*/
void Xpilot::loop(void)
{
    nowMs = millis();
    processIMU();

    // Process output to servos at 50Hz intervals
    if (nowMs - outputLastMs >= 20)
    {
        rx.processInput();
        processOutput();
        outputLastMs = nowMs;
    }

#if defined(IO_DEBUG) || defined(IMU_DEBUG)
    static unsigned long debugLastMs = 0;
    if (nowMs - debugLastMs >= 1000)
    {
#if defined(IMU_DEBUG)
        print_imu();
#endif
#if defined(IO_DEBUG)
        print_output();
#endif
        debugLastMs = nowMs;
    }
#endif

#if defined(LOOP_DEBUG)
    unsigned long loopMillis = millis() - nowMs;
    Serial.print("Loop time: ");
    Serial.print(loopMillis);
    Serial.println("ms");
    Serial.print("Loop rate: ");
    loopMillis = loopMillis <= 0 ? 1 : loopMillis; // To prevent division by 0
    Serial.print(1000 / loopMillis);
    Serial.println("Hz");
#endif
}

void Xpilot::processIMU(void)
{
    if (imu.update())
    {
#if defined(REVERSE_ROLL_STABILIZE)
        ahrs_roll = -((int16_t)(imu.getRoll() + IMU_ROLL_TRIM));
#else
        ahrs_roll = (int16_t)(imu.getRoll() + IMU_ROLL_TRIM);
#endif

#if defined(REVERSE_PITCH_STABILIZE)
        ahrs_pitch = -((int16_t)(imu.getPitch() + IMU_PITCH_TRIM));
#else
        ahrs_pitch = (int16_t)(imu.getPitch() + IMU_PITCH_TRIM);
#endif

#if defined(REVERSE_YAW_STABILIZE)
        ahrs_yaw = -((int16_t)(imu.getYaw() + IMU_YAW_TRIM));
#else
        ahrs_yaw = (int16_t)(imu.getYaw() + IMU_YAW_TRIM);
#endif
    }

#if defined(REVERSE_ROLL_GYRO)
    gyroX = -((int16_t)imu.getGyroX());
#else
    gyroX = (int16_t)imu.getGyroX();
#endif
#if defined(REVERSE_PITCH_GYRO)
    gyroY = -((int16_t)imu.getGyroY());
#else
    gyroY = (int16_t)imu.getGyroY();
#endif
#if defined(REVERSE_YAW_GYRO)
    gyroZ = -((int16_t)imu.getGyroZ());
#else
    gyroZ = (int16_t)imu.getGyroZ();
#endif
}

void Xpilot::processOutput(void)
{
    modeController.process();

    aileron1_out = constrain(aileron1_out, SERVO_MIN_PWM, SERVO_MAX_PWM);
    aileron2_out = constrain(aileron2_out, SERVO_MIN_PWM, SERVO_MAX_PWM);
    elevator_out = constrain(elevator_out, SERVO_MIN_PWM, SERVO_MAX_PWM);
    rudder_out = constrain(rudder_out, SERVO_MIN_PWM, SERVO_MAX_PWM);

    aileron1Servo.writeMicroseconds(aileron1_out);
    aileron2Servo.writeMicroseconds(aileron2_out);
    elevatorServo.writeMicroseconds(elevator_out);
    rudderServo.writeMicroseconds(rudder_out);
}

// IO Debug functions
#if defined(IMU_DEBUG)
void Xpilot::print_imu(void)
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

#if defined(IO_DEBUG)
void Xpilot::print_output(void)
{
    Serial.print("Elevator Servo: ");
    Serial.println(elevator_out);
    Serial.print("Aileron Servo 1: ");
    Serial.println(aileron1_out);
    Serial.print("Aileron Servo 2: ");
    Serial.println(aileron2_out);
    Serial.print("Rudder Servo: ");
    Serial.println(rudder_out);
    Serial.println();
}
#endif

#if defined(CALIBRATE_DEBUG)
void Xpilot::print_calibration(void)
{
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(imu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(imu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(imu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(imu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(imu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(imu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(imu.getMagBiasX());
    Serial.print(", ");
    Serial.print(imu.getMagBiasY());
    Serial.print(", ");
    Serial.print(imu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(imu.getMagScaleX());
    Serial.print(", ");
    Serial.print(imu.getMagScaleY());
    Serial.print(", ");
    Serial.print(imu.getMagScaleZ());
    Serial.println();
}
#endif
// ---------------------------

Xpilot xpilot;