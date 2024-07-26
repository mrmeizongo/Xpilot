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
#include <PinChangeInterrupt.h>
#include "Mode.h"

#define IMU_WARMUP_LOOP 1000U
#define I2C_CLOCK_1MHZ 1000000U

// Timer variables
unsigned long nowMs, outputLastMs = 0;

#if IO_DEBUG
unsigned long debugLastMs = 0;
#endif

#if LOOP_DEBUG
unsigned long loopLastMs = 0;
#endif
// -------------------------

// Aileron variables
volatile long aileronCurrentTime, aileronStartTime, aileronPulses = 0;

// Elevator variables
volatile long elevatorCurrentTIme, elevatorStartTime, elevatorPulses = 0;

// Rudder variables
volatile long rudderCurrentTIme, rudderStartTime, rudderPulses = 0;
unsigned long yawLastMs = 0;

// Mode variables
volatile long modeCurrentTime, modeStartTime, modePulses = 0;
// -------------------------

Xpilot::Xpilot(void)
{
}

void Xpilot::setup(void)
{
#if !defined(AILPIN_INT) || !defined(ELEVPIN_INT) || !defined(RUDDPIN_INT) || !defined(MODEPIN_INT)
#error "Ensure interrupt pins for aileron, elevator, rudder and mode are defined"
#endif

    Wire.begin();
    Wire.setClock(I2C_CLOCK_1MHZ); // Overclocking I2C to 1Mhz

#if DEBUG
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
#if DEBUG
            Serial.println("No MPU found! Check connection");
#endif
            delay(1000);
        }
    }

#if DEBUG
    Serial.println("Accel Gyro calibration will start in 3sec.");
    Serial.println("Please leave the device still on the flat plane.");
    imu.verbose(true);
    delay(3000);
    // Calibrate IMU accelerometer and gyro
    imu.calibrateAccelGyro();
    Serial.println("Calibration complete.");
    print_calibration();
#else
    imu.verbose(false);
    imu.calibrateAccelGyro();
#endif

    // All input pins use pin change interrupts
    // Aileron setup
    aileronServo.attach(AILPIN_OUTPUT);
    pinMode(AILPIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(AILPIN_INT, CHANGE);

    // Elevator setup
    elevatorServo.attach(ELEVPIN_OUTPUT);
    pinMode(ELEVPIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(ELEVPIN_INT, CHANGE);

    // Rudder setup
    rudderServo.attach(RUDDPIN_OUTPUT);
    pinMode(RUDDPIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(RUDDPIN_INT, CHANGE);

    // Mode setup
    pinMode(MODEPIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(MODEPIN_INT, CHANGE);

    warmupIMU(IMU_WARMUP_LOOP);
}

/*
    Main xpilot execution loop
    Read input, read imu data, process output to servos
*/
void Xpilot::loop(void)
{
    nowMs = millis();
    processInput();

    // Only run IMU processing in auto modes i.e FBW, STABILIZE
    // if (currentMode != FLIGHT_MODE::PASSTHROUGH)
    //     processIMU();
    processIMU();

    // Process output to servos at 50Hz intervals
    if (nowMs - outputLastMs >= 20)
    {
        processOutput();
        outputLastMs = nowMs;
    }

#if IO_DEBUG
    if (nowMs - debugLastMs >= 1000)
    {
        print_imu();
        print_input();
        print_output();
        debugLastMs = nowMs;
    }
#endif

#if LOOP_DEBUG
    loopLastMs = millis();
    Serial.print("Loop time: ");
    long loopMillis = loopLastMs - nowMs;
    Serial.print(loopMillis);
    Serial.println("ms");
    Serial.print("Loop rate: ");
    loopMillis = loopMillis <= 0 ? 1 : loopMillis; // To prevent division by 0
    Serial.print(1000 / loopMillis);
    Serial.println("Hz");
    Serial.println();
#endif
}

void Xpilot::processInput(void)
{
    uint8_t oldSREG = SREG;

    // Disable interrupts as pulses are being read to avoid race conditionS
    cli();
    if (modePulses >= SERVO_MIN_PWM && modePulses <= SERVO_MAX_PWM)
        mode.update(modePulses);

    if (aileronPulses >= SERVO_MIN_PWM && aileronPulses <= SERVO_MAX_PWM)
        aileronPulseWidth = aileronPulses;

    if (elevatorPulses >= SERVO_MIN_PWM && elevatorPulses <= SERVO_MAX_PWM)
        elevatorPulseWidth = elevatorPulses;

    if (rudderPulses >= SERVO_MIN_PWM && rudderPulses <= SERVO_MAX_PWM)
        rudderPulseWidth = rudderPulses;

    SREG = oldSREG;
}

void Xpilot::processIMU(void)
{
    if (imu.update())
    {
        ahrs_roll = imu.getRoll();
        ahrs_pitch = imu.getPitch();
        ahrs_yaw = imu.getYaw();
    }
}

void Xpilot::processOutput(void)
{
    aileron_out = constrain(aileronPulseWidth, SERVO_MIN_PWM, SERVO_MAX_PWM);
    elevator_out = constrain(elevatorPulseWidth, SERVO_MIN_PWM, SERVO_MAX_PWM);
    rudder_out = constrain(rudderPulseWidth, SERVO_MIN_PWM, SERVO_MAX_PWM);

    mode.process();

    aileron_out = constrain(aileron_out, SERVO_MIN_PWM, SERVO_MAX_PWM);
    elevator_out = constrain(elevator_out, SERVO_MIN_PWM, SERVO_MAX_PWM);
    rudder_out = constrain(rudder_out, SERVO_MIN_PWM, SERVO_MAX_PWM);

    aileronServo.writeMicroseconds(aileron_out);
    elevatorServo.writeMicroseconds(elevator_out);
    rudderServo.writeMicroseconds(rudder_out);
}

void Xpilot::warmupIMU(uint16_t warmUpTimer)
{
    // Warm up the IMU
    for (uint16_t i = 0; i < warmUpTimer; i++)
    {
        processIMU();
    }
}

// ISR
void PinChangeInterruptEvent(AILPIN_INT)(void)
{
    aileronCurrentTime = micros();
    if (aileronCurrentTime > aileronStartTime)
    {
        aileronPulses = aileronCurrentTime - aileronStartTime;
        aileronStartTime = aileronCurrentTime;
    }
}

void PinChangeInterruptEvent(ELEVPIN_INT)(void)
{
    elevatorCurrentTIme = micros();
    if (elevatorCurrentTIme > elevatorStartTime)
    {
        elevatorPulses = elevatorCurrentTIme - elevatorStartTime;
        elevatorStartTime = elevatorCurrentTIme;
    }
}

void PinChangeInterruptEvent(RUDDPIN_INT)(void)
{
    rudderCurrentTIme = micros();
    if (rudderCurrentTIme > rudderStartTime)
    {
        rudderPulses = rudderCurrentTIme - rudderStartTime;
        rudderStartTime = rudderCurrentTIme;
    }
}

void PinChangeInterruptEvent(MODEPIN_INT)(void)
{
    modeCurrentTime = micros();
    if (modeCurrentTime > modeStartTime)
    {
        modePulses = modeCurrentTime - modeStartTime;
        modeStartTime = modeCurrentTime;
    }
}
// ----------------------------

// IO Debug functions
#if IO_DEBUG
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

void Xpilot::print_input(void)
{
    Serial.print("Elevator Pulse: ");
    Serial.println(elevatorPulseWidth);
    Serial.print("Aileron Pulse: ");
    Serial.println(aileronPulseWidth);
    Serial.print("Rudder Pulse: ");
    Serial.println(rudderPulseWidth);
    Serial.print("Flight Mode: ");
    Serial.println((int)currentMode);
    Serial.println();
}

void Xpilot::print_output(void)
{
    Serial.print("Elevator Servo: ");
    Serial.println(elevator_out);
    Serial.print("Aileron Servo: ");
    Serial.println(aileron_out);
    Serial.print("Rudder Servo: ");
    Serial.println(rudder_out);
    Serial.println();
}
#endif

#if DEBUG
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