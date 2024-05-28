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
#include "Mode.h"
#include <PinChangeInterrupt.h>
#include <Utils.cpp>

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral.
// With the MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
#define imuKp 25.0
#define imuKi 0.0

// Globals for AHRS loop timing
unsigned long nowUs = 0, lastUs = 0; // micros() timers
double dt = 0;                       // loop time in seconds
// -------------------------

// Timer variables
unsigned long nowMs, inputLastMs = 0;

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

// Mode variables
volatile long modeCurrentTime, modeStartTime, modePulses = 0;
// -------------------------

Xpilot::Xpilot(void)
{
    aileronInputPin = AILPIN_INPUT;
    elevatorInputPin = ELEVPIN_INPUT;
    rudderInputPin = RUDDPIN_INPUT;
    modeInputPin = MODEPIN_INPUT;

    aileronOutputPin = AILPIN_OUTPUT;
    elevatorOutputPin = ELEVPIN_OUTPUT;
    rudderOutputPin = RUDDPIN_OUTPUT;
}

void Xpilot::setup(void)
{
#if !defined(AILPIN_INT) || !defined(ELEVPIN_INT) || !defined(RUDDPIN_INT) || !defined(MODEPIN_INT)
#error "Ensure interrupt pins for aileron, elevator, rudder and mode are defined"
#endif

    Wire.begin();
#if DEBUG
    Serial.begin(9600);
    while (!Serial)
    {
        Serial.println("Serial port communication cannot be established.");
        delay(1000);
    }
#endif

    // initialize device
    imu.initialize();
    // test mpu for connection after initialization
    while (!imu.testConnection())
    {
#if IO_DEBUG
        Serial.println("No MPU found! Check connection");
#endif
        delay(1000);
    }

    // All input pins use pin change interrupts
    // Aileron setup
    aileronServo.attach(aileronOutputPin);
    pinMode(aileronInputPin, INPUT_PULLUP);
    attachPinChangeInterrupt(AILPIN_INT, CHANGE);

    // Elevator setup
    elevatorServo.attach(elevatorOutputPin);
    pinMode(elevatorInputPin, INPUT_PULLUP);
    attachPinChangeInterrupt(ELEVPIN_INT, CHANGE);

    // Rudder setup
    rudderServo.attach(rudderOutputPin);
    pinMode(rudderInputPin, INPUT_PULLUP);
    attachPinChangeInterrupt(RUDDPIN_INT, CHANGE);

    // Mode setup
    pinMode(modeInputPin, INPUT_PULLUP);
    attachPinChangeInterrupt(MODEPIN_INT, CHANGE);
}

// Main execution xpilot execution loop
void Xpilot::loop(void)
{
    // Read input, read imu data, process output to servos
    // Process input at 50Hz, everything else runs at loop speed
    nowMs = millis();
    if (nowMs - inputLastMs >= 20)
    {
        processInput();
        inputLastMs = nowMs;
    }

    // Only run IMU processing in auto modes i.e FBW, STABILIZE
    if (currentMode != FLIGHT_MODE::PASSTHROUGH)
        processIMU();

    // Output to servos
    processOutput();

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
    Serial.print("Loop rate: ");
    Serial.print((double)(1000 / (loopLastMs - nowMs)));
    Serial.println("Hz");
#endif
}

void Xpilot::processInput(void)
{
    uint8_t oldSREG = SREG;
    cli();
    if (modePulses >= RECEIVER_LOW && modePulses <= RECEIVER_HIGH)
        modePulseWidth = modePulses;

    if (aileronPulses >= RECEIVER_LOW && aileronPulses <= RECEIVER_HIGH)
        aileronPulseWidth = aileronPulses;

    if (elevatorPulses >= RECEIVER_LOW && elevatorPulses <= RECEIVER_HIGH)
        elevatorPulseWidth = elevatorPulses;

    if (rudderPulses >= RECEIVER_LOW && rudderPulses <= RECEIVER_HIGH)
        rudderPulseWidth = rudderPulses;

    mode.update();
    SREG = oldSREG;
}

void Xpilot::processIMU(void)
{
    nowUs = micros();
    get_imu_scaled();
    dt = (nowUs - lastUs) * 1e-6; // seconds since last update
    lastUs = nowUs;

    //  Standard orientation: X North, Y West, Z Up
    //  Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw,
    // pitch, and then roll.
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.
    mahoganyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[1], Mxyz[0], -Mxyz[2], dt);

    // Strictly valid only for approximately level movement
    // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
    // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock
    ahrs_roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    ahrs_pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    ahrs_yaw = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
    //  to degrees
    ahrs_yaw *= 180.0 / PI;
    ahrs_pitch *= 180.0 / PI;
    ahrs_roll *= 180.0 / PI;

    // http://www.ngdc.noaa.gov/geomag-web/#declination
    // conventional nav, yaw increases CW from North, corrected for local magnetic declination
    ahrs_yaw = -ahrs_yaw + 11.5;

    if (ahrs_yaw < 0)
        ahrs_yaw += 360.0;
    if (ahrs_yaw > 360.0)
        ahrs_yaw -= 360.0;
}

void Xpilot::processOutput(void)
{
    aileron_out = constrain(aileronPulseWidth, SERVO_MIN_PWM, SERVO_MAX_PWM);
    elevator_out = constrain(elevatorPulseWidth, SERVO_MIN_PWM, SERVO_MAX_PWM);
    rudder_out = constrain(rudderPulseWidth, SERVO_MIN_PWM, SERVO_MAX_PWM);

    mode.process();

    aileron_out = constrain(aileron_out, SERVO_MIN_PWM, SERVO_MAX_PWM);
    elevator_out = constrain(elevator_out, SERVO_MIN_PWM, SERVO_MAX_PWM);
    rudder_out = constrain(rudderPulseWidth, SERVO_MIN_PWM, SERVO_MAX_PWM);

    aileronServo.writeMicroseconds(aileron_out);
    elevatorServo.writeMicroseconds(elevator_out);
    rudderServo.writeMicroseconds(rudder_out);
}

void Xpilot::get_imu_scaled(void)
{
    double temp[3];
    int i;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // 250 LSB(d/s) default to radians/s
    Gxyz[0] = ((double)gx - G_off[0]) * gScale;
    Gxyz[1] = ((double)gy - G_off[1]) * gScale;
    Gxyz[2] = ((double)gz - G_off[2]) * gScale;

    Axyz[0] = (double)ax;
    Axyz[1] = (double)ay;
    Axyz[2] = (double)az;

    // apply offsets (bias) and scale factors from Magneto
    for (i = 0; i < 3; i++)
        temp[i] = (Axyz[i] - A_B[i]);

    Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
    Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
    Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
    vectorNormalize(Axyz);

    Mxyz[0] = (double)mx;
    Mxyz[1] = (double)my;
    Mxyz[2] = (double)mz;

    for (i = 0; i < 3; i++)
        temp[i] = (Mxyz[i] - M_B[i]);
    Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
    Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
    Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
    vectorNormalize(Mxyz);
}

// Mahogany scheme uses proportional and integral filtering on
// the error between estimated reference vectors and measured ones.
void Xpilot::mahoganyQuaternionUpdate(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz, double deltat)
{
    // Vector to hold integral error for Mahogany method
    static double eInt[3] = {0.0, 0.0, 0.0};
    // short name local variable for readability
    double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    double norm;
    double hx, hy, bx, bz;
    double vx, vy, vz, wx, wy, wz;
    double ex, ey, ez;

    // Auxiliary variables to avoid repeated arithmetic
    double q1q1 = q1 * q1;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q1q4 = q1 * q4;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q2q4 = q2 * q4;
    double q3q3 = q3 * q3;
    double q3q4 = q3 * q4;
    double q4q4 = q4 * q4;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

    // Error is cross product between estimated direction and measured direction of the reference vectors
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (imuKi > 0.0f)
    {
        eInt[0] += ex; // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
        // Apply I feedback
        gx += imuKi * eInt[0];
        gy += imuKi * eInt[1];
        gz += imuKi * eInt[2];
    }

    // Apply P feedback
    gx = gx + imuKp * ex;
    gy = gy + imuKp * ey;
    gz = gz + imuKp * ez;

    // Integrate rate of change of quaternion
    // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
    gx = gx * (0.5 * deltat); // pre-multiply common factors
    gy = gy * (0.5 * deltat);
    gz = gz * (0.5 * deltat);
    double qa = q1;
    double qb = q2;
    double qc = q3;
    q1 += (-qb * gx - qc * gy - q4 * gz);
    q2 += (qa * gx + qc * gz - q4 * gy);
    q3 += (qa * gy - qb * gz + q4 * gx);
    q4 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
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
// ---------------------------

Xpilot xpilot;