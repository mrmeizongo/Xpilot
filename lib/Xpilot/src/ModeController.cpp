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
#include <Arduino.h>
#include <PIDF.h>
#include <PlaneConfig.h>
#include "ModeController.h"
#include "Radio.h"
#include "IMU.h"
#include "Actuators.h"

// Helper functions
static void planeMixer(int16_t, int16_t, int16_t);
static void rudderMixer(void);
static void yawController(void);
// -----------------------------------------------------------------------------------------------------------------

// Servo channel out
static int16_t SRVout[Channel::NUM_CHANNELS]{0, 0, 0, 0};
// -----------------------------------------------------------------------------------------------------------------

// PID controllers
static PIDF rollPIDF;
static PIDF pitchPIDF;
static PIDF yawPIDF;
// -----------------------------------------------------------------------------------------------------------------

// Radio input variables
static int16_t rollInput, pitchInput, yawInput = 0;
static FlightMode currentFlightMode;
static FlightMode previousFlightMode;
// -----------------------------------------------------------------------------------------------------------------

ModeController::ModeController(void)
{
}

void ModeController::init(void)
{
    rollPIDF = PIDF(ROLL_KP, ROLL_KI, ROLL_KD, ROLL_KF, ROLL_I_WINDUP_MAX);
    pitchPIDF = PIDF(PITCH_KP, PITCH_KI, PITCH_KD, PITCH_KF, PITCH_I_WINDUP_MAX);
    yawPIDF = PIDF(YAW_KP, YAW_KI, YAW_KD, YAW_KF, YAW_I_WINDUP_MAX);
    currentFlightMode = FlightMode::INITIALIZING;
    previousFlightMode = currentFlightMode;
}

void ModeController::processMode(void)
{
    rollInput = radio.getRxRoll();
    pitchInput = radio.getRxPitch();
    yawInput = radio.getRxYaw();
    currentFlightMode = radio.getRxCurrentMode();
    if (currentFlightMode != previousFlightMode)
    {
        rollPIDF.Reset();
        pitchPIDF.Reset();
        yawPIDF.Reset();
        previousFlightMode = currentFlightMode;
    }

    switch (currentFlightMode)
    {
    case PASSTHROUGH:
        passthroughMode();
        SRVout[AILERON1] = map(SRVout[AILERON1], -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[AILERON2] = map(SRVout[AILERON2], -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[ELEVATOR] = map(SRVout[ELEVATOR], -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[RUDDER] = map(SRVout[RUDDER], -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        break;
    default:
    case RATE:
        rateMode();
        SRVout[AILERON1] = map(SRVout[AILERON1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[AILERON2] = map(SRVout[AILERON2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[ELEVATOR] = map(SRVout[ELEVATOR], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[RUDDER] = map(SRVout[RUDDER], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        break;
    case STABILIZE:
        stabilizeMode();
        SRVout[AILERON1] = map(SRVout[AILERON1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[AILERON2] = map(SRVout[AILERON2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[ELEVATOR] = map(SRVout[ELEVATOR], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[RUDDER] = map(SRVout[RUDDER], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        break;
    case INITIALIZING:
        SRVout[AILERON1] = SERVO_MID_PWM;
        SRVout[AILERON2] = SERVO_MID_PWM;
        SRVout[ELEVATOR] = SERVO_MID_PWM;
        SRVout[RUDDER] = SERVO_MID_PWM;
        break;
    }

    actuators.setServoOut(SRVout);
}

// Manual mode gives full control of the rc plane flight surfaces
// No stabilization or rate control. For experienced rc pilots, use with caution!
void ModeController::passthroughMode(void)
{
#if defined(RUDDER_MIX_IN_PASS)
    rudderMixer();
#endif

    planeMixer(rollInput, pitchInput, yawInput);
    SRVout[AILERON1] = constrain(SRVout[AILERON1], -PASSTHROUGH_RES, PASSTHROUGH_RES);
    SRVout[AILERON2] = constrain(SRVout[AILERON2], -PASSTHROUGH_RES, PASSTHROUGH_RES);
    SRVout[ELEVATOR] = constrain(SRVout[ELEVATOR], -PASSTHROUGH_RES, PASSTHROUGH_RES);
    SRVout[RUDDER] = constrain(SRVout[RUDDER], -PASSTHROUGH_RES, PASSTHROUGH_RES);
}

// Rate mode uses gyroscope values to maintain a desired rate of change
// Flight surfaces counteract sudden changes in plane attitude due to external forces
void ModeController::rateMode(void)
{
#if defined(RUDDER_MIX_IN_RATE)
    rudderMixer();
#endif
    yawController();

    int16_t roll = rollPIDF.Compute(rollInput, imu.getGyroX());
    int16_t pitch = pitchPIDF.Compute(pitchInput, imu.getGyroY());
    int16_t yaw = yawPIDF.Compute(yawInput, imu.getGyroZ());

    planeMixer(roll, pitch, yaw);
    SRVout[AILERON1] = constrain(SRVout[AILERON1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[AILERON2] = constrain(SRVout[AILERON2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[ELEVATOR] = constrain(SRVout[ELEVATOR], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[RUDDER] = constrain(SRVout[RUDDER], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
}

// Roll and pitch follow stick input up to set limits
// Flight surfaces counteract sudden changes in attitude on stick release
// Roll and pitch leveling on stick release
// CAUTION: At extreme pitch angles +/-90, gimbal lock may occur!
void ModeController::stabilizeMode(void)
{
#if defined(RUDDER_MIX_IN_STABILIZE)
    rudderMixer();
#endif
    yawController();

    float rollDemand = rollInput - imu.getRoll();
    float pitchDemand = pitchInput - imu.getPitch();
    rollDemand = map(rollDemand, -MAX_ROLL_ANGLE_DEGS, MAX_ROLL_ANGLE_DEGS, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
    pitchDemand = map(pitchDemand, -MAX_PITCH_ANGLE_DEGS, MAX_PITCH_ANGLE_DEGS, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);

    int16_t roll = rollPIDF.Compute(rollDemand, imu.getGyroX());
    int16_t pitch = pitchPIDF.Compute(pitchDemand, imu.getGyroY());
    int16_t yaw = yawPIDF.Compute(yawInput, imu.getGyroZ());

    planeMixer(roll, pitch, yaw);
    SRVout[AILERON1] = constrain(SRVout[AILERON1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[AILERON2] = constrain(SRVout[AILERON2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[ELEVATOR] = constrain(SRVout[ELEVATOR], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[RUDDER] = constrain(SRVout[RUDDER], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
}

/*
 * Mixer for airplane type
 * Only tested with a full plane i.e. ailerons, elevator and rudder
 * Proceed with caution. Perform thorough pre-flight checks and reverse servo direction as needed.
 *
 * Default
 * For V-Tail - If servos are mounted on top, with face of servo facing the same direction
 *
 *                 Rudder Left
 *
 *                                ^
 *                 _       _      |
 *         <======|O|     |O|=====>
 *         |  Face| | Face| |
 *         v      |_|     |_|
 *
 *   Push surface out   Pull surface in
 *
 * For Flying Wings, reverse install elevon servos same as you would ailerons for planes
 */
static void planeMixer(int16_t roll, int16_t pitch, int16_t yaw)
{
#if defined(FULL_PLANE)
    SRVout[AILERON1] = roll;
    SRVout[AILERON2] = roll;
    SRVout[ELEVATOR] = pitch;
    SRVout[RUDDER] = yaw;
#elif defined(FULL_PLANE_V_TAIL)
    SRVout[AILERON1] = roll;
    SRVout[AILERON2] = roll;
    SRVout[ELEVATOR] = yaw + pitch;
    SRVout[RUDDER] = yaw - pitch;
#elif defined(RUDDER_ELEVATOR_ONLY_V_TAIL)
    SRVout[AILERON1] = 0;
    SRVout[AILERON2] = 0;
    SRVout[ELEVATOR] = yaw + pitch;
    SRVout[RUDDER] = yaw - pitch;
#elif defined(FLYING_WING_W_RUDDER)
    SRVout[AILERON1] = roll - pitch;
    SRVout[AILERON2] = roll + pitch;
    SRVout[ELEVATOR] = 0;
    SRVout[RUDDER] = yaw;
#elif defined(FLYING_WING_NO_RUDDER)
    SRVout[AILERON1] = roll - pitch;
    SRVout[AILERON2] = roll + pitch;
    SRVout[ELEVATOR] = 0;
    SRVout[RUDDER] = 0;
#elif defined(RUDDER_ELEVATOR_ONLY_PLANE)
    SRVout[AILERON1] = 0;
    SRVout[AILERON2] = 0;
    SRVout[ELEVATOR] = pitch;
    SRVout[RUDDER] = yaw;
#else
#error No airplane type selected!
#endif
}

static void rudderMixer(void)
{
#if defined(FULL_PLANE) || defined(FULL_PLANE_V_TAIL) || defined(FLYING_WING_W_RUDDER)
#if defined(REVERSE_RUDDER_MIX)
    yawInput = yawInput - (rollInput * RUDDER_MIXING);
#else
    yawInput = yawInput + (rollInput * RUDDER_MIXING);
#endif
#endif
}

static void yawController(void)
{
#if defined(USE_HEADING_HOLD)
    // If yaw is centered and we're using heading hold, add YAW_KI to PID(if any is defined)
    // Remove it otherwise
    if (yawInput == 0)
    {
        yawPIDF.setKi(YAW_KI);
    }
    else
    {
        yawPIDF.setKi(0.f);
        yawPIDF.Reset();
    }
#else
    yawPIDF.setKi(0.f);
    yawPIDF.Reset();
#endif
}

ModeController modeController;