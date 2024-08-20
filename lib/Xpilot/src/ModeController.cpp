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

#include "ModeController.h"
#include "Xpilot.h"
#include "Radio.h"
#include "IMU.h"
#include "Actuators.h"
#include "config.h"

// Helper functions
#define RESETPIDCONTROLLERS() \
    rollPID->ResetPID();      \
    pitchPID->ResetPID();     \
    yawPID->ResetPID();
// -----------------------------------------------------------------------------------------------------------------
// Servo channel out
static int16_t SRVout[MAX_SERVO_CHANNELS]{0, 0, 0, 0};
// -----------------------------------------------------------------------------------------------------------------

ModeController::ModeController(void)
{
    rollPID = new PID(ROLL_KP, ROLL_KI, ROLL_KD);
    pitchPID = new PID(PITCH_KP, PITCH_KI, PITCH_KD);
    yawPID = new PID(YAW_KP, YAW_KI, YAW_KD);
}

// Update flight mode from mode switch position
// Do nothing if we're already in the selected mode
// Otherwise reset PID values and set current mode
void ModeController::updateMode(void)
{
    if (radio.rx.previousMode == radio.rx.currentMode)
        return;

    RESETPIDCONTROLLERS();
    radio.rx.previousMode = radio.rx.currentMode;
}

void ModeController::processMode(void)
{
    switch (radio.rx.currentMode)
    {
    case FlightMode::passthrough:
        passthroughMode();
#if defined(RUDDER_MIX_IN_PASS)
        rudderMixer();
#endif
        SRVout[Actuators::Control::AILERON1] = map(SRVout[Actuators::Control::AILERON1], -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[Actuators::Control::AILERON2] = map(SRVout[Actuators::Control::AILERON2], -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[Actuators::Control::ELEVATOR] = map(SRVout[Actuators::Control::ELEVATOR], -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[Actuators::Control::RUDDER] = map(SRVout[Actuators::Control::RUDDER], -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        actuators.setServoOut(SRVout);
        break;
    default:
    case FlightMode::rate:
        rateMode();
#if defined(RUDDER_MIX_IN_RATE)
        rudderMixer();
#endif
        SRVout[Actuators::Control::AILERON1] = map(SRVout[Actuators::Control::AILERON1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[Actuators::Control::AILERON2] = map(SRVout[Actuators::Control::AILERON2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[Actuators::Control::ELEVATOR] = map(SRVout[Actuators::Control::ELEVATOR], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[Actuators::Control::RUDDER] = map(SRVout[Actuators::Control::RUDDER], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        actuators.setServoOut(SRVout);
        break;
    case FlightMode::stabilize:
        stabilizeMode();
#if defined(RUDDER_MIX_IN_STABILIZE)
        rudderMixer();
#endif
        SRVout[Actuators::Control::AILERON1] = map(SRVout[Actuators::Control::AILERON1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[Actuators::Control::AILERON2] = map(SRVout[Actuators::Control::AILERON2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[Actuators::Control::ELEVATOR] = map(SRVout[Actuators::Control::ELEVATOR], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        SRVout[Actuators::Control::RUDDER] = map(SRVout[Actuators::Control::RUDDER], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        actuators.setServoOut(SRVout);
        break;
    }
}

// Manual mode gives full control of the rc plane flight surfaces
// No stabilization and rate control
void ModeController::passthroughMode(void)
{
    planeMixer(radio.rx.roll, radio.rx.pitch, radio.rx.yaw);
    SRVout[Actuators::Control::AILERON1] = constrain(SRVout[Actuators::Control::AILERON1], -PASSTHROUGH_RES, PASSTHROUGH_RES);
    SRVout[Actuators::Control::AILERON2] = constrain(SRVout[Actuators::Control::AILERON2], -PASSTHROUGH_RES, PASSTHROUGH_RES);
    SRVout[Actuators::Control::ELEVATOR] = constrain(SRVout[Actuators::Control::ELEVATOR], -PASSTHROUGH_RES, PASSTHROUGH_RES);
    SRVout[Actuators::Control::RUDDER] = constrain(SRVout[Actuators::Control::RUDDER], -PASSTHROUGH_RES, PASSTHROUGH_RES);
}

// Rate mode uses gyroscope values to maintain a desired rate of change
// Flight surfaces move to prevent sudden changes in direction
void ModeController::rateMode(void)
{
    float rollDemand = radio.rx.roll - imu.gyroX;
    float pitchDemand = radio.rx.pitch - imu.gyroY;
    float yawDemand = radio.rx.yaw - imu.gyroZ;

    int16_t roll = rollPID->Compute(rollDemand);
    int16_t pitch = pitchPID->Compute(pitchDemand);
    int16_t yaw = yawPID->Compute(yawDemand);

    planeMixer(roll, pitch, yaw);
    SRVout[Actuators::Control::AILERON1] = constrain(SRVout[Actuators::Control::AILERON1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[Actuators::Control::AILERON2] = constrain(SRVout[Actuators::Control::AILERON2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[Actuators::Control::ELEVATOR] = constrain(SRVout[Actuators::Control::ELEVATOR], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[Actuators::Control::RUDDER] = constrain(SRVout[Actuators::Control::RUDDER], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
}

// Roll and pitch follow stick input up to set limits
// Roll and pitch leveling on stick release
void ModeController::stabilizeMode(void)
{
    float rollDemand = radio.rx.roll - imu.ahrs_roll;
    float pitchDemand = radio.rx.pitch - imu.ahrs_pitch;
    float yawDemand = radio.rx.yaw - imu.gyroZ;

    rollDemand = map(rollDemand, -MAX_ROLL_ANGLE_DEGS, MAX_ROLL_ANGLE_DEGS, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
    pitchDemand = map(pitchDemand, -MAX_PITCH_ANGLE_DEGS, MAX_PITCH_ANGLE_DEGS, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);

    rollDemand = constrain(rollDemand, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
    pitchDemand = constrain(pitchDemand, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);

    rollDemand = rollDemand - imu.gyroX;
    pitchDemand = pitchDemand - imu.gyroY;

    int16_t roll = rollPID->Compute(rollDemand);
    int16_t pitch = pitchPID->Compute(pitchDemand);
    int16_t yaw = yawPID->Compute(yawDemand);

    planeMixer(roll, pitch, yaw);
    SRVout[Actuators::Control::AILERON1] = constrain(SRVout[Actuators::Control::AILERON1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[Actuators::Control::AILERON2] = constrain(SRVout[Actuators::Control::AILERON2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[Actuators::Control::ELEVATOR] = constrain(SRVout[Actuators::Control::ELEVATOR], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    SRVout[Actuators::Control::RUDDER] = constrain(SRVout[Actuators::Control::RUDDER], -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
}

/*
 * Mixer for airplane type
 * Only tested with a full plane i.e. ailerons, elevator and rudder
 * Proceed with caution. Depends heavily on the direction of servo travel.
 * Perform thorough pre flight checks and reverse as needed.
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
void ModeController::planeMixer(int16_t roll, int16_t pitch, int16_t yaw)
{
#if defined(FULL_PLANE)
    SRVout[Actuators::Control::AILERON1] = roll;
    SRVout[Actuators::Control::AILERON2] = roll;
    SRVout[Actuators::Control::ELEVATOR] = pitch;
    SRVout[Actuators::Control::RUDDER] = yaw;
#elif defined(FULL_PLANE_V_TAIL)
    SRVout[Actuators::Control::AILERON1] = roll;
    SRVout[Actuators::Control::AILERON2] = roll;
    SRVout[Actuators::Control::ELEVATOR] = yaw + pitch;
    SRVout[Actuators::Control::RUDDER] = yaw - pitch;
#elif defined(RUDDER_ELEVATOR_ONLY_V_TAIL)
    SRVout[Actuators::Control::AILERON1] = 0;
    SRVout[Actuators::Control::AILERON2] = 0;
    SRVout[Actuators::Control::ELEVATOR] = yaw + pitch;
    SRVout[Actuators::Control::RUDDER] = yaw - pitch;
#elif defined(FLYING_WING_W_RUDDER)
    SRVout[Actuators::Control::AILERON1] = roll - pitch;
    SRVout[Actuators::Control::AILERON2] = roll + pitch;
    SRVout[Actuators::Control::ELEVATOR] = 0;
    SRVout[Actuators::Control::RUDDER] = yaw;
#elif defined(FLYING_WING_NO_RUDDER)
    SRVout[Actuators::Control::AILERON1] = roll - pitch;
    SRVout[Actuators::Control::AILERON2] = roll + pitch;
    SRVout[Actuators::Control::ELEVATOR] = 0;
    SRVout[Actuators::Control::RUDDER] = 0;
#elif defined(RUDDER_ELEVATOR_ONLY_PLANE)
    SRVout[Actuators::Control::AILERON1] = 0;
    SRVout[Actuators::Control::AILERON2] = 0;
    SRVout[Actuators::Control::ELEVATOR] = pitch;
    SRVout[Actuators::Control::RUDDER] = yaw;
#else
#error No airplane type selected!
#endif
}

void ModeController::rudderMixer(void)
{
#if defined(FULL_PLANE) || defined(FULL_PLANE_V_TAIL) || defined(FLYING_WING_W_RUDDER)
#if defined(REVERSE_RUDDER_MIX)
    SRVout[Actuators::Control::RUDDER] = SRVout[Actuators::Control::RUDDER] - (SRVout[Actuators::Control::AILERON1] * RUDDER_MIXING);
#else
    SRVout[Actuators::Control::RUDDER] = SRVout[Actuators::Control::RUDDER] + (SRVout[Actuators::Control::AILERON1] * RUDDER_MIXING);
#endif
#endif
}

ModeController modeController;