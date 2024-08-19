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
#include "config.h"

// Helper functions
#define RESETPIDCONTROLLERS() \
    rollPID->ResetPID();      \
    pitchPID->ResetPID();     \
    yawPID->ResetPID();
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
    if (radio.rx.currentMode == FlightMode::passthrough)
    {
        if (radio.rx.previousMode == FlightMode::passthrough)
            return;

        radio.rx.previousMode = radio.rx.currentMode;
    }
    else if (radio.rx.currentMode == FlightMode::rate)
    {
        if (radio.rx.previousMode == FlightMode::rate)
            return;

        RESETPIDCONTROLLERS();
        radio.rx.previousMode = radio.rx.currentMode;
    }
    else if (radio.rx.currentMode == FlightMode::stabilize)
    {
        if (radio.rx.previousMode == FlightMode::stabilize)
            return;

        RESETPIDCONTROLLERS();
        radio.rx.previousMode = radio.rx.currentMode;
    }
}

void ModeController::process(void)
{
    switch (radio.rx.currentMode)
    {
    case FlightMode::passthrough:
        passthroughMode();
#if defined(RUDDER_MIX_IN_PASS)
        rudderMixer();
#endif
        xpilot.aileron1_out = map(xpilot.aileron1_out, -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.aileron2_out = map(xpilot.aileron2_out, -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.elevator_out = map(xpilot.elevator_out, -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.rudder_out = map(xpilot.rudder_out, -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        break;
    default:
    case FlightMode::rate:
        rateMode();
#if defined(RUDDER_MIX_IN_RATE)
        rudderMixer();
#endif
        xpilot.aileron1_out = map(xpilot.aileron1_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.aileron2_out = map(xpilot.aileron2_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.elevator_out = map(xpilot.elevator_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.rudder_out = map(xpilot.rudder_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        break;
    case FlightMode::stabilize:
        stabilizeMode();
#if defined(RUDDER_MIX_IN_STABILIZE)
        rudderMixer();
#endif
        xpilot.aileron1_out = map(xpilot.aileron1_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.aileron2_out = map(xpilot.aileron2_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.elevator_out = map(xpilot.elevator_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.rudder_out = map(xpilot.rudder_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        break;
    }
}

// Manual mode gives full control of the rc plane flight surfaces
// No stabilization and rate control
void ModeController::passthroughMode(void)
{
    planeMixer(radio.rx.roll, radio.rx.pitch, radio.rx.yaw);
    xpilot.aileron1_out = constrain(xpilot.aileron1_out, -PASSTHROUGH_RES, PASSTHROUGH_RES);
    xpilot.aileron2_out = constrain(xpilot.aileron2_out, -PASSTHROUGH_RES, PASSTHROUGH_RES);
    xpilot.elevator_out = constrain(xpilot.elevator_out, -PASSTHROUGH_RES, PASSTHROUGH_RES);
    xpilot.rudder_out = constrain(xpilot.rudder_out, -PASSTHROUGH_RES, PASSTHROUGH_RES);
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
    xpilot.aileron1_out = constrain(xpilot.aileron1_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    xpilot.aileron2_out = constrain(xpilot.aileron2_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    xpilot.elevator_out = constrain(xpilot.elevator_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    xpilot.rudder_out = constrain(xpilot.rudder_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
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
    xpilot.aileron1_out = constrain(xpilot.aileron1_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    xpilot.aileron2_out = constrain(xpilot.aileron2_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    xpilot.elevator_out = constrain(xpilot.elevator_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
    xpilot.rudder_out = constrain(xpilot.rudder_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT);
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
    xpilot.aileron1_out = roll;
    xpilot.aileron2_out = roll;
    xpilot.elevator_out = pitch;
    xpilot.rudder_out = yaw;
#elif defined(FULL_PLANE_V_TAIL)
    xpilot.aileron1_out = roll;
    xpilot.aileron2_out = roll;
    xpilot.elevator_out = yaw + pitch;
    xpilot.rudder_out = yaw - pitch;
#elif defined(RUDDER_ELEVATOR_ONLY_V_TAIL)
    xpilot.aileron1_out = 0;
    xpilot.aileron2_out = 0;
    xpilot.elevator_out = yaw + pitch;
    xpilot.rudder_out = yaw - pitch;
#elif defined(FLYING_WING_W_RUDDER)
    xpilot.aileron1_out = roll - pitch;
    xpilot.aileron2_out = roll + pitch;
    xpilot.elevator_out = 0;
    xpilot.rudder_out = yaw;
#elif defined(FLYING_WING_NO_RUDDER)
    xpilot.aileron1_out = roll - pitch;
    xpilot.aileron2_out = roll + pitch;
    xpilot.elevator_out = 0;
    xpilot.rudder_out = 0;
#elif defined(RUDDER_ELEVATOR_ONLY_PLANE)
    xpilot.aileron1_out = 0;
    xpilot.aileron2_out = 0;
    xpilot.elevator_out = pitch;
    xpilot.rudder_out = yaw;
#else
#error No airplane type selected!
#endif
}

void ModeController::rudderMixer(void)
{
#if defined(FULL_PLANE) || defined(FULL_PLANE_V_TAIL) || defined(FLYING_WING_W_RUDDER)
#if defined(REVERSE_RUDDER_MIX)
    xpilot.rudder_out = xpilot.rudder_out - (xpilot.aileron1_out * RUDDER_MIXING);
#else
    xpilot.rudder_out = xpilot.rudder_out + (xpilot.aileron1_out * RUDDER_MIXING);
#endif
#endif
}

ModeController modeController;