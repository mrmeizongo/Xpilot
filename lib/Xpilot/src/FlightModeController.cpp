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

#include "FlightModeController.h"
#include "Xpilot.h"
#include "Radio.h"
#include "config.h"

// Helper functions
#define RESETPIDCONTROLLERS() \
    rollPID->ResetPID();      \
    pitchPID->ResetPID();     \
    yawPID->ResetPID();

#define CROPPID(x) ((x) < -(MAX_PID_OUTPUT)) ? -(MAX_PID_OUTPUT) : (((x) > MAX_PID_OUTPUT) ? (MAX_PID_OUTPUT) : (x))
// -----------------------------------------------------------------------------------------------------------------

FlightModeController::FlightModeController(void)
{
    rollPID = new PID(ROLL_KP, ROLL_KI, ROLL_KD);
    pitchPID = new PID(PITCH_KP, PITCH_KI, PITCH_KD);
    yawPID = new PID(YAW_KP, YAW_KI, YAW_KD);
}

// Update flight mode from mode switch position
// Do nothing if we're already in the selected mode
// Otherwise reset PID values and set current mode
void FlightModeController::update(long pulse)
{
    if (pulse >= SERVO_MAX_PWM - INPUT_THRESHOLD)
    {
        if (xpilot.getCurrentMode() == Xpilot::FLIGHT_MODE::PASSTHROUGH)
            return;

        xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::PASSTHROUGH);
    }
    else if (pulse >= SERVO_MIN_PWM + INPUT_THRESHOLD && pulse <= SERVO_MAX_PWM - INPUT_THRESHOLD)
    {
        if (xpilot.getCurrentMode() == Xpilot::FLIGHT_MODE::RATE)
            return;

        RESETPIDCONTROLLERS();
        xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::RATE);
    }
    else if (pulse <= SERVO_MIN_PWM + INPUT_THRESHOLD)
    {
        if (xpilot.getCurrentMode() == Xpilot::FLIGHT_MODE::STABILIZE)
            return;

        RESETPIDCONTROLLERS();
        xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::STABILIZE);
    }
}

void FlightModeController::process(void)
{
    switch (xpilot.getCurrentMode())
    {
    default:
    case Xpilot::FLIGHT_MODE::PASSTHROUGH:
        passthroughMode();
        planeMixer();
        xpilot.aileron1_out = map(xpilot.aileron1_out, -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.aileron2_out = map(xpilot.aileron2_out, -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.elevator_out = map(xpilot.elevator_out, -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.rudder_out = map(xpilot.rudder_out, -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
        break;
    case Xpilot::FLIGHT_MODE::RATE:
        rudderMixer();
        rateMode();
        planeMixer();
        xpilot.aileron1_out = map(xpilot.aileron1_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.aileron2_out = map(xpilot.aileron2_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.elevator_out = map(xpilot.elevator_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.rudder_out = map(xpilot.rudder_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        break;
    case Xpilot::FLIGHT_MODE::STABILIZE:
        rudderMixer();
        stabilizeMode();
        planeMixer();
        xpilot.aileron1_out = map(xpilot.aileron1_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.aileron2_out = map(xpilot.aileron2_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.elevator_out = map(xpilot.elevator_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.rudder_out = map(xpilot.rudder_out, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
        break;
    }
}

// Manual mode gives full control of the rc plane flight surfaces
// No stabilization and rate control
void FlightModeController::passthroughMode(void)
{
    xpilot.aileron1_out = constrain(xpilot.aileron1_out, -PASSTHROUGH_RES, PASSTHROUGH_RES);
    xpilot.aileron2_out = xpilot.aileron1_out;
    xpilot.elevator_out = constrain(xpilot.elevator_out, -PASSTHROUGH_RES, PASSTHROUGH_RES);
    xpilot.rudder_out = constrain(xpilot.rudder_out, -PASSTHROUGH_RES, PASSTHROUGH_RES);
}

// Rate mode uses gyroscope values to maintain a desired rate of change
// Flight surfaces move to prevent sudden changes in direction
void FlightModeController::rateMode(void)
{
    int16_t rollDemand = xpilot.aileron1_out - xpilot.gyroX;
    int16_t pitchDemand = xpilot.elevator_out - xpilot.gyroY;
    int16_t yawDemand = xpilot.rudder_out - xpilot.gyroZ;

    xpilot.aileron1_out = rollPID->Compute(rollDemand);
    xpilot.elevator_out = pitchPID->Compute(pitchDemand);
    xpilot.rudder_out = yawPID->Compute(yawDemand);

    xpilot.aileron1_out = CROPPID(xpilot.aileron1_out);
    xpilot.aileron2_out = xpilot.aileron1_out;
    xpilot.elevator_out = CROPPID(xpilot.elevator_out);
    xpilot.rudder_out = CROPPID(xpilot.rudder_out);
}

// Roll and pitch follow stick input up to set limits
// Roll and pitch leveling on stick release
void FlightModeController::stabilizeMode(void)
{
    int16_t rollDemand = xpilot.aileron1_out - xpilot.ahrs_roll;
    int16_t pitchDemand = xpilot.elevator_out - xpilot.ahrs_pitch;
    int16_t yawDemand = xpilot.rudder_out - xpilot.gyroZ;

    xpilot.aileron1_out = map(rollDemand, -MAX_ROLL_ANGLE_DEGS, MAX_ROLL_ANGLE_DEGS, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
    xpilot.elevator_out = map(pitchDemand, -MAX_PITCH_ANGLE_DEGS, MAX_PITCH_ANGLE_DEGS, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);

    xpilot.aileron1_out = rollPID->Compute(rollDemand - xpilot.gyroX);
    xpilot.elevator_out = pitchPID->Compute(pitchDemand - xpilot.gyroY);
    xpilot.rudder_out = yawPID->Compute(yawDemand);

    xpilot.aileron1_out = CROPPID(xpilot.aileron1_out);
    xpilot.aileron2_out = xpilot.aileron1_out;
    xpilot.elevator_out = CROPPID(xpilot.elevator_out);
    xpilot.rudder_out = CROPPID(xpilot.rudder_out);
}

/*
 * Mixer for airplane type
 * Only tested with a full plane i.e. ailerons, elevator and rudder
 * Proceed with caution. Depends heavily on the direction of servo travel.
 * Perform thorough pre flight checks and reverse as needed.
 *
 * Default
 * For V-Tail - If servos are mounted on top, with face of servo facing the same direction
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
void FlightModeController::planeMixer(void)
{
#if defined(FULL_PLANE)
    xpilot.aileron1_out = xpilot.aileron1_out;
    xpilot.aileron2_out = xpilot.aileron2_out;
    xpilot.elevator_out = xpilot.elevator_out;
    xpilot.rudder_out = xpilot.rudder_out;
#elif defined(FULL_PLANE_V_TAIL)
    xpilot.aileron1_out = xpilot.aileron1_out;
    xpilot.aileron2_out = xpilot.aileron2_out;
    xpilot.elevator_out = xpilot.rudder_out + xpilot.elevator_out;
    xpilot.rudder_out = xpilot.rudder_out - xpilot.elevator_out;
#elif defined(RUDDER_ELEVATOR_ONLY_V_TAIL)
    xpilot.aileron1_out = 0;
    xpilot.aileron2_out = 0;
    xpilot.elevator_out = xpilot.rudder_out + xpilot.elevator_out;
    xpilot.rudder_out = xpilot.rudder_out - xpilot.elevator_out;
#elif defined(FLYING_WING_RUDDER)
    xpilot.aileron1_out = xpilot.aileron1_out - xpilot.elevator_out;
    xpilot.aileron2_out = xpilot.aileron2_out + xpilot.elevator_out;
    xpilot.elevator_out = 0;
    xpilot.rudder_out = xpilot.rudder_out;
#elif defined(FLYING_WING_NO_RUDDER)
    xpilot.aileron1_out = xpilot.aileron1_out - xpilot.elevator_out;
    xpilot.aileron2_out = xpilot.aileron2_out + xpilot.elevator_out;
    xpilot.elevator_out = 0;
    xpilot.rudder_out = 0;
#elif defined(RUDDER_ELEVATOR_ONLY)
    xpilot.aileron1_out = 0;
    xpilot.aileron2_out = 0;
    xpilot.elevator_out = xpilot.elevator_out;
    xpilot.rudder_out = xpilot.rudder_out;
#else
#error No airplane type selected!
#endif
}

void FlightModeController::rudderMixer(void)
{
#if defined(FULL_PLANE) || defined(FULL_PLANE_V_TAIL) || defined(FLYING_WING_RUDDER)
    xpilot.rudder_out = xpilot.rudder_out + (xpilot.aileron1_out * RUDDER_MIXING);
#else
    xpilot.rudder_out = xpilot.rudder_out;
#endif
}

FlightModeController modeController;