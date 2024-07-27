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

#include "Mode.h"
#include "config.h"

bool headingSet = false;

// Helper function to reset all PID controllers
#define ResetPIDControllers() \
    rollPID->ResetPID();      \
    pitchPID->ResetPID();     \
    yawPID->ResetPID();

// Helper functions
bool isCentered(int16_t stickInput);
bool allowInput(double angle, int16_t stickInput, uint8_t angleLimit, bool reverse = false);

Mode::Mode()
{
    rollPID = new PID(ROLL_KP, ROLL_KI, ROLL_KD);
    pitchPID = new PID(PITCH_KP, PITCH_KI, PITCH_KD);
    yawPID = new PID(YAW_KP, YAW_KI, YAW_KD);
}

// Update flight mode from mode switch position
// Do nothing if we're already in the selected mode
// Otherwise reset PID values and set current mode
void Mode::update(long pulseIn)
{
    if (abs(pulseIn - SERVO_MIN_PWM) < INPUT_THRESHOLD)
    {
        if (xpilot.getCurrentMode() == Xpilot::FLIGHT_MODE::STABILIZE)
            return;

        ResetPIDControllers();
        xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::STABILIZE);
    }
    else if ((pulseIn >= SERVO_MID_PWM - INPUT_THRESHOLD) && (pulseIn <= SERVO_MID_PWM + INPUT_THRESHOLD))
    {
        if (xpilot.getCurrentMode() == Xpilot::FLIGHT_MODE::FBW)
            return;

        ResetPIDControllers();
        xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::FBW);
    }
    else if (abs(pulseIn - SERVO_MAX_PWM) < INPUT_THRESHOLD)
    {
        if (xpilot.getCurrentMode() == Xpilot::FLIGHT_MODE::PASSTHROUGH)
            return;

        xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::PASSTHROUGH);
    }
}

void Mode::process()
{
    switch (xpilot.getCurrentMode())
    {
    case Xpilot::FLIGHT_MODE::PASSTHROUGH:
        passthroughMode();
        break;
    case Xpilot::FLIGHT_MODE::FBW:
        FBWMode();
        break;
    case Xpilot::FLIGHT_MODE::STABILIZE:
        stabilizeMode();
        break;
    default:
        // Should not get here, but if we do, default to passthrough mode i.e flight mode 1
#if DEBUG
        Serial.println("Invalid mode. Defaulting to passthrough.");
#endif
        passthroughMode();
        break;
    }
}

// Manual mode gives full control of the rc plane flight surfaces
// No stabilization and no deflection limits on flight surfaces
void Mode::passthroughMode()
{
    // This is to stop the fluctuation experienced in center position due to stick drift
    xpilot.aileron_out = isCentered(xpilot.aileronPulseWidth) ? SERVO_MID_PWM : xpilot.aileronPulseWidth;
    xpilot.elevator_out = isCentered(xpilot.elevatorPulseWidth) ? SERVO_MID_PWM : xpilot.elevatorPulseWidth;
    xpilot.rudder_out = isCentered(xpilot.rudderPulseWidth) ? SERVO_MID_PWM : xpilot.rudderPulseWidth;
}

// FBW mode is like manual mode
// Roll and pitch follow stick input up to set limits
void Mode::FBWMode()
{
    passthroughMode();

    // Compute error
    float rollError = ROLL_LIMIT - abs(xpilot.ahrs_roll);
    float pitchError = PITCH_LIMIT - abs(xpilot.ahrs_pitch);

    rollError = xpilot.ahrs_roll >= 0 ? rollError : -(rollError);
    pitchError = xpilot.ahrs_pitch >= 0 ? pitchError : -(pitchError);

    // Pass to PID controller
    int rollAdjust = rollPID->Compute(rollError);
    int pitchAdjust = pitchPID->Compute(pitchError);
#if REVERSE_ROLL_STABILIZE
    rollAdjust = -(rollAdjust);
#endif
#if REVERSE_PITCH_STABILIZE
    pitchAdjust = -(pitchAdjust);
#endif
    xpilot.aileron_out = allowInput(xpilot.ahrs_roll, xpilot.aileronPulseWidth, ROLL_LIMIT, true) ? xpilot.aileronPulseWidth : SERVO_MID_PWM + rollAdjust;
    xpilot.elevator_out = allowInput(xpilot.ahrs_pitch, xpilot.elevatorPulseWidth, PITCH_LIMIT) ? xpilot.elevatorPulseWidth : SERVO_MID_PWM + pitchAdjust;
}

// Roll and pitch follow stick input up to set limits
// Roll and pitch leveling on stick release
void Mode::stabilizeMode()
{
    float rollError = 0 - xpilot.ahrs_roll;
    int rollAdjust = rollPID->Compute(rollError);
    rollAdjust = isCentered(xpilot.aileronPulseWidth) ? rollAdjust : 0;

    float pitchError = 0 - xpilot.ahrs_pitch;
    int pitchAdjust = pitchPID->Compute(pitchError);
    pitchAdjust = isCentered(xpilot.elevatorPulseWidth) ? pitchAdjust : 0;

#if REVERSE_ROLL_STABILIZE
    rollAdjust = -(rollAdjust);
#endif
#if REVERSE_PITCH_STABILIZE
    pitchAdjust = -(pitchAdjust);
#endif
    // #if REVERSE_YAW_STABILIZE
    //     yawAdjust = -(yawAdjust);
    // #endif

    xpilot.aileron_out = allowInput(xpilot.ahrs_roll, xpilot.aileronPulseWidth, ROLL_LIMIT, true) ? xpilot.aileronPulseWidth + rollAdjust : SERVO_MID_PWM + rollAdjust;
    xpilot.elevator_out = allowInput(xpilot.ahrs_pitch, xpilot.elevatorPulseWidth, PITCH_LIMIT) ? xpilot.elevatorPulseWidth + pitchAdjust : SERVO_MID_PWM + pitchAdjust;
    xpilot.rudder_out = xpilot.rudder_out;
}

// Helper functions
bool isCentered(int16_t stickInput)
{
    return abs(stickInput - SERVO_MID_PWM) <= 20;
}

// Allow input based on angle
bool allowInput(double angle, int16_t input, uint8_t angleLimit, bool reverse)
{
    // If we haven't reached the angle limit allow input
    if (abs(angle) < angleLimit)
        return true;

    // If limit is reached, prevent increasing it but allow decreasing
    // Typically, both roll and pitch angle readings from the IMU are 0 when level
    // Roll angle is positive when rolling left and negative when rolling right, pitch angle is negative pitching down and positive when pitching up
    // Aileron stick input is positive when pushed left and negative when pushed right
    // Elevator stick input is positive when pushed up and negative when pushed down
    // Stick and servo positions are 1500 when centered, increases up to limit 2000 when rolling left or pitching up and decreases up to limit 1000 otherwise
    if (reverse)
        return (angle > 0 && input < SERVO_MID_PWM) || (angle < 0 && input > SERVO_MID_PWM);
    else
        return (angle > 0 && input > SERVO_MID_PWM) || (angle < 0 && input < SERVO_MID_PWM);
}
// --------------------------------------

Mode mode;