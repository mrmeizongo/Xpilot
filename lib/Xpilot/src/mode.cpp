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

// Helper function to reset all PID controllers
#define ResetPIDControllers() \
    rollPID->ResetPID();      \
    pitchPID->ResetPID();     \
    yawPID->ResetPID();

Mode::Mode()
{
    rollPID = new PID(ROLL_KP, ROLL_KI, ROLL_KD);
    pitchPID = new PID(PITCH_KP, PITCH_KI, PITCH_KD);
    yawPID = new PID(YAW_KP, YAW_KI, YAW_KD);
}

// Update flight mode from mode switch position
// Do nothing if we're already in the selected mode
// Otherwise reset PID values and set current mode
void Mode::update(long pulse)
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

        ResetPIDControllers();
        xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::RATE);
    }
    else if (pulse <= SERVO_MIN_PWM + INPUT_THRESHOLD)
    {
        if (xpilot.getCurrentMode() == Xpilot::FLIGHT_MODE::STABILIZE)
            return;

        ResetPIDControllers();
        xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::STABILIZE);
    }
}

void Mode::process()
{
    switch (xpilot.getCurrentMode())
    {
    case Xpilot::FLIGHT_MODE::PASSTHROUGH:
        passthroughMode();
        break;
    case Xpilot::FLIGHT_MODE::RATE:
        rateMode();
        xpilot.aileron_out = map(xpilot.aileron_out, -MAX_ROLL_RATE_PID, MAX_ROLL_RATE_PID, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.elevator_out = map(xpilot.elevator_out, -MAX_PITCH_RATE_PID, MAX_PITCH_RATE_PID, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.rudder_out = map(xpilot.rudder_out, -MAX_YAW_RATE_PID, MAX_YAW_RATE_PID, SERVO_MIN_PWM, SERVO_MAX_PWM);
        break;
    case Xpilot::FLIGHT_MODE::STABILIZE:
        stabilizeMode();
        xpilot.aileron_out = map(xpilot.aileron_out, -MAX_ROLL_ANGLE_PID, MAX_ROLL_ANGLE_PID, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.elevator_out = map(xpilot.elevator_out, -MAX_PITCH_ANGLE_PID, MAX_PITCH_ANGLE_PID, SERVO_MIN_PWM, SERVO_MAX_PWM);
        xpilot.rudder_out = map(xpilot.rudder_out, -MAX_YAW_RATE_PID, MAX_YAW_RATE_PID, SERVO_MIN_PWM, SERVO_MAX_PWM);
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
// No stabilization and rate control
void Mode::passthroughMode()
{
    // This is to stop the fluctuation experienced in center position due to stick drift
    xpilot.aileron_out = (abs(xpilot.aileronPulseWidth - SERVO_MID_PWM) <= ROLL_INPUT_DEADBAND) ? SERVO_MID_PWM : xpilot.aileronPulseWidth;
    xpilot.elevator_out = (abs(xpilot.elevatorPulseWidth - SERVO_MID_PWM) <= PITCH_INPUT_DEADBAND) ? SERVO_MID_PWM : xpilot.elevatorPulseWidth;
    xpilot.rudder_out = (abs(xpilot.rudderPulseWidth - SERVO_MID_PWM) <= YAW_INPUT_DEADBAND) ? SERVO_MID_PWM : xpilot.rudderPulseWidth;
}

// Rate mode uses gyroscope values to maintain a desired rate of change
// Flight surfaces move to prevent sudden changes in direction
void Mode::rateMode()
{
    xpilot.aileron_out = (abs(xpilot.aileronPulseWidth - SERVO_MID_PWM) <= ROLL_INPUT_DEADBAND)
                             ? 0
                             : map(xpilot.aileronPulseWidth, SERVO_MIN_PWM, SERVO_MAX_PWM, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
    xpilot.elevator_out = (abs(xpilot.elevatorPulseWidth - SERVO_MID_PWM) <= PITCH_INPUT_DEADBAND)
                              ? 0
                              : map(xpilot.elevatorPulseWidth, SERVO_MIN_PWM, SERVO_MAX_PWM, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);
    xpilot.rudder_out = (abs(xpilot.rudderPulseWidth - SERVO_MID_PWM) <= YAW_INPUT_DEADBAND)
                            ? 0
                            : map(xpilot.rudderPulseWidth, SERVO_MIN_PWM, SERVO_MAX_PWM, -MAX_YAW_RATE_DEGS, MAX_YAW_RATE_DEGS);

    int16_t rollDemand = xpilot.aileron_out - xpilot.gyroX;
    int16_t pitchDemand = xpilot.elevator_out - xpilot.gyroY;
    int16_t yawDemand = xpilot.rudder_out - xpilot.gyroZ;

    xpilot.aileron_out = rollPID->Compute(rollDemand);
    xpilot.elevator_out = pitchPID->Compute(pitchDemand);
    xpilot.rudder_out = yawPID->Compute(yawDemand);
}

// Roll and pitch follow stick input up to set limits
// Roll and pitch leveling on stick release
void Mode::stabilizeMode()
{
    xpilot.aileron_out = (abs(xpilot.aileronPulseWidth - SERVO_MID_PWM) <= ROLL_INPUT_DEADBAND)
                             ? 0
                             : map(xpilot.aileronPulseWidth, SERVO_MIN_PWM, SERVO_MAX_PWM, -MAX_ROLL_ANGLE_DEGS, MAX_ROLL_ANGLE_DEGS);
    xpilot.elevator_out = (abs(xpilot.elevatorPulseWidth - SERVO_MID_PWM) <= PITCH_INPUT_DEADBAND)
                              ? 0
                              : map(xpilot.elevatorPulseWidth, SERVO_MIN_PWM, SERVO_MAX_PWM, -MAX_PITCH_ANGLE_DEGS, MAX_PITCH_ANGLE_DEGS);
    xpilot.rudder_out = (abs(xpilot.rudderPulseWidth - SERVO_MID_PWM) <= YAW_INPUT_DEADBAND)
                            ? 0
                            : map(xpilot.rudderPulseWidth, SERVO_MIN_PWM, SERVO_MAX_PWM, -MAX_YAW_RATE_DEGS, MAX_YAW_RATE_DEGS);

    int16_t rollDemand = xpilot.aileron_out - xpilot.ahrs_roll;
    int16_t pitchDemand = xpilot.elevator_out - xpilot.ahrs_pitch;
    int16_t yawDemand = xpilot.rudder_out - xpilot.gyroZ;

    xpilot.aileron_out = rollPID->Compute(rollDemand);
    xpilot.elevator_out = pitchPID->Compute(pitchDemand);
    xpilot.rudder_out = yawPID->Compute(yawDemand);
}

Mode mode;