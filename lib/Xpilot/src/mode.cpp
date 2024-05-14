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

#include <Xpilot.h>
#include <PID_v1.h>
#include "Mode.h"

// Mode channel input timer variables. See setMode and ISR functions
volatile long modeCurrentTIme, modeStartTime, modePulses = 0;

// Propportional gain. Keep number low; increase as needed
uint8_t stabilizeKp = 2;

// Helper functions
bool isCentered(uint8_t stickInput);
bool allowInput(float angle, uint8_t stickInput, uint8_t angleLimit, bool reverse = false);

Mode::Mode()
{
    modePinInt = MODEPIN_INT;
}

void Mode::init()
{
    // Mode setup
    // Mode pin uses pin change interrupts to read tx channel data
    // Mode channel should be set up correctly on the tx & rx
    pinMode(modePinInt, INPUT_PULLUP);
    PCICR |= B00000100;
    PCMSK2 |= B00010000;
}

// Update flight mode from mode switch position
void Mode::update()
{
    if (modePulses >= RECEIVER_LOW && modePulses <= RECEIVER_HIGH)
    {
        if (abs(modePulses - RECEIVER_LOW) < INPUT_THRESHOLD)
        {
            if (xpilot.getCurrentMode() == Xpilot::FLIGHT_MODE::STABILIZE)
                return;

            xpilot.rollDeflectionLim = AILERON_DEFLECTION_LIM;
            xpilot.pitchDeflectionLim = ELEVATOR_DEFLECTION_LIM;
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::STABILIZE);
        }
        else if (abs(modePulses - RECEIVER_MID) < INPUT_THRESHOLD)
        {
            if (xpilot.getCurrentMode() == Xpilot::FLIGHT_MODE::FBW)
                return;

            xpilot.rollDeflectionLim = AILERON_DEFLECTION_LIM;
            xpilot.pitchDeflectionLim = ELEVATOR_DEFLECTION_LIM;
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::FBW);
        }
        else if (abs(modePulses - RECEIVER_HIGH) < INPUT_THRESHOLD)
        {
            if (xpilot.getCurrentMode() == Xpilot::FLIGHT_MODE::PASSTHROUGH)
                return;

            xpilot.rollDeflectionLim = 0;
            xpilot.pitchDeflectionLim = 0;
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::PASSTHROUGH);
        }
    }
}

void Mode::process()
{
    switch (xpilot.getCurrentMode())
    {
    case Xpilot::FLIGHT_MODE::PASSTHROUGH:
        manualMode();
        break;
    case Xpilot::FLIGHT_MODE::FBW:
        FBWMode();
        break;
    case Xpilot::FLIGHT_MODE::STABILIZE:
        stabilizeMode();
        break;
    default:
        // Should not get here, but if we do, default to manual mode i.e flight mode 1
#if DEBUG
        Serial.println("Invalid mode. Defaulting to manual.");
#endif
        manualMode();
        break;
    }
}

// Manual mode gives full control of the rc plane flight surfaces
// No stabilization and no deflection limits on flight surfaces
void Mode::manualMode()
{
    // This is to stop the fluctuation experienced in center position due to stick drift
    xpilot.aileron_out = isCentered(xpilot.aileron_out) ? CENTER_DEFLECTION_POS : xpilot.aileron_out;
    xpilot.elevator_out = isCentered(xpilot.elevator_out) ? CENTER_DEFLECTION_POS : xpilot.elevator_out;
}

// FBW mode is like manual mode
// Roll and pitch follow stick input up to set limits
void Mode::FBWMode()
{
    manualMode();
    float rollStabilize = ROLL_LIMIT - abs(xpilot.ahrs_roll);
    float pitchStabilize = PITCH_LIMIT - abs(xpilot.ahrs_pitch);
    rollStabilize = xpilot.ahrs_roll >= 0 ? rollStabilize : -(rollStabilize);
    pitchStabilize = xpilot.ahrs_pitch >= 0 ? pitchStabilize : -(pitchStabilize);

    rollStabilize *= stabilizeKp;
    pitchStabilize *= stabilizeKp;

    xpilot.aileron_out = allowInput(xpilot.ahrs_roll, xpilot.aileron_out, ROLL_LIMIT, true) ? xpilot.aileron_out : CENTER_DEFLECTION_POS + rollStabilize;
    xpilot.elevator_out = allowInput(xpilot.ahrs_pitch, xpilot.elevator_out, PITCH_LIMIT) ? xpilot.elevator_out : CENTER_DEFLECTION_POS - pitchStabilize;
}

// Roll and pitch follow stick input up to set limits
// Wing and pitch leveling on stick release
// Uses a simple P controller; add to roll and subtract from pitch
void Mode::stabilizeMode()
{
    float rollStabilize = 0 - xpilot.ahrs_roll;
    float pitchStabilize = 0 - xpilot.ahrs_pitch;
    bool allowRoll = allowInput(xpilot.ahrs_roll, xpilot.aileron_out, ROLL_LIMIT, true);
    bool allowPitch = allowInput(xpilot.ahrs_pitch, xpilot.elevator_out, PITCH_LIMIT);

    rollStabilize = isCentered(xpilot.aileron_out) || !allowRoll ? rollStabilize : 0;
    pitchStabilize = isCentered(xpilot.elevator_out) || !allowPitch ? pitchStabilize : 0;

    rollStabilize *= stabilizeKp;
    pitchStabilize *= stabilizeKp;

    xpilot.aileron_out = allowRoll ? xpilot.aileron_out + rollStabilize : CENTER_DEFLECTION_POS + rollStabilize;
    xpilot.elevator_out = allowPitch ? xpilot.elevator_out - pitchStabilize : CENTER_DEFLECTION_POS - pitchStabilize;
}

// Helper functions
bool isCentered(uint8_t stickInput)
{
    return abs(stickInput - CENTER_DEFLECTION_POS) <= 3;
}

// Allow input based on angle
bool allowInput(float angle, uint8_t input, uint8_t angleLimit, bool reverse /*=false*/)
{
    // If we haven't reached the angle limit or we're not touching the input sticks, allow input
    if (abs(angle) < angleLimit)
        return true;

    // If limit is reached, prevent increasing it but allow decreasing
    // Typically, both roll and pitch angle readings from the IMU are 0 when level
    // Roll angle is positive when rolling left and negative when rolling right, pitch angle is negative pitching down and positive when pitching up
    // Aileron stick input is positive when pushed left and negative when pushed right
    // Elevator stick input is positive when pushed up and negative when pushed down
    // Stick and servo positions are 90 when centered, increases up to limit 180 when rolling left or pitching up and decreases up to limit 0 otherwise
    if (reverse)
        return (angle > 0 && input < CENTER_DEFLECTION_POS) || (angle < 0 && input > CENTER_DEFLECTION_POS);
    else
        return (angle > 0 && input > CENTER_DEFLECTION_POS) || (angle < 0 && input < CENTER_DEFLECTION_POS);
}
// --------------------------------------

// Pin change interrupt function
ISR(PCINT2_vect)
{
    modeCurrentTIme = micros();
    if (modeCurrentTIme > modeStartTime)
    {
        modePulses = modeCurrentTIme - modeStartTime;
        modeStartTime = modeCurrentTIme;
    }
}
// --------------------------------------

Mode mode;