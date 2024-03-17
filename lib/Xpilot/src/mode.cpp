#include "mode.h"

// Mode variables
volatile long modeCurrentTIme, modeStartTime, modePulses = 0;
unsigned char kp = 2;

// Helper functions
bool isCentered(unsigned char stickInput);
bool allowInput(float angle, unsigned char stickInput, float angleLimit);

Mode::Mode()
{
    modePinInt = MODEPIN_INT;
}

void Mode::init()
{
    // Mode setup
    // Mode pin uses pin change interrupts to read tx channel 6 data
    pinMode(modePinInt, INPUT_PULLUP);
    PCICR |= B00000100;
    PCMSK2 |= B00010000;
}

void Mode::setMode()
{
    if (modePulses >= RECEIVER_LOW && modePulses <= RECEIVER_HIGH)
    {
        if (abs(modePulses - RECEIVER_LOW) < MODE_THRESHOLD && xpilot.getCurrentMode() != Xpilot::FLIGHT_MODE::STABILIZE)
        {
            xpilot.rollDeflectionLim = AILERON_DEFLECTION_LIM;
            xpilot.pitchDeflectinLim = ELEVATOR_DEFLECTION_LIM;
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::STABILIZE);
        }
        else if (abs(modePulses - RECEIVER_MID) < MODE_THRESHOLD && xpilot.getCurrentMode() != Xpilot::FLIGHT_MODE::FBW)
        {
            xpilot.rollDeflectionLim = AILERON_DEFLECTION_LIM;
            xpilot.pitchDeflectinLim = ELEVATOR_DEFLECTION_LIM;
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::FBW);
        }
        else if (abs(modePulses - RECEIVER_HIGH) < MODE_THRESHOLD && xpilot.getCurrentMode() != Xpilot::FLIGHT_MODE::MANUAL)
        {
            xpilot.rollDeflectionLim = 0;
            xpilot.pitchDeflectinLim = 0;
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::MANUAL);
        }
    }
}

void Mode::updateMode()
{
    switch (xpilot.getCurrentMode())
    {
    case Xpilot::FLIGHT_MODE::MANUAL:
        manualMode(xpilot);
        break;
    case Xpilot::FLIGHT_MODE::FBW:
        FBWMode(xpilot);
        break;
    case Xpilot::FLIGHT_MODE::STABILIZE:
        stabilizeMode(xpilot);
        break;
    default:
        manualMode(xpilot);
        break;
    }
}

// Manual mode gives full control of the rc plane flight surfaces
// No stabilization and no deflection limits on flight surfaces
void Mode::manualMode(Xpilot &xpilot)
{
    // This is to stop the fluctuation experienced due to stick drift
    xpilot.aileron_out = isCentered(xpilot.aileron_out) ? CENTER_SRV_POS : xpilot.aileron_out;
    xpilot.elevator_out = isCentered(xpilot.elevator_out) ? CENTER_SRV_POS : xpilot.elevator_out;
}

// FBW mode is like manual mode
// Roll and pitch follow stick input up to set limits
void Mode::FBWMode(Xpilot &xpilot)
{
    unsigned char aileronOut = isCentered(xpilot.aileron_out) ? CENTER_SRV_POS : xpilot.aileron_out;
    unsigned char elevatorOut = isCentered(xpilot.elevator_out) ? CENTER_SRV_POS : xpilot.elevator_out;

    xpilot.aileron_out = allowInput(xpilot.ahrs_roll, aileronOut, ROLL_LIMIT) ? xpilot.aileron_out : CENTER_SRV_POS;
    xpilot.elevator_out = allowInput(xpilot.ahrs_pitch, elevatorOut, PITCH_LIMIT) ? xpilot.elevator_out : CENTER_SRV_POS;
}

// Roll and pitch follow stick input up to set limits
// Wing and pitch leveling on stick release
void Mode::stabilizeMode(Xpilot &xpilot)
{
    float desiredRoll = 0 - xpilot.ahrs_roll;
    float desiredPitch = 0 - xpilot.ahrs_pitch;

    float rollStabilize = isCentered(xpilot.aileron_out) ? desiredRoll * kp : 0;
    float pitchStabilize = isCentered(xpilot.elevator_out) ? desiredPitch * kp : 0;

    xpilot.aileron_out = allowInput(xpilot.ahrs_roll, xpilot.aileron_out + rollStabilize, ROLL_LIMIT) ? xpilot.aileron_out + rollStabilize : CENTER_SRV_POS;
    xpilot.elevator_out = allowInput(xpilot.ahrs_pitch, xpilot.elevator_out - pitchStabilize, PITCH_LIMIT) ? xpilot.elevator_out - pitchStabilize : CENTER_SRV_POS;
}

// Helper functions
bool isCentered(unsigned char stickInput)
{
    return abs(stickInput - CENTER_SRV_POS) <= 3;
}

// Allow input based on angle
// If limit is reached, prevent increasing it but allow decreasing
bool allowInput(float angle, unsigned char stickInput, float angleLimit)
{
    // If we haven't reached the limit, allow input
    if (abs(angle) <= angleLimit)
        return true;

    return (angle <= -angleLimit && stickInput <= CENTER_SRV_POS) || (angle >= angleLimit && stickInput >= CENTER_SRV_POS);
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

Mode mode;