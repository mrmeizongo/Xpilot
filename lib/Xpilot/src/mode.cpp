#include "mode.h"

// Mode channel input timer variables. See setMode and ISR functions
volatile long modeCurrentTIme, modeStartTime, modePulses = 0;

// Propportional gain. Keep number low; increase as needed
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

void Mode::set()
{
    if (modePulses >= RECEIVER_LOW && modePulses <= RECEIVER_HIGH)
    {
        if (abs(modePulses - RECEIVER_LOW) < MODE_THRESHOLD)
        {
            xpilot.rollDeflectionLim = AILERON_DEFLECTION_LIM;
            xpilot.pitchDeflectionLim = ELEVATOR_DEFLECTION_LIM;
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::STABILIZE);
        }
        else if (abs(modePulses - RECEIVER_MID) < MODE_THRESHOLD)
        {
            xpilot.rollDeflectionLim = AILERON_DEFLECTION_LIM;
            xpilot.pitchDeflectionLim = ELEVATOR_DEFLECTION_LIM;
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::FBW);
        }
        else if (abs(modePulses - RECEIVER_HIGH) < MODE_THRESHOLD)
        {
            xpilot.rollDeflectionLim = 0;
            xpilot.pitchDeflectionLim = 0;
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::MANUAL);
        }
    }
}

void Mode::update()
{
    switch (xpilot.getCurrentMode())
    {
    case Xpilot::FLIGHT_MODE::MANUAL:
        manualMode();
        break;
    case Xpilot::FLIGHT_MODE::FBW:
        FBWMode();
        break;
    case Xpilot::FLIGHT_MODE::STABILIZE:
        stabilizeMode();
        break;
    default:
        manualMode();
        break;
    }
}

// Manual mode gives full control of the rc plane flight surfaces
// No stabilization and no deflection limits on flight surfaces
void Mode::manualMode()
{
    // This is to stop the fluctuation experienced in center position due to stick drift
    xpilot.aileron_out = isCentered(xpilot.aileron_out) ? CENTER_SRV_POS : xpilot.aileron_out;
    xpilot.elevator_out = isCentered(xpilot.elevator_out) ? CENTER_SRV_POS : xpilot.elevator_out;
}

// FBW mode is like manual mode
// Roll and pitch follow stick input up to set limits
void Mode::FBWMode()
{
    xpilot.aileron_out = isCentered(xpilot.aileron_out) ? CENTER_SRV_POS : xpilot.aileron_out;
    xpilot.elevator_out = isCentered(xpilot.elevator_out) ? CENTER_SRV_POS : xpilot.elevator_out;

    xpilot.aileron_out = allowInput(xpilot.ahrs_roll, xpilot.aileron_out, ROLL_LIMIT) ? xpilot.aileron_out : CENTER_SRV_POS;
    xpilot.elevator_out = allowInput(xpilot.ahrs_pitch, xpilot.elevator_out, PITCH_LIMIT) ? xpilot.elevator_out : CENTER_SRV_POS;
}

// Roll and pitch follow stick input up to set limits
// Wing and pitch leveling on stick release
// Uses a simple P controller; add to roll and subtract from pitch
void Mode::stabilizeMode()
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
bool allowInput(float angle, unsigned char stickInput, float angleLimit)
{
    // If we haven't reached the angle limit, allow input
    if (abs(angle) <= angleLimit)
        return true;

    // If limit is reached, prevent increasing it but allow decreasing
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