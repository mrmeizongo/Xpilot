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
            if (xpilot.getCurrentMode() == Xpilot::FLIGHT_MODE::MANUAL)
                return;

            xpilot.rollDeflectionLim = 0;
            xpilot.pitchDeflectionLim = 0;
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::MANUAL);
        }
    }
}

void Mode::process()
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
#if DEBUG
        Serial.println("Invalid mode. Defaulting to FBW.");
#endif
        FBWMode();
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

    xpilot.aileron_out = allowInput(xpilot.ahrs_roll, xpilot.aileron_out, ROLL_LIMIT) ? xpilot.aileron_out : CENTER_DEFLECTION_POS;
    xpilot.elevator_out = allowInput(xpilot.ahrs_pitch, xpilot.elevator_out, PITCH_LIMIT) ? xpilot.elevator_out : CENTER_DEFLECTION_POS;
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

    xpilot.aileron_out = allowInput(xpilot.ahrs_roll, xpilot.aileron_out, ROLL_LIMIT) ? xpilot.aileron_out + rollStabilize : CENTER_DEFLECTION_POS;
    xpilot.elevator_out = allowInput(xpilot.ahrs_pitch, xpilot.elevator_out, PITCH_LIMIT) ? xpilot.elevator_out - pitchStabilize : CENTER_DEFLECTION_POS;
}

// Helper functions
bool isCentered(unsigned char stickInput)
{
    return abs(stickInput - CENTER_DEFLECTION_POS) <= 3;
}

// Allow input based on angle
bool allowInput(float angle, unsigned char input, float angleLimit)
{
    // If we haven't reached the angle limit or we're not touching the input sticks, allow input
    if (abs(angle) <= angleLimit || isCentered(input))
        return true;

    // If limit is reached, prevent increasing it but allow decreasing
    // Typically, both roll and pitch angle readings from the IMU are 0 when level
    // Roll angle is positive when rolling left and negative when rolling right, pitch angle is negative pitching down and positive when pitching up
    // Aileron stick input is positive when pushed left and negative when pushed right
    // Elevator stick input is positive when pushed up and negative when pushed down
    // Stick and servo positions are 90 when centered, increases up to limit 180 when rolling left or pitching up and decreases up to limit 0 otherwise
    return (angle > 0 && input < CENTER_DEFLECTION_POS) || (angle < 0 && input > CENTER_DEFLECTION_POS);
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