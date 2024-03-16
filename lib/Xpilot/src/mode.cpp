#include "mode.h"

// Mode variables
volatile long modeCurrentTIme, modeStartTime, modePulses = 0;
unsigned char kp = 2;

// Helper functions
bool isCentered(unsigned char stickInput);
bool limitMvmnt(float angle);

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
            xpilot.rollLimit = DEFAULT_ROLL_LIMIT;
            xpilot.pitchLimit = DEFAULT_PITCH_LIMIT;
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::STABILIZE);
        }
        else if (abs(modePulses - RECEIVER_MID) < MODE_THRESHOLD && xpilot.getCurrentMode() != Xpilot::FLIGHT_MODE::FBW)
        {
            xpilot.rollLimit = DEFAULT_ROLL_LIMIT;
            xpilot.pitchLimit = DEFAULT_PITCH_LIMIT;
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::FBW);
        }
        else if (abs(modePulses - RECEIVER_HIGH) < MODE_THRESHOLD && xpilot.getCurrentMode() != Xpilot::FLIGHT_MODE::MANUAL)
        {
            xpilot.rollLimit = 0;
            xpilot.pitchLimit = 0;
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

void Mode::manualMode(Xpilot &xpilot)
{
    // Do nothing for now
    // Pass raw tx values to servos
}

void Mode::FBWMode(Xpilot &xpilot)
{
}

void Mode::stabilizeMode(Xpilot &xpilot)
{
    float desiredRoll = 0 - xpilot.ahrs_roll;
    float desiredPitch = 0 - xpilot.ahrs_pitch;

    if (isCentered(xpilot.aileron_out))
        xpilot.aileron_out += desiredRoll * kp;

    if (isCentered(xpilot.aileron_out))
        xpilot.elevator_out -= desiredPitch * kp;
}

bool isCentered(unsigned char stickInput)
{
    return abs(stickInput - 90) <= 3 ? true : false;
}

bool limitMvmnt(float angle, float limit)
{
    return abs(angle - limit) <= 1;
}

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