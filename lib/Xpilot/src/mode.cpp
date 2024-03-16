#include "mode.h"

// Mode variables
volatile long modeCurrentTIme, modeStartTime, modePulses = 0;

Mode::Mode()
{
}

void Mode::init(Xpilot &xpilot)
{
    // Mode setup
    // Mode pin uses pin change interrupts
    pinMode(xpilot.modePinInt, INPUT_PULLUP);
    PCICR |= B00000100;
    PCMSK2 |= B00010000;
}

void Mode::setMode()
{
    if (modePulses >= RECEIVER_LOW && modePulses <= RECEIVER_HIGH)
    {
        if (abs(modePulses - RECEIVER_LOW) < MODE_THRESHOLD && xpilot.getCurrentMode() != Xpilot::FLIGHT_MODE::STABILIZE)
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::STABILIZE);
        else if (abs(modePulses - RECEIVER_MID) < MODE_THRESHOLD && xpilot.getCurrentMode() != Xpilot::FLIGHT_MODE::FBW)
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::FBW);
        else if (abs(modePulses - RECEIVER_HIGH) < MODE_THRESHOLD && xpilot.getCurrentMode() != Xpilot::FLIGHT_MODE::MANUAL)
            xpilot.setCurrentMode(Xpilot::FLIGHT_MODE::MANUAL);
    }
}

void Mode::updateMode()
{
    switch (xpilot.getCurrentMode())
    {
    case Xpilot::FLIGHT_MODE::MANUAL:
        break;
    case Xpilot::FLIGHT_MODE::FBW:
        break;
    case Xpilot::FLIGHT_MODE::STABILIZE:
        break;
    default:
        break;
    }
}

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