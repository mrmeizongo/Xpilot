#include "Xpilot.h"

// Proportional gain value. Increase or decrease number to adjust auto mode sentivity
unsigned char kp = 2;
unsigned char ailOut, eleOut = 0;

bool isCentered(unsigned char stick);

void modeController(Xpilot &xpilot)
{
    ailOut = xpilot.aileron_out;
    eleOut = xpilot.elevator_out;

    switch (xpilot.currentMode)
    {
    case Xpilot::Mode::STABILIZE:
        stabilizeMode(xpilot);
        break;
    case Xpilot::Mode::FBW:
        flyByWireMode(xpilot);
        break;
    case Xpilot::Mode::MANUAL:
        manualMode(xpilot);
        break;
    default:
        break;
    }
}

void manualMode(Xpilot &xpilot)
{
    // Do nothing for now, pass raw values from tx to rx
    // Allow full aileron and elevator travel
    ailOut = constrain(ailOut, 0, 180);
    eleOut = constrain(eleOut, 0, 180);

    xpilot.aileron_out = ailOut;
    xpilot.elevator_out = eleOut;
}

void flyByWireMode(Xpilot &xpilot)
{
    // float rollError = desiredRoll - xpilot.ahrs_roll;
    // float pitchError = desiredPitch - xpilot.ahrs_pitch;

    // xpilot.aileron_out += lockRoll ? rollError * kp : 0;
    // xpilot.elevator_out -= lockPitch ? pitchError * kp : 0;

    // Constrain aileron and elevator servo travel
    // This value can be changed in xpilot config file
    ailOut = constrain(ailOut, ROLL_LIMIT, 180 - ROLL_LIMIT);
    eleOut = constrain(eleOut, PITCH_LIMIT, 180 - PITCH_LIMIT);

    xpilot.aileron_out = ailOut;
    xpilot.elevator_out = eleOut;
}

void stabilizeMode(Xpilot &xpilot)
{
    float rollError = 0 - xpilot.ahrs_roll;
    float pitchError = 0 - xpilot.ahrs_pitch;

    ailOut += isCentered(ailOut) ? rollError * kp : 0;
    eleOut -= isCentered(eleOut) ? pitchError * kp : 0;

    // Constrain aileron and elevator servo travel
    // This value can be changed in xpilot config file
    ailOut = constrain(ailOut, ROLL_LIMIT, 180 - ROLL_LIMIT);
    eleOut = constrain(eleOut, PITCH_LIMIT, 180 - PITCH_LIMIT);

    xpilot.aileron_out = ailOut;
    xpilot.elevator_out = eleOut;
}

bool isCentered(unsigned char stick)
{
    return abs(stick - 90) <= 3 ? true : false;
}