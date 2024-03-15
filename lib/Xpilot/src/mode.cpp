#include "Xpilot.h"

unsigned char kp = 2; // Proportional gain value. Increase or decrease number to adjust auto mode sentivity

void modeController(Xpilot &xpilot)
{
    switch (xpilot.currentMode)
    {
    case Xpilot::Mode::STABILIZE:
        stabilizeMode(xpilot);
        break;
    case Xpilot::Mode::FBW:
        flyByWireMode(xpilot);
        break;
    case Xpilot::Mode::MANUAL:
        // Do nothing for now, just pass raw values from tx to rx
        manualMode(xpilot);
        break;
    default:
        break;
    }
}

void manualMode(Xpilot &xpilot)
{
    Serial.println("In MANUAL mode");
}

void flyByWireMode(Xpilot &xpilot)
{
    // TODO: implement FBW mode
    Serial.println("In FBW mode");
}

void stabilizeMode(Xpilot &xpilot)
{
    Serial.println("In STABILIZE mode");
    float rollError = 0 - xpilot.ahrs_roll;
    float pitchError = 0 - xpilot.ahrs_pitch;

    xpilot.aileron_out -= rollError * kp;
    xpilot.elevator_out -= pitchError * kp;
}