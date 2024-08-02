#include <Arduino.h>
#include "Radio.h"
#include "Xpilot.h"
#include "config.h"
#include "FlightModeController.h"
#include <PinChangeInterrupt.h>

// Aileron variables
volatile long aileronCurrentTime, aileronStartTime, aileronPulses = 0;

// Elevator variables
volatile long elevatorCurrentTIme, elevatorStartTime, elevatorPulses = 0;

// Rudder variables
volatile long rudderCurrentTIme, rudderStartTime, rudderPulses = 0;

// Mode variables
volatile long modeCurrentTime, modeStartTime, modePulses = 0;
// -------------------------

// Helper function
#define SETINPUT(rawValue, deadBand, inLowRange, inMidRange, inHighRange, outLowRange, outHighRange) \
    (abs((rawValue) - (inMidRange)) <= (deadBand) ? 0 : map((rawValue), (inLowRange), (inHighRange), (outLowRange), (outHighRange)))

Radio::Radio(void)
{
}

void Radio::init(void)
{
    // All input pins use pin change interrupts
    attachPinChangeInterrupt(AILPIN_INT, CHANGE);
    attachPinChangeInterrupt(ELEVPIN_INT, CHANGE);
    attachPinChangeInterrupt(RUDDPIN_INT, CHANGE);
    attachPinChangeInterrupt(MODEPIN_INT, CHANGE);
}

void Radio::processInput(void)
{
    uint8_t oldSREG = SREG;

    // Disable interrupts as pulses are being read to avoid race conditionS
    cli();
    if (modePulses >= SERVO_MIN_PWM && modePulses <= SERVO_MAX_PWM)
        modeController.update(modePulses);

    if (aileronPulses >= SERVO_MIN_PWM && aileronPulses <= SERVO_MAX_PWM)
        aileronPulseWidth = aileronPulses;

    if (elevatorPulses >= SERVO_MIN_PWM && elevatorPulses <= SERVO_MAX_PWM)
        elevatorPulseWidth = elevatorPulses;

    if (rudderPulses >= SERVO_MIN_PWM && rudderPulses <= SERVO_MAX_PWM)
        rudderPulseWidth = rudderPulses;

    switch (xpilot.getCurrentMode())
    {
    default:
    case Xpilot::FLIGHT_MODE::PASSTHROUGH:
        xpilot.aileron1_out = SETINPUT(aileronPulseWidth, ROLL_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        xpilot.aileron2_out = xpilot.aileron1_out;
        xpilot.elevator_out = SETINPUT(elevatorPulseWidth, PITCH_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        xpilot.rudder_out = SETINPUT(rudderPulseWidth, YAW_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        break;
    case Xpilot::FLIGHT_MODE::RATE:
        xpilot.aileron1_out = SETINPUT(aileronPulseWidth, ROLL_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
        xpilot.aileron2_out = xpilot.aileron1_out;
        xpilot.elevator_out = SETINPUT(elevatorPulseWidth, PITCH_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);
        xpilot.rudder_out = SETINPUT(rudderPulseWidth, YAW_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -MAX_YAW_RATE_DEGS, MAX_YAW_RATE_DEGS);
        break;
    case Xpilot::FLIGHT_MODE::STABILIZE:
        xpilot.aileron1_out = SETINPUT(aileronPulseWidth, ROLL_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -MAX_ROLL_ANGLE_DEGS, MAX_ROLL_ANGLE_DEGS);
        xpilot.aileron2_out = xpilot.aileron1_out;
        xpilot.elevator_out = SETINPUT(elevatorPulseWidth, PITCH_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -MAX_PITCH_ANGLE_DEGS, MAX_PITCH_ANGLE_DEGS);
        xpilot.rudder_out = SETINPUT(rudderPulseWidth, YAW_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -MAX_YAW_RATE_DEGS, MAX_YAW_RATE_DEGS);
        break;
    }

    SREG = oldSREG;
}

// ISR
void PinChangeInterruptEvent(AILPIN_INT)(void)
{
    aileronCurrentTime = micros();
    if (aileronCurrentTime > aileronStartTime)
    {
        aileronPulses = aileronCurrentTime - aileronStartTime;
        aileronStartTime = aileronCurrentTime;
    }
}

void PinChangeInterruptEvent(ELEVPIN_INT)(void)
{
    elevatorCurrentTIme = micros();
    if (elevatorCurrentTIme > elevatorStartTime)
    {
        elevatorPulses = elevatorCurrentTIme - elevatorStartTime;
        elevatorStartTime = elevatorCurrentTIme;
    }
}

void PinChangeInterruptEvent(RUDDPIN_INT)(void)
{
    rudderCurrentTIme = micros();
    if (rudderCurrentTIme > rudderStartTime)
    {
        rudderPulses = rudderCurrentTIme - rudderStartTime;
        rudderStartTime = rudderCurrentTIme;
    }
}

void PinChangeInterruptEvent(MODEPIN_INT)(void)
{
    modeCurrentTime = micros();
    if (modeCurrentTime > modeStartTime)
    {
        modePulses = modeCurrentTime - modeStartTime;
        modeStartTime = modeCurrentTime;
    }
}
// ----------------------------

#if defined(IO_DEBUG)
void Radio::printInput(void)
{
    Serial.print("Elevator Pulse: ");
    Serial.println(elevatorPulseWidth);
    Serial.print("Aileron 1 Pulse: ");
    Serial.println(aileronPulseWidth);
    Serial.print("Aileron 2 Pulse: ");
    Serial.println(aileronPulseWidth);
    Serial.print("Rudder Pulse: ");
    Serial.println(rudderPulseWidth);
    Serial.print("Flight Mode: ");
    Serial.println((int)xpilot.getCurrentMode());
    Serial.println();
}
#endif

Radio rx;