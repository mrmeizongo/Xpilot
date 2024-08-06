#include <Arduino.h>
#include "Radio.h"
#include "config.h"
#include "FlightModeController.h"
#include <PinChangeInterrupt.h>

volatile unsigned long aileronCurrentTime, aileronStartTime, aileronPulses = 0;
volatile unsigned long elevatorCurrentTime, elevatorStartTime, elevatorPulses = 0;
volatile unsigned long rudderCurrentTime, rudderStartTime, rudderPulses = 0;
volatile unsigned long modeCurrentTime, modeStartTime, modePulses = 0;

uint16_t aileronPulseWidth, elevatorPulseWidth, rudderPulseWidth = 0;
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
    // AIleron setup
    pinMode(AILPIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(AILPIN_INT, CHANGE);
    // Elevator setup
    pinMode(ELEVPIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(ELEVPIN_INT, CHANGE);
    // Rudder setup
    pinMode(RUDDPIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(RUDDPIN_INT, CHANGE);
    // Mode setup
    pinMode(MODEPIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(MODEPIN_INT, CHANGE);
}

void Radio::processInput(void)
{
    uint8_t oldSREG = SREG;

    // Disable interrupts as pulses are being read to avoid race conditionS
    cli();
    if (modePulses >= SERVO_MIN_PWM && modePulses <= SERVO_MAX_PWM)
    {
        if (modePulses >= SERVO_MAX_PWM - INPUT_THRESHOLD)
            rx.mode = SwitchState::low;
        else if (modePulses >= SERVO_MIN_PWM + INPUT_THRESHOLD && modePulses <= SERVO_MAX_PWM - INPUT_THRESHOLD)
            rx.mode = SwitchState::mid;
        else if (modePulses <= SERVO_MIN_PWM + INPUT_THRESHOLD)
            rx.mode = SwitchState::high;
    }

    if (aileronPulses >= SERVO_MIN_PWM && aileronPulses <= SERVO_MAX_PWM)
        aileronPulseWidth = aileronPulses;
    if (elevatorPulses >= SERVO_MIN_PWM && elevatorPulses <= SERVO_MAX_PWM)
        elevatorPulseWidth = elevatorPulses;
    if (rudderPulses >= SERVO_MIN_PWM && rudderPulses <= SERVO_MAX_PWM)
        rudderPulseWidth = rudderPulses;

    modeController.updateFlightMode();

    switch (rx.mode)
    {
    case SwitchState::low:
        rx.roll = SETINPUT(aileronPulseWidth, ROLL_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        rx.pitch = SETINPUT(elevatorPulseWidth, PITCH_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        rx.yaw = SETINPUT(rudderPulseWidth, YAW_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        break;
    default:
    case SwitchState::mid:
        rx.roll = SETINPUT(aileronPulseWidth, ROLL_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
        rx.pitch = SETINPUT(elevatorPulseWidth, PITCH_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);
        rx.yaw = SETINPUT(rudderPulseWidth, YAW_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -MAX_YAW_RATE_DEGS, MAX_YAW_RATE_DEGS);
        break;
    case SwitchState::high:
        rx.roll = SETINPUT(aileronPulseWidth, ROLL_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -MAX_ROLL_ANGLE_DEGS, MAX_ROLL_ANGLE_DEGS);
        rx.pitch = SETINPUT(elevatorPulseWidth, PITCH_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -MAX_PITCH_ANGLE_DEGS, MAX_PITCH_ANGLE_DEGS);
        rx.yaw = SETINPUT(rudderPulseWidth, YAW_INPUT_DEADBAND, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM, -MAX_YAW_RATE_DEGS, MAX_YAW_RATE_DEGS);
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
    elevatorCurrentTime = micros();
    if (elevatorCurrentTime > elevatorStartTime)
    {
        elevatorPulses = elevatorCurrentTime - elevatorStartTime;
        elevatorStartTime = elevatorCurrentTime;
    }
}

void PinChangeInterruptEvent(RUDDPIN_INT)(void)
{
    rudderCurrentTime = micros();
    if (rudderCurrentTime > rudderStartTime)
    {
        rudderPulses = rudderCurrentTime - rudderStartTime;
        rudderStartTime = rudderCurrentTime;
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
//  ----------------------------

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
    Serial.println((int)rx.mode);
    Serial.println();
}
#endif

Radio radio;