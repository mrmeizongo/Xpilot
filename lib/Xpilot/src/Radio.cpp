#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include "Radio.h"
#include "ModeController.h"
#include "config.h"

volatile unsigned long aileronCurrentTime, aileronStartTime, aileronPulses = 0;
volatile unsigned long elevatorCurrentTime, elevatorStartTime, elevatorPulses = 0;
volatile unsigned long rudderCurrentTime, rudderStartTime, rudderPulses = 0;
volatile unsigned long modeCurrentTime, modeStartTime, modePulses = 0;

uint16_t aileronPulseWidth, elevatorPulseWidth, rudderPulseWidth = 0;
// -------------------------

// Helper function to set Radio rx values
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
    // Disable interrupts as pulses are being read
    cli();
    // Record the length of the pulse if it is within tx/rx range (pulse is in uS)
    if (modePulses >= INPUT_MIN_PWM && modePulses <= INPUT_MAX_PWM)
    {
        if (modePulses >= INPUT_MAX_PWM - INPUT_THRESHOLD)
            rx.currentMode = FlightMode::passthrough;
        else if (modePulses > INPUT_MIN_PWM + INPUT_THRESHOLD)
            rx.currentMode = FlightMode::rate;
        else if (modePulses >= INPUT_MIN_PWM)
            rx.currentMode = FlightMode::stabilize;
    }

    if (aileronPulses >= INPUT_MIN_PWM && aileronPulses <= INPUT_MAX_PWM)
        aileronPulseWidth = aileronPulses;
    if (elevatorPulses >= INPUT_MIN_PWM && elevatorPulses <= INPUT_MAX_PWM)
        elevatorPulseWidth = elevatorPulses;
    if (rudderPulses >= INPUT_MIN_PWM && rudderPulses <= INPUT_MAX_PWM)
        rudderPulseWidth = rudderPulses;
    // Enable interrupts

    sei();

    // Set stick resolutions
    switch (rx.currentMode)
    {
    case FlightMode::passthrough:
        rx.roll = SETINPUT(aileronPulseWidth, ROLL_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        rx.pitch = SETINPUT(elevatorPulseWidth, PITCH_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        rx.yaw = SETINPUT(rudderPulseWidth, YAW_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        break;
    case FlightMode::rate:
        rx.roll = SETINPUT(aileronPulseWidth, ROLL_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
        rx.pitch = SETINPUT(elevatorPulseWidth, PITCH_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);
        rx.yaw = SETINPUT(rudderPulseWidth, YAW_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_YAW_RATE_DEGS, MAX_YAW_RATE_DEGS);
        break;
    case FlightMode::stabilize:
        rx.roll = SETINPUT(aileronPulseWidth, ROLL_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_ROLL_ANGLE_DEGS, MAX_ROLL_ANGLE_DEGS);
        rx.pitch = SETINPUT(elevatorPulseWidth, PITCH_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_PITCH_ANGLE_DEGS, MAX_PITCH_ANGLE_DEGS);
        rx.yaw = SETINPUT(rudderPulseWidth, YAW_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_YAW_RATE_DEGS, MAX_YAW_RATE_DEGS);
        break;
    default:
        break;
    }

    modeController.updateMode();
}

/*
 * ISR
 * Typical hobby servos expect to see a pulse every 20ms and the length of the pulse determines the position to set the servo
 * The length of the pulse is typically between 1ms - 2ms with 1ms setting the servo position to 0°, 1.5ms to 90° and 2ms to 180°
 * RC transmitters are designed to send a pulse to the receiver every 20ms within this range, going HIGH for the duration of the pulse and LOW otherwise
 * If the output of the channel on the receiver is attached to an interrupt pin on the arduino, we can use it to drive a pin change interrupt ISR
 * The ISR simply records the time between the changes
 * millis() causes problems when called in an ISR so we play it safe and use micros() instead (1us -> 1000ms)
 */
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

Radio radio;