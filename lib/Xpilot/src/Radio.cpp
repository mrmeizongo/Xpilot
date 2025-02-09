#include <Arduino.h>
#include <util/atomic.h>
#include <PinChangeInterrupt.h>
#include <BoardConfig.h>
#include "Radio.h"

volatile static unsigned long aileronCurrentTime, aileronStartTime, aileronPulses = 0;
volatile static unsigned long elevatorCurrentTime, elevatorStartTime, elevatorPulses = 0;
volatile static unsigned long rudderCurrentTime, rudderStartTime, rudderPulses = 0;
volatile static unsigned long modeCurrentTime, modeStartTime, modePulses = 0;
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
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        // Record the length of the pulse if it is within tx/rx range (pulse is in uS)
        if (modePulses >= INPUT_MIN_PWM && modePulses <= INPUT_MAX_PWM)
        {
            if (modePulses >= INPUT_MAX_PWM - INPUT_SEPARATOR)
                rx.currentMode = PASSTHROUGH;
            else if (modePulses <= INPUT_MIN_PWM + INPUT_SEPARATOR)
                rx.currentMode = STABILIZE;
            else
                rx.currentMode = RATE;
        }

        if (aileronPulses >= INPUT_MIN_PWM && aileronPulses <= INPUT_MAX_PWM)
            rx.rollPWM = aileronPulses;
        if (elevatorPulses >= INPUT_MIN_PWM && elevatorPulses <= INPUT_MAX_PWM)
            rx.pitchPWM = elevatorPulses;
        if (rudderPulses >= INPUT_MIN_PWM && rudderPulses <= INPUT_MAX_PWM)
            rx.yawPWM = rudderPulses;
    }

    // Set stick resolutions
    switch (rx.currentMode)
    {
    case PASSTHROUGH:
        rx.roll = SETINPUT(rx.rollPWM, ROLL_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        rx.pitch = SETINPUT(rx.pitchPWM, PITCH_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        rx.yaw = SETINPUT(rx.yawPWM, YAW_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        break;
    case STABILIZE:
        rx.roll = SETINPUT(rx.rollPWM, ROLL_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_ROLL_ANGLE_DEGS, MAX_ROLL_ANGLE_DEGS);
        rx.pitch = SETINPUT(rx.pitchPWM, PITCH_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_PITCH_ANGLE_DEGS, MAX_PITCH_ANGLE_DEGS);
        rx.yaw = SETINPUT(rx.yawPWM, YAW_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_YAW_RATE_DEGS, MAX_YAW_RATE_DEGS);
        break;
    case RATE:
    default:
        rx.roll = SETINPUT(rx.rollPWM, ROLL_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
        rx.pitch = SETINPUT(rx.pitchPWM, PITCH_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);
        rx.yaw = SETINPUT(rx.yawPWM, YAW_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_YAW_RATE_DEGS, MAX_YAW_RATE_DEGS);
        break;
    }
}

/*
 * ISR
 * Typical hobby servos expect to see a pulse every 20ms and the length of the pulse determines the position to set the servo
 * The length of the pulse is typically between 1ms - 2ms with 1ms setting the servo position to 0°, 1.5ms to 90° and 2ms to 180°
 * RC transmitters are designed to send a pulse to the receiver every 20ms within this range, going HIGH for the duration of the pulse and LOW otherwise
 * If the output of the channel on the receiver is attached to an interrupt pin on the arduino, we can use it to drive a pin change interrupt ISR
 * The ISR simply records the time between the changes
 * millis() causes problems when called in an ISR so we play it safe and use micros() instead (1000us -> 1ms)
 */
void PinChangeInterruptEvent(AILPIN_INT)(void)
{
    aileronCurrentTime = micros();
    if (aileronCurrentTime > aileronStartTime)
    {
        aileronPulses = aileronCurrentTime - aileronStartTime;
        aileronStartTime = aileronCurrentTime;
    }
    else
        aileronStartTime = 0;
}

void PinChangeInterruptEvent(ELEVPIN_INT)(void)
{
    elevatorCurrentTime = micros();
    if (elevatorCurrentTime > elevatorStartTime)
    {
        elevatorPulses = elevatorCurrentTime - elevatorStartTime;
        elevatorStartTime = elevatorCurrentTime;
    }
    else
        elevatorStartTime = 0;
}

void PinChangeInterruptEvent(RUDDPIN_INT)(void)
{
    rudderCurrentTime = micros();
    if (rudderCurrentTime > rudderStartTime)
    {
        rudderPulses = rudderCurrentTime - rudderStartTime;
        rudderStartTime = rudderCurrentTime;
    }
    else
        rudderStartTime = 0;
}

void PinChangeInterruptEvent(MODEPIN_INT)(void)
{
    modeCurrentTime = micros();
    if (modeCurrentTime > modeStartTime)
    {
        modePulses = modeCurrentTime - modeStartTime;
        modeStartTime = modeCurrentTime;
    }
    else
        modeStartTime = 0;
}
//  ----------------------------

Radio radio;