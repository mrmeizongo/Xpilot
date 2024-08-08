#include <Arduino.h>
#include "Radio.h"
#include "config.h"
#include "ModeController.h"
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
    // Record the length of the pulse if it is within the 1ms to 2ms range
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

    modeController.updateMode();

    // Set stick resolutions
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
    else
    {
        // micros() overflows after approximately 70min
        aileronStartTime = 0;
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
    else
    {
        // micros() overflows after approximately 70min
        elevatorStartTime = 0;
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
    else
    {
        // micros() overflows after approximately 70min
        rudderStartTime = 0;
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
    else
    {
        // micros() overflows after approximately 70min
        modeStartTime = 0;
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