#include <Arduino.h>
#include <util/atomic.h>
#include <PinChangeInterrupt.h>
#include <SystemConfig.h>
#include "Radio.h"

volatile static unsigned long aileronCurrentTime, aileronStartTime, aileronPulses = 0;
volatile static unsigned long elevatorCurrentTime, elevatorStartTime, elevatorPulses = 0;
volatile static unsigned long rudderCurrentTime, rudderStartTime, rudderPulses = 0;
volatile static unsigned long auxCurrentTime, auxStartTime, auxPulses = 0;
// -------------------------

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
    // Auxiliary switch setup
    pinMode(AUXPIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(AUXPIN_INT, CHANGE);
}

void Radio::processInput(void)
{
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        // Record the length of the pulse if it is within tx/rx range (pulse is in uS)
        if (auxPulses >= INPUT_MIN_PWM && auxPulses <= INPUT_MAX_PWM)
        {
            if (auxPulses >= INPUT_MAX_PWM - INPUT_SEPARATOR)
                rx.auxSwitchPos = THREE_POS_SW::HIGH_POS;
            else if (auxPulses <= INPUT_MIN_PWM + INPUT_SEPARATOR)
                rx.auxSwitchPos = THREE_POS_SW::LOW_POS;
            else
                rx.auxSwitchPos = THREE_POS_SW::MID_POS;
        }
        if (aileronPulses >= INPUT_MIN_PWM && aileronPulses <= INPUT_MAX_PWM)
            rx.rollPWM = aileronPulses;
        if (elevatorPulses >= INPUT_MIN_PWM && elevatorPulses <= INPUT_MAX_PWM)
            rx.pitchPWM = elevatorPulses;
        if (rudderPulses >= INPUT_MIN_PWM && rudderPulses <= INPUT_MAX_PWM)
            rx.yawPWM = rudderPulses;

        FailSafeLogic(); // Check for failsafe conditions
    }
}

void Radio::FailSafeLogic()
{
    // Big brain failsafe logic
    failsafe = false;
}

/*
 * ISR
 * Typical hobby servos expect to see a pulse every 20ms and the length of the pulse determines the position to set the servo
 * The length of the pulse is typically between 1ms - 2ms with 1ms setting the servo position to 0°, 1.5ms to 90° and 2ms to 180°
 * This gives us a servo refresh rate of 22ms(20ms interval + actual 1ms-2ms pulse duration)
 * RC transmitters are designed to send a pulse to the receiver within this range, going HIGH for the duration of the pulse and LOW otherwise
 * If the output of the channel on the receiver is attached to an interrupt pin, we can use it to drive a PCISR
 * The ISR simply records the time between the changes. We're only interested in pulses lasting between INPUT_MIN_PWM and INPUT_MAX_PWM
 * Due to this input capture mechanism, implementing a failsafe is largely dependent on the receiver's behavior when the signal is lost
 * Example: The Spektrum tx/rx I use will either hold the last known position when the signal is lost or default to a preset position determined at bind time
 * The failsafe implementation is left to the user
 * See plane config INPUT_MIN_PWM and INPUT_MAX_PWM
 */
void PinChangeInterruptEvent(AILPIN_INT)(void)
{
    aileronCurrentTime = micros();
    aileronPulses = aileronCurrentTime - aileronStartTime;
    aileronStartTime = aileronCurrentTime;
}

void PinChangeInterruptEvent(ELEVPIN_INT)(void)
{
    elevatorCurrentTime = micros();
    elevatorPulses = elevatorCurrentTime - elevatorStartTime;
    elevatorStartTime = elevatorCurrentTime;
}

void PinChangeInterruptEvent(RUDDPIN_INT)(void)
{
    rudderCurrentTime = micros();
    rudderPulses = rudderCurrentTime - rudderStartTime;
    rudderStartTime = rudderCurrentTime;
}

void PinChangeInterruptEvent(AUXPIN_INT)(void)
{
    auxCurrentTime = micros();
    auxPulses = auxCurrentTime - auxStartTime;
    auxStartTime = auxCurrentTime;
}
//  ----------------------------

Radio radio;