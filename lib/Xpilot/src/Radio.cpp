#include <Arduino.h>
#include <util/atomic.h>
#include <PinChangeInterrupt.h>
#include <SystemConfig.h>
#include "Radio.h"

// Helper macro to set PWM values and switch positions based on pulse lengths
#define SET_SWITCH_POS(controlPWM, controlSwitch, pulse)   \
    if (pulse >= INPUT_MIN_PWM && pulse <= INPUT_MAX_PWM)  \
    {                                                      \
        controlPWM = pulse;                                \
        if (pulse >= INPUT_MAX_PWM - INPUT_SEPARATOR)      \
            controlSwitch = THREE_POS_SW::HIGH_POS;        \
        else if (pulse <= INPUT_MIN_PWM + INPUT_SEPARATOR) \
            controlSwitch = THREE_POS_SW::LOW_POS;         \
        else                                               \
            controlSwitch = THREE_POS_SW::MID_POS;         \
    }

// Helper macro to set PWM values based on pulse lengths
#define SET_PWM(controlPWM, pulse)                        \
    if (pulse >= INPUT_MIN_PWM && pulse <= INPUT_MAX_PWM) \
        controlPWM = pulse;

volatile static unsigned long aileronCurrentTime, aileronStartTime, aileronPulses = 0;
volatile static unsigned long elevatorCurrentTime, elevatorStartTime, elevatorPulses = 0;
volatile static unsigned long rudderCurrentTime, rudderStartTime, rudderPulses = 0;
volatile static unsigned long aux1CurrentTime, aux1StartTime, aux1Pulses = 0;
#if defined(USE_FLAPERONS)
volatile static unsigned long aux2CurrentTime, aux2StartTime, aux2Pulses = 0;
#endif
#if defined(USE_AUX3)
volatile static unsigned long aux3CurrentTime, aux3StartTime, aux3Pulses = 0;
#endif
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
    // Auxiliary switch 1 setup
    pinMode(AUX1PIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(AUX1PIN_INT, CHANGE);
#if defined(USE_FLAPERONS)
    // Auxiliary switch 2 setup
    pinMode(AUX2PIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(AUX2PIN_INT, CHANGE);
#endif
#if defined(USE_AUX3)
    // Auxiliary switch 3 setup
    pinMode(AUX3PIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(AUX3PIN_INT, CHANGE);
#endif

    failSafe = false;
}

void Radio::processInput(void)
{
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        SET_SWITCH_POS(currentRx.aux1PWM, currentRx.aux1SwitchPos, aux1Pulses);

#if defined(USE_FLAPERONS)
        SET_SWITCH_POS(currentRx.aux2PWM, currentRx.aux2SwitchPos, aux2Pulses);
#endif

#if defined(USE_AUX3)
        SET_SWITCH_POS(currentRx.aux3PWM, currentRx.aux3SwitchPos, aux3Pulses);
#endif

        SET_PWM(currentRx.rollPWM, aileronPulses);
        SET_PWM(currentRx.pitchPWM, elevatorPulses);
        SET_PWM(currentRx.yawPWM, rudderPulses);
    }

    FailSafe();
}

// During binding, I set up my transmitter's failsafe position to be the maximum value for roll, pitch and yaw
// Failsafe logic is highly user/system peculiar, modify test logic accordingly
void Radio::FailSafe()
{
    // Check transmitter failsafe position i.e. max for roll, pitch and yaw
    bool signalLost = (abs(INPUT_MAX_PWM - currentRx.rollPWM) <= FAILSAFE_TOLERANCE) &&
                      (abs(INPUT_MAX_PWM - currentRx.pitchPWM) <= FAILSAFE_TOLERANCE) &&
                      (abs(INPUT_MAX_PWM - currentRx.yawPWM) <= FAILSAFE_TOLERANCE);

    if (signalLost)
        failSafe = true;
    else
        failSafe = false; // Reset failsafe condition
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
 * I set up my transmitter's failsafe position to be the maximum value for roll, pitch and yaw
 * Failsafe is triggered if the input pulse is at maximum value with a FAILSAFE_TOLERANCE for more than FAILSAFE_TIMEOUT_MS
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

void PinChangeInterruptEvent(AUX1PIN_INT)(void)
{
    aux1CurrentTime = micros();
    aux1Pulses = aux1CurrentTime - aux1StartTime;
    aux1StartTime = aux1CurrentTime;
}

#if defined(USE_FLAPERONS)
void PinChangeInterruptEvent(AUX2PIN_INT)(void)
{
    aux2CurrentTime = micros();
    aux2Pulses = aux2CurrentTime - aux2StartTime;
    aux2StartTime = aux2CurrentTime;
}
#endif

#if defined(USE_AUX3)
void PinChangeInterruptEvent(AUX3PIN_INT)(void)
{
    aux3CurrentTime = micros();
    aux3Pulses = aux3CurrentTime - aux3StartTime;
    aux3StartTime = aux3CurrentTime;
}
#endif
// ----------------------------

Radio radio;