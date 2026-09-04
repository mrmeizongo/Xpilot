#include <Arduino.h>
#include <util/atomic.h>
#include "GPIODef.h"
#include "Radio.h"
#include "PinChangeInterrupt.h"

volatile static uint32_t aileronCurrentTime = 0, aileronStartTime = 0, aileronPulses = 0;
volatile static uint32_t elevatorCurrentTime = 0, elevatorStartTime = 0, elevatorPulses = 0;
volatile static uint32_t rudderCurrentTime = 0, rudderStartTime = 0, rudderPulses = 0;
volatile static uint32_t aux1CurrentTime = 0, aux1StartTime = 0, aux1Pulses = 0;
#if defined(USE_AUXIN2)
volatile static uint32_t aux2CurrentTime = 0, aux2StartTime = 0, aux2Pulses = 0;
#endif
// -------------------------

Radio::Radio(void)
{
    failSafe = false;
    failSafeTimerStarted = false;

    signalLossTimeMs = 0;
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
#if defined(USE_AUXIN2)
    // Auxiliary switch 2 setup
    pinMode(AUX2PIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(AUX2PIN_INT, CHANGE);
#endif
}

void Radio::processInput(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        setPWM(aileronPulses, CHANNEL::ROLL);
        setPWM(elevatorPulses, CHANNEL::PITCH);
        setPWM(rudderPulses, CHANNEL::YAW);
        setPWM(aux1Pulses, CHANNEL::AUX1);
#if defined(USE_AUXIN2)
        setPWM(aux2Pulses, CHANNEL::AUX2);
#endif
    }

    FailSafe();
}

void Radio::setPWM(uint32_t pulse, CHANNEL ch)
{
    if (pulse < RX_PWM_MIN || pulse > RX_PWM_MAX)
        return;

    raw[ch] = static_cast<int16_t>(pulse);

    lastValidRxTimeMs[ch] = millis();
}

/**
 * Only roll, pitch and yaw channels are monitored for a failsafe
 * Rx should be configured to set rpy channels to max on signal loss
 */
void Radio::FailSafe()
{
    const uint32_t now = millis();

    const uint8_t req = requiredChannels(config().airframeConfig.type);

    bool timeout = false;
    bool rxFailsafe = true;

    for (uint8_t i = 0; i < 3; ++i)
    {
        const uint8_t mask = 1 << i;

        // Skip iteration if rx channel is not required for this airframe
        if (!(req & mask))
            continue;

        // Only one channel is required to trigger a timeout
        timeout |= (now - lastValidRxTimeMs[i]) >= RX_TIMEOUT_MS;

        // All channels are required to trigger a failsafe
        rxFailsafe &=
            abs(RX_FAILSAFE_PWM - raw[i]) <= RX_FAILSAFE_TOLERANCE;
    }

    const bool signalLost = timeout || rxFailsafe;

    if (!signalLost)
    {
        failSafe = false;
        failSafeTimerStarted = false;
        return;
    }

    if (!failSafeTimerStarted)
    {
        signalLossTimeMs = now;
        failSafeTimerStarted = true;
        return;
    }

    if (now - signalLossTimeMs >= 2000)
        failSafe = true;
}

/*
 * ISR
 * RC receivers are designed to send a 1000us-2000us pulse to the servos every 20ms - 22ms, going HIGH for the duration of the pulse and LOW otherwise
 * The receiver PWM output is used to drive a pin change interrupt routine
 * The ISR simply records the time between the pulses.
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

#if defined(USE_AUXIN2)
void PinChangeInterruptEvent(AUX2PIN_INT)(void)
{
    aux2CurrentTime = micros();
    aux2Pulses = aux2CurrentTime - aux2StartTime;
    aux2StartTime = aux2CurrentTime;
}
#endif
// ----------------------------

Radio radio;