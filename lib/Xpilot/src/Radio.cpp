#include <Arduino.h>
#include <util/atomic.h>
#include "GPIOConfig.h"
#include "Radio.h"
#include "PinChangeInterrupt.h"

volatile static uint32_t throttleRiseTimeUs = 0, aileronRiseTimeUs = 0, elevatorRiseTimeUs = 0, rudderRiseTimeUs = 0,
                         aux1RiseTimeUs = 0;
volatile static uint16_t throttlePulseUs = 0, aileronPulseUs = 0, elevatorPulseUs = 0, rudderPulseUs = 0, aux1PulseUs = 0;

#if defined(USE_AUX2IN)
volatile static uint32_t aux2RiseTimeUs = 0;
volatile static uint16_t aux2PulseUs = 0;
#endif

volatile static uint32_t lastValidTimeUs[Radio::CHANNEL::CHANNEL_COUNT];
// -------------------------

Radio::Radio(void)
{
    failSafe = false;
    failSafeTimerStarted = false;

    signalLossTimeUs = 0;
}

void Radio::init(void)
{
    // All input pins use pin change interrupts
    // Throttle setup
    pinMode(THROTTLEPIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(THROTTLEPIN_INT, CHANGE);
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
#if defined(USE_AUX2IN)
    // Auxiliary switch 2 setup
    pinMode(AUX2PIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(AUX2PIN_INT, CHANGE);
#endif
}

void Radio::processInput()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        setPWM(CHANNEL::THROTTLE, throttlePulseUs, lastValidTimeUs[CHANNEL::THROTTLE]);
        setPWM(CHANNEL::ROLL, aileronPulseUs, lastValidTimeUs[CHANNEL::ROLL]);
        setPWM(CHANNEL::PITCH, elevatorPulseUs, lastValidTimeUs[CHANNEL::PITCH]);
        setPWM(CHANNEL::YAW, rudderPulseUs, lastValidTimeUs[CHANNEL::YAW]);
        setPWM(CHANNEL::AUX1, aux1PulseUs, lastValidTimeUs[CHANNEL::AUX1]);

#if defined(USE_AUX2IN)
        setPWM(CHANNEL::AUX2, aux2PulseUs, lastValidTimeUs[CHANNEL::AUX2]);
#endif
    }

    FailSafe();
}

uint8_t Radio::requiredChannels()
{
    switch (config().airframeConfig.type)
    {
        case Config::AirframeType::CONVENTIONAL:
        case Config::AirframeType::V_TAIL:
        case Config::AirframeType::FLYING_WING_RUDDER:
        case Config::AirframeType::CUSTOM:
            return REQ_THROTTLE | REQ_ROLL | REQ_PITCH | REQ_YAW;

        case Config::AirframeType::FLYING_WING_NO_RUDDER:
        case Config::AirframeType::AILERON_ELEVATOR:
            return REQ_THROTTLE | REQ_ROLL | REQ_PITCH;

        case Config::AirframeType::RUDDER_ELEVATOR:
            return REQ_THROTTLE | REQ_PITCH | REQ_YAW;

        default:
            return NONE;
    }
}

/**
 * Only roll, pitch and yaw channels are monitored for a failsafe
 * Rx should be configured to set rpy channels to max on signal loss
 */
void Radio::FailSafe()
{
    const uint32_t now = micros();

    const uint8_t req = requiredChannels();

    bool timeout = false;
    bool rxFailsafe = false;

    // Only checking throttle, roll, pitch and yaw channels
    for (uint8_t i = 0; i < 4; ++i)
    {
        const uint8_t mask = 1 << i;

        // Skip iteration if rx channel is not required for this airframe
        if (!(req & mask))
            continue;

        // On stale or invalid input, any one of the 4 control channels can trigger a timeout failsafe
        timeout |= (now - lastValidRxTimeUs[i]) >= RX_TIMEOUT_US;
    }

    // During rx bind, throttle is set to a value below min(through throttle cut) to indicate loss of signal
    rxFailsafe = raw[CHANNEL::THROTTLE] < (config().throttleRxConfig.min - RX_THROTTLE_FAILSAFE_TOL);

    const bool signalLost = timeout || rxFailsafe;

    if (!signalLost)
    {
        failSafe = false;
        failSafeTimerStarted = false;
        return;
    }

    if (!failSafeTimerStarted)
    {
        signalLossTimeUs = now;
        failSafeTimerStarted = true;
        return;
    }

    if (now - signalLossTimeUs >= 2000000UL)
        failSafe = true;
}

/*
 * ISR
 * RC receivers are designed to send a 1000us-2000us pulse to the servos every 20ms - 22ms, going HIGH for the duration of the pulse and LOW otherwise
 * The receiver PWM output is used to drive a pin change interrupt service routine
 * The ISR simply records the time between the pulses.
 */
void PinChangeInterruptEvent(THROTTLEPIN_INT)(void)
{
    capturePWMEdge(THROTTLEPIN_INPUT, throttleRiseTimeUs, throttlePulseUs, lastValidTimeUs[Radio::CHANNEL::THROTTLE]);
}

void PinChangeInterruptEvent(AILPIN_INT)(void)
{
    capturePWMEdge(AILPIN_INPUT, aileronRiseTimeUs, aileronPulseUs, lastValidTimeUs[Radio::CHANNEL::ROLL]);
}

void PinChangeInterruptEvent(ELEVPIN_INT)(void)
{
    capturePWMEdge(ELEVPIN_INPUT, elevatorRiseTimeUs, elevatorPulseUs, lastValidTimeUs[Radio::CHANNEL::PITCH]);
}

void PinChangeInterruptEvent(RUDDPIN_INT)(void)
{
    capturePWMEdge(RUDDPIN_INPUT, rudderRiseTimeUs, rudderPulseUs, lastValidTimeUs[Radio::CHANNEL::YAW]);
}

void PinChangeInterruptEvent(AUX1PIN_INT)(void)
{
    capturePWMEdge(AUX1PIN_INPUT, aux1RiseTimeUs, aux1PulseUs, lastValidTimeUs[Radio::CHANNEL::AUX1]);
}

#if defined(USE_AUX2IN)
void PinChangeInterruptEvent(AUX2PIN_INT)(void)
{
    capturePWMEdge(AUX2PIN_INPUT, aux2RiseTimeUs, aux2PulseUs, lastValidTimeUs[Radio::CHANNEL::AUX2]);
}
#endif
// ----------------------------

Radio radio;