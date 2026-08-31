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
    uint16_t minPulse;
    uint16_t maxPulse;

    switch (ch)
    {
    case CHANNEL::ROLL:
        minPulse = config().rollRC.min;
        maxPulse = config().rollRC.max;
        break;

    case CHANNEL::PITCH:
        minPulse = config().pitchRC.min;
        maxPulse = config().pitchRC.max;
        break;

    case CHANNEL::YAW:
        minPulse = config().yawRC.min;
        maxPulse = config().yawRC.max;
        break;

    case CHANNEL::AUX1:
#if defined(USE_AUXIN2)
    case CHANNEL::AUX2:
#endif
        minPulse = RX_PWM_MIN;
        maxPulse = RX_PWM_MAX;
        break;

    default:
        return;
    }

    if (pulse < minPulse || pulse > maxPulse)
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

    const uint8_t req = requiredChannels(config().airframeType.type);

    const int16_t rxMax[3] = {config().rollRC.max, config().pitchRC.max, config().yawRC.max};

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
            abs(rxMax[i] - raw[i]) <= RX_FAILSAFE_TOLERANCE;
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
 * Typical hobby servos expect to see a pulse every 20ms and the length of the pulse determines the position to set the servo
 * The length of the pulse is typically between 1ms - 2ms with 1ms setting the servo position to 0°, 1.5ms to 90° and 2ms to 180°
 * This gives us a servo refresh rate of 22ms(20ms interval + actual 1ms-2ms pulse duration)
 * RC transmitters are designed to send a pulse to the receiver within this range, going HIGH for the duration of the pulse and LOW otherwise
 * If the output of the channel on the receiver is attached to an interrupt pin, we can use it to drive a PCISR
 * The ISR simply records the time between the changes. We're only interested in pulses lasting between INPUT_MIN_PWM and INPUT_MAX_PWM
 * Due to this input capture mechanism, implementing a failsafe is largely dependent on the receiver's behavior when the signal is lost
 * Example: The Spektrum tx/rx I use will either hold the last known position when the signal is lost or default to a preset position determined at bind time
 * I set up my transmitter's failsafe position to be the maximum value for roll, pitch and yaw
 * Failsafe is triggered if the input pulse is at maximum value with a FAILSAFE_TOLERANCE for more than 2 seconds
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