#include <Arduino.h>
#include <util/atomic.h>
#include "GPIODef.h"
#include "Radio.h"
#include "FlightConfigAccess.h"
#include "PinChangeInterrupt.h"

volatile static uint32_t aileronCurrentTime = 0, aileronStartTime = 0, aileronPulses = 0;
volatile static uint32_t elevatorCurrentTime = 0, elevatorStartTime = 0, elevatorPulses = 0;
volatile static uint32_t rudderCurrentTime = 0, rudderStartTime = 0, rudderPulses = 0;
volatile static uint32_t aux1CurrentTime = 0, aux1StartTime = 0, aux1Pulses = 0;
#if defined(USE_AUX2)
volatile static uint32_t aux2CurrentTime = 0, aux2StartTime = 0, aux2Pulses = 0;
#endif
// -------------------------

Radio::Radio(void)
{
    failSafe = false;
    failSafeTimerStarted = false;
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
#if defined(USE_AUX2)
    // Auxiliary switch 2 setup
    pinMode(AUX2PIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(AUX2PIN_INT, CHANGE);
#endif
}

void Radio::processInput(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        setPWM(raw[CHANNELS::ROLL], aileronPulses, CHANNELS::ROLL);
        setPWM(raw[CHANNELS::PITCH], elevatorPulses, CHANNELS::PITCH);
        setPWM(raw[CHANNELS::YAW], rudderPulses, CHANNELS::YAW);
        setPWM(raw[CHANNELS::AUX1], aux1Pulses, CHANNELS::AUX1);
#if defined(USE_AUX2)
        setPWM(raw[CHANNELS::AUX2], aux2Pulses, CHANNELS::AUX2);
#endif
    }

    FailSafe();
}

void Radio::setPWM(int16_t &dest, uint32_t pulse, CHANNELS ch)
{
    auto setIfValid = [&dest, pulse](uint16_t minPulse, uint16_t maxPulse)
    {
        if (pulse >= minPulse && pulse <= maxPulse)
            dest = static_cast<int16_t>(pulse);
    };

    switch (ch)
    {
    case CHANNELS::ROLL:
        setIfValid(config().rollRC.min, config().rollRC.max);
        break;

    case CHANNELS::PITCH:
        setIfValid(config().pitchRC.min, config().pitchRC.max);
        break;

    case CHANNELS::YAW:
        setIfValid(config().yawRC.min, config().yawRC.max);
        break;

    case CHANNELS::AUX1:
#if defined(USE_AUX2)
    case CHANNELS::AUX2:
#endif
        setIfValid(RX_PWM_MIN, RX_PWM_MAX);
        break;

    default:
        break;
    }
}

/**
 * Failsafe logic is highly user/system peculiar, modify test logic accordingly
 * The implemented failsafe assumes roll, pitch and yaw go to maximum on signal loss
 * Failsafe is triggered after 2 seconds if signal is not recovered
 */
void Radio::FailSafe()
{
    uint32_t now = millis();

    const bool noRecentSignal = (now - lastValidRxTimeMs) >= RX_TIMEOUT_MS;

    // Check transmitter failsafe position i.e. max for roll, pitch and yaw
    const bool rxFailsafePosition = (abs(RX_PWM_MAX - raw[CHANNELS::ROLL]) <= RX_FAILSAFE_TOLERANCE) &&
                                    (abs(RX_PWM_MAX - raw[CHANNELS::PITCH]) <= RX_FAILSAFE_TOLERANCE) &&
                                    (abs(RX_PWM_MAX - raw[CHANNELS::YAW]) <= RX_FAILSAFE_TOLERANCE);

    const bool signalLost = noRecentSignal || rxFailsafePosition;

    if (!signalLost)
    {
        failSafe = false;
        failSafeTimerStarted = false;
        lastValidRxTimeMs = now;
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

#if defined(USE_AUX2)
void PinChangeInterruptEvent(AUX2PIN_INT)(void)
{
    aux2CurrentTime = micros();
    aux2Pulses = aux2CurrentTime - aux2StartTime;
    aux2StartTime = aux2CurrentTime;
}
#endif
// ----------------------------

Radio radio;