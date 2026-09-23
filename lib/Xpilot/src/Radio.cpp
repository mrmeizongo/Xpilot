#include <Arduino.h>
#include <util/atomic.h>
#include "GPIOConfig.h"
#include "Radio.h"
#include "PinChangeInterrupt.h"

volatile static uint32_t throttleRiseTimeUs = 0, aileronRiseTimeUs = 0, elevatorRiseTimeUs = 0, rudderRiseTimeUs = 0,
                         aux1RiseTimeUs = 0, aux2RiseTimeUs = 0;
volatile static uint16_t throttlePulseUs = 0, aileronPulseUs = 0, elevatorPulseUs = 0, rudderPulseUs = 0, aux1PulseUs = 0,
                         aux2PulseUs = 0;

volatile static uint32_t lastValidTimeUs[Radio::CHANNEL::CHANNEL_COUNT];
// -------------------------

Radio::Radio(void)
{
    failSafe = false;
    failSafeTimerStarted = false;

    signalLossTimeUs = 0;

    txThrottleCut = false;
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

    // Auxiliary switch 2 setup
    pinMode(AUX2PIN_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(AUX2PIN_INT, CHANGE);
}

void Radio::processInput()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        setRawPWM(CHANNEL::THROTTLE, throttlePulseUs, lastValidTimeUs[CHANNEL::THROTTLE]);
        setRawPWM(CHANNEL::ROLL, aileronPulseUs, lastValidTimeUs[CHANNEL::ROLL]);
        setRawPWM(CHANNEL::PITCH, elevatorPulseUs, lastValidTimeUs[CHANNEL::PITCH]);
        setRawPWM(CHANNEL::YAW, rudderPulseUs, lastValidTimeUs[CHANNEL::YAW]);
        setRawPWM(CHANNEL::AUX1, aux1PulseUs, lastValidTimeUs[CHANNEL::AUX1]);
        setRawPWM(CHANNEL::AUX2, aux2PulseUs, lastValidTimeUs[CHANNEL::AUX2]);
    }

    FailSafeDetector();
}

uint8_t Radio::requiredChannels()
{
    switch (config().airframeConfig.type)
    {
        case Config::AirframeType::CONVENTIONAL:
        case Config::AirframeType::V_TAIL:
        case Config::AirframeType::ELEVON_WITH_RUDDER:
        case Config::AirframeType::CUSTOM:
            return CHANNELMASK::REQ_ROLL | CHANNELMASK::REQ_PITCH | CHANNELMASK::REQ_YAW;

        case Config::AirframeType::ELEVON_NO_RUDDER:
        case Config::AirframeType::AILERON_ELEVATOR:
            return CHANNELMASK::REQ_ROLL | CHANNELMASK::REQ_PITCH;

        case Config::AirframeType::RUDDER_ELEVATOR:
            return CHANNELMASK::REQ_PITCH | CHANNELMASK::REQ_YAW;

        default:
            return CHANNELMASK::NONE;
    }
}

// See FAILSAFE.md for more information
Radio::THROTTLE_STATE Radio::decodeThrottleState(uint32_t timeNow)
{
    uint16_t pwm = rawPWM[CHANNEL::THROTTLE];

    if (pwm < THROTTLE_FAILSAFE_THRESHOLD)
        return THROTTLE_STATE::FAILSAFE;

    if (pwm < THROTTLE_CUT_THRESHOLD)
        return THROTTLE_STATE::CUT;

    if (timeNow - lastRawPWMTimeUS[CHANNEL::THROTTLE] >= TIMEOUT_US)
        return THROTTLE_STATE::SIGNAL_LOST;

    return THROTTLE_STATE::NORMAL;
}

// Failsafe is triggered 2 seconds after an input timeout or loss of signal
void Radio::FailSafeDetector()
{
    const uint32_t now = micros();

    const uint8_t req = requiredChannels();

    // Stale or invalid pwm input flag
    bool timeout = false;

    // Rx state
    bool rxFailsafe = false;

    // Check only roll, pitch and yaw channels for valid signals
    // This is preferred to checking all channels
    // It prevents loose wire connections from channels like throttle and the auxiliaries from initiating a failsafe
    // Airplane is still flyable in the event of a loss of those channels
    // Throttle is considered separately
    for (uint8_t i = CHANNEL::ROLL; i <= CHANNEL::YAW; ++i)
    {
        const uint8_t mask = 1 << i;

        if (!(req & mask))
            continue;

        // Any one of the 3 control channels can trigger a timeout failsafe
        timeout |= (now - lastRawPWMTimeUS[i]) >= TIMEOUT_US;
    }

    // Check throttle signal
    switch (decodeThrottleState(now))
    {
        case THROTTLE_STATE::NORMAL:
            txThrottleCut = false;
            rxFailsafe = false;
            break;

        case THROTTLE_STATE::CUT:
        case THROTTLE_STATE::SIGNAL_LOST:
            txThrottleCut = true;
            rxFailsafe = false;
            break;

        case THROTTLE_STATE::FAILSAFE:
            txThrottleCut = true;
            rxFailsafe = true;
            break;

        default:
            break;
    }

    const bool signalLost = timeout || rxFailsafe;

    if (!signalLost)
    {
        failSafe = false;
        failSafeTimerStarted = false;

        // Update all last valid pwm values
        lastValidPWM[CHANNEL::THROTTLE] = txThrottleCut ? config().throttleRxConfig.min : rawPWM[CHANNEL::THROTTLE];
        for (uint8_t i = CHANNEL::ROLL; i < CHANNEL::CHANNEL_COUNT; i++)
        {
            lastValidPWM[i] = rawPWM[i];
        }

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

Radio::THREE_POS_SW Radio::getThreeSwitchPos(CHANNEL ch, uint16_t trim, uint16_t threshold)
{
    if (ch >= CHANNEL::CHANNEL_COUNT)
        return THREE_POS_SW::UNDEFINED;

    if (rawPWM[ch] < trim - threshold)
        return THREE_POS_SW::LOW_POS;

    if (rawPWM[ch] > trim + threshold)
        return THREE_POS_SW::HIGH_POS;

    return THREE_POS_SW::MID_POS;
}

bool Radio::getValidControlPWM(uint16_t* dest, uint8_t count)
{
    if (failSafe || failSafeTimerStarted || (count > Radio::CHANNEL::CHANNEL_COUNT))
        return false;

    for (uint8_t i = 0; i < count; i++)
    {
        dest[i] = lastValidPWM[i];
    }

    return true;
}

uint16_t Radio::getPWM(CHANNEL ch)
{
    if (ch >= CHANNEL::CHANNEL_COUNT)
        return 0;

    return lastValidPWM[ch];
}

void Radio::setRawPWM(CHANNEL ch, const volatile uint16_t& rawPulse, const volatile uint32_t& validTime)
{
    rawPWM[ch] = rawPulse;
    lastRawPWMTimeUS[ch] = validTime;
}

/*
 * ISR
 * Typical RC receivers are designed to send a 1ms-2ms pulse to the servos every 20ms, going HIGH for the duration of the pulse and LOW otherwise
 * The receiver pulse output is used to drive a pin change interrupt service routine
 * The ISR simply records the time between the pulses and store the pulses that fall within normal PWM range
 */

inline void
capturePWMEdge(uint8_t pin, volatile uint32_t& riseTimeUs, volatile uint16_t& pulseUs, volatile uint32_t& lastValid)
{
    const uint32_t now = micros();

    if (PIN_HIGH(pin))
    {
        riseTimeUs = now;
        return;
    }

    const uint16_t rawPulse = static_cast<uint16_t>(now - riseTimeUs);

    if (rawPulse >= PWM_MIN_US && rawPulse <= PWM_MAX_US)
    {
        pulseUs = rawPulse;
        lastValid = now;
    }
}

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

void PinChangeInterruptEvent(AUX2PIN_INT)(void)
{
    capturePWMEdge(AUX2PIN_INPUT, aux2RiseTimeUs, aux2PulseUs, lastValidTimeUs[Radio::CHANNEL::AUX2]);
}
// ----------------------------

Radio radio;