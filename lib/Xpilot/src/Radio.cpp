#include <Arduino.h>
#include <util/atomic.h>
#include <PinChangeInterrupt.h>
#include <SystemConfig.h>
#include "Radio.h"

volatile static unsigned long aileronCurrentTime, aileronStartTime, aileronPulses = 0;
volatile static unsigned long elevatorCurrentTime, elevatorStartTime, elevatorPulses = 0;
volatile static unsigned long rudderCurrentTime, rudderStartTime, rudderPulses = 0;
volatile static unsigned long aux1CurrentTime, aux1StartTime, aux1Pulses = 0;
#if defined(USE_AUX2)
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
    pinMode(AUXPIN1_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(AUXPIN1_INT, CHANGE);
#if defined(USE_AUX2)
    // Auxiliary switch 2 setup
    pinMode(AUXPIN2_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(AUXPIN2_INT, CHANGE);
#endif
#if defined(USE_AUX3)
    // Auxiliary switch 3 setup
    pinMode(AUXPIN3_INPUT, INPUT_PULLUP);
    attachPinChangeInterrupt(AUXPIN3_INT, CHANGE);
#endif

    failsafe = false;      // Initialize failsafe state
    failsafeStartTime = 0; // Initialize failsafe start time
}

void Radio::processInput(void)
{
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        // Record the length of the pulse if it is within tx/rx range (pulse is in uS)
        if (aux1Pulses >= INPUT_MIN_PWM && aux1Pulses <= INPUT_MAX_PWM)
        {
            if (aux1Pulses >= INPUT_MAX_PWM - INPUT_SEPARATOR)
                rx.aux1SwitchPos = THREE_POS_SW::HIGH_POS;
            else if (aux1Pulses <= INPUT_MIN_PWM + INPUT_SEPARATOR)
                rx.aux1SwitchPos = THREE_POS_SW::LOW_POS;
            else
                rx.aux1SwitchPos = THREE_POS_SW::MID_POS;
        }
#if defined(USE_AUX2)
        if (aux2Pulses >= INPUT_MIN_PWM && aux2Pulses <= INPUT_MAX_PWM)
        {
            if (aux2Pulses >= INPUT_MAX_PWM - INPUT_SEPARATOR)
                rx.aux2SwitchPos = THREE_POS_SW::HIGH_POS;
            else if (aux2Pulses <= INPUT_MIN_PWM + INPUT_SEPARATOR)
                rx.aux2SwitchPos = THREE_POS_SW::LOW_POS;
            else
                rx.aux2SwitchPos = THREE_POS_SW::MID_POS;
        }
#endif
#if defined(USE_AUX3)
        if (aux3Pulses >= INPUT_MIN_PWM && aux3Pulses <= INPUT_MAX_PWM)
        {
            if (aux3Pulses >= INPUT_MAX_PWM - INPUT_SEPARATOR)
                rx.aux3SwitchPos = THREE_POS_SW::HIGH_POS;
            else if (aux3Pulses <= INPUT_MIN_PWM + INPUT_SEPARATOR)
                rx.aux3SwitchPos = THREE_POS_SW::LOW_POS;
            else
                rx.aux3SwitchPos = THREE_POS_SW::MID_POS;
        }
#endif

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
    // During binding, I set up my transmitter's failsafe position to be the maximum value for roll, pitch and yaw
    // Yours might be different, so adjust the values accordingly
    // The failsafe logic is dependent on the receiver's behavior when the signal is lost
    bool isFailsafe = false;
#if defined(FULL_PLANE_TRADITIONAL) || defined(FULL_PLANE_V_TAIL) || defined(FLYING_WING_W_RUDDER)
    isFailsafe = (abs(INPUT_MAX_PWM - rx.rollPWM) <= FAILSAFE_TOLERANCE) &&
                 (abs(INPUT_MAX_PWM - rx.pitchPWM) <= FAILSAFE_TOLERANCE) &&
                 (abs(INPUT_MAX_PWM - rx.yawPWM) <= FAILSAFE_TOLERANCE);
#elif defined(RUDDER_ELEVATOR_ONLY_V_TAIL) || defined(RUDDER_ELEVATOR_ONLY_PLANE)
    isFailsafe = (abs(INPUT_MAX_PWM - rx.pitchPWM) <= FAILSAFE_TOLERANCE) &&
                 (abs(INPUT_MAX_PWM - rx.yawPWM) <= FAILSAFE_TOLERANCE);
#elif defined(FLYING_WING_NO_RUDDER) || defined(AILERON_ELEVATOR_ONLY)
    isFailsafe = (abs(INPUT_MAX_PWM - rx.rollPWM) <= FAILSAFE_TOLERANCE) &&
                 (abs(INPUT_MAX_PWM - rx.pitchPWM) <= FAILSAFE_TOLERANCE);
#endif
    if (isFailsafe)
    {
        if (failsafeStartTime == 0)
            failsafeStartTime = millis();                        // Start the failsafe timer
        if (millis() - failsafeStartTime >= FAILSAFE_TIMEOUT_MS) // Trigger failsafe condition after timeout
            failsafe = true;                                     // Failsafe condition met
    }
    else
    {
        failsafe = false;      // Reset failsafe condition
        failsafeStartTime = 0; // Reset failsafe timer
    }
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

void PinChangeInterruptEvent(AUXPIN1_INT)(void)
{
    aux1CurrentTime = micros();
    aux1Pulses = aux1CurrentTime - aux1StartTime;
    aux1StartTime = aux1CurrentTime;
}

#if defined(USE_AUX2)
void PinChangeInterruptEvent(AUXPIN2_INT)(void)
{
    aux2CurrentTime = micros();
    aux2Pulses = aux2CurrentTime - aux2StartTime;
    aux2StartTime = aux2CurrentTime;
}
#endif

#if defined(USE_AUX3)
void PinChangeInterruptEvent(AUXPIN3_INT)(void)
{
    aux3CurrentTime = micros();
    aux3Pulses = aux3CurrentTime - aux3StartTime;
    aux3StartTime = aux3CurrentTime;
}
#endif
// ----------------------------

Radio radio;