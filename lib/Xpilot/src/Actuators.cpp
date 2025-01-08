#include <BoardConfig.h>
#include <PlaneConfig.h>
#include "Actuators.h"

Actuators::Actuators(void)
{
}

void Actuators::init(void)
{
    // Set up output servos
    // This assumes consecutive numbering of channels and output pins
    for (uint8_t ch = AILERON1, pin = AILPIN1_OUTPUT; ch < NUM_CHANNELS; ch++, pin++)
    {
        controlServo[ch].attach(pin, SERVO_MIN_PWM, SERVO_MAX_PWM);
    }
}

void Actuators::setServoOut(const int16_t (&SRVout)[NUM_CHANNELS])
{
    for (uint8_t ch = AILERON1; ch < NUM_CHANNELS; ch++)
    {
        channelOut[ch] = SRVout[ch];
    }
}

void Actuators::setServoOut(Channel ch, int16_t value)
{
    if (ch < AILERON1 || ch >= NUM_CHANNELS)
        return;

    channelOut[ch] = value;
}

int16_t Actuators::getServoOut(Channel ch)
{
    if (ch < AILERON1 || ch >= NUM_CHANNELS)
        return -1;

    return channelOut[ch];
}

void Actuators::writeServos(void)
{
    for (uint8_t ch = AILERON1; ch < NUM_CHANNELS; ch++)
    {
        controlServo[ch].writeMicroseconds(channelOut[ch]);
    }
}

Actuators actuators;