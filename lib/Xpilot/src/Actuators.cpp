#include <BoardConfig.h>
#include <PlaneConfig.h>
#include "Actuators.h"

Actuators::Actuators(void)
{
}

void Actuators::init(void)
{
    // Set up output servos
    for (uint8_t channel = AILERON1; channel < NUM_CHANNELS; channel++)
        controlServo[channel].attach(channel, SERVO_MIN_PWM, SERVO_MAX_PWM);
}

void Actuators::setServoOut(const int16_t (&SRVout)[NUM_CHANNELS])
{
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        channelOut[i] = SRVout[i];
    }
}

int16_t Actuators::getServoOut(Channel channel)
{
    if (channel < 0 || channel >= NUM_CHANNELS)
        return -1;

    return channelOut[channel];
}

void Actuators::writeServos(void)
{
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        controlServo[i].writeMicroseconds(channelOut[i]);
    }
}

Actuators actuators;