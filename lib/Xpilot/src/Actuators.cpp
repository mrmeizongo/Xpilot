#include "Actuators.h"

Actuators::Actuators(void)
{
}

void Actuators::init(void)
{
    // Set up output servos
    controlServo[AILERON1].attach(AILPIN1_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    controlServo[AILERON2].attach(AILPIN2_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
    controlServo[ELEVATOR].attach(ELEVPIN_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
    controlServo[RUDDER].attach(RUDDPIN_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
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