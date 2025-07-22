#include <SystemConfig.h>
#include <PlaneConfig.h>
#include "Actuators.h"

Actuators::Actuators(void)
{
}

void Actuators::init(void)
{
    // Set up output servos
    controlServo[CH1].attach(AIL1PIN_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    controlServo[CH2].attach(AIL2PIN_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    controlServo[CH3].attach(ELEVPIN_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    controlServo[CH4].attach(RUDDPIN_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
}

void Actuators::setServoOut(const int16_t (&SRVout)[NUM_CHANNELS])
{
    channelOut[CH1] = SRVout[CH1];
    channelOut[CH2] = SRVout[CH2];
    channelOut[CH3] = SRVout[CH3];
    channelOut[CH4] = SRVout[CH4];
}

void Actuators::setServoOut(Actuators::Channel ch, int16_t value)
{
    if (ch < 0 || ch >= NUM_CHANNELS)
        return;

    channelOut[ch] = value;
}

int16_t Actuators::getServoOut(Actuators::Channel ch)
{
    if (ch < 0 || ch >= NUM_CHANNELS)
        return -1;

    return channelOut[ch];
}

void Actuators::writeServos(void)
{
    controlServo[CH1].writeMicroseconds(channelOut[CH1]);
    controlServo[CH2].writeMicroseconds(channelOut[CH2]);
    controlServo[CH3].writeMicroseconds(channelOut[CH3]);
    controlServo[CH4].writeMicroseconds(channelOut[CH4]);
}

Actuators actuators;