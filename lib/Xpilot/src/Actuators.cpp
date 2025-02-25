#include <BoardConfig.h>
#include <PlaneConfig.h>
#include "Actuators.h"

Actuators::Actuators(void)
{
}

void Actuators::init(void)
{
    // Set up output servos
    controlServo[AILERON1].attach(AILPIN1_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    controlServo[AILERON2].attach(AILPIN2_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    controlServo[ELEVATOR].attach(ELEVPIN_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    controlServo[RUDDER].attach(RUDDPIN_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
}

void Actuators::setServoOut(const int16_t (&SRVout)[NUM_CHANNELS])
{
    channelOut[AILERON1] = SRVout[AILERON1];
    channelOut[AILERON2] = SRVout[AILERON2];
    channelOut[ELEVATOR] = SRVout[ELEVATOR];
    channelOut[RUDDER] = SRVout[RUDDER];
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
    controlServo[AILERON1].writeMicroseconds(channelOut[AILERON1]);
    controlServo[AILERON2].writeMicroseconds(channelOut[AILERON2]);
    controlServo[ELEVATOR].writeMicroseconds(channelOut[ELEVATOR]);
    controlServo[RUDDER].writeMicroseconds(channelOut[RUDDER]);
}

Actuators actuators;