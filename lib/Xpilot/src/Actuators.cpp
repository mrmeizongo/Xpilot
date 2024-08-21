#include "Actuators.h"
#include "config.h"

Actuators::Actuators(void)
{
}

void Actuators::init(void)
{
    // Set up output servos
    controlServo[Control::AILERON1].attach(AILPIN1_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    controlServo[Control::AILERON2].attach(AILPIN2_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
    controlServo[Control::ELEVATOR].attach(ELEVPIN_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
    controlServo[Control::RUDDER].attach(RUDDPIN_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
}

void Actuators::setServoOut(const int16_t (&SRVout)[MAX_SERVO_CHANNELS])
{
    for (uint8_t i = 0; i < MAX_SERVO_CHANNELS; i++)
    {
        channelOut[i] = SRVout[i];
    }
}

int16_t Actuators::getServoOut(Control control)
{
    if (control < 0 || control >= MAX_SERVO_CHANNELS)
        return -1;

    return channelOut[control];
}

void Actuators::writeServos(void)
{
    for (uint8_t i = 0; i < MAX_SERVO_CHANNELS; i++)
    {
        controlServo[i].writeMicroseconds(channelOut[i]);
    }
}

Actuators actuators;