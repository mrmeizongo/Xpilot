#include "Actuators.h"
#include "config.h"

Actuators::Actuators(void)
{
}

void Actuators::init(void)
{
    // Set up output servos
    controlSurfaces[Control::AILERON1].attach(AILPIN1_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    controlSurfaces[Control::AILERON2].attach(AILPIN2_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
    controlSurfaces[Control::ELEVATOR].attach(ELEVPIN_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
    controlSurfaces[Control::RUDDER].attach(RUDDPIN_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
}

void Actuators::writeServo(Control surface, int16_t value)
{
    if (surface < 0 || surface >= MAX_CHANNELS)
        return;

    controlSurfaces[surface].writeMicroseconds(value);
}

Actuators actuators;