#include "Actuators.h"
#include "config.h"

Actuators::Actuators(void)
{
}

void Actuators::init(void)
{
    // Set up output servos
    aileron1Servo.attach(AILPIN1_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    aileron2Servo.attach(AILPIN2_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
    elevatorServo.attach(ELEVPIN_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
    rudderServo.attach(RUDDPIN_OUTPUT, SERVO_MID_PWM, SERVO_MAX_PWM);
}

void Actuators::writeServo(ControlSurface surface, int16_t value)
{
    switch (surface)
    {
    case ControlSurface::AILERON1:
        aileron1Servo.writeMicroseconds(value);
        break;

    case ControlSurface::AILERON2:
        aileron2Servo.writeMicroseconds(value);
        break;

    case ControlSurface::ELEVATOR:
        elevatorServo.writeMicroseconds(value);
        break;

    case ControlSurface::RUDDER:
        rudderServo.writeMicroseconds(value);
        break;
    default:
        break;
    }
}

Actuators actuators;