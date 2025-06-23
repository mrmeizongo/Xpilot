#include "Mode.h"

void PassthroughMode::process(void)
{
    if (!radio.inFailsafe())
    {
        Mode::rollOut = GETINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
        Mode::pitchOut = GETINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
        Mode::yawOut = GETINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
    }
    else
        Mode::controlFailsafe();
}

void PassthroughMode::run(void)
{
    process();

    Mode::planeMixer(Mode::rollOut, Mode::pitchOut, Mode::yawOut);
    Mode::SRVout[Actuators::Channel::AILERON1] = map(Mode::SRVout[Actuators::Channel::AILERON1], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::AILERON2] = map(Mode::SRVout[Actuators::Channel::AILERON2], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::ELEVATOR] = map(Mode::SRVout[Actuators::Channel::ELEVATOR], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::RUDDER] = map(Mode::SRVout[Actuators::Channel::RUDDER], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::setServoOut();
}