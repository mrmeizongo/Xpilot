#include "Mode.h"

void PassthroughMode::process(void)
{
    if (!radio.inFailsafe())
    {
        Mode::rollOut = SETINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_PASS_THROUGH, 0, MAX_PASS_THROUGH);
        Mode::pitchOut = SETINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_PASS_THROUGH, 0, MAX_PASS_THROUGH);
        Mode::yawOut = SETINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -MAX_PASS_THROUGH, 0, MAX_PASS_THROUGH);
    }
    else
        controlFailsafe();
}

void PassthroughMode::run(void)
{
    process();
#if defined(RUDDER_MIX_IN_PASS)
    Mode::rudderMixer();
#endif
    Mode::planeMixer(Mode::rollOut, Mode::pitchOut, Mode::yawOut);
    Mode::SRVout[Actuators::Channel::AILERON1] = constrain(Mode::SRVout[Actuators::Channel::AILERON1], -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
    Mode::SRVout[Actuators::Channel::AILERON2] = constrain(Mode::SRVout[Actuators::Channel::AILERON2], -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
    Mode::SRVout[Actuators::Channel::ELEVATOR] = constrain(Mode::SRVout[Actuators::Channel::ELEVATOR], -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
    Mode::SRVout[Actuators::Channel::RUDDER] = constrain(Mode::SRVout[Actuators::Channel::RUDDER], -MAX_PASS_THROUGH, MAX_PASS_THROUGH);

    Mode::SRVout[Actuators::Channel::AILERON1] = map(Mode::SRVout[Actuators::Channel::AILERON1], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::AILERON2] = map(Mode::SRVout[Actuators::Channel::AILERON2], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::ELEVATOR] = map(Mode::SRVout[Actuators::Channel::ELEVATOR], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::RUDDER] = map(Mode::SRVout[Actuators::Channel::RUDDER], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::setServoOut();
}

void PassthroughMode::controlFailsafe(void)
{
    Mode::rollOut = SERVO_MID_PWM;
    Mode::pitchOut = SERVO_MID_PWM;
    Mode::yawOut = SERVO_MID_PWM;
}