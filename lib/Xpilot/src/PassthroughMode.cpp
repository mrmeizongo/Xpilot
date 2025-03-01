#include "Mode.h"

void PassthroughMode::process(void)
{
    if (!radio.inFailsafe())
    {
        Mode::rollOut = SETINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM);
        Mode::pitchOut = SETINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM);
        Mode::yawOut = SETINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, SERVO_MIN_PWM, SERVO_MID_PWM, SERVO_MAX_PWM);
    }
    else
        controlFailsafe();
}

void PassthroughMode::run(void)
{
    process();
#if defined(RUDDER_MIX_IN_PASS)
    rudderMixer();
#endif
    Mode::planeMixer(Mode::rollOut, Mode::pitchOut, Mode::yawOut);
    Mode::SRVout[Actuators::Channel::AILERON1] = constrain(Mode::SRVout[Actuators::Channel::AILERON1], SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::AILERON2] = constrain(Mode::SRVout[Actuators::Channel::AILERON2], SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::ELEVATOR] = constrain(Mode::SRVout[Actuators::Channel::ELEVATOR], SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::RUDDER] = constrain(Mode::SRVout[Actuators::Channel::RUDDER], SERVO_MIN_PWM, SERVO_MAX_PWM);
}

void PassthroughMode::controlFailsafe(void)
{
    Mode::rollOut = SERVO_MID_PWM;
    Mode::pitchOut = SERVO_MID_PWM;
    Mode::yawOut = SERVO_MID_PWM;
}