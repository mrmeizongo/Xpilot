#include "Mode.h"

void PassthroughMode::process(void)
{
    if (!radio.inFailsafe())
    {
        rollOut = SETINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, SERVO_MIN_PWM, SERVO_MAX_PWM);
        pitchOut = SETINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, SERVO_MIN_PWM, SERVO_MAX_PWM);
        yawOut = SETINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, SERVO_MIN_PWM, SERVO_MAX_PWM);
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
    planeMixer(rollOut, pitchOut, yawOut);
    SRVout[Actuators::Channel::AILERON1] = constrain(SRVout[Actuators::Channel::AILERON1], SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::AILERON2] = constrain(SRVout[Actuators::Channel::AILERON2], SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::ELEVATOR] = constrain(SRVout[Actuators::Channel::ELEVATOR], SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::RUDDER] = constrain(SRVout[Actuators::Channel::RUDDER], SERVO_MIN_PWM, SERVO_MAX_PWM);
}