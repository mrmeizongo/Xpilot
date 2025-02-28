#include "Mode.h"

void PassthroughMode::process(void)
{
    if (!radio.inFailsafe())
    {
        rollInput = SETINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        pitchInput = SETINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
        yawInput = SETINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, INPUT_MIN_PWM, INPUT_MID_PWM, INPUT_MAX_PWM, -PASSTHROUGH_RES, PASSTHROUGH_RES);
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
    planeMixer(rollInput, pitchInput, yawInput);
    SRVout[Actuators::Channel::AILERON1] = constrain(SRVout[Actuators::Channel::AILERON1], -PASSTHROUGH_RES, PASSTHROUGH_RES);
    SRVout[Actuators::Channel::AILERON2] = constrain(SRVout[Actuators::Channel::AILERON2], -PASSTHROUGH_RES, PASSTHROUGH_RES);
    SRVout[Actuators::Channel::ELEVATOR] = constrain(SRVout[Actuators::Channel::ELEVATOR], -PASSTHROUGH_RES, PASSTHROUGH_RES);
    SRVout[Actuators::Channel::RUDDER] = constrain(SRVout[Actuators::Channel::RUDDER], -PASSTHROUGH_RES, PASSTHROUGH_RES);

    SRVout[Actuators::Channel::AILERON1] = map(SRVout[Actuators::Channel::AILERON1], -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::AILERON2] = map(SRVout[Actuators::Channel::AILERON2], -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::ELEVATOR] = map(SRVout[Actuators::Channel::ELEVATOR], -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
    SRVout[Actuators::Channel::RUDDER] = map(SRVout[Actuators::Channel::RUDDER], -PASSTHROUGH_RES, PASSTHROUGH_RES, SERVO_MIN_PWM, SERVO_MAX_PWM);
}