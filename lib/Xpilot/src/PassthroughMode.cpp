#include "Mode.h"
#include <LowpassFilter.h>

void PassthroughMode::process(void)
{
    if (radio.inFailsafe())
    {
        Mode::controlFailsafe();
        return;
    }

    Mode::input_rpy[0] = GETFILTEREDINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
    Mode::input_rpy[1] = GETFILTEREDINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
    Mode::input_rpy[2] = GETFILTEREDINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
#if defined(USE_FLAPERONS)
    flaperonInput();
#endif
}

void PassthroughMode::run(void)
{
    process();

    Mode::output_rpy[0] = Mode::input_rpy[0];
    Mode::output_rpy[1] = Mode::input_rpy[1];
    Mode::output_rpy[2] = Mode::input_rpy[2];
    Mode::planeMixer(Mode::output_rpy[0], Mode::output_rpy[1], Mode::output_rpy[2]);

    Mode::SRVout[Actuators::Channel::CH1] = map(Mode::SRVout[Actuators::Channel::CH1], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH2] = map(Mode::SRVout[Actuators::Channel::CH2], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH3] = map(Mode::SRVout[Actuators::Channel::CH3], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH4] = map(Mode::SRVout[Actuators::Channel::CH4], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
#if defined(USE_AUXOUT1)
    Mode::SRVout[Actuators::Channel::CH5] = map(Mode::SRVout[Actuators::Channel::CH5], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
#endif
#if defined(USE_FLAPERONS)
    Mode::setFlaperons();
#endif
}