#include "Mode.h"
#include <LowpassFilter.h>

#if defined(USE_FILTER_IN_PT)
static FirstOrderLPF<int16_t> rollLPF{PASSTHROUGH_LPF_FREQ, LPF_DT};
static FirstOrderLPF<int16_t> pitchLPF{PASSTHROUGH_LPF_FREQ, LPF_DT};
static FirstOrderLPF<int16_t> yawLPF{PASSTHROUGH_LPF_FREQ, LPF_DT};
#endif

void PassthroughMode::enter(void)
{
#if defined(USE_FILTER_IN_PT)
    Mode::resetControllers();
#endif
}

void PassthroughMode::process(void)
{
    if (radio.inFailsafe())
    {
        Mode::controlFailsafe();
        return;
    }

    Mode::rollOut = GETFILTEREDINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
    Mode::pitchOut = GETFILTEREDINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
    Mode::yawOut = GETFILTEREDINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
#if defined(USE_FLAPERONS)
    flaperonInput();
#endif
}

void PassthroughMode::run(void)
{
    process();

#if defined(USE_FILTER_IN_PT)
    int16_t roll = rollLPF.Process(Mode::rollOut);
    int16_t pitch = pitchLPF.Process(Mode::pitchOut);
    int16_t yaw = yawLPF.Process(Mode::yawOut);
    Mode::planeMixer(roll, pitch, yaw);
#else
    Mode::planeMixer(Mode::rollOut, Mode::pitchOut, Mode::yawOut);
#endif

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