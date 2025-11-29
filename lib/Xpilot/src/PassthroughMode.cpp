#include "Mode.h"
#include <LowpassFilter.h>

#if defined(USE_FILTER_IN_PT)
static FirstOrderLPF<int16_t> rollLPF{PT_LPF_FREQ};
static FirstOrderLPF<int16_t> pitchLPF{PT_LPF_FREQ};
static FirstOrderLPF<int16_t> yawLPF{PT_LPF_FREQ};
#endif

void PassthroughMode::enter(void)
{
#if defined(USE_FILTER_IN_PT)
    // Reset LPF states
    rollLPF.Reset();
    pitchLPF.Reset();
    yawLPF.Reset();
#endif
}

void PassthroughMode::process(void)
{
    if (!radio.inFailsafe())
    {
        Mode::rollOut = GETFILTEREDINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
        Mode::pitchOut = GETFILTEREDINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
        Mode::yawOut = GETFILTEREDINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
#if defined(USE_FLAPERONS)
        Mode::flaperonOut = GETRAWINPUT(radio.getRxAux2PWM(), 0, FLAPERON_RANGE);
#endif
    }
    else
        Mode::controlFailsafe();
}

void PassthroughMode::run(void)
{
    process();

#if defined(USE_FILTER_IN_PT)
    int16_t roll = rollLPF.Process(Mode::rollOut, LPF_DT);
    int16_t pitch = pitchLPF.Process(Mode::pitchOut, LPF_DT);
    int16_t yaw = yawLPF.Process(Mode::yawOut, LPF_DT);
    Mode::planeMixer(roll, pitch, yaw);
#else
    Mode::planeMixer(Mode::rollOut, Mode::pitchOut, Mode::yawOut);
#endif

    Mode::SRVout[Actuators::Channel::CH1] = map(Mode::SRVout[Actuators::Channel::CH1], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH2] = map(Mode::SRVout[Actuators::Channel::CH2], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH3] = map(Mode::SRVout[Actuators::Channel::CH3], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH4] = map(Mode::SRVout[Actuators::Channel::CH4], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
#if defined(USE_FLAPERONS)
    Mode::setFlaperons();
#endif
    Mode::setServoOut();
}