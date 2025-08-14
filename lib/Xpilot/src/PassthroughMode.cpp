#include "Mode.h"
#include <LowpassFilter.h>

static SecondOrderLPF<int16_t> rollLPF{PT_LPF_FREQ};
static SecondOrderLPF<int16_t> pitchLPF{PT_LPF_FREQ};
static SecondOrderLPF<int16_t> yawLPF{PT_LPF_FREQ};
static unsigned long previousTime = 0;

void PassthroughMode::process(void)
{
    if (!radio.inFailsafe())
    {
        Mode::rollOut = GETINPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
        Mode::pitchOut = GETINPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
        Mode::yawOut = GETINPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND, -MAX_PASS_THROUGH, MAX_PASS_THROUGH);
#if defined(USE_FLAPERONS)
        Mode::flaperonOut = GETRAWINPUT(radio.getRxAux2PWM(), SERVO_MID_PWM, SERVO_MAX_PWM, 0, FLAPERON_RANGE);
#endif
    }
    else
        Mode::controlFailsafe();
}

void PassthroughMode::run(void)
{
    process();
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - previousTime;
    float deltaTime = (float)dt * 0.001f;
    previousTime = currentTime;

    int16_t roll = rollLPF.Process(Mode::rollOut, deltaTime);
    int16_t pitch = pitchLPF.Process(Mode::pitchOut, deltaTime);
    int16_t yaw = yawLPF.Process(Mode::yawOut, deltaTime);

    Mode::planeMixer(roll, pitch, yaw);
    Mode::SRVout[Actuators::Channel::CH1] = map(Mode::SRVout[Actuators::Channel::CH1], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH2] = map(Mode::SRVout[Actuators::Channel::CH2], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH3] = map(Mode::SRVout[Actuators::Channel::CH3], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH4] = map(Mode::SRVout[Actuators::Channel::CH4], -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
#if defined(USE_FLAPERONS)
    Mode::setFlaperons();
#endif
    Mode::setServoOut();
}