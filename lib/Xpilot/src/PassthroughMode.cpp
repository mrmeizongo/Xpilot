#include "Mode.h"
#include "SlewRateLimiter.h"

static SlewRateLimiter<int16_t> rollSlew(PT_SLEW_RATE, PROCESS_DT);
static SlewRateLimiter<int16_t> pitchSlew(PT_SLEW_RATE, PROCESS_DT);
static SlewRateLimiter<int16_t> yawSlew(PT_SLEW_RATE, PROCESS_DT);

void PassthroughMode::enter(void)
{
    rollSlew.reset();
    pitchSlew.reset();
    yawSlew.reset();
    Mode::airplaneMixer.setCommandLimit(MAX_PASS_THROUGH);
}

void PassthroughMode::update(void)
{
    if (radio.inFailsafe())
    {
        Mode::controlFailsafe();
        return;
    }

    Mode::input_rpy[0] = FILTERED_NORM_INPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND) * MAX_PASS_THROUGH;
    Mode::input_rpy[1] = FILTERED_NORM_INPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND) * MAX_PASS_THROUGH;
    Mode::input_rpy[2] = FILTERED_NORM_INPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND) * MAX_PASS_THROUGH;

    Mode::update();
}

void PassthroughMode::run(void)
{
    // update();

    Mode::output_rpy[0] = rollSlew.update(Mode::input_rpy[0]);
    Mode::output_rpy[1] = pitchSlew.update(Mode::input_rpy[1]);
    Mode::output_rpy[2] = yawSlew.update(Mode::input_rpy[2]);

    AirplaneMixer::Outputs outputs = airplaneMixer.mix(Mode::output_rpy[0], Mode::output_rpy[1], Mode::output_rpy[2]);

    Mode::SRVout[Actuators::Channel::CH1] = map(outputs.leftAileron, -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH2] = map(outputs.rightAileron, -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH3] = map(outputs.elevator, -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH4] = map(outputs.rudder, -MAX_PASS_THROUGH, MAX_PASS_THROUGH, SERVO_MIN_PWM, SERVO_MAX_PWM);
#if defined(USE_FLAPERONS)
    Mode::setFlaperons();
#endif
    actuators.setServoOut(SRVout); // Set servo output
}