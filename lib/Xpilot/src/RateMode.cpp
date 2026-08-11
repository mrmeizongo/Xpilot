#include "Mode.h"
#include "IMU.h"

void RateMode::enter(void)
{
    Mode::resetControllers();
    Mode::airplaneMixer.setCommandLimit(MAX_PID_OUTPUT);
}

void RateMode::process(void)
{
    if (radio.inFailsafe())
    {
        Mode::controlFailsafe();
        return;
    }

    Mode::input_rpy[0] = FILTERED_NORM_INPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND) * MAX_ROLL_RATE_DEGS;
    Mode::input_rpy[1] = FILTERED_NORM_INPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND) * MAX_PITCH_RATE_DEGS;
    Mode::input_rpy[2] = FILTERED_NORM_INPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND) * MAX_YAW_RATE_DEGS;
#if defined(USE_FLAPERONS)
    flaperonInput();
#endif
}

void RateMode::run(void)
{
    if (!Mode::imuDataHealthy())
    {
        return;
    }

    process();
#if defined(RUDDER_MIX_IN_RATE)
    Mode::rudderMixer();
#endif

    Mode::output_rpy[0] = Mode::rollPIDF.Compute(Mode::input_rpy[0], imu.getGyroX());
    Mode::output_rpy[1] = Mode::pitchPIDF.Compute(Mode::input_rpy[1], imu.getGyroY());
    Mode::output_rpy[2] = Mode::yawPIDF.Compute(Mode::input_rpy[2], imu.getGyroZ());

    AirplaneMixer::Outputs outputs = airplaneMixer.mix(Mode::output_rpy[0], Mode::output_rpy[1], Mode::output_rpy[2]);

    Mode::SRVout[Actuators::Channel::CH1] = map(outputs.leftAileron, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH2] = map(outputs.rightAileron, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH3] = map(outputs.elevator, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH4] = map(outputs.rudder, -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
#if defined(USE_FLAPERONS)
    Mode::setFlaperons();
#endif
    actuators.setServoOut(SRVout); // Set servo output
}