#include "Mode.h"
#include "IMU.h"

void StabilizeMode::enter(void)
{
    Mode::resetControllers();
}

void StabilizeMode::process(void)
{
    if (radio.inFailsafe())
    {
        Mode::controlFailsafe();
        return;
    }

    Mode::input_rpy[0] = FILTERED_NORM_INPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND) * MAX_ROLL_ANGLE_DEGS;
    Mode::input_rpy[1] = FILTERED_NORM_INPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND) * MAX_PITCH_ANGLE_DEGS;
    Mode::input_rpy[2] = FILTERED_NORM_INPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND) * MAX_YAW_RATE_DEGS;
#if defined(USE_FLAPERONS)
    flaperonInput();
#endif
}

void StabilizeMode::run(void)
{
    if (!Mode::imuDataHealthy())
    {
        return;
    }

    process();
#if defined(RUDDER_MIX_IN_STABILIZE)
    Mode::rudderMixer();
#endif

    int16_t rollDemand = Mode::input_rpy[0] - imu.getRoll();
    int16_t pitchDemand = Mode::input_rpy[1] - imu.getPitch();
    rollDemand = map(rollDemand, -MAX_ROLL_ANGLE_DEGS, MAX_ROLL_ANGLE_DEGS, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
    pitchDemand = map(pitchDemand, -MAX_PITCH_ANGLE_DEGS, MAX_PITCH_ANGLE_DEGS, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);

    Mode::output_rpy[0] = Mode::rollPIDF.Compute(rollDemand, imu.getGyroX());
    Mode::output_rpy[1] = Mode::pitchPIDF.Compute(pitchDemand, imu.getGyroY());
    Mode::output_rpy[2] = Mode::yawPIDF.Compute(Mode::input_rpy[2], imu.getGyroZ());

    Mode::planeMixer(Mode::output_rpy[0], Mode::output_rpy[1], Mode::output_rpy[2]);
    Mode::SRVout[Actuators::Channel::CH1] = map(Mode::SRVout[Actuators::Channel::CH1], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH2] = map(Mode::SRVout[Actuators::Channel::CH2], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH3] = map(Mode::SRVout[Actuators::Channel::CH3], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
    Mode::SRVout[Actuators::Channel::CH4] = map(Mode::SRVout[Actuators::Channel::CH4], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
#if defined(USE_AUXOUT1)
    Mode::SRVout[Actuators::Channel::CH5] = map(Mode::SRVout[Actuators::Channel::CH5], -MAX_PID_OUTPUT, MAX_PID_OUTPUT, SERVO_MIN_PWM, SERVO_MAX_PWM);
#endif
#if defined(USE_FLAPERONS)
    Mode::setFlaperons();
#endif
}

void StabilizeMode::controlFailsafe(void)
{
    Mode::input_rpy[0] = 5; // 5 degree roll to the right
    Mode::input_rpy[1] = 0;
    Mode::input_rpy[2] = 0;
#if defined(USE_FLAPERONS)
    Mode::flaperonOut = 0;
#endif
}