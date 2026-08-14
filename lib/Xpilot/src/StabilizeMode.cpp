#include "Mode.h"
#include "IMU.h"

void StabilizeMode::enter(void)
{
    Mode::resetControllers();
    Mode::airplaneMixer.setCommandLimit(MAX_PID_OUTPUT);
}

void StabilizeMode::update(void)
{
    if (radio.inFailsafe())
    {
        Mode::controlFailsafe();
        return;
    }

    Mode::input_rpy[0] = FILTERED_NORM_INPUT(radio.getRxRollPWM(), ROLL_INPUT_DEADBAND) * MAX_ROLL_ANGLE_DEGS;
    Mode::input_rpy[1] = FILTERED_NORM_INPUT(radio.getRxPitchPWM(), PITCH_INPUT_DEADBAND) * MAX_PITCH_ANGLE_DEGS;
    Mode::input_rpy[2] = FILTERED_NORM_INPUT(radio.getRxYawPWM(), YAW_INPUT_DEADBAND) * MAX_YAW_RATE_DEGS;

    Mode::update();
}

void StabilizeMode::run(void)
{
    int16_t rollDemand = Mode::input_rpy[0] - Mode::imu_rpy[0];
    int16_t pitchDemand = Mode::input_rpy[1] - Mode::imu_rpy[1];
    rollDemand = map(rollDemand, -MAX_ROLL_ANGLE_DEGS, MAX_ROLL_ANGLE_DEGS, -MAX_ROLL_RATE_DEGS, MAX_ROLL_RATE_DEGS);
    pitchDemand = map(pitchDemand, -MAX_PITCH_ANGLE_DEGS, MAX_PITCH_ANGLE_DEGS, -MAX_PITCH_RATE_DEGS, MAX_PITCH_RATE_DEGS);

    Mode::output_rpy[0] = Mode::rollPIDF.Compute(rollDemand, Mode::imu_g[0]);
    Mode::output_rpy[1] = Mode::pitchPIDF.Compute(pitchDemand, Mode::imu_g[0]);
    Mode::output_rpy[2] = Mode::yawPIDF.Compute(Mode::input_rpy[2], Mode::imu_g[0]);

#if defined(RUDDER_MIX_IN_STABILIZE)
    Mode::rudderMixer();
#endif

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