#include "Mode.h"
#include "IMU.h"

void StabilizeMode::enter(void)
{
    resetControllers();
}

void StabilizeMode::update(void)
{
    if (radio.inFailsafe())
    {
        controlFailsafe();
        return;
    }

    input_rpy[0] = normalizeInput(radio.getPWM(Radio::CHANNELS::ROLL), config().rollRC.min, config().rollRC.trim, config().rollRC.max, config().rollRC.deadband) *
                   config().flightConfig.maxRollAngleDegs;

    input_rpy[1] = normalizeInput(radio.getPWM(Radio::CHANNELS::PITCH), config().pitchRC.min, config().pitchRC.trim, config().pitchRC.max, config().pitchRC.deadband) *
                   config().flightConfig.maxPitchAngleDegs;

    input_rpy[2] = normalizeInput(radio.getPWM(Radio::CHANNELS::YAW), config().yawRC.min, config().yawRC.trim, config().yawRC.max, config().yawRC.deadband) *
                   config().flightConfig.maxYawRateDegs;

    rudderMixer();
    Mode::update();
}

void StabilizeMode::run(void)
{

    int16_t rollError = input_rpy[0] - imu_rpy[0];
    int16_t pitchError = input_rpy[1] - imu_rpy[1];

    rollError = rollError * config().flightConfig.rollAngleKp;
    pitchError = pitchError * config().flightConfig.pitchAngleKp;

    rollError = constrain(rollError, -config().flightConfig.maxRollRateDegs, config().flightConfig.maxRollRateDegs);
    pitchError = constrain(pitchError, -config().flightConfig.maxPitchRateDegs, config().flightConfig.maxPitchRateDegs);

    output_rpy[0] = rollPIDF.Compute(rollError, imu_g[0]);
    output_rpy[1] = pitchPIDF.Compute(pitchError, imu_g[1]);
    output_rpy[2] = yawPIDF.Compute(input_rpy[2], imu_g[2]);

    output_rpy[0] = constrain(output_rpy[0], -config().flightConfig.controlResolution, config().flightConfig.controlResolution);
    output_rpy[1] = constrain(output_rpy[1], -config().flightConfig.controlResolution, config().flightConfig.controlResolution);
    output_rpy[2] = constrain(output_rpy[2], -config().flightConfig.controlResolution, config().flightConfig.controlResolution);

    AirplaneMixer::Outputs outputs = airplaneMixer.mix(output_rpy[0], output_rpy[1], output_rpy[2]);

    SRVout[Actuators::Channel::CH1] = map(outputs.leftAileron, config().flightConfig.controlResolution,
                                          config().srvConfig.min, config().srvConfig.max);

    SRVout[Actuators::Channel::CH2] = map(outputs.rightAileron, config().flightConfig.controlResolution,
                                          config().srvConfig.min, config().srvConfig.max);

    SRVout[Actuators::Channel::CH3] = map(outputs.elevator, config().flightConfig.controlResolution,
                                          config().srvConfig.min, config().srvConfig.max);

    SRVout[Actuators::Channel::CH4] = map(outputs.rudder, config().flightConfig.controlResolution,
                                          config().srvConfig.min, config().srvConfig.max);
#if defined(USE_FLAPERONS)
    flaperonMixer();
#endif
    actuators.setServoOut(SRVout); // Set servo output
}