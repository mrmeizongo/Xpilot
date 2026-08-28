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

    input_rpy[0] = getNormalizedInput(radio.getPWM(Radio::CHANNELS::ROLL), rollConfig.min, rollConfig.trim, rollConfig.max, rollConfig.deadband) *
                   fConfig.maxRollAngleDegs;

    input_rpy[1] = getNormalizedInput(radio.getPWM(Radio::CHANNELS::PITCH), pitchConfig.min, pitchConfig.trim, pitchConfig.max, pitchConfig.deadband) *
                   fConfig.maxPitchAngleDegs;

    input_rpy[2] = getNormalizedInput(radio.getPWM(Radio::CHANNELS::YAW), yawConfig.min, yawConfig.trim, yawConfig.max, yawConfig.deadband) *
                   fConfig.maxYawRateDegs;

    Mode::update();
}

void StabilizeMode::run(void)
{
    int16_t rollDemand = input_rpy[0] - imu_rpy[0];
    int16_t pitchDemand = input_rpy[1] - imu_rpy[1];

    rollDemand = map(rollDemand, fConfig.maxRollAngleDegs,
                     -fConfig.maxRollRateDegs, fConfig.maxRollRateDegs);

    pitchDemand = map(pitchDemand, fConfig.maxPitchAngleDegs,
                      -fConfig.maxPitchRateDegs, fConfig.maxPitchRateDegs);

    output_rpy[0] = rollPIDF.Compute(rollDemand, imu_g[0]);
    output_rpy[1] = pitchPIDF.Compute(pitchDemand, imu_g[1]);
    output_rpy[2] = yawPIDF.Compute(input_rpy[2], imu_g[2]);

    rudderMixer();

    AirplaneMixer::Outputs outputs = airplaneMixer.mix(output_rpy[0], output_rpy[1], output_rpy[2]);

    SRVout[Actuators::Channel::CH1] = map(outputs.leftAileron, fConfig.controlResolution,
                                          srvConfig.min, srvConfig.max);

    SRVout[Actuators::Channel::CH2] = map(outputs.rightAileron, fConfig.controlResolution,
                                          srvConfig.min, srvConfig.max);

    SRVout[Actuators::Channel::CH3] = map(outputs.elevator, fConfig.controlResolution,
                                          srvConfig.min, srvConfig.max);

    SRVout[Actuators::Channel::CH4] = map(outputs.rudder, fConfig.controlResolution,
                                          srvConfig.min, srvConfig.max);
#if defined(USE_FLAPERONS)
    flaperonMixer();
#endif
    actuators.setServoOut(SRVout); // Set servo output
}