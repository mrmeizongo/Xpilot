#include "Mode.h"
#include "IMU.h"

void RateMode::enter(void)
{
    resetControllers();
}

void RateMode::update(void)
{
    if (radio.inFailsafe())
    {
        controlFailsafe();
        return;
    }

    input_rpy[0] = getNormalizedInput(radio.getPWM(Radio::CHANNELS::ROLL), rollConfig.min, rollConfig.trim, rollConfig.max, rollConfig.deadband) *
                   fConfig.maxRollRateDegs;

    input_rpy[1] = getNormalizedInput(radio.getPWM(Radio::CHANNELS::PITCH), pitchConfig.min, pitchConfig.trim, pitchConfig.max, pitchConfig.deadband) *
                   fConfig.maxRollRateDegs;

    input_rpy[2] = getNormalizedInput(radio.getPWM(Radio::CHANNELS::YAW), yawConfig.min, yawConfig.trim, yawConfig.max, yawConfig.deadband) *
                   fConfig.maxRollRateDegs;

    Mode::update();
}

void RateMode::run(void)
{
    output_rpy[0] = rollPIDF.Compute(input_rpy[0], imu_g[0]);
    output_rpy[1] = pitchPIDF.Compute(input_rpy[1], imu_g[1]);
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