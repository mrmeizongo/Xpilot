#include "IMU.h"
#include "Mode.h"

void RateMode::enter(void) { resetControllers(); }

// Convert radio input to rate demands for all channels
void RateMode::update(void)
{
    Mode::update();

    input_rpy[0] *= config().flightConfig.maxRollRateDegs;

    input_rpy[1] *= config().flightConfig.maxPitchRateDegs;

    input_rpy[2] *= config().flightConfig.maxYawRateDegs;
}

void RateMode::run(void)
{
    output_rpy[0] = rollPIDF.Compute(input_rpy[0], imu_g[0]);
    output_rpy[1] = pitchPIDF.Compute(input_rpy[1], imu_g[1]);
    output_rpy[2] = yawPIDF.Compute(input_rpy[2], imu_g[2]);

    output_rpy[0] = constrain(output_rpy[0], -Control::RESOLUTION, Control::RESOLUTION);
    output_rpy[1] = constrain(output_rpy[1], -Control::RESOLUTION, Control::RESOLUTION);
    output_rpy[2] = constrain(output_rpy[2], -Control::RESOLUTION, Control::RESOLUTION);

    AirplaneMixer::Outputs outputs = airplaneMixer.mix(output_rpy[0], output_rpy[1], output_rpy[2]);

    SRVout[Actuators::Channel::CH1] = mapToSRV(outputs.leftAileron);

    SRVout[Actuators::Channel::CH2] = mapToSRV(outputs.rightAileron);

    SRVout[Actuators::Channel::CH3] = mapToSRV(outputs.elevator);

    SRVout[Actuators::Channel::CH4] = mapToSRV(outputs.rudder);
#if defined(USE_FLAPERONS)
    flaperonMixer();
#endif
    actuators.setServoOut(SRVout); // Set servo output
}