#include "IMU.h"
#include "Mode.h"

void RateMode::enter(void)
{
    rollPIDF.reset();
    pitchPIDF.reset();
    yawPIDF.reset();
}

// Convert radio input to rate demands for all channels
void RateMode::update(void)
{
    Mode::update();

    input_rpy[0] *= config().flightConfig.maxRollRateDegs;

    input_rpy[1] *= config().flightConfig.maxPitchRateDegs;

    input_rpy[2] *= config().flightConfig.maxYawRateDegs;

    applyRudderMix();
}

void RateMode::run(void)
{
    output_rpy[0] = rollPIDF.Compute(input_rpy[0], imu_g[0]);
    output_rpy[1] = pitchPIDF.Compute(input_rpy[1], imu_g[1]);
    output_rpy[2] = yawPIDF.Compute(input_rpy[2], imu_g[2]);
}