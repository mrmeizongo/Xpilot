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

    input_trpy[Radio::CHANNEL::ROLL] *= config().flightConfig.maxRollRateDegs;

    input_trpy[Radio::CHANNEL::PITCH] *= config().flightConfig.maxPitchRateDegs;

    input_trpy[Radio::CHANNEL::YAW] *= config().flightConfig.maxYawRateDegs;

    input_trpy[Radio::CHANNEL::YAW] =
        airplaneMixer.mixRudderInput(input_trpy[Radio::CHANNEL::ROLL], input_trpy[Radio::CHANNEL::YAW]);
}

void RateMode::run(void)
{
    output_trpy[Radio::CHANNEL::ROLL] = rollPIDF.Compute(input_trpy[Radio::CHANNEL::ROLL], imu_g[0]);
    output_trpy[Radio::CHANNEL::PITCH] = pitchPIDF.Compute(input_trpy[Radio::CHANNEL::PITCH], imu_g[1]);
    output_trpy[Radio::CHANNEL::YAW] = yawPIDF.Compute(input_trpy[Radio::CHANNEL::YAW], imu_g[2]);
}