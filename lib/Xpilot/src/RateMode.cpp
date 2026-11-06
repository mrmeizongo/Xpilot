#include "Mode.h"
#include "IMU.h"

void RateMode::enter(void) { Mode::resetControllers(); }

// Convert radio input to rate demands for all channels
void RateMode::update(void)
{
    Mode::update();

    input_trpy[Radio::CHANNEL::ROLL] *= config().flightConfig.maxRollRateDegs;

    input_trpy[Radio::CHANNEL::PITCH] *= config().flightConfig.maxPitchRateDegs;

    input_trpy[Radio::CHANNEL::YAW] *= config().flightConfig.maxYawRateDegs;
}

void RateMode::run(void)
{
    output_trpy[Radio::CHANNEL::ROLL] = rollPIDF.Compute(input_trpy[Radio::CHANNEL::ROLL], imu.getG(IMU::X_AXIS));
    output_trpy[Radio::CHANNEL::PITCH] = pitchPIDF.Compute(input_trpy[Radio::CHANNEL::PITCH], imu.getG(IMU::Y_AXIS));
    output_trpy[Radio::CHANNEL::YAW] = yawPIDF.Compute(input_trpy[Radio::CHANNEL::YAW], imu.getG(IMU::Z_AXIS));
}