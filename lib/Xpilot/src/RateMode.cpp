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
    output_trpy[Radio::CHANNEL::ROLL] = rollPIDF.Compute(input_trpy[Radio::CHANNEL::ROLL], getIMU_G(IMU::X_AXIS));
    output_trpy[Radio::CHANNEL::PITCH] = pitchPIDF.Compute(input_trpy[Radio::CHANNEL::PITCH], getIMU_G(IMU::Axis::Y_AXIS));
    output_trpy[Radio::CHANNEL::YAW] = yawPIDF.Compute(input_trpy[Radio::CHANNEL::YAW], getIMU_G(IMU::Axis::Z_AXIS));
}