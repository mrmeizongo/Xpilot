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

    ROLL_INPUT *= config().flightConfig.maxRollRateDegs;

    PITCH_INPUT *= config().flightConfig.maxPitchRateDegs;

    YAW_INPUT *= config().flightConfig.maxYawRateDegs;

    YAW_INPUT = airplaneMixer.mixRudderInput(ROLL_INPUT, YAW_INPUT);
}

void RateMode::run(void)
{
    ROLL_OUTPUT = rollPIDF.Compute(ROLL_INPUT, imu_g[0]);
    PITCH_OUTPUT = pitchPIDF.Compute(PITCH_INPUT, imu_g[1]);
    YAW_OUTPUT = yawPIDF.Compute(YAW_INPUT, imu_g[2]);
}