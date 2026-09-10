#include "IMU.h"
#include "Mode.h"

inline int32_t stabilizeDemand(int32_t input, int32_t angle, int16_t maxRate, int16_t maxAngle, float levelKp)
{
    const int32_t rateLimit = static_cast<int32_t>(maxRate) * config().controlConfig.controlResolution;

    const int32_t angleLimit = static_cast<int32_t>(maxAngle) * config().controlConfig.controlResolution;

    const int32_t correctionTarget = input > 0 ? angleLimit : (input < 0) ? -angleLimit : 0;

    int32_t demand;

    const bool correctAttitude = input == 0 || (input > 0 && angle > angleLimit) || (input < 0 && angle < -angleLimit);

    if (correctAttitude)
    {
        demand = (correctionTarget - angle) * levelKp;
    }
    else
    {
        demand = input * maxRate;
    }

    return constrain(demand, -rateLimit, rateLimit);
}

void StabilizeMode::enter(void)
{
    rollPIDF.reset();
    pitchPIDF.reset();
    yawPIDF.reset();
}

// Yaw is rate controlled
void StabilizeMode::update(void)
{
    Mode::update();

    YAW_INPUT = airplaneMixer.mixRudderInput(ROLL_INPUT, YAW_INPUT);

    YAW_INPUT *= config().flightConfig.maxYawRateDegs;
}

void StabilizeMode::run(void)
{
    int32_t rollDemand = stabilizeDemand(ROLL_INPUT,
                                         imu_rpy[0],
                                         config().flightConfig.maxRollRateDegs,
                                         config().flightConfig.maxRollAngleDegs,
                                         config().flightConfig.rollAngleKp);

    int32_t pitchDemand = stabilizeDemand(PITCH_INPUT,
                                          imu_rpy[1],
                                          config().flightConfig.maxPitchRateDegs,
                                          config().flightConfig.maxPitchAngleDegs,
                                          config().flightConfig.pitchAngleKp);

    ROLL_OUTPUT = rollPIDF.Compute(rollDemand, imu_g[0]);
    PITCH_OUTPUT = pitchPIDF.Compute(pitchDemand, imu_g[1]);
    YAW_OUTPUT = yawPIDF.Compute(input_trpy[3], imu_g[2]);
}