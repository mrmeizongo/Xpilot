#include "IMU.h"
#include "Mode.h"

inline int32_t stabilizeDemand(int16_t input,    // -1000 : +1000
                               int32_t angle,    // deg * Control::RESOLUTION
                               int16_t maxRate,  // deg/s
                               int16_t maxAngle, // deg
                               float levelKp)
{
    const int32_t rateLimit = static_cast<int32_t>(maxRate) * Control::RESOLUTION;

    const int32_t angleLimit = static_cast<int32_t>(maxAngle) * Control::RESOLUTION;

    const int32_t target = input > 0 ? angleLimit : (input < 0) ? -angleLimit : 0;

    int32_t demand;

    const bool correctAttitude =
        input == 0 || (input > 0 && angle > angleLimit) || (input < 0 && angle < -angleLimit);

    if (correctAttitude)
    {
        demand = (target - angle) * levelKp;
    }
    else
    {
        demand = static_cast<int32_t>(input) * maxRate;
    }

    return constrain(demand, -rateLimit, rateLimit);
}

void StabilizeMode::enter(void) { resetControllers(); }

// Yaw is rate controlled
void StabilizeMode::update(void)
{
    Mode::update();

    input_rpy[2] *= config().flightConfig.maxYawRateDegs;
}

void StabilizeMode::run(void)
{
    int32_t rollDemand =
        stabilizeDemand(input_rpy[0], imu_rpy[0], config().flightConfig.maxRollRateDegs,
                        config().flightConfig.maxRollAngleDegs, config().flightConfig.rollAngleKp);

    int32_t pitchDemand = stabilizeDemand(
        input_rpy[1], imu_rpy[1], config().flightConfig.maxPitchRateDegs,
        config().flightConfig.maxPitchAngleDegs, config().flightConfig.pitchAngleKp);

    output_rpy[0] = rollPIDF.Compute(rollDemand, imu_g[0]);
    output_rpy[1] = pitchPIDF.Compute(pitchDemand, imu_g[1]);
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