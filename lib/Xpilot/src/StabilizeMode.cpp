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

    input_rpy[0] = normalizeInput(radio.getPWM(Radio::CHANNEL::ROLL), config().rollRC.min, config().rollRC.trim, config().rollRC.max, config().rollRC.deadband) *
                   config().flightConfig.maxRollAngleDegs;

    input_rpy[1] = normalizeInput(radio.getPWM(Radio::CHANNEL::PITCH), config().pitchRC.min, config().pitchRC.trim, config().pitchRC.max, config().pitchRC.deadband) *
                   config().flightConfig.maxPitchAngleDegs;

    input_rpy[2] = normalizeInput(radio.getPWM(Radio::CHANNEL::YAW), config().yawRC.min, config().yawRC.trim, config().yawRC.max, config().yawRC.deadband) *
                   config().flightConfig.maxYawRateDegs;

    Mode::update();
}

void StabilizeMode::run(void)
{
    int16_t rollError = input_rpy[0] - imu_rpy[0];
    int16_t pitchError = input_rpy[1] - imu_rpy[1];

    rollError = rollError * config().flightConfig.rollAngleKp;
    pitchError = pitchError * config().flightConfig.pitchAngleKp;

    rollError = constrain(rollError, -config().flightConfig.maxRollRateDegs, config().flightConfig.maxRollRateDegs);
    pitchError = constrain(pitchError, -config().flightConfig.maxPitchRateDegs, config().flightConfig.maxPitchRateDegs);

    output_rpy[0] = rollPIDF.Compute(rollError, imu_g[0]);
    output_rpy[1] = pitchPIDF.Compute(pitchError, imu_g[1]);
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