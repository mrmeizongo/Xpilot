#include "Mode.h"

void PassthroughMode::enter(void)
{
    rollSlew.reset(output_rpy[0]);
    pitchSlew.reset(output_rpy[1]);
    yawSlew.reset(output_rpy[2]);
}

void PassthroughMode::update(void)
{
    if (radio.inFailsafe())
    {
        controlFailsafe();
        return;
    }

    input_rpy[0] = getNormalizedInput(radio.getPWM(Radio::CHANNELS::ROLL), config().rollRC.min, config().rollRC.trim, config().rollRC.max, config().rollRC.deadband) *
                   config().flightConfig.controlResolution;

    input_rpy[1] = getNormalizedInput(radio.getPWM(Radio::CHANNELS::PITCH), config().pitchRC.min, config().pitchRC.trim, config().pitchRC.max, config().pitchRC.deadband) *
                   config().flightConfig.controlResolution;

    input_rpy[2] = getNormalizedInput(radio.getPWM(Radio::CHANNELS::YAW), config().yawRC.min, config().yawRC.trim, config().yawRC.max, config().yawRC.deadband) *
                   config().flightConfig.controlResolution;

    Mode::update();
}

void PassthroughMode::run(void)
{
    output_rpy[0] = rollSlew.update(input_rpy[0]);
    output_rpy[1] = pitchSlew.update(input_rpy[1]);
    output_rpy[2] = yawSlew.update(input_rpy[2]);

    output_rpy[0] = constrain(output_rpy[0], -config().flightConfig.controlResolution, config().flightConfig.controlResolution);
    output_rpy[1] = constrain(output_rpy[1], -config().flightConfig.controlResolution, config().flightConfig.controlResolution);
    output_rpy[2] = constrain(output_rpy[2], -config().flightConfig.controlResolution, config().flightConfig.controlResolution);

    AirplaneMixer::Outputs outputs = airplaneMixer.mix(output_rpy[0], output_rpy[1], output_rpy[2]);

    SRVout[Actuators::Channel::CH1] = map(outputs.leftAileron, config().flightConfig.controlResolution,
                                          config().srvConfig.min, config().srvConfig.max);

    SRVout[Actuators::Channel::CH2] = map(outputs.rightAileron, config().flightConfig.controlResolution,
                                          config().srvConfig.min, config().srvConfig.max);

    SRVout[Actuators::Channel::CH3] = map(outputs.elevator, config().flightConfig.controlResolution,
                                          config().srvConfig.min, config().srvConfig.max);

    SRVout[Actuators::Channel::CH4] = map(outputs.rudder, config().flightConfig.controlResolution,
                                          config().srvConfig.min, config().srvConfig.max);
#if defined(USE_FLAPERONS)
    flaperonMixer();
#endif
    actuators.setServoOut(SRVout); // Set servo output
}