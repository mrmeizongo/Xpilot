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

    input_rpy[0] = getNormalizedInput(radio.getPWM(Radio::CHANNELS::ROLL), rollConfig.min, rollConfig.trim, rollConfig.max, rollConfig.deadband) *
                   fConfig.controlResolution;

    input_rpy[1] = getNormalizedInput(radio.getPWM(Radio::CHANNELS::PITCH), pitchConfig.min, pitchConfig.trim, pitchConfig.max, pitchConfig.deadband) *
                   fConfig.controlResolution;

    input_rpy[2] = getNormalizedInput(radio.getPWM(Radio::CHANNELS::YAW), yawConfig.min, yawConfig.trim, yawConfig.max, yawConfig.deadband) *
                   fConfig.controlResolution;

    Mode::update();
}

void PassthroughMode::run(void)
{
    output_rpy[0] = rollSlew.update(input_rpy[0]);
    output_rpy[1] = pitchSlew.update(input_rpy[1]);
    output_rpy[2] = yawSlew.update(input_rpy[2]);

    AirplaneMixer::Outputs outputs = airplaneMixer.mix(output_rpy[0], output_rpy[1], output_rpy[2]);

    SRVout[Actuators::Channel::CH1] = map(outputs.leftAileron, fConfig.controlResolution,
                                          srvConfig.min, srvConfig.max);

    SRVout[Actuators::Channel::CH2] = map(outputs.rightAileron, fConfig.controlResolution,
                                          srvConfig.min, srvConfig.max);

    SRVout[Actuators::Channel::CH3] = map(outputs.elevator, fConfig.controlResolution,
                                          srvConfig.min, srvConfig.max);

    SRVout[Actuators::Channel::CH4] = map(outputs.rudder, fConfig.controlResolution,
                                          srvConfig.min, srvConfig.max);
#if defined(USE_FLAPERONS)
    flaperonMixer();
#endif
    actuators.setServoOut(SRVout); // Set servo output
}