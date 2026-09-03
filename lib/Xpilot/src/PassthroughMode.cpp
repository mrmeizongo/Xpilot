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

    input_rpy[0] = normalizeInput(radio.getPWM(Radio::CHANNEL::ROLL),
                                  config().rollRC.min, config().rollRC.trim, config().rollRC.max, config().rollRC.deadband);

    input_rpy[1] = normalizeInput(radio.getPWM(Radio::CHANNEL::PITCH),
                                  config().pitchRC.min, config().pitchRC.trim, config().pitchRC.max, config().pitchRC.deadband);

    input_rpy[2] = normalizeInput(radio.getPWM(Radio::CHANNEL::YAW),
                                  config().yawRC.min, config().yawRC.trim, config().yawRC.max, config().yawRC.deadband);

    Mode::update();
}

void PassthroughMode::run(void)
{
    output_rpy[0] = rollSlew.update(input_rpy[0]);
    output_rpy[1] = pitchSlew.update(input_rpy[1]);
    output_rpy[2] = yawSlew.update(input_rpy[2]);

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