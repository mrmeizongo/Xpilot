#include "Mode.h"

void PassthroughMode::enter(void)
{
    rollSlew.reset(output_trpy[Radio::CHANNEL::ROLL]);
    pitchSlew.reset(output_trpy[Radio::CHANNEL::PITCH]);
    yawSlew.reset(output_trpy[Radio::CHANNEL::YAW]);
}

void PassthroughMode::run(void)
{
    output_trpy[Radio::CHANNEL::ROLL] = rollSlew.update(input_trpy[Radio::CHANNEL::ROLL]);
    output_trpy[Radio::CHANNEL::PITCH] = pitchSlew.update(input_trpy[Radio::CHANNEL::PITCH]);
    output_trpy[Radio::CHANNEL::YAW] = yawSlew.update(input_trpy[Radio::CHANNEL::YAW]);
}