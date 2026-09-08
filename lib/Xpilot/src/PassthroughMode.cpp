#include "Mode.h"

void PassthroughMode::enter(void)
{
    rollSlew.reset(output_rpy[0]);
    pitchSlew.reset(output_rpy[1]);
    yawSlew.reset(output_rpy[2]);
}

void PassthroughMode::run(void)
{
    output_rpy[0] = rollSlew.update(input_rpy[0]);
    output_rpy[1] = pitchSlew.update(input_rpy[1]);
    output_rpy[2] = yawSlew.update(input_rpy[2]);
}