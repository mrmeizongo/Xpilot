#include "Mode.h"

void PassthroughMode::enter(void)
{
    rollSlew.reset(ROLL_OUTPUT);
    pitchSlew.reset(PITCH_OUTPUT);
    yawSlew.reset(YAW_OUTPUT);
}

void PassthroughMode::run(void)
{
    ROLL_OUTPUT = rollSlew.update(ROLL_INPUT);
    PITCH_OUTPUT = pitchSlew.update(PITCH_INPUT);
    YAW_OUTPUT = yawSlew.update(YAW_INPUT);
}