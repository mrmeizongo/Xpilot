#include <Arduino.h>
#include "Actuators.h"
#include "GPIOConfig.h"
#include "FlightConfigAccess.h"

// ISO C++ forbids in-class initialization of non-const static members
// We define them here instead
Servo Actuators::controlServo[CHANNEL_COUNT]{};
int16_t Actuators::channelOut[CHANNEL_COUNT]{};

Actuators::Actuators(void) {}

// Set up output servos
void Actuators::init(void)
{
    controlServo[CH1].attach(THROTTLEPIN_OUTPUT, config().auxSrvConfig.min, config().auxSrvConfig.max);
    controlServo[CH2].attach(AIL1PIN_OUTPUT, config().rollSrvConfig.min, config().rollSrvConfig.max);
    controlServo[CH3].attach(AIL2PIN_OUTPUT, config().rollSrvConfig.min, config().rollSrvConfig.max);
    controlServo[CH4].attach(ELEVPIN_OUTPUT, config().pitchSrvConfig.min, config().pitchSrvConfig.max);
    controlServo[CH5].attach(RUDDPIN_OUTPUT, config().yawSrvConfig.min, config().yawSrvConfig.max);
}

// Set individual servo output values in microseconds
void Actuators::setServoOut(Actuators::Channel ch, int16_t value)
{
    if (ch < 0U || ch >= CHANNEL_COUNT)
        return;

    channelOut[ch] = value;
}

// Set all servo output values at once using an array
void Actuators::setServoOut(const int16_t (&SRVout)[CHANNEL_COUNT])
{
    channelOut[CH1] = SRVout[CH1];
    channelOut[CH2] = SRVout[CH2];
    channelOut[CH3] = SRVout[CH3];
    channelOut[CH4] = SRVout[CH4];
    channelOut[CH5] = SRVout[CH5];
}

// Get individual servo output value in microseconds
int16_t Actuators::getServoOut(Actuators::Channel ch)
{
    if (ch < 0U || ch >= CHANNEL_COUNT)
        return -1;

    return controlServo[ch].readMicroseconds();
}

// Write current servo output values to the servos
void Actuators::writeServos(void) { writeServos(channelOut); }

// Write all servo output values at once using an array
void Actuators::writeServos(const int16_t (&SRVout)[CHANNEL_COUNT])
{
    controlServo[CH1].writeMicroseconds(SRVout[CH1]);
    controlServo[CH2].writeMicroseconds(SRVout[CH2]);
    controlServo[CH3].writeMicroseconds(SRVout[CH3]);
    controlServo[CH4].writeMicroseconds(SRVout[CH4]);
    controlServo[CH5].writeMicroseconds(SRVout[CH5]);
}

Actuators actuators;