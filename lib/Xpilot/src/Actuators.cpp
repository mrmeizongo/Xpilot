#include <Arduino.h>
#include "Actuators.h"
#include "GPIODef.h"
#include "FlightConfigAccess.h"

// ISO C++ forbids in-class initialization of non-const static members
// We define them here instead
Servo Actuators::controlServo[CHANNEL_COUNT]{};
int16_t Actuators::channelOut[CHANNEL_COUNT]{};

Actuators::Actuators(void)
{
}

// Set up output servos
void Actuators::init(void)
{
    controlServo[CH1].attach(AIL1PIN_OUTPUT, config().srvConfig.min, config().srvConfig.max);
    controlServo[CH2].attach(AIL2PIN_OUTPUT, config().srvConfig.min, config().srvConfig.max);
    controlServo[CH3].attach(ELEVPIN_OUTPUT, config().srvConfig.min, config().srvConfig.max);
    controlServo[CH4].attach(RUDDPIN_OUTPUT, config().srvConfig.min, config().srvConfig.max);
#if defined(USE_AUXOUT1)
    controlServo[CH5].attach(AUX1PIN_OUTPUT, config().srvConfig.min, config().srvConfig.max);
#endif
}

// Set individual servo output values in microseconds
void Actuators::setServoOut(Actuators::Channel ch, int16_t value)
{
    if (ch < CHANNEL_START || ch >= CHANNEL_COUNT)
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
#if defined(USE_AUXOUT1)
    channelOut[CH5] = SRVout[CH5];
#endif
}

// Get individual servo output value in microseconds
int16_t Actuators::getServoOut(Actuators::Channel ch)
{
    if (ch < CHANNEL_START || ch >= CHANNEL_COUNT)
        return -1;

    return controlServo[ch].readMicroseconds();
}

// Write current servo output values to the servos
void Actuators::writeServos(void)
{
    writeServos(channelOut);
}

// Write all servo output values at once using an array
void Actuators::writeServos(const int16_t (&SRVout)[CHANNEL_COUNT])
{
    int16_t srv1 = constrain(SRVout[Actuators::Channel::CH1], config().srvConfig.min, config().srvConfig.max);
    int16_t srv2 = constrain(SRVout[Actuators::Channel::CH2], config().srvConfig.min, config().srvConfig.max);
    int16_t srv3 = constrain(SRVout[Actuators::Channel::CH3], config().srvConfig.min, config().srvConfig.max);
    int16_t srv4 = constrain(SRVout[Actuators::Channel::CH4], config().srvConfig.min, config().srvConfig.max);
#if defined(USE_AUXOUT1)
    uint16_t srv5 = constrain((uint16_t)SRVout[Actuators::Channel::CH5], config().actuatorSRV.min, config().actuatorSRV.max);
#endif

    controlServo[CH1].writeMicroseconds(srv1);
    controlServo[CH2].writeMicroseconds(srv2);
    controlServo[CH3].writeMicroseconds(srv3);
    controlServo[CH4].writeMicroseconds(srv4);
#if defined(USE_AUXOUT1)
    controlServo[CH5].writeMicroseconds(srv5);
#endif
}

Actuators actuators;