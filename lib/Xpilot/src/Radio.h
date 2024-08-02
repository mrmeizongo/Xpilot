#ifndef _RADIO_H
#define _RADIO_H

#include <Arduino.h>
#include "config.h"

class Radio
{
public:
    Radio(void);
    void init(void);
    void processInput(void);

    uint16_t getAileronPWM(void) { return aileronPulseWidth; }
    uint16_t getElevatorPWM(void) { return elevatorPulseWidth; }
    uint16_t getRudderPWM(void) { return rudderPulseWidth; }

#if defined(IO_DEBUG)
    void printInput(void);
#endif

private:
    uint16_t aileronPulseWidth = 0;  // Aileron values obtained from transmitter through interrupts
    uint16_t elevatorPulseWidth = 0; // Elevator values obtained from transmitter through interrupts
    uint16_t rudderPulseWidth = 0;   // Rudder values obtained from transmitter through interrupts
    uint16_t modePulseWidth = 0;     // Mode values obtained from transmitter through interrupts
};

extern Radio rx;
#endif