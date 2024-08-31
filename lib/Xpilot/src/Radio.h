#ifndef _RADIO_H
#define _RADIO_H

#include <inttypes.h>
#include "config.h"

enum FlightMode : uint8_t
{
    PASSTHROUGH = 1U,
    RATE,
    STABILIZE
};

struct Control
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    FlightMode currentMode;

    Control(void) {}

    Control(int16_t _roll, int16_t _pitch, int16_t _yaw, FlightMode _currentMode)
        : roll{_roll}, pitch{_pitch}, yaw{_yaw}, currentMode{_currentMode} {}
};

class Radio
{
public:
    Radio(void);
    void init(void);
    void processInput(void);

    int16_t getRxRoll(void) { return rx.roll; }
    int16_t getRxPitch(void) { return rx.pitch; }
    int16_t getRxYaw(void) { return rx.yaw; }
    FlightMode getRxCurrentMode(void) { return rx.currentMode; }

private:
    Control rx;
};

extern Radio radio;
#endif