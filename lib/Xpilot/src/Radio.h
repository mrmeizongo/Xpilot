#ifndef _RADIO_H
#define _RADIO_H

#include <inttypes.h>
#include "ModeController.h"
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
    FlightMode previousMode;
};

class Radio
{
public:
    Radio(void);
    void init(void);
    void processInput(void);

    friend void ModeController::updateMode();

    int16_t getRxRoll(void) { return rx.roll; }
    int16_t getRxPitch(void) { return rx.pitch; }
    int16_t getRxYaw(void) { return rx.yaw; }
    FlightMode getRxCurrentMode(void) { return rx.currentMode; }

private:
    Control rx;
};

extern Radio radio;
#endif