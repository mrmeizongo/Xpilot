#ifndef _RADIO_H
#define _RADIO_H

#include <inttypes.h>
#include "ModeController.h"
#include "config.h"

typedef enum : uint8_t
{
    passthrough = 1U,
    rate = 2U,
    stabilize = 3U
} FlightMode;

typedef struct
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    FlightMode currentMode;
    FlightMode previousMode;
} Control;

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
    Control rx{0, 0, 0, FlightMode::rate, FlightMode::rate}; // Sets default radio values on power up
};

extern Radio radio;
#endif